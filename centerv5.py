#!/usr/bin/env python3
"""
Hallway centering node with integrated obstacle stop + recovery.

Three lidar cones:
  FRONT  ±15°   around   0°  -> stop if anything within 0.5 m
  LEFT   ±22.5° around  90°  -> left wall distance for centering
  RIGHT  ±22.5° around -90°  -> right wall distance for centering

Behaviour is a small state machine:
  CENTERING   -> drive forward, keep equidistant from walls
  BACKING_UP  -> obstacle hit, reverse for self.backup_time seconds
  TURNING     -> rotate toward the more open side until front is clear

Published topics:
  /cmd_vel    geometry_msgs/Twist
"""

import math
import time
import collections
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class HallwayCenterNode(Node):

    def __init__(self):
        super().__init__('hallway_center_node')

        # -- Cone half-widths -------------------------------------------------
        self.front_half_cone = math.radians(15)    # +/-15  front
        self.side_half_cone  = math.radians(22.5)  # +/-22.5 each side

        # -- Distances --------------------------------------------------------
        self.stop_dis        = 0.3   # stop if front cone reading <= this (m)
        self.clear_dis       = 0.7   # must exceed this to resume centering (m)
        self.wall_target     = 0.8   # desired distance from a single wall (m)

        # -- Motion parameters ------------------------------------------------
        self.forward_speed   = 0.2   # m/s
        self.backup_speed    = 0.15  # m/s (magnitude; applied as negative)
        self.backup_time     = 1.0   # seconds to reverse after a hit
        self.kp              = 0.4   # proportional steering gain (position)
        self.kd              = 1.0   # derivative steering gain
        self.filter_alpha    = 0.2   # low-pass on corrected_error before derivative
        self.max_angular_z   = 1.2   # rad/s clamp (was 10 -- way too fast)
        self.turn_speed      = 0.8   # rad/s while actively turning away

        # -- PD state --------------------------------------------------------
        self.filtered_error = 0.0
        self.prev_error     = 0.0
        self.prev_time      = None

        # -- Hallway right-turn (servo) parameters ---------------------------
        self.kp_dist                = 0.12   # distance servo gain (from their code)
        self.kp_angle               = 0.3    # parallel-alignment gain (from their code)
        self.max_turn               = 0.9    # clip for servo angular output
        self.turn_target_dist       = 1.8    # fallback target if snapshot is bad (m)
        self.hallway_turn_trigger   = 4.0    # right_dist median threshold (m) to trigger
        self.turn_exit_slope_thresh = 0.05   # |slope_mag| below this = parallel (m)
        self.turn_exit_dist_max     = 3.0    # d90 must be under this for exit (m)
        self.turn_exit_consecutive  = 3      # scans of "parallel" before exiting
        self.turn_min_time          = 0.5    # min seconds in mode before exit allowed
        self.turn_max_time          = 10.0   # hard timeout (s)

        # Rolling buffer of recent dist_at_90 readings (for snapshotting target)
        self.right_dist_buffer  = collections.deque(maxlen=3)
        self.active_turn_target = 0.0
        self.turn_parallel_count = 0

        # -- Centering deadband ----------------------------------------------
        # Fraction of hallway half-width that counts as "close enough to
        # center." Inside this zone the controller commands straight-line
        # motion. Outside, the deadband is subtracted from the error so the
        # proportional response ramps smoothly from zero at the zone edge.
        # Only applies when BOTH walls are visible.
        self.deadband_frac   = 0.05   # 0.1 = middle 20% of hallway is dead

        # -- State machine ----------------------------------------------------
        self.MODE_CENTERING                = 'CENTERING'
        self.MODE_BACKING_UP               = 'BACKING_UP'
        self.MODE_TURNING                  = 'TURNING'
        self.MODE_HALLWAY_TURN_RIGHT       = 'HALLWAY_TURN_RIGHT'
        self.mode            = self.MODE_CENTERING
        self.mode_start_time = 0.0
        self.turn_dir        = 1.0   # +1 = left, -1 = right
        self.suppress_right_turn  = False

        # -- Startup delay ----------------------------------------------------
        self.ready = False
        self.create_timer(3.0, self.set_ready)

        # -- Pub / sub --------------------------------------------------------
        self.vel_pub      = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info('Hallway center node started. Waiting 3 seconds for lidar to stabilize...')

    def set_ready(self):
        if self.ready:
            return
        self.ready = True
        self.get_logger().info('Lidar stabilized. Starting hallway centering.')

    # -- Index / range helpers ------------------------------------------------

    def _cone_indices(self, msg: LaserScan, center_rad: float, half_cone_rad: float):
        start_idx = int(((center_rad - half_cone_rad) - msg.angle_min) / msg.angle_increment)
        end_idx   = int(((center_rad + half_cone_rad) - msg.angle_min) / msg.angle_increment)
        n = len(msg.ranges)
        start_idx = max(0, min(start_idx, n - 1))
        end_idx   = max(0, min(end_idx,   n - 1))
        return start_idx, end_idx

    def _valid_min(self, ranges_np: np.ndarray, msg: LaserScan,
                   center_rad: float, half_cone_rad: float) -> float:
        """Minimum valid range in a cone. Returns inf when no valid reading."""
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & (np.isfinite(cone))]
        return float(np.min(valid)) if valid.size > 0 else float('inf')

    def _valid_median(self, ranges_np: np.ndarray, msg: LaserScan,
                      center_rad: float, half_cone_rad: float) -> float:
        """Median valid range in a cone. Returns nan when no valid reading."""
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & (np.isfinite(cone))]
        return float(np.median(valid)) if valid.size > 0 else float('nan')

    def _wall_angle(self, ranges_np: np.ndarray, msg: LaserScan,
                    center_rad: float, half_cone_rad: float) -> float:
        """Fit a line through the cone's points and return wall slope angle (rad).
        Returns nan if fewer than 5 valid points or fit residuals are too high."""
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        angles = msg.angle_min + np.arange(s, e + 1) * msg.angle_increment
        mask = (cone > msg.range_min) & np.isfinite(cone)
        if np.count_nonzero(mask) < 5:
            return float('nan')
        r = cone[mask]
        a = angles[mask]
        x = r * np.cos(a)
        y = r * np.sin(a)
        m, b = np.polyfit(x, y, 1)
        # Reject bad fits (non-wall clutter, doorways, etc.)
        rms = float(np.sqrt(np.mean((y - (m * x + b)) ** 2)))
        if rms > 0.1:
            return float('nan')
        return float(np.arctan(m))

    def _dist_at_90(self, ranges_np: np.ndarray, msg: LaserScan) -> float:
        """Range of the beam closest to exactly -90 deg. Inf maps to range_max.
        Returns nan for NaN or below-range_min readings."""
        target_angle = -math.pi / 2
        idx = int(round((target_angle - msg.angle_min) / msg.angle_increment))
        idx = max(0, min(idx, len(ranges_np) - 1))
        val = ranges_np[idx]
        if np.isinf(val):
            return float(msg.range_max)
        if np.isnan(val) or val <= msg.range_min:
            return float('nan')
        return float(val)

    def _right_cone_min_and_angle(self, ranges_np: np.ndarray, msg: LaserScan):
        """Return (min_dist, min_angle_rad) in the right cone, mapping inf to range_max.
        Returns (nan, nan) if nothing valid."""
        s, e = self._cone_indices(msg, -math.pi / 2, self.side_half_cone)
        cone = ranges_np[s:e + 1].copy()
        angles = msg.angle_min + np.arange(s, e + 1) * msg.angle_increment
        cone = np.where(np.isinf(cone), msg.range_max, cone)
        valid = np.isfinite(cone) & (cone > msg.range_min)
        if not np.any(valid):
            return float('nan'), float('nan')
        r = cone[valid]
        a = angles[valid]
        i = int(np.argmin(r))
        return float(r[i]), float(a[i])

    # -- Mode transitions -----------------------------------------------------

    def _enter_mode(self, new_mode: str):
        self.mode = new_mode
        self.mode_start_time = time.time()

    def _pick_turn_direction(self, ranges: np.ndarray, msg: LaserScan) -> float:
        """Return +1 (left) or -1 (right) based on which side is more open.
        Treats NaN (nothing seen) as 'very open' so we prefer unseen sides."""
        left_dist  = self._valid_median(ranges, msg,  math.pi / 2, self.side_half_cone)
        right_dist = self._valid_median(ranges, msg, -math.pi / 2, self.side_half_cone)
        l = left_dist  if not math.isnan(left_dist)  else float('inf')
        r = right_dist if not math.isnan(right_dist) else float('inf')
        return 1.0 if l >= r else -1.0

    # -- Main callback --------------------------------------------------------

    def scan_callback(self, msg: LaserScan):
        if not self.ready:
            return

        ranges    = np.array(msg.ranges)
        front_min = self._valid_min(ranges, msg, 0.0, self.front_half_cone)
        now       = time.time()

        # Keep a rolling buffer of recent dist_at_90 for turn-target snapshotting
        self.right_dist_buffer.append(self._dist_at_90(ranges, msg))

        # ------------------------------------------------------------------
        # BACKING_UP: reverse in place for a fixed duration, then pick a
        # turn direction and switch to TURNING.
        # ------------------------------------------------------------------
        if self.mode == self.MODE_BACKING_UP:
            if now - self.mode_start_time < self.backup_time:
                twist = Twist()
                twist.linear.x = -self.backup_speed
                self.vel_pub.publish(twist)
                self.get_logger().info(
                    f'[BACKING_UP] front: {front_min:.2f} m',
                    throttle_duration_sec=0.5)
                return

            # Done reversing -> choose turn direction
            self.turn_dir = self._pick_turn_direction(ranges, msg)
            self._enter_mode(self.MODE_TURNING)
            self.get_logger().info(
                f'Backup complete. Turning {"left" if self.turn_dir > 0 else "right"}.')
            # fall through to TURNING this cycle

        # ------------------------------------------------------------------
        # TURNING: rotate in place toward the open side until the front
        # cone is clearly clear (hysteresis: clear_dis > stop_dis).
        # ------------------------------------------------------------------
        if self.mode == self.MODE_TURNING:
            if front_min >= self.clear_dis:
                self._enter_mode(self.MODE_CENTERING)
                self.vel_pub.publish(Twist())   # brief stop before resuming
                self.get_logger().info('Front clear. Resuming centering.')
                return

            twist = Twist()
            twist.angular.z = self.turn_dir * self.turn_speed
            self.vel_pub.publish(twist)
            self.get_logger().info(
                f'[TURNING {("L" if self.turn_dir > 0 else "R")}] '
                f'front: {front_min:.2f} m',
                throttle_duration_sec=0.5)
            return

        # ------------------------------------------------------------------
        # HALLWAY_TURN_RIGHT: servo-based right wall follow that naturally
        # turns through right-hand corners via inf -> range_max mapping.
        # Exits when parallel to the new right wall for N consecutive scans.
        # Front obstacle still preempts.
        # ------------------------------------------------------------------
        if self.mode == self.MODE_HALLWAY_TURN_RIGHT:
            # Safety first: front obstacle preempts
            if front_min <= self.stop_dis:
                self.get_logger().warn(
                    f'Obstacle at {front_min:.2f} m during servo turn -- backing up.')
                self.vel_pub.publish(Twist())
                self.suppress_right_turn = False
                self._enter_mode(self.MODE_BACKING_UP)
                return

            elapsed = now - self.mode_start_time

            # Hard timeout failsafe
            if elapsed > self.turn_max_time:
                self.get_logger().warn(
                    f'Servo turn timed out after {elapsed:.1f}s. Returning to centering.')
                self.suppress_right_turn = False
                self._enter_mode(self.MODE_CENTERING)
                return

            # Compute servo feedback from right cone
            d90 = self._dist_at_90(ranges, msg)
            min_dist, min_angle = self._right_cone_min_and_angle(ranges, msg)

            if math.isnan(d90) or math.isnan(min_dist):
                # Nothing usable; go straight and hope
                twist = Twist()
                twist.linear.x = self.forward_speed
                self.vel_pub.publish(twist)
                self.get_logger().info(
                    '[SERVO_TURN] no valid readings, driving straight',
                    throttle_duration_sec=0.5)
                return

            # Distance term
            dist_error = d90 - self.active_turn_target
            dist_turn  = -self.kp_dist * dist_error

            # Angle (parallel alignment) term
            slope_mag     = d90 - min_dist
            min_angle_deg = math.degrees(min_angle)
            angle_sign    = np.sign(min_angle_deg - (-90.0))
            angle_error   = float(angle_sign * slope_mag)
            angle_turn    = self.kp_angle * angle_error

            turn = float(np.clip(dist_turn + angle_turn,
                                 -self.max_turn, self.max_turn))

            twist = Twist()
            twist.linear.x  = self.forward_speed
            twist.angular.z = turn
            self.vel_pub.publish(twist)

            # Exit condition: parallel to right wall for N consecutive scans
            if (elapsed >= self.turn_min_time
                    and abs(slope_mag) < self.turn_exit_slope_thresh
                    and d90 < self.turn_exit_dist_max):
                self.turn_parallel_count += 1
            else:
                self.turn_parallel_count = 0

            self.get_logger().info(
                f'[SERVO_TURN] d90={d90:.2f} min={min_dist:.2f}@{min_angle_deg:+.1f}° '
                f'slope={slope_mag:+.3f} turn={turn:+.3f} par_cnt={self.turn_parallel_count}',
                throttle_duration_sec=0.5)

            if self.turn_parallel_count >= self.turn_exit_consecutive:
                self.suppress_right_turn = False
                self._enter_mode(self.MODE_CENTERING)
                self.get_logger().info(
                    f'Parallel to right wall ({slope_mag:+.3f} m). Resuming centering.')
                return

            return

        # ------------------------------------------------------------------
        # CENTERING: normal wall-following. If we see an obstacle, switch
        # to BACKING_UP on the next cycle.
        # ------------------------------------------------------------------
        if front_min <= self.stop_dis:
            self.get_logger().warn(
                f'Obstacle at {front_min:.2f} m -- backing up.')
            self.vel_pub.publish(Twist())   # immediate stop this cycle
            self._enter_mode(self.MODE_BACKING_UP)
            return

        # -- Right-opening detection: trigger a servo-based right turn when
        # the right cone's median distance exceeds the threshold. Snapshot
        # the buffered dist_at_90 as the servo target, or fall back.
        right_dist_check = self._valid_median(
            ranges, msg, -math.pi / 2, self.side_half_cone)
        if (not self.suppress_right_turn
                and not math.isnan(right_dist_check)
                and right_dist_check > self.hallway_turn_trigger):
            buf = [v for v in self.right_dist_buffer
                   if not math.isnan(v) and 0.1 < v < 3.0]
            if len(buf) >= 2:
                self.active_turn_target = float(np.median(buf))
            else:
                self.active_turn_target = self.turn_target_dist
            self.turn_parallel_count = 0
            self.suppress_right_turn = True
            self._enter_mode(self.MODE_HALLWAY_TURN_RIGHT)
            self.get_logger().info(
                f'Right opening detected ({right_dist_check:.2f} m). '
                f'Servo target {self.active_turn_target:.2f} m.')
            return

        # -- Side wall sampling -----------------------------------------------
        left_dist  = self._valid_median(ranges, msg,  math.pi / 2, self.side_half_cone)
        right_dist = self._valid_median(ranges, msg, -math.pi / 2, self.side_half_cone)

        # -- Steering correction ----------------------------------------------
        # Compute raw error and a deadband. Deadband only applies in the
        # two-wall case where "center of hallway" is meaningful.
        left_boundary = 1.8
        right_boundary = 1.8
        left_bumper = 1
        right_bumper = 1

        if math.isnan(left_dist) and math.isnan(right_dist):
            corrected_error = 0.0
            branch = 'none'
        elif math.isnan(left_dist):
            if right_dist < right_bumper:
                corrected_error = -(right_dist - right_bumper)
            elif right_dist > right_boundary:
                corrected_error = -(right_dist - right_boundary)
            else:
                corrected_error = 0.0
            branch = 'right'
        elif math.isnan(right_dist):
            if left_dist < left_bumper:
                corrected_error = left_dist - left_bumper
            elif left_dist > left_boundary:
                corrected_error = left_dist - left_boundary
            else:
                corrected_error = 0.0
            branch = 'left'
        else:
            deadband = (left_boundary + right_boundary) - (left_dist + right_dist)
            if deadband < 0.3:
                # Wide hallway or divot -- prioritize the closer wall.
                if left_dist < right_dist:
                    if left_dist < left_bumper:
                        corrected_error = left_dist - left_bumper
                    elif left_dist > left_boundary:
                        corrected_error = left_dist - left_boundary
                    else:
                        corrected_error = 0.0
                    branch = 'left'
                else:
                    if right_dist < right_bumper:
                        corrected_error = -(right_dist - right_bumper)
                    elif right_dist > right_boundary:
                        corrected_error = -(right_dist - right_boundary)
                    else:
                        corrected_error = 0.0
                    branch = 'right'
            else:
                # Normal hallway -- stay within the 1.8 m zone of both walls.
                if left_dist > left_boundary:
                    corrected_error = left_dist - left_boundary
                elif right_dist > right_boundary:
                    corrected_error = -(right_dist - right_boundary)
                else:
                    corrected_error = 0.0
                branch = 'center'
        # positive error -> too close to left -> steer right (negative angular.z)
        # PD control on the position error, with a low-pass filter on the
        # error before taking the derivative.
        self.filtered_error = (self.filter_alpha * corrected_error
                               + (1.0 - self.filter_alpha) * self.filtered_error)

        if self.prev_time is None:
            d_error = 0.0
        else:
            dt = now - self.prev_time
            d_error = ((self.filtered_error - self.prev_error) / dt
                       if dt > 0 else 0.0)

        self.prev_error = self.filtered_error
        self.prev_time  = now

        pos_term = self.kp * self.filtered_error
        d_term   = self.kd * d_error
        angular_z = float(np.clip(pos_term + d_term, -self.max_angular_z, self.max_angular_z))

        # -- Readout ----------------------------------------------------------
        left_str  = f'{left_dist:.2f} m'  if not math.isnan(left_dist)  else '  --  '
        right_str = f'{right_dist:.2f} m' if not math.isnan(right_dist) else '  --  '

        if angular_z > 0.05:
            direction = f'turning left  (w={angular_z:+.2f})'
        elif angular_z < -0.05:
            direction = f'turning right (w={angular_z:+.2f})'
        else:
            direction = 'straight'

        self.get_logger().info(
            f'left: {left_str} | front: {front_min:.2f} m | '
            f'right: {right_str} | [{branch}] | pos={pos_term:+.3f} d={d_term:+.3f} '
            f'(err={corrected_error:+.2f} filt={self.filtered_error:+.2f}) | {direction}',
            throttle_duration_sec=0.5)

        twist = Twist()
        twist.linear.x  = self.forward_speed
        twist.angular.z = angular_z
        self.vel_pub.publish(twist)


# -- Entry point --------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HallwayCenterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.vel_pub.publish(Twist())   # zero-stop on shutdown
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
