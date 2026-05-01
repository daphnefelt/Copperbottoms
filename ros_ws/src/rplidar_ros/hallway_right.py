#!/usr/bin/env python3
"""
Hallway centering node with right-turn-at-opening behavior.

Three lidar cones:
  FRONT  ±15°   around   0°  -> stop if anything within 0.5 m
  LEFT   ±22.5° around  90°  -> left wall distance for centering
  RIGHT  ±22.5° around -90°  -> right wall distance; trigger turn if > 4 m

State machine:
  CENTERING          -> drive forward, keep centered, watch for right opening
  DRIVE_PAST_CORNER  -> right opening detected; drive straight briefly so the
                        robot body clears the corner before rotating
  TURNING_RIGHT      -> rotate in place until parallel with the left wall
  BACKING_UP         -> obstacle hit, reverse for backup_time seconds
  TURNING            -> recovery rotation after backup, until front is clear

Published topics:
  /cmd_vel    geometry_msgs/Twist
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class HallwayCenterNode(Node):

    def __init__(self):
        super().__init__('hallway_center_node')

        # -- Cone half-widths -------------------------------------------------
        self.front_half_cone = math.radians(15)
        self.side_half_cone  = math.radians(22.5)

        # -- Distances --------------------------------------------------------
        self.stop_dis        = 0.5
        self.clear_dis       = 0.5
        self.wall_target     = 0.8

        # -- Right-turn trigger ----------------------------------------------
        self.right_open_dis      = 4.0   # right_dist > this => opening
        self.right_open_debounce = 4     # consecutive scans required
        self.drive_past_time     = 0.05   # seconds to drive past the corner
        self.parallel_tol        = math.radians(5)  # |left_angle| <= this => done

        # -- Motion parameters ------------------------------------------------
        self.forward_speed   = 0.2
        self.backup_speed    = 0.15
        self.backup_time     = 1.0
        self.kp              = 0.6
        self.kp_heading      = 0.6
        self.max_angular_z   = 0.8
        self.turn_speed      = 1.2

        # -- Centering deadband ----------------------------------------------
        self.deadband_frac   = 0.1

        # -- State machine ----------------------------------------------------
        self.MODE_CENTERING         = 'CENTERING'
        self.MODE_DRIVE_PAST_CORNER = 'DRIVE_PAST_CORNER'
        self.MODE_TURNING_RIGHT     = 'TURNING_RIGHT'
        self.MODE_BACKING_UP        = 'BACKING_UP'
        self.MODE_TURNING           = 'TURNING'
        self.mode            = self.MODE_CENTERING
        self.mode_start_time = 0.0
        self.turn_dir        = 1.0

        # Debounce counter for right-opening detection
        self.right_open_count = 0

        # -- Startup delay ----------------------------------------------------
        self.ready = False
        self.create_timer(3.0, self.set_ready)

        # -- Pub / sub --------------------------------------------------------
        self.vel_pub      = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info(
            'Hallway center node started. Waiting 3 seconds for lidar to stabilize...')

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

    def _valid_min(self, ranges_np, msg, center_rad, half_cone_rad):
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & (np.isfinite(cone))]
        return float(np.min(valid)) if valid.size > 0 else float('inf')

    def _valid_median(self, ranges_np, msg, center_rad, half_cone_rad):
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & (np.isfinite(cone))]
        return float(np.median(valid)) if valid.size > 0 else float('nan')

    def _wall_angle(self, ranges_np, msg, center_rad, half_cone_rad):
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
        rms = float(np.sqrt(np.mean((y - (m * x + b)) ** 2)))
        if rms > 0.1:
            return float('nan')
        return float(np.arctan(m))

    # -- Mode transitions -----------------------------------------------------

    def _enter_mode(self, new_mode: str):
        self.mode = new_mode
        self.mode_start_time = time.time()
        self.right_open_count = 0   # reset on any mode change

    def _pick_turn_direction(self, ranges, msg):
        left_dist  = self._valid_median(ranges, msg,  math.pi / 2, self.side_half_cone)
        right_dist = self._valid_median(ranges, msg, -math.pi / 2, self.side_half_cone)
        l = left_dist  if not math.isnan(left_dist)  else float('inf')
        r = right_dist if not math.isnan(right_dist) else float('inf')
        return 1.0 if l >= r else -1.0

    # -- Centering controller (shared) ---------------------------------------

    def _compute_centering_cmd(self, ranges, msg):
        """Return a Twist that keeps the robot centered and parallel."""
        left_dist  = self._valid_median(ranges, msg,  math.pi / 2, self.side_half_cone)
        right_dist = self._valid_median(ranges, msg, -math.pi / 2, self.side_half_cone)

        deadband = 0.0
        if math.isnan(left_dist) and math.isnan(right_dist):
            error = 0.0
        elif math.isnan(left_dist):
            error = -(right_dist - self.wall_target)
        elif math.isnan(right_dist):
            error = left_dist - self.wall_target
        else:
            error = (left_dist - right_dist) / 2.0
            hallway_width = left_dist + right_dist
            deadband = self.deadband_frac * hallway_width / 2.0

        if error > deadband:
            corrected_error = error - deadband
        elif error < -deadband:
            corrected_error = error + deadband
        else:
            corrected_error = 0.0

        left_angle  = self._wall_angle(ranges, msg,  math.pi / 2, self.side_half_cone)
        right_angle = self._wall_angle(ranges, msg, -math.pi / 2, self.side_half_cone)
        headings = []
        if not math.isnan(left_angle):
            headings.append(left_angle)
        if not math.isnan(right_angle):
            headings.append(-right_angle)
        heading_error = float(np.mean(headings)) if headings else 0.0

        angular_z = float(np.clip(self.kp * corrected_error
                                  - self.kp_heading * heading_error,
                                  -self.max_angular_z, self.max_angular_z))

        twist = Twist()
        twist.linear.x  = self.forward_speed
        twist.angular.z = angular_z
        return twist, left_dist, right_dist

    # -- Main callback --------------------------------------------------------

    def scan_callback(self, msg: LaserScan):
        if not self.ready:
            return

        ranges    = np.array(msg.ranges)
        front_min = self._valid_min(ranges, msg, 0.0, self.front_half_cone)
        now       = time.time()

        # ------------------------------------------------------------------
        # BACKING_UP
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

            self.turn_dir = self._pick_turn_direction(ranges, msg)
            self._enter_mode(self.MODE_TURNING)
            self.get_logger().info(
                f'Backup complete. Turning {"left" if self.turn_dir > 0 else "right"}.')

        # ------------------------------------------------------------------
        # TURNING (recovery after backup)
        # ------------------------------------------------------------------
        if self.mode == self.MODE_TURNING:
            if front_min >= self.clear_dis:
                self._enter_mode(self.MODE_CENTERING)
                self.vel_pub.publish(Twist())
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
        # DRIVE_PAST_CORNER: opening detected; keep centering for a short
        # time so the robot body clears the corner, then rotate.
        # ------------------------------------------------------------------
        if self.mode == self.MODE_DRIVE_PAST_CORNER:
            if front_min <= self.stop_dis:
                self.get_logger().warn(
                    f'Obstacle at {front_min:.2f} m while clearing corner -- backing up.')
                self.vel_pub.publish(Twist())
                self._enter_mode(self.MODE_BACKING_UP)
                return

            if now - self.mode_start_time < self.drive_past_time:
                twist, _, _ = self._compute_centering_cmd(ranges, msg)
                self.vel_pub.publish(twist)
                self.get_logger().info(
                    f'[DRIVE_PAST_CORNER] t={now - self.mode_start_time:.2f}s',
                    throttle_duration_sec=0.5)
                return

            self._enter_mode(self.MODE_TURNING_RIGHT)
            self.get_logger().info('Corner cleared. Rotating right.')

        # ------------------------------------------------------------------
        # TURNING_RIGHT: rotate in place until parallel with the left wall
        # (|left_angle| <= parallel_tol). Require a valid left_angle reading
        # *and* enough elapsed time so we don't exit on the first frame.
        # ------------------------------------------------------------------
        if self.mode == self.MODE_TURNING_RIGHT:
            left_angle = self._wall_angle(
                ranges, msg, math.pi / 2, self.side_half_cone)

            # Require at least 0.5 s of rotation before we allow completion,
            # so we don't latch onto a stale pre-turn reading.
            elapsed = now - self.mode_start_time
            if elapsed > 0.5 and not math.isnan(left_angle) \
                    and abs(left_angle) <= self.parallel_tol:
                self._enter_mode(self.MODE_CENTERING)
                self.vel_pub.publish(Twist())
                self.get_logger().info(
                    f'Parallel with left wall (angle={math.degrees(left_angle):+.1f}°). '
                    f'Resuming centering.')
                return

            twist = Twist()
            twist.angular.z = -self.turn_speed   # negative = right
            self.vel_pub.publish(twist)
            angle_str = (f'{math.degrees(left_angle):+.1f}°'
                         if not math.isnan(left_angle) else '  --  ')
            self.get_logger().info(
                f'[TURNING_RIGHT] left_angle: {angle_str}',
                throttle_duration_sec=0.5)
            return

        # ------------------------------------------------------------------
        # CENTERING
        # ------------------------------------------------------------------
        if front_min <= self.stop_dis:
            self.get_logger().warn(
                f'Obstacle at {front_min:.2f} m -- backing up.')
            self.vel_pub.publish(Twist())
            self._enter_mode(self.MODE_BACKING_UP)
            return

        twist, left_dist, right_dist = self._compute_centering_cmd(ranges, msg)

        # -- Right-opening detection (debounced) -----------------------------
        right_open = (not math.isnan(right_dist)) and (right_dist > self.right_open_dis)
        if right_open:
            self.right_open_count += 1
        else:
            self.right_open_count = 0

        if self.right_open_count >= self.right_open_debounce:
            self.get_logger().info(
                f'Right opening confirmed (right_dist={right_dist:.2f} m). '
                f'Driving past corner.')
            self._enter_mode(self.MODE_DRIVE_PAST_CORNER)
            # Keep moving forward this cycle to start clearing the corner.
            self.vel_pub.publish(twist)
            return

        # -- Readout ----------------------------------------------------------
        left_str  = f'{left_dist:.2f} m'  if not math.isnan(left_dist)  else '  --  '
        right_str = f'{right_dist:.2f} m' if not math.isnan(right_dist) else '  --  '
        az = twist.angular.z
        if az > 0.05:
            direction = f'turning left  (w={az:+.2f})'
        elif az < -0.05:
            direction = f'turning right (w={az:+.2f})'
        else:
            direction = 'straight'

        self.get_logger().info(
            f'left: {left_str} | front: {front_min:.2f} m | '
            f'right: {right_str} | {direction} | '
            f'open_count: {self.right_open_count}',
            throttle_duration_sec=0.5)

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
            node.vel_pub.publish(Twist())
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
