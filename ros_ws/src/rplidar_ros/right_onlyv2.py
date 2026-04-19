#!/usr/bin/env python3
"""
Right-wall-parallel follower with right-turn-at-corner behavior.

Sensing:
  FRONT cone  : +/-15 degrees around 0 degrees. Obstacle detection ONLY --
                no steering input comes from the front cone.
  RIGHT wedge : from -15 degrees (right edge of the front cone) going
                clockwise to -180 degrees (directly behind the robot).
                A single 165-degree wedge on the right side. All steering
                decisions in FOLLOWING come from a line fit across this
                wedge.
  FRONT-RIGHT half of the wedge : -90 to -15 degrees. Used only during
                TURNING_RIGHT to decide when the new right wall has been
                acquired.

Control:
  FOLLOWING       - drive forward. Fit a line to the right wedge and steer
                    to stay parallel to it (zero slope in the body frame).
                    If the fit is rejected (high RMS, short span, too few
                    points -- e.g. disjoint wall segments like "| | |"),
                    drive dead straight.
  TURNING_RIGHT   - rotate in place to the right until the FRONT-RIGHT half
                    of the wedge returns a valid parallel fit, with a
                    debounce so a single lucky frame cannot exit the turn.
  BACKING_UP      - very close obstacle (< stop_dis). Reverse briefly, then
                    TURNING_RIGHT (only right turns on the course).

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

        # -- Front cone (obstacle detection only) ----------------------------
        self.front_half_cone = math.radians(15)

        # -- Right wedge: [-120 deg, -15 deg] --------------------------------
        # Excludes directly-behind rays which tend to capture unrelated walls
        # (far end of corridor, rear wall, etc.) and contaminate the line fit.
        self.right_wedge_min = math.radians(-120.0)
        self.right_wedge_max = math.radians(-15.0)

        # -- Front-right half of wedge: [-90 deg, -15 deg] -------------------
        # Used during TURNING_RIGHT to reacquire the new right wall.
        self.front_right_min = math.radians(-90.0)
        self.front_right_max = math.radians(-15.0)

        # -- Front-cone triggers ---------------------------------------------
        self.stop_dis     = 0.5   # front < this => BACKING_UP (emergency only)

        # -- Right-side turn trigger -----------------------------------------
        # Sample a narrow cone centered at -90 deg (directly right). When the
        # distance stays above open_distance for debounce scans, treat it as
        # a corridor opening and trigger TURNING_RIGHT.
        self.right90_half_cone = math.radians(10.0)
        self.open_distance     = 4.0   # m; sustained right distance to trigger
        self.open_debounce     = 4     # consecutive frames required

        # -- Line-fit quality gates ------------------------------------------
        self.fit_min_points = 20     # need at least this many valid returns
        self.fit_min_span   = 0.3    # wall segment must span >= this (m)
        self.fit_max_rms    = 0.15   # line-fit RMS must be <= this (m)

        # -- Parallel-exit debounce (TURNING_RIGHT) --------------------------
        self.parallel_tol      = math.radians(3.0)
        self.parallel_debounce = 4
        self.turn_min_time     = 1.0   # don't allow exit before this (s)
        self.turn_max_time     = 4.0   # hard timeout (s)

        # -- Motion parameters -----------------------------------------------
        self.forward_speed   = 0.25
        self.backup_speed    = 0.15
        self.backup_time     = 1.0
        self.kp_heading      = 1.2
        self.kp_distance     = 0.8
        self.wall_target     = 1.4   # desired distance from right wall (m)
        self.wall_deadband   = 0.1   # |error| < this => no distance correction
        self.max_angular_z   = 0.8
        # Ackermann: must have linear.x > 0 to produce yaw. TURNING_RIGHT
        # drives a forward arc with this steering rate. Radius = forward_speed
        # / turn_angular_z. Tune to your robot's minimum turning radius and
        # the corner width.
        self.turn_angular_z  = 0.6

        # -- State machine ---------------------------------------------------
        self.MODE_FOLLOWING     = 'FOLLOWING'
        self.MODE_TURNING_RIGHT = 'TURNING_RIGHT'
        self.MODE_BACKING_UP    = 'BACKING_UP'

        self.mode            = self.MODE_FOLLOWING
        self.mode_start_time = 0.0
        self.parallel_count  = 0

        # Open-count tracks consecutive frames with right90 > open_distance
        self.open_count = 0

        # -- Startup delay ---------------------------------------------------
        self.ready = False
        self.create_timer(3.0, self.set_ready)

        # -- Pub / sub -------------------------------------------------------
        self.vel_pub      = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info(
            'Hallway node started. Waiting 3 seconds for lidar to stabilize...')

    def set_ready(self):
        if self.ready:
            return
        self.ready = True
        self.get_logger().info('Lidar stabilized. Beginning right-wall follow.')

    # -- Mode transitions -----------------------------------------------------
    def _enter_mode(self, new_mode: str):
        self.mode = new_mode
        self.mode_start_time = time.time()
        self.parallel_count = 0
        self.open_count = 0

    # -- Helpers --------------------------------------------------------------
    @staticmethod
    def _wrap_to_pi(a):
        """Wrap angles to [-pi, pi]."""
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _front_min(self, ranges_np: np.ndarray, msg: LaserScan) -> float:
        """Minimum valid range in the front cone."""
        n = len(msg.ranges)
        all_angles = self._wrap_to_pi(
            msg.angle_min + np.arange(n) * msg.angle_increment)
        mask = ((all_angles >= -self.front_half_cone)
                & (all_angles <=  self.front_half_cone)
                & (ranges_np > msg.range_min)
                & np.isfinite(ranges_np))
        if not np.any(mask):
            return float('inf')
        return float(np.min(ranges_np[mask]))

    def _right90_distance(self, ranges_np: np.ndarray, msg: LaserScan) -> float:
        """Median valid range in a narrow cone centered at -90 deg."""
        n = len(msg.ranges)
        all_angles = self._wrap_to_pi(
            msg.angle_min + np.arange(n) * msg.angle_increment)
        center = -math.pi / 2.0
        mask = ((all_angles >= center - self.right90_half_cone)
                & (all_angles <= center + self.right90_half_cone)
                & (ranges_np > msg.range_min)
                & np.isfinite(ranges_np))
        if not np.any(mask):
            return float('nan')
        return float(np.median(ranges_np[mask]))

    def _wedge_points(self, ranges_np: np.ndarray, msg: LaserScan,
                      ang_min: float, ang_max: float):
        """Return (x, y) arrays for valid returns in [ang_min, ang_max]."""
        n = len(msg.ranges)
        all_angles = self._wrap_to_pi(
            msg.angle_min + np.arange(n) * msg.angle_increment)
        mask = ((all_angles >= ang_min)
                & (all_angles <= ang_max)
                & (ranges_np > msg.range_min)
                & np.isfinite(ranges_np))
        if not np.any(mask):
            return np.empty(0), np.empty(0)
        r = ranges_np[mask]
        a = all_angles[mask]
        return r * np.cos(a), r * np.sin(a)

    def _fit_wall_angle(self, x: np.ndarray, y: np.ndarray):
        """Gated line fit. Returns (angle_rad, rms, n_points).

        angle_rad is arctan(slope) in the body frame -- the wall's deviation
        from parallel to the robot's heading. Returns NaN for angle if any
        quality gate fails (too few points, too short a span, too much
        scatter -- e.g. disjoint segments like "| | |").
        """
        n_pts = int(x.size)
        if n_pts < self.fit_min_points:
            return float('nan'), float('nan'), n_pts

        # For a wall roughly parallel to the robot (runs front-back in body
        # frame), x spans more than y. Use the larger extent as the "length".
        span = max(float(x.max() - x.min()), float(y.max() - y.min()))
        if span < self.fit_min_span:
            return float('nan'), float('nan'), n_pts

        # y = m*x + b fits a near-parallel wall cleanly. A near-vertical
        # segment (all points at similar x) would blow up, but the span gate
        # rejects those before we reach polyfit.
        m, b = np.polyfit(x, y, 1)
        rms = float(np.sqrt(np.mean((y - (m * x + b)) ** 2)))
        if rms > self.fit_max_rms:
            return float('nan'), rms, n_pts

        return float(np.arctan(m)), rms, n_pts

    # -- Main callback --------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        if not self.ready:
            return

        ranges = np.array(msg.ranges)
        front_min = self._front_min(ranges, msg)
        now = time.time()

        # ------------------------------------------------------------------
        # BACKING_UP (straight reverse -- Ackermann safe)
        # ------------------------------------------------------------------
        if self.mode == self.MODE_BACKING_UP:
            if now - self.mode_start_time < self.backup_time:
                t = Twist()
                t.linear.x = -self.backup_speed
                self.vel_pub.publish(t)
                self.get_logger().info(
                    f'[BACKING_UP] front={front_min:.2f} m',
                    throttle_duration_sec=0.5)
                return
            self._enter_mode(self.MODE_TURNING_RIGHT)
            self.get_logger().info('Backup complete. Arcing right.')

        # ------------------------------------------------------------------
        # TURNING_RIGHT (forward arc -- Ackermann)
        # ------------------------------------------------------------------
        if self.mode == self.MODE_TURNING_RIGHT:
            elapsed = now - self.mode_start_time

            # Mid-arc obstacle check: if something gets too close while
            # arcing, bail to BACKING_UP. Front cone is still the monitor.
            if front_min <= self.stop_dis:
                self.get_logger().warn(
                    f'Obstacle at {front_min:.2f} m during arc -- backing up.')
                self.vel_pub.publish(Twist())
                self._enter_mode(self.MODE_BACKING_UP)
                return

            if elapsed >= self.turn_max_time:
                self._enter_mode(self.MODE_FOLLOWING)
                self.vel_pub.publish(Twist())
                self.get_logger().warn(
                    f'TURNING_RIGHT timeout after {elapsed:.2f} s -- '
                    f'resuming follow.')
                return

            # Check ONLY the front-right half for exit. A wall that reads
            # parallel in the back half is the wall we're turning away from
            # and must not count.
            fx, fy = self._wedge_points(
                ranges, msg, self.front_right_min, self.front_right_max)
            angle, rms, n_pts = self._fit_wall_angle(fx, fy)

            parallel_ok = (not math.isnan(angle)
                           and abs(angle) <= self.parallel_tol)
            if elapsed > self.turn_min_time and parallel_ok:
                self.parallel_count += 1
            else:
                self.parallel_count = 0

            if self.parallel_count >= self.parallel_debounce:
                self._enter_mode(self.MODE_FOLLOWING)
                self.vel_pub.publish(Twist())
                self.get_logger().info(
                    f'Front-right parallel (angle={math.degrees(angle):+.1f} deg, '
                    f'rms={rms:.3f} m, n={n_pts}). Resuming follow.')
                return

            # Ackermann: forward motion + steering. Publishing angular.z
            # without linear.x produces no yaw.
            t = Twist()
            t.linear.x  = self.forward_speed
            t.angular.z = -self.turn_angular_z
            self.vel_pub.publish(t)

            ang_str = (f'{math.degrees(angle):+.1f} deg'
                       if not math.isnan(angle) else '  --  ')
            rms_str = f'{rms:.3f}' if not math.isnan(rms) else ' -- '
            self.get_logger().info(
                f'[TURNING_RIGHT] t={elapsed:.2f}s angle={ang_str} '
                f'rms={rms_str} n={n_pts} par_count={self.parallel_count}',
                throttle_duration_sec=0.5)
            return

        # ------------------------------------------------------------------
        # FOLLOWING
        # ------------------------------------------------------------------
        if front_min <= self.stop_dis:
            self.get_logger().warn(
                f'Obstacle at {front_min:.2f} m -- backing up.')
            self.vel_pub.publish(Twist())
            self._enter_mode(self.MODE_BACKING_UP)
            return

        # -- Right-side turn trigger ----------------------------------------
        # If right-90 distance stays above open_distance for open_debounce
        # consecutive scans, commit to a right turn.
        right90 = self._right90_distance(ranges, msg)

        if not math.isnan(right90) and right90 > self.open_distance:
            self.open_count += 1
        else:
            self.open_count = 0

        if self.open_count >= self.open_debounce:
            self.get_logger().info(
                f'Right opening confirmed (right90={right90:.2f} m). '
                f'Turning right.')
            self.vel_pub.publish(Twist())
            self._enter_mode(self.MODE_TURNING_RIGHT)
            return

        # Full right wedge -> line fit -> stay parallel and at target distance.
        rx, ry = self._wedge_points(
            ranges, msg, self.right_wedge_min, self.right_wedge_max)
        angle, rms, n_pts = self._fit_wall_angle(rx, ry)

        t = Twist()
        t.linear.x = self.forward_speed

        if math.isnan(angle):
            # Disjoint segments, too few points, or too much scatter -- no
            # clean line to follow. Drive straight and wait to reacquire.
            # Freeze both heading and distance correction.
            t.angular.z = 0.0
            reason = 'no-fit -> straight'
        else:
            # Heading term: angle > 0 means the wall tilts away from the
            # nose, robot is yawed left. Steer right to realign.
            heading_cmd = -self.kp_heading * angle

            # Distance term: right90 > target means too far from wall,
            # steer right (negative angular.z). Apply a +/-deadband.
            distance_cmd = 0.0
            if not math.isnan(right90):
                dist_err = right90 - self.wall_target
                if dist_err > self.wall_deadband:
                    dist_err -= self.wall_deadband
                elif dist_err < -self.wall_deadband:
                    dist_err += self.wall_deadband
                else:
                    dist_err = 0.0
                distance_cmd = -self.kp_distance * dist_err

            az = float(np.clip(heading_cmd + distance_cmd,
                               -self.max_angular_z, self.max_angular_z))
            t.angular.z = az
            r90_str = (f'{right90:.2f}m' if not math.isnan(right90)
                       else '--')
            reason = (f'angle={math.degrees(angle):+.1f}deg '
                      f'r90={r90_str} -> w={az:+.2f}')

        self.vel_pub.publish(t)
        self.get_logger().info(
            f'[FOLLOWING] front={front_min:.2f} m | n={n_pts} | {reason}',
            throttle_duration_sec=0.5)


# -- Entry point -------------------------------------------------------------
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
            try:
                node.vel_pub.publish(Twist())
            except Exception:
                pass
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
