#!/usr/bin/env python3
"""
Lidar hallway node — merged version.

Drives BACKWARDS down a hallway using two-wall centering when both walls are
visible, falling back to single-right-wall tracking at wall_target_inital
when only the right wall is seen. Robust to wall divots and inlets because
the left wall keeps the robot centered when the right wall has gaps.

State machine retained from the original:
  STRAIGHT / DIVOT / INLET -> all use two-wall centering controller
  TURN     -> rotate at fixed rate when a perpendicular wall appears ahead
  OBSTACLE -> backup + recovery turn

The DIVOT and INLET modes still exist for logging/observability, but their
behavior is now identical to STRAIGHT so divots/inlets in the right wall no
longer pull the robot off-center.
"""
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarDebugNode(Node):

    def __init__(self):
        super().__init__('lidar_debug_node')

        # -- distances --------------------------------------------------------
        self.stop_dist = 1.0
        self.wall_target_inital = 1.47
        self.wall_target = 1.47        # used only when one wall missing
        self.dist_tol = 0.1

        # -- misc variables ---------------------------------------------------
        self.prev_cls = 'PARALLEL'
        self.prev_state = 'STRAIGHT'
        self.turn_rate = -0.3
        self.forward_speed = 0.12
        self.OB_forward_speed = 0.22
        self.phase_start_time = None
        self.last_correction = 0.0
        self.OB_turn_rate = 1.0

        # PD gains — tune these
        self.Kp_dist  =  1.2
        self.Kd_dist  =  0.5
        self.Kp_angle =  1.2
        self.Kd_angle =  0.1
        self.K_dist_to_heading = 1.2

        # PD state
        self.prev_angle_error = 0.0
        self.prev_dist_error  = 0.0
        self.prev_time        = time.time()

        # -- Cone half-widths -------------------------------------------------
        self.front_half_cone = math.radians(10)   # ±10° front
        self.side_half_cone  = math.radians(10)   # ±10° each side
        self.angle_half_cone = math.radians(2)    # ±2°  angled lookahead

        # -- Parallel / perpendicular tolerance -------------------------------
        self.angle_tol = math.radians(10)
        self.perp_tol  = math.radians(10)

        # -- Two-wall centering deadband --------------------------------------
        # Fraction of hallway half-width that counts as "close enough to
        # center." Inside this band, lateral error is treated as zero so the
        # robot doesn't oscillate. Only applies when BOTH walls visible.
        self.deadband_frac = 0.05

        # -- State machine ----------------------------------------------------
        self.MODE_STRAIGHT   = 'STRAIGHT'
        self.MODE_DIVOT      = 'DIVOT'
        self.MODE_INLET      = 'INLET'
        self.MODE_TURN       = 'TURN'
        self.MODE_OBSTACLE   = 'OBSTACLE'
        self.obstacle_phase  = 'BACKUP'
        self.mode            = self.MODE_STRAIGHT
        self.mode_start_time = 0.0

        self.num_lidar_itrs = 0
        self.rolling_avg_dist = np.zeros(3)
        self.rolling_dist_thres = 2

        # -- PUB / SUB --------------------------------------------------------
        self.vel_pub      = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info('Lidar debug node started.')

    # -------------------------------------------------------------------------
    # Cone / median / wall-angle helpers (unchanged from original)
    # -------------------------------------------------------------------------

    def _cone_indices(self, msg: LaserScan, center_rad: float, half_cone_rad: float):
        start_idx = int(((center_rad - half_cone_rad) - msg.angle_min) / msg.angle_increment)
        end_idx   = int(((center_rad + half_cone_rad) - msg.angle_min) / msg.angle_increment)
        n = len(msg.ranges)
        start_idx = max(0, min(start_idx, n - 1))
        end_idx   = max(0, min(end_idx,   n - 1))
        return start_idx, end_idx

    def _valid_median(self, ranges_np: np.ndarray, msg: LaserScan,
                      center_rad: float, half_cone_rad: float) -> float:
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & np.isfinite(cone)]
        return float(np.median(valid)) if valid.size > 0 else float('nan')

    def _wall_angle_from_cones(self, ranges_np, msg, cone_specs,
                            median_filter_ratio=1.5,
                            residual_thresh=0.05,
                            min_points=8):
        segments = []
        for center_rad, half_cone in cone_specs:
            s, e = self._cone_indices(msg, center_rad, half_cone)
            cone = ranges_np[s:e+1]
            angles = msg.angle_min + np.arange(s, e+1) * msg.angle_increment
            mask = (cone > msg.range_min) & np.isfinite(cone)
            r, a = cone[mask], angles[mask]
            if r.size > 0:
                segments.append(np.column_stack((r * np.cos(a), r * np.sin(a))))

        if not segments:
            return float('nan')

        xy = np.vstack(segments)
        if len(xy) < min_points:
            return float('nan')

        dists = np.linalg.norm(xy, axis=1)
        med = np.median(dists)
        xy = xy[dists < median_filter_ratio * med]
        if len(xy) < min_points:
            return float('nan')

        def _svd_angle(pts):
            _, _, vt = np.linalg.svd(pts - pts.mean(axis=0))
            return vt[0]

        dx, dy = _svd_angle(xy)
        cx, cy = xy.mean(axis=0)
        residuals = np.abs(dx * (xy[:, 1] - cy) - dy * (xy[:, 0] - cx))
        xy = xy[residuals < residual_thresh]
        if len(xy) < min_points:
            return float('nan')

        dx, dy = _svd_angle(xy)
        return float(np.arctan2(dy, dx))

    def _wall_angle_right(self, ranges_np, msg):
        return self._wall_angle_from_cones(
            ranges_np, msg,
            [(math.pi / 2, self.side_half_cone)])

    def _wall_angle_left(self, ranges_np, msg):
        """Linear regression on the left cone (−90°)."""
        return self._wall_angle_from_cones(
            ranges_np, msg,
            [(-math.pi / 2, self.side_half_cone)])

    def _wall_angle_angle(self, ranges_np, msg):
        return self._wall_angle_from_cones(
            ranges_np, msg,
            [(3 * math.pi / 4, self.angle_half_cone)])

    def _wall_angle_rear(self, ranges_np, msg):
        return self._wall_angle_from_cones(ranges_np, msg, [(math.pi / 4, self.angle_half_cone)])

    def _wrap(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    # -------------------------------------------------------------------------
    # Two-wall centering: compute the (dist_error, angle_error) pair to feed
    # into PD_steering. Replaces the old single-wall right_dist tracking.
    # -------------------------------------------------------------------------
    def _centering_errors(self, right_dist, left_dist, right_angle, left_angle):
        """
        Returns (dist_error, angle_error).

        Sign convention notes (REVERSE driving + lidar with right wall at +π/2):
          - dist_error > 0  -> robot is too far from right wall (too close to left)
                            -> we want to nudge toward the right wall.
          - dist_error < 0  -> robot is too close to right wall.
          - angle_error wraps right_angle - π so that 0 = parallel-to-right-wall.

        Two-wall mode (both walls visible):
          dist_error = (right_dist - left_dist) / 2
            > 0 means right gap larger than left gap -> drift right to center.
          Deadband: zero out small errors so we don't oscillate.

        One-wall right (left missing): track right wall to wall_target_inital.
          dist_error = right_dist - wall_target_inital  (matches original sign).

        One-wall left (right missing): track left wall to wall_target_inital.
          dist_error = wall_target_inital - left_dist
          (mirror so + still means "too far from right side / too close to left").

        No walls: zero error, hold heading.
        """
        right_ok = not math.isnan(right_dist) and right_dist < 99.0
        left_ok  = not math.isnan(left_dist)  and left_dist  < 99.0

        # ---- distance error ----
        if right_ok and left_ok:
            raw = (right_dist - left_dist) / 2.0
            hallway_half = (right_dist + left_dist) / 2.0
            deadband = self.deadband_frac * hallway_half
            if   raw >  deadband: dist_error = raw - deadband
            elif raw < -deadband: dist_error = raw + deadband
            else:                 dist_error = 0.0
        elif right_ok and not left_ok:
            # fallback: hold the original right-wall target distance
            dist_error = right_dist - self.wall_target_inital
        elif left_ok and not right_ok:
            # mirror: pretend the left wall is the reference at the same offset
            dist_error = self.wall_target_inital - left_dist
        else:
            dist_error = 0.0

        # ---- angle error: average the two walls when both available ----
        # right wall is parallel when right_angle ≈ ±π
        # left  wall is parallel when left_angle  ≈ 0
        # Convert each into a common "heading vs hallway axis" frame.
        headings = []
        if not math.isnan(right_angle):
            headings.append(self._wrap(right_angle - math.pi))
        if not math.isnan(left_angle):
            headings.append(self._wrap(left_angle))   # already 0 when parallel

        if headings:
            angle_error = float(np.mean(headings))
        else:
            angle_error = float('nan')   # PD_steering falls back to pure-distance

        return dist_error, angle_error

    # -------------------------------------------------------------------------
    # PD steering (unchanged)
    # -------------------------------------------------------------------------
    def PD_steering(self, angle_error, dist_error):
        now = time.time()
        dt = now - self.prev_time if (now - self.prev_time) > 0 else 0.05
        self.prev_time = now

        if math.isnan(angle_error):
            d_dist = (dist_error - self.prev_dist_error) / dt
            self.prev_dist_error  = dist_error
            self.prev_angle_error = 0.0

            correction = self.Kp_dist * dist_error + self.Kd_dist * d_dist
            if math.isnan(correction):
                correction = 0.0
            correction = max(-12.0, min(12.0, correction))
            self.last_correction = correction
            return correction

        desired_heading_offset = self.K_dist_to_heading * dist_error
        max_offset = math.radians(20)
        desired_heading_offset = max(-max_offset, min(max_offset, desired_heading_offset))

        heading_error = angle_error - desired_heading_offset
        d_heading = (heading_error - self.prev_angle_error) / dt

        self.prev_angle_error = heading_error
        self.prev_dist_error  = dist_error

        correction = self.Kp_angle * heading_error + self.Kd_angle * d_heading
        if math.isnan(correction):
            correction = 0.0
        correction = max(-12.0, min(12.0, correction))
        self.last_correction = correction
        return correction

    # -------------------------------------------------------------------------
    # Mode transition
    # -------------------------------------------------------------------------
    def _enter_mode(self, new_mode: str):
        if new_mode == self.MODE_OBSTACLE:
            self.obstacle_phase = 'BACKUP'
        if new_mode == self.mode:
            return
        self.get_logger().info(f'Mode: {self.mode} -> {new_mode}')
        self.mode            = new_mode
        self.mode_start_time = time.time()

    # -------------------------------------------------------------------------
    # Scan callback
    # -------------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), 99.0, ranges)

        front_dist       = self._valid_median(ranges, msg, math.pi,         self.front_half_cone)
        right_dist       = self._valid_median(ranges, msg, math.pi / 2,     self.side_half_cone)
        angle_dist       = self._valid_median(ranges, msg, 3 * math.pi / 4, self.angle_half_cone)
        rear_dist        = self._valid_median(ranges, msg, math.pi / 4,     self.angle_half_cone)
        left_dist        = self._valid_median(ranges, msg, -math.pi / 2,    self.side_half_cone)
        right_angle      = self._wall_angle_right(ranges, msg)
        left_angle       = self._wall_angle_left(ranges, msg)
        lookahead_angle  = self._wall_angle_angle(ranges, msg)
        lookbehind_angle = self._wall_angle_rear(ranges, msg)

        def classify(a):
            if math.isnan(a):
                return 'NO READING'
            elif abs(abs(a) - math.pi) < self.angle_tol:
                return 'PARALLEL'
            elif abs(abs(a) - math.pi / 2) < self.perp_tol:
                return 'PERPENDICULAR'
            else:
                return 'NOT PARALLEL'

        right_cls      = classify(right_angle)
        lookahead_cls  = classify(lookahead_angle)
        lookbehind_cls = classify(lookbehind_angle)

        # ------------------------------------------------------------------
        # State switching (kept for OBSTACLE + TURN; STRAIGHT/DIVOT/INLET
        # all do two-wall centering now, but we still log the geometry
        # via the mode label).
        # ------------------------------------------------------------------
        if self.mode in (self.MODE_STRAIGHT, self.MODE_DIVOT, self.MODE_INLET, self.MODE_OBSTACLE):

            cond_sideways = (right_cls == 'PERPENDICULAR' and front_dist < 3)
            cond_turn     = (right_cls == 'PARALLEL' and lookahead_cls == 'PERPENDICULAR'
                             and 4.5 <= angle_dist < 99)
            cond_inlet    = (right_dist > angle_dist)
            cond_divot    = (angle_dist >= 99 and (lookahead_cls == 'NOT PARALLEL'
                             or lookahead_cls == 'NO READING'))
            cond_obstacle = (front_dist < self.stop_dist)

            if cond_obstacle or cond_sideways:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.vel_pub.publish(twist)
                self._enter_mode(self.MODE_OBSTACLE)

            elif cond_turn:
                self._enter_mode(self.MODE_TURN)

            elif cond_divot:
                self._enter_mode(self.MODE_DIVOT)

            elif cond_inlet:
                self._enter_mode(self.MODE_INLET)

            else:
                self._enter_mode(self.MODE_STRAIGHT)

        self.num_lidar_itrs += 1
        if self.num_lidar_itrs > 3:
            self.rolling_avg_dist[0:2] = self.rolling_avg_dist[1:]
            self.rolling_avg_dist[-1]  = right_dist
        else:
            self.rolling_avg_dist[self.num_lidar_itrs - 1] = right_dist

        # ----------------------------------------------------------------------------------------------------
        # ---- STATES ----------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------

        # STRAIGHT, DIVOT, INLET all run the same two-wall centering controller.
        # This is the key change: divots in the right wall no longer cause the
        # robot to chase them, because the left wall keeps it centered.
        if self.mode in (self.MODE_STRAIGHT, self.MODE_DIVOT, self.MODE_INLET):
            self.prev_state = self.mode
            twist = Twist()

            dist_error, angle_error = self._centering_errors(
                right_dist, left_dist, right_angle, left_angle)

            twist.linear.x  = -self.forward_speed
            twist.angular.z = self.PD_steering(angle_error, dist_error)
            self.vel_pub.publish(twist)

        elif self.mode == self.MODE_TURN:
            twist = Twist()
            self.prev_state = self.MODE_TURN
            elapsed = time.time() - self.mode_start_time
            turn_complete = (angle_dist <= 2.0 and elapsed > 0.5)
            if turn_complete:
                twist.angular.z = 0.0
                self._enter_mode(self.MODE_STRAIGHT)
            twist.linear.x  = -self.forward_speed
            twist.angular.z = self.turn_rate
            self.vel_pub.publish(twist)

        elif self.mode == self.MODE_OBSTACLE:
            self.get_logger().info(f'OBSTACLE [{self.obstacle_phase}]')
            twist = Twist()
            self.prev_state = self.MODE_OBSTACLE

            if self.obstacle_phase == 'BACKUP':
                if self.phase_start_time is None:
                    self.phase_start_time = time.time()

                if time.time() - self.phase_start_time < 3.0:
                    twist.linear.x  = self.OB_forward_speed
                    twist.angular.z = 0.0
                    self.vel_pub.publish(twist)
                else:
                    twist.linear.x  = 0.0
                    twist.angular.z = 0.0
                    self.vel_pub.publish(twist)
                    self.phase_start_time = None
                    self.obstacle_phase = 'OB TURN'

            elif self.obstacle_phase == 'OB TURN':
                if self.phase_start_time is None:
                    self.phase_start_time = time.time()
                if (right_dist > 5.0 or right_dist < 0.6):
                    twist.linear.x  = -self.OB_forward_speed
                    twist.angular.z = self.OB_turn_rate
                    self.vel_pub.publish(twist)
                    if right_cls == 'PARALLEL':
                        self.obstacle_phase = 'BACKUP'
                        self.phase_start_time = None
                        self._enter_mode(self.MODE_STRAIGHT)
                else:
                    twist.linear.x  = -self.OB_forward_speed
                    twist.angular.z = -self.OB_turn_rate
                    self.vel_pub.publish(twist)

                    if time.time() - self.phase_start_time > 2.0:
                        twist.linear.x  = -self.OB_forward_speed
                        twist.angular.z = self.OB_turn_rate
                        self.vel_pub.publish(twist)
                        if right_cls == 'PARALLEL':
                            self.obstacle_phase = 'BACKUP'
                            self.phase_start_time = None
                            self._enter_mode(self.MODE_STRAIGHT)

        # ------------------------------------------------------------------
        # Sensor summary
        # ------------------------------------------------------------------
        left_str  = f'{left_dist:6.2f}'  if not math.isnan(left_dist)  else '  --  '
        right_str = f'{right_dist:6.2f}' if not math.isnan(right_dist) else '  --  '
        self.get_logger().info(
            f'L: {left_str} m | R: {right_str} m ({right_cls}) | '
            f'lookahead: {angle_dist:6.2f} m ({lookahead_cls}) | '
            f'lookbehind: {rear_dist:6.2f} m ({lookbehind_cls}) | '
            f'corr: {self.last_correction:+.3f} | mode: {self.mode}',
            throttle_duration_sec=0.2
        )


# -- Entry point --------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarDebugNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
