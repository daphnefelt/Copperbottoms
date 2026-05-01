#!/usr/bin/env python3
"""
Lidar hallway node — hybrid version.

STRAIGHT  -> ORIGINAL single-right-wall tracking with wall_target update logic.
             Trusts the right wall when geometry says it's clean.
DIVOT     -> Two-wall centering. The left wall keeps the robot on track when
             the right wall has a gap.
INLET     -> Two-wall centering. Same idea — left wall is the reliable
             reference while the right wall protrudes.
TURN      -> Unchanged: fixed-rate open-loop curve.
OBSTACLE  -> Unchanged: BACKUP -> OB TURN recovery.

Drives BACKWARDS (linear.x is negative when moving forward in robot's intent).
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
        self.wall_target = 1.47
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

        # PD gains
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

        # -- Two-wall centering deadband (DIVOT/INLET only) -------------------
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
    # Cone / median / wall-angle helpers (unchanged)
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
    # Two-wall centering errors (used by DIVOT and INLET only).
    # -------------------------------------------------------------------------
    def _centering_errors(self, right_dist, left_dist, right_angle, left_angle):
        """
        Returns (dist_error, angle_error) using both walls when available.

        Sign convention (matches your original STRAIGHT):
          dist_error > 0  => 'too far from right wall' (need to drift right)
          dist_error < 0  => 'too close to right wall'

        Two walls visible:
          dist_error = (right_dist - left_dist) / 2  with deadband.
          A right-wall divot makes right_dist spike high; that gets averaged
          against a normal left_dist, so the controller barely moves.

        Right only (left missing): same as your original — track wall_target.
        Left only  (right missing): mirror so the sign convention holds.
        Neither: zero error, hold heading.
        """
        right_ok = not math.isnan(right_dist) and right_dist < 99.0
        left_ok  = not math.isnan(left_dist)  and left_dist  < 99.0

        if right_ok and left_ok:
            raw = (right_dist - left_dist) / 2.0
            hallway_half = (right_dist + left_dist) / 2.0
            deadband = self.deadband_frac * hallway_half
            if   raw >  deadband: dist_error = raw - deadband
            elif raw < -deadband: dist_error = raw + deadband
            else:                 dist_error = 0.0
        elif right_ok and not left_ok:
            dist_error = right_dist - self.wall_target
        elif left_ok and not right_ok:
            dist_error = self.wall_target - left_dist
        else:
            dist_error = 0.0

        # Average heading from whichever walls are valid
        headings = []
        if not math.isnan(right_angle):
            headings.append(self._wrap(right_angle - math.pi))
        if not math.isnan(left_angle):
            headings.append(self._wrap(left_angle))
        angle_error = float(np.mean(headings)) if headings else float('nan')

        return dist_error, angle_error

    # -------------------------------------------------------------------------
    # PD steering (unchanged from original)
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
    # Mode transitions
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
        # State switching (unchanged from original)
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

        # ---- STRAIGHT: ORIGINAL single-right-wall logic (unchanged) ----
        if self.mode == self.MODE_STRAIGHT:
            twist = Twist()

            if self.prev_state == self.MODE_INLET or self.prev_state == self.MODE_DIVOT:
                if (right_dist > 1.47 and right_dist != 99):
                    self.wall_target = right_dist
                    self.get_logger().info('UPDATE WALL TARGET')
                else:
                    self.wall_target = self.wall_target_inital
                    self.get_logger().info('UPDATE WALL TARGET INITIAL')

            self.prev_state = self.MODE_STRAIGHT
            dist_error  = right_dist - self.wall_target
            angle_error = self._wrap(right_angle - math.pi)
            twist.linear.x  = -self.forward_speed
            twist.angular.z = self.PD_steering(angle_error, dist_error)
            self.vel_pub.publish(twist)

        # ---- DIVOT: two-wall centering with one-sided clamp ----
        elif self.mode == self.MODE_DIVOT:
            twist = Twist()
            self.prev_state = self.MODE_DIVOT
            dist_error, angle_error = self._centering_errors(
                right_dist, left_dist, right_angle, left_angle)
            twist.linear.x  = -self.forward_speed
            correction = self.PD_steering(angle_error, dist_error)
            # Clamp correction to non-negative: only allow steering AWAY from
            # divots, never INTO them. Without this, the controller can still
            # be tricked into the gap on certain geometries.
            twist.angular.z = max(0.0, correction)
            self.vel_pub.publish(twist)
            self.prev_cls = right_cls

        # ---- INLET: two-wall centering with one-sided clamp ----
        elif self.mode == self.MODE_INLET:
            twist = Twist()
            self.prev_state = self.MODE_INLET
            dist_error, angle_error = self._centering_errors(
                right_dist, left_dist, right_angle, left_angle)
            twist.linear.x  = -self.forward_speed
            correction = self.PD_steering(angle_error, dist_error)
            twist.angular.z = max(0.0, correction)
            self.vel_pub.publish(twist)

        # ---- TURN (unchanged) ----
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

        # ---- OBSTACLE (unchanged) ----
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
