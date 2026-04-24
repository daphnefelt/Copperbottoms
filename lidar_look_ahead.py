#!/usr/bin/env python3
"""
Hallway centering node with integrated obstacle stop + recovery.

Lidar cones:
  FRONT  ±10°  around   0°      -> stop if anything within stop_dis
  RIGHT  ±5°   around  -90°     -> primary wall distance (PD control)
  ANGLE  ±1°   around  -45°     -> lookahead cone for state switching

State machine:
  STRAIGHT    -> parallel to right wall, PD control to hold_distance
  DIVOT       -> angle_dist jumped (< turn_threshold): ignore, maintain heading, recalc hold
  INLET       -> angle_dist < right_dist: wall closing in, maintain heading, recalc hold
  TURN        -> angle_dist >= turn_threshold (wall gone): execute right turn until parallel
  BACKING_UP  -> obstacle hit, reverse for backup_time seconds then return to STRAIGHT
"""

""" ------------------- TO DO ------------------------------------
-check turn direction
-tune distance variables 
-obstace detection wider to the left / exception handling (fuck those open doors)
-use 3 linear regression lines for divot and inlet logic maybe??
-possibly use angle linear regression for turn logic - (will go from perpendicular to parallel theoretically)
-tune PD
-test if perpendicular is worth it
-test 

------------------------------------------------------------------
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
        self.front_half_cone = math.radians(10)   # ±10° front
        self.side_half_cone  = math.radians(5)    # ±5°  left side 90°
        self.angle_half_cone = math.radians(1)    # ±1°  angled lookahead 135°

        # -- Distances --------------------------------------------------------
        self.stop_dis          = 0.5    # (m) front obstacle triggers BACKING_UP
        self.clear_dis         = 0.5    # (m) front must exceed this to leave BACKING_UP
        self.turn_threshold    = 6.0    # (m) angle_dist above this -> TURN (wall gone)
        self.wall_target       = 1.4    # (m) initial hold distance from right wall

        # -- Hold-distance stabilisation window -------------------------------
        self.hold_window_sec   = 0.25    # collect right_dist readings for this long
        self._hold_buf         = []     # (timestamp, right_dist) pairs
        self._collecting_hold  = False
        self._hold_start       = 0.0
        self.hold_distance     = self.wall_target
        self.hold_timout       = 2

        # -- Motion parameters ------------------------------------------------
        self.forward_speed     = 0.2    # m/s
        self.backup_speed      = 0.15   # m/s magnitude
        self.backup_time       = 1.5    # seconds to reverse
        self.kp                = 0.4    # position error gain
        self.kp_heading        = 0.4    # heading error gain
        self.max_angular_z     = 1.2    # rad/s clamp
        self.turn_speed        = -0.5    # rad/s during TURN mode REVERSED FOR BACKWARDS

        # -- Parallel-detection tolerance (used to exit TURN) -----------------
        self.angle_tol = math.radians(5)   # wall angle considered "parallel"
        self.parallel_dist_tol  = 0.15     # right_dist stable within ± x m

        # -- State machine ----------------------------------------------------
        self.MODE_STRAIGHT   = 'STRAIGHT'
        self.MODE_DIVOT      = 'DIVOT'
        self.MODE_INLET      = 'INLET'
        self.MODE_TURN       = 'TURN'
        self.MODE_BACKING_UP = 'BACKING_UP'

        self.mode            = self.MODE_STRAIGHT
        self.mode_start_time = 0.0

        # -- Startup delay ----------------------------------------------------
        self.ready = False
        self.create_timer(3.0, self.set_ready)

        # -- Pub / sub --------------------------------------------------------
        self.vel_pub      = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info(
            'Hallway center node started. Waiting 3 s for lidar to stabilize...')

    def set_ready(self):
        if self.ready:
            return
        self.ready = True
        self.get_logger().info('Lidar stabilized. Starting hallway centering.')

    # -------------------------------------------------------------------------
    # Index / range helpers
    # -------------------------------------------------------------------------

    def _cone_indices(self, msg: LaserScan, center_rad: float, half_cone_rad: float):
        start_idx = int(((center_rad - half_cone_rad) - msg.angle_min) / msg.angle_increment)
        end_idx   = int(((center_rad + half_cone_rad) - msg.angle_min) / msg.angle_increment)
        n = len(msg.ranges)
        start_idx = max(0, min(start_idx, n - 1))
        end_idx   = max(0, min(end_idx,   n - 1))
        return start_idx, end_idx

    def _valid_min(self, ranges_np: np.ndarray, msg: LaserScan,
                   center_rad: float, half_cone_rad: float) -> float:
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & np.isfinite(cone)]
        return float(np.min(valid)) if valid.size > 0 else float('inf')

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
        """
        Fit a line through points collected from one or more cones.
        cone_specs: list of (center_rad, half_cone_rad) tuples.
        Returns wall angle in radians, or nan if fit is unreliable.
        """
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

        # 1. Distance-gate
        dists = np.linalg.norm(xy, axis=1)
        med = np.median(dists)
        xy = xy[dists < median_filter_ratio * med]
        if len(xy) < min_points:
            return float('nan')

        # 2. SVD fit + RANSAC-lite refit
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

    # ---  Individual cone wall angles --------------------------------------

    def _wall_angle_right(self, ranges_np, msg):
        """Linear regression on the right cone only (90°)."""
        return self._wall_angle_from_cones(
            ranges_np, msg,
            [(math.pi / 2, self.side_half_cone)])

    def _wall_angle_angle(self, ranges_np, msg):
        """Linear regression on the angle/lookahead cone only (135°)."""
        return self._wall_angle_from_cones(
            ranges_np, msg,
            [(3 * math.pi / 4, self.angle_half_cone)])

    def _wall_angle_combined(self, ranges_np, msg):
        """Linear regression across both cones combined."""
        return self._wall_angle_from_cones(
            ranges_np, msg,
            [(math.pi / 2, self.side_half_cone),
            (3 * math.pi / 4, self.angle_half_cone)])

    # -------------------------------------------------------------------------
    # Hold-distance helper
    # -------------------------------------------------------------------------

    def _start_hold_collection(self):
        """Begin a fresh window to recalculate hold_distance."""
        self._collecting_hold = True
        self._hold_start      = time.time()
        self._hold_buf        = []

    def _update_hold_collection(self, right_dist: float, wall_angle: float) -> bool:
        """
        Feed the current right_dist reading into the buffer.
        Returns True (and updates self.hold_distance) when the window is complete.
        """
        if not self._collecting_hold:
            return False
        if math.isnan(wall_angle) or abs(wall_angle) > self.angle_tol:
            return False
        now = time.time()
        if not math.isnan(right_dist) and math.isfinite(right_dist):
            self._hold_buf.append(right_dist)
        if now - self._hold_start >= self.hold_window_sec:
            self._collecting_hold = False
            if self._hold_buf:
                self.hold_distance = float(np.median(self._hold_buf))
                self.get_logger().info(
                    f'Hold distance updated -> {self.hold_distance:.3f} m '
                    f'(n={len(self._hold_buf)})')
            return True
        return False

    # -------------------------------------------------------------------------
    # Mode transition
    # -------------------------------------------------------------------------

    def _enter_mode(self, new_mode: str):
        if new_mode == self.mode:
            return
        self.get_logger().info(f'Mode: {self.mode} -> {new_mode}')
        self.mode            = new_mode
        self.mode_start_time = time.time()
        # Start a fresh hold-distance collection whenever entering a wall-following mode
        if new_mode in (self.MODE_STRAIGHT, self.MODE_DIVOT, self.MODE_INLET):
            self._start_hold_collection()

    # -------------------------------------------------------------------------
    # PD steering (used by STRAIGHT, DIVOT, INLET)
    # -------------------------------------------------------------------------

    def _pd_steering(self, ranges_np: np.ndarray, msg: LaserScan,
                     right_dist: float) -> float:
        """
        Returns angular_z using position error (right_dist vs hold_distance)
        and heading error from right wall angle.
        Positive error = too far from wall = steer right (negative angular_z).
        """
        pos_error = right_dist - self.hold_distance   # + means too far, steer right

        right_angle = self._wall_angle_right(self.side_half_cone, ranges_np, msg)
        heading_error = -right_angle if not math.isnan(right_angle) else 0.0

        angular_z = float(np.clip(
            -self.kp * pos_error - self.kp_heading * heading_error,
            -self.max_angular_z,
            self.max_angular_z))
        return angular_z

    # -------------------------------------------------------------------------
    # Main scan callback
    # -------------------------------------------------------------------------

    def scan_callback(self, msg: LaserScan):
        if not self.ready:
            return

        ranges    = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), 12.0, ranges)
        now       = time.time()
        front_min = self._valid_min(ranges,  msg, 0.0,           self.front_half_cone)
        right_dist = self._valid_median(ranges, msg, math.pi / 2, self.side_half_cone)
        angle_dist = self._valid_median(ranges, msg, 3 * math.pi / 4, self.angle_half_cone)
        right_angle = self._wall_angle_right(ranges, msg)
        # ------------------------------------------------------------------
        # BACKING_UP: reverse for a fixed duration, then return to STRAIGHT.
        # The normal obstacle-avoidance steering will handle turning once
        # STRAIGHT resumes.
        # ------------------------------------------------------------------
        if self.mode == self.MODE_BACKING_UP:
            if now - self.mode_start_time < self.backup_time:
                twist = Twist()
                twist.linear.x = self.backup_speed
                self.vel_pub.publish(twist)
                self.get_logger().info(
                    f'[BACKING_UP] front: {front_min:.2f} m',
                    throttle_duration_sec=0.5)
                return

            # Done reversing -> back to STRAIGHT (PD will steer us clear)
            self._enter_mode(self.MODE_STRAIGHT)
            self.get_logger().info('Backup complete. Resuming STRAIGHT.')
            # fall through to STRAIGHT this cycle

        # ------------------------------------------------------------------
        # Front obstacle guard (applies in all forward-moving modes)
        # ------------------------------------------------------------------
        if self.mode != self.MODE_BACKING_UP and front_min <= self.stop_dis:
            self.get_logger().warn(f'Obstacle at {front_min:.2f} m -- backing up.')
            self.vel_pub.publish(Twist())
            self._enter_mode(self.MODE_BACKING_UP)
            return

        # ------------------------------------------------------------------
        # State switching logic (angle cone drives transitions)
        # ------------------------------------------------------------------
        angle_valid = not math.isnan(angle_dist) and math.isfinite(angle_dist)
        right_valid = not math.isnan(right_dist) and math.isfinite(right_dist)

        if self.mode in (self.MODE_STRAIGHT, self.MODE_DIVOT, self.MODE_INLET):
            if angle_valid and angle_dist >= self.turn_threshold:
                # Wall has disappeared ahead -> corner, execute turn
                self._enter_mode(self.MODE_TURN)

            elif angle_valid and right_valid and angle_dist < right_dist:
                # Angle cone sees closer wall than side cone -> inlet
                if self.mode != self.MODE_INLET:
                    self._enter_mode(self.MODE_INLET)

            elif angle_valid and right_valid and angle_dist > right_dist:
                # Angle cone sees farther wall but below turn threshold -> divot
                if self.mode != self.MODE_DIVOT:
                    self._enter_mode(self.MODE_DIVOT)

            else:
                # Walls appear parallel -> STRAIGHT
                if self.mode != self.MODE_STRAIGHT:
                    self._enter_mode(self.MODE_STRAIGHT)

        # ------------------------------------------------------------------
        # STRAIGHT: PD control to hold_distance from right wall
        # ------------------------------------------------------------------
        if self.mode == self.MODE_STRAIGHT:
            self._update_hold_collection(right_dist, right_angle)

            if not right_valid:
                # No right wall reading — drive straight
                twist = Twist()
                twist.linear.x = -self.forward_speed
                self.vel_pub.publish(twist)
                return

            angular_z = self._pd_steering(ranges, msg, right_dist)
            self.get_logger().info(
                f'[STRAIGHT] right: {right_dist:.2f} m  '
                f'hold: {self.hold_distance:.2f} m  '
                f'w: {angular_z:+.2f}',
                throttle_duration_sec=0.5)
            twist = Twist()
            twist.linear.x  = -self.forward_speed
            twist.angular.z = angular_z
            self.vel_pub.publish(twist)
            return

        # ------------------------------------------------------------------
        # DIVOT: wall receded slightly. Maintain heading, recalc hold dist.
        # ------------------------------------------------------------------
        if self.mode == self.MODE_DIVOT:
            done = self._update_hold_collection(right_dist, right_angle)
            # if parallel check keeps failing for too long, force transition anyway
            if time.time() - self.mode_start_time > self.hold_timout:
                self._enter_mode(self.MODE_STRAIGHT)
            if done:
                self._enter_mode(self.MODE_STRAIGHT)

            angular_z = self._pd_steering(ranges, msg, right_dist) if right_valid else 0.0
            self.get_logger().info(
                f'[DIVOT] right: {right_dist:.2f} m  '
                f'angle: {angle_dist:.2f} m',
                throttle_duration_sec=0.5)
            twist = Twist()
            twist.linear.x  = -self.forward_speed
            twist.angular.z = angular_z
            self.vel_pub.publish(twist)
            return

        # ------------------------------------------------------------------
        # INLET: wall closing in. Maintain heading, recalc hold dist.
        # ------------------------------------------------------------------
        if self.mode == self.MODE_INLET:
            done = self._update_hold_collection(right_dist, right_angle)
            # if parallel check keeps failing for too long, force transition anyway
            if time.time() - self.mode_start_time > self.hold_timout:
                self._enter_mode(self.MODE_STRAIGHT)
            if done:
                self._enter_mode(self.MODE_STRAIGHT)

            angular_z = self._pd_steering(ranges, msg, right_dist) if right_valid else 0.0
            self.get_logger().info(
                f'[INLET] right: {right_dist:.2f} m  '
                f'angle: {angle_dist:.2f} m',
                throttle_duration_sec=0.5)
            twist = Twist()
            twist.linear.x  = -self.forward_speed
            twist.angular.z = angular_z
            self.vel_pub.publish(twist)
            return

        # ------------------------------------------------------------------
        # TURN: right wall gone. Rotate right until parallel again.
        # "Parallel" = wall angle near 0 AND right_dist is stable.
        # ------------------------------------------------------------------
        if self.mode == self.MODE_TURN:
            right_angle = self._wall_angle_right(self.side_half_cone, ranges, msg)
            parallel = (
                not math.isnan(right_angle)
                and abs(right_angle) < self.angle_tol
                and right_valid
                and right_dist < self.turn_threshold
            )
            if parallel:
                self.get_logger().info(
                    f'[TURN] Parallel re-acquired. right_angle={math.degrees(right_angle):.1f}°')
                self._enter_mode(self.MODE_STRAIGHT)
                # Fall through is fine; STRAIGHT block won't be reached this cycle
                # because _enter_mode just set the mode. Next cycle picks it up.
                self.vel_pub.publish(Twist())
                return

            twist = Twist()
            twist.linear.x  = -self.forward_speed         # forward while turning
            twist.angular.z = -self.turn_speed            # negative = right turn
            self.vel_pub.publish(twist)
            self.get_logger().info(
                f'[TURN] right_angle: '
                f'{"nan" if math.isnan(right_angle) else f"{math.degrees(right_angle):.1f}°"}  '
                f'right: {right_dist:.2f} m',
                throttle_duration_sec=0.5)
            return

        # --------------------------------------------------------------------
        # -------- Test Angle Logic
        #---------------------------------------------------------------------

        # Perpendicular would be:
        abs(abs(right_angle) - math.pi / 2) < self.angle_tol
        # TEST IF THIS IS WORKING


        # --------------------------------------------------------------------

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
