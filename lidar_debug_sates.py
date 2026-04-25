#!/usr/bin/env python3
"""
Lidar debug node — prints right_dist, angle_dist, front_dist, and wall angle status.
No motion commands. Read-only.
Uses EXACT logic from hallway_center_node.
"""
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarDebugNode(Node):

    def __init__(self):
        super().__init__('lidar_debug_node')

        # -- distances --------------------------------------------------------
        self.stop_dist = 0.5
        self.wall_target = 1.3
        self.dist_tol = 0.1

        # -- Cone half-widths (exact from hallway_center_node) ----------------
        self.front_half_cone = math.radians(10)   # ±10° front
        self.side_half_cone  = math.radians(5)    # ±5°  right side 90°
        self.angle_half_cone = math.radians(2)    # ±1°  angled lookahead 135°

        # -- Parallel / perpendicular tolerance (exact from hallway_center_node)
        self.angle_tol = math.radians(10)
        self.perp_tol  = math.radians(10)

        # -- State Machine --
        self.MODE_STRAIGHT   = 'STRAIGHT'
        self.MODE_DIVOT      = 'DIVOT'
        self.MODE_INLET      = 'INLET'
        self.MODE_TURN       = 'TURN'
        self.MODE_OBSTACLE   = 'OBSTACLE'
        self.mode            = self.MODE_STRAIGHT
        self.mode_start_time = 0.0

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info('Lidar debug node started.')

    # -------------------------------------------------------------------------
    # Exact copies from hallway_center_node
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
    
    # --------------------------------------------------------------------------
    # -- PD Steering Control -------
    # -------------------------------------------------------------------------
    def PD_steering():
        # takes target from HandD
        # PD control to meet target
        return
    
    # -------------------------------------------------------------------------
    # -- Heading and Distance Control Helper
    # -------------------------------------------------------------------------
    def HandD():
        # ------ DETERMINES TARGET FOR PD CONTROL TO MAINTAIN --------
        # takes current mode, right_dist, right_angle, angle_dist, lookahead_angle

        # Mode Straight
            # prioritize staying parallel on right_angle and staying within tolerance of right dist

        # Mode Divot
            # Maintain right_dist until right_angle not parallel or no reading
            # coast when not parallel or no reading
            # when parallel again, re-assess right_dist and maintain

        # Mode Inlet
            # maintain right_dist until mode is straight?
            # robust to rapid decrease in right_dist

        # Mode Turn
            # calculated turn logic?

        # Mode Obstacle
            # avoid obstacle?
        

        return
    # -------------------------------------------------------------------------
    # State Machine Mode transition
    # -------------------------------------------------------------------------

    def _enter_mode(self, new_mode: str):
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

        front_dist       = self._valid_median(ranges, msg, 180, self.front_half_cone)
        right_dist      = self._valid_median(ranges, msg, math.pi / 2, self.side_half_cone)
        angle_dist      = self._valid_median(ranges, msg, 3 * math.pi / 4, self.angle_half_cone)
        right_angle     = self._wall_angle_right(ranges, msg)
        lookahead_angle = self._wall_angle_angle(ranges, msg)

        def classify(a):
            if math.isnan(a):
                return 'NO READING'
            elif abs(abs(a) - math.pi) < self.angle_tol:
                return 'PARALLEL'
            elif abs(abs(a) - math.pi / 2) < self.perp_tol:
                return 'PERPENDICULAR'
            else:
                return 'NOT PARALLEL'

        right_cls     = classify(right_angle)
        lookahead_cls = classify(lookahead_angle)
        right_deg     = math.degrees(right_angle)     if not math.isnan(right_angle)     else float('nan')
        lookahead_deg = math.degrees(lookahead_angle) if not math.isnan(lookahead_angle) else float('nan')

        # ------------------------------------------------------------------
        # State switching logic
        # ------------------------------------------------------------------

        if self.mode in (self.MODE_STRAIGHT, self.MODE_DIVOT, self.MODE_INLET):

            # Evaluate each condition and log it explicitly
            cond_turn   = (right_cls == 'PARALLEL' and lookahead_cls == 'PERPENDICULAR' and 5 <= angle_dist < 99)
            cond_inlet  = (right_dist > angle_dist)
            cond_divot  = (right_cls == 'PARALLEL' and angle_dist >= 99 and (lookahead_cls == 'NOT PARALLEL' or lookahead_cls == 'NO READING'))
            cond_obstacle = (front_dist < self.stop_dist)

            if cond_turn:
                self._enter_mode(self.MODE_TURN)

            elif cond_inlet:
                self._enter_mode(self.MODE_INLET)

            elif cond_divot:
                self._enter_mode(self.MODE_DIVOT)
            
            elif cond_obstacle:
                self._enter_mode(self.MODE_OBSTACLE)

            else:
                self._enter_mode(self.MODE_STRAIGHT)

        # ----------------------------------------------------------------------------------------------------
        # ---- STATES ----------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------
        if self.mode == self.MODE_STRAIGHT:
            self.get_logger().info('STRAIGHT')
            # PD control staying a distance from the wall and parallel

        if self.mode == self.MODE_DIVOT:
            self.get_logger().info('DIVOT')
            # PD control to maintain straight regardless of divot

        if self.mode == self.MODE_INLET:
            self.get_logger().info('INLET')
            # analyze if the inlet will result in a collision / adjust if needed
            # PD control to maintain straight regardless of inlet

        if self.mode == self.MODE_TURN:
            self.get_logger().info('TURN')
            elapsed = time.time() - self.mode_start_time
            turn_complete = (lookahead_cls == 'PARALLEL' and elapsed > 0.1)  # minimum dwell to avoid instant re-exit
            if turn_complete:
                self._enter_mode(self.MODE_STRAIGHT)
            # turn motor commands
        
        if self.mode == self.MODE_OBSTACLE:
            self.get_logger().info('OBSTACLE')
            # backup
            # analyze and go around, preferably to the right
            # re-align parallel in the hallway
            obstacle_cleared = (front_dist > self.stop_dist)
            if obstacle_cleared:
                self._enter_mode(self.MODE_STRAIGHT)



    # ------------------------------------------------------------------
    # Sensor summary
    # ------------------------------------------------------------------
        self.get_logger().info(
            f'front: {front_dist:6.2f} m  |  '
            f'right: {right_dist:6.2f} m  angle: {right_deg:+7.2f}°  ({right_cls})  |  '
            f'lookahead: {angle_dist:6.2f} m  angle: {lookahead_deg:+7.2f}°  ({lookahead_cls})  |  '
            f'mode: {self.mode}',
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
