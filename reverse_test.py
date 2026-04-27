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
from geometry_msgs.msg import Twist


class LidarDebugNode(Node):

    def __init__(self):
        super().__init__('lidar_debug_node')

        # -- distances --------------------------------------------------------
        self.stop_dist = 0.5
        self.wall_target_inital = 1.47
        self.wall_target = 1.47
        self.dist_tol = 0.1

        # -- misc variables --------------------------------------------------
        self.prev_cls = 'PARALLEL'
        self.prev_state = ''
        self.turn_rate = 1.0
        self.forward_speed = 0.12
        self.OB_forward_speed = 0.2
        self.phase_start_time = 0.0
        self.last_correction = 0.0
    

        # PD gains — tune these
        self.Kp_dist  =  1.2   # proportional to lateral distance error
        self.Kd_dist  =  0.5   # damping on distance error
        self.Kp_angle =  1.2   # proportional to angle error
        self.Kd_angle =  0.05  # damping on angle error

        # PD state
        self.prev_angle_error = 0.0
        self.prev_dist_error  = 0.0
        self.prev_time        = time.time()

        # -- Cone half-widths  ------------------------------------------------
        self.front_half_cone = math.radians(10)   # ±10° front
        self.side_half_cone  = math.radians(5)    # ±5°  right side 90°
        self.angle_half_cone = math.radians(2)    # ±1°  angled lookahead 135°

        # -- Parallel / perpendicular tolerance --------------------------------
        self.angle_tol = math.radians(10)
        self.perp_tol  = math.radians(10)


        # -- State Machine --
        self.MODE_STRAIGHT   = 'STRAIGHT'
        self.MODE_DIVOT      = 'DIVOT'
        self.MODE_INLET      = 'INLET'
        self.MODE_TURN       = 'TURN'
        self.MODE_OBSTACLE   = 'OBSTACLE'
        self.obstacle_phase  = 'BACKUP'
        self.mode            = self.MODE_STRAIGHT
        self.mode_start_time = 0.0

        # -- PUB / SUB --------
        self.vel_pub      = self.create_publisher(Twist, 'cmd_vel', 10)
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
    def PD_steering(self, angle_error, dist_error):
        """
        PD controller for wall-following steering.
        
        angle_error: abs(right_angle) - pi  → 0 when parallel to wall
        dist_error:  right_dist - wall_target     → + too far, - too close
        
        Returns angular.z correction.
        """
        now = time.time()
        dt = now - self.prev_time if hasattr(self, 'prev_time') and (now - self.prev_time) > 0 else 0.05
        self.prev_time = now

        # --- Derivative terms ---
        d_angle = (angle_error - self.prev_angle_error) / dt
        d_dist  = (dist_error  - self.prev_dist_error)  / dt

        # --- Store for next cycle ---
        self.prev_angle_error = angle_error
        self.prev_dist_error  = dist_error

        # --- PD output ---
        # dist_error:  steer left (positive z) when too far, right (negative z) when too close
        # angle_error: steer to restore parallel alignment
        correction = (
            self.Kp_dist  * dist_error  + self.Kd_dist  * d_dist
            + self.Kp_angle * angle_error + self.Kd_angle * d_angle
        )
        if math.isnan(correction):
            correction = 0.0
        self.last_correction = correction
        return -correction
    
    # -----------------------------------------------------------------------
    # -- parallel helper ----------------------------------------------------
    # -----------------------------------------------------------------------
    def _wrap(self, a):
        """Wrap an angle to [-pi, pi]."""
        return math.atan2(math.sin(a), math.cos(a))
    # ------------------------------------------------------------------------
    # -- Update Wall Target
    # ------------------------------------------------------------------------
    #def update_wall_target(right_dist):
        #right_dist = self._valid_median(ranges, msg, math.pi / 2, self.side_half_cone)
        #self.wall_target = right_dist
    # -------------------------------------------------------------------------
    # State Machine Mode transition
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

        front_dist       = self._valid_median(ranges, msg, math.pi, self.front_half_cone)
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

        if self.mode in (self.MODE_STRAIGHT, self.MODE_DIVOT, self.MODE_INLET, self.MODE_TURN, self.MODE_OBSTACLE):

            # Evaluate each condition and log it explicitly
            cond_turn   = (right_cls == 'PARALLEL' and lookahead_cls == 'PERPENDICULAR' and 5 <= angle_dist < 99)
            cond_inlet  = (right_dist > angle_dist)
            cond_divot  = (right_cls == 'PARALLEL' and angle_dist >= 99 and (lookahead_cls == 'NOT PARALLEL' or lookahead_cls == 'NO READING'))
            cond_obstacle = (front_dist < self.stop_dist)

            if cond_obstacle:
                self._enter_mode(self.MODE_OBSTACLE)
            
            elif cond_divot:
                self._enter_mode(self.MODE_DIVOT)

            elif cond_inlet:
                self._enter_mode(self.MODE_INLET)

            elif cond_turn:
                self._enter_mode(self.MODE_TURN)

            else:
                self._enter_mode(self.MODE_STRAIGHT)

        # ----------------------------------------------------------------------------------------------------
        # ---- STATES ----------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------
        if self.mode == self.MODE_STRAIGHT:
            twist = Twist()
            if self.prev_state == self.MODE_INLET:
                self.wall_target = right_dist
                self.get_logger().info('UPDATE WALL TARGET')

            self.prev_state = self.MODE_STRAIGHT
            # error values
            dist_error = right_dist - self.wall_target   # + too far, - too close
            angle_error = self._wrap(right_angle - math.pi)
            
            # PD control staying a distance from the wall and parallel
            twist.linear.x  = -self.forward_speed
            twist.angular.z = self.PD_steering(angle_error, dist_error)
            self.vel_pub.publish(twist)

            # If the separate cases need to be broken up -----------------------------------------
            # prioritize staying parallel on right_angle and staying within tolerance of right dist
            #if right_cls == 'PARALLEL':
                #if abs(dist_error) <= self.dist_tol:
                    # forward speed
                    #twist.linear.x  = -self.forward_speed
                    # PD Steering straight
                    #twist.angular.z = self.PD_steering(angle_error,dist_error)
                #else:
                    # forward speed
                    #twist.linear.x  = -self.forward_speed
                    # PD Steering correction
                    #twist.angular.z = self.PD_steering(angle_error, dist_error)
            #else:
                # correct to parallel
                # forward speed
                #twist.linear.x  = -self.forward_speed
                # PD Steering correction
                #twist.angular.z = self.PD_steering(angle_error, dist_error)

            

        elif self.mode == self.MODE_DIVOT:
            twist = Twist()
            self.prev_state = self.MODE_DIVOT
            # Maintain right_dist until right_angle not parallel or no reading
            # coast when not parallel or no reading
            # when parallel again, re-assess right_dist and maintain
            if right_cls == 'PARALLEL':
                if self.prev_cls != 'PARALLEL':
                    self.wall_target = right_dist
                    self.get_logger().info('UPDATE WALL TARGET')
                dist_error = right_dist - self.wall_target
                angle_error = self._wrap(right_angle - math.pi)
                twist.linear.x = -self.forward_speed
                twist.angular.z = self.PD_steering(angle_error, dist_error)
                self.vel_pub.publish(twist)
            else:
                twist.linear.x = -self.forward_speed
                twist.angular.z = 0.0
                self.vel_pub.publish(twist)
                self.get_logger().info('COAST')
            
            self.prev_cls = right_cls

        elif self.mode == self.MODE_INLET:
            twist = Twist()
            self.prev_state = self.MODE_INLET
            dist_error = right_dist - self.wall_target
            angle_error = self._wrap(right_angle - math.pi)
            # analyze if the inlet will result in a collision / adjust if needed??
            # forward speed
            twist.linear.x  = -self.forward_speed
            # PD Steering correction
            twist.angular.z = self.PD_steering(angle_error, dist_error)
            self.vel_pub.publish(twist)


        elif self.mode == self.MODE_TURN:
            twist = Twist()
            self.prev_state = self.MODE_TURN
            elapsed = time.time() - self.mode_start_time
            turn_complete = (lookahead_cls == 'PARALLEL' and elapsed > 0.1)  # minimum dwell to avoid instant re-exit
            if turn_complete:
                self._enter_mode(self.MODE_STRAIGHT)
            # turn motor commands
            twist.linear.x = -self.forward_speed
            twist.angular.z = self.turn_rate
            self.vel_pub.publish(twist)
            

        elif self.mode == self.MODE_OBSTACLE:
            self.get_logger().info(f'OBSTACLE [{self.obstacle_phase}]')
            twist = Twist()
            self.prev_state = self.MODE_OBSTACLE

            if self.obstacle_phase == 'BACKUP':
                # Drive backward, straight
                twist.linear.x  = self.OB_forward_speed   # positive = backward in your convention
                twist.angular.z = 0.0
                self.vel_pub.publish(twist)
                # Once we have enough clearance, start turning right
                if front_dist > self.stop_dist * 1.5:   # gives a comfortable buffer
                    self.phase_start_time = time.time()
                    self.obstacle_phase = 'OB TURN'

            elif self.obstacle_phase == 'OB TURN':
                self.obstacle_turn_duration = 1
                elapsed = time.time() - self.phase_start_time
                
                if elapsed < self.obstacle_turn_duration:   # turn right first
                    twist.linear.x  = -self.OB_forward_speed
                    twist.angular.z = -self.turn_rate
                    self.vel_pub.publish(twist)
                else:                                        # then turn left until parallel
                    twist.linear.x  = -self.OB_forward_speed
                    twist.angular.z = self.turn_rate
                    self.vel_pub.publish(twist)
                    if right_cls == 'PARALLEL':
                        self.obstacle_phase = 'BACKUP'
                        self._enter_mode(self.MODE_STRAIGHT)
                            


    # ------------------------------------------------------------------
    # Sensor summary
    # ------------------------------------------------------------------
        self.get_logger().info(
            f'front: {front_dist:6.2f} m  |  '
            f'right: {right_dist:6.2f} m   ({right_cls})  |  '
            f'lookahead: {angle_dist:6.2f} m  ({lookahead_cls})  |  '
            f'correction: {self.last_correction:+.3f}  |  '
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
