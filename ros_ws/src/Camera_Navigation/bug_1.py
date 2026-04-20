#!/usr/bin/env python3
"""
Bug 1 wall follower — LiDAR only.

Two sensor cones
  RIGHT  : single beam at -90 deg   (perpendicular to right wall)
  FRONT  : single beam at   0 deg   (straight ahead)
  QUADRANT : beams 0 deg to -90 deg (used to decide which way to turn)

State machine
─────────────
  FOLLOW   : right arm in wall (<= right_wall_dist)
              → drive forward with gentle right-wall P control
  FIND_WALL: right arm not in wall
              → spin right until right arm sees wall
  BLOCKED  : front arm in wall (<= front_wall_dist)
              → backup then turn (from centerv3 logic)
  BACKING_UP / TURNING : recovery sub-states (from centerv3)

Transitions
  any non-BLOCKED state: if front_dist <= front_wall_dist → BACKING_UP
  FOLLOW → FIND_WALL : right_dist  > right_wall_dist
  FIND_WALL → FOLLOW : right_dist <= right_wall_dist
  BACKING_UP → TURNING: after backup_time seconds
  TURNING → FOLLOW   : front is clear
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Bug1(Node):

    MODE_FOLLOW     = 'FOLLOW'
    MODE_FIND_WALL  = 'FIND_WALL'
    MODE_BACKING_UP = 'BACKING_UP'
    MODE_TURNING    = 'TURNING'

    def __init__(self):
        super().__init__('bug_1')

        # ── tunable params ───────────────────────────────────────────────
        self.right_wall_dist  = 1.8   # m — "right arm in wall" threshold
        self.front_warn_dist  = 1.5   # m — long front arm: steer to avoid in FOLLOW
        self.front_stop_dist  = 0.5   # m — short front arm: trigger BACKING_UP
        self.front_clear_dist = 1.0   # m — hysteresis: need this to leave TURNING
        self.wall_fov_lo      = -105.0  # deg — left edge of right-wall FOV
        self.wall_fov_hi      = -75.0   # deg — right edge of right-wall FOV
        self.quadrant_open_thresh = 2.0  # m — beam reading above this = open space

        self.forward_speed  = 0.25    # m/s
        self.turn_speed     = 0.3     # rad/s (spin in place)
        self.sharp_turn_speed = 0.75     # rad/s
        self.backup_speed   = 0.25    # m/s magnitude during backup
        self.backup_time    = 1.0     # seconds to reverse
        self.kp_angle       = 3.85     # P gain when correction is leftward (+z)
        self.kp_angle_right = 3.85      # P gain when correction is rightward (-z)
        self.n_min_avg      = 3       # number of closest beams to average for min point
        self.shift_time     = 0.5     # seconds for each leg of the nudge-right maneuver
        self.shift_speed    = 0.3     # rad/s used during the nudge

        # half-cone for sampling (single beam is fine, small cone is more robust)
        self.cone_half = math.radians(5.0)

        # ── state ────────────────────────────────────────────────────────
        self.mode            = self.MODE_FIND_WALL  # start by finding the wall
        self.mode_start_time = time.time()
        self._shift_phase    = 'NONE'   # 'NONE', 'RIGHT', 'LEFT'
        self._shift_start    = 0.0
        self._nudge_cooldown = 0.0  # time.time() when last nudge finished

        # startup settling delay
        self.ready = False
        self.create_timer(3.0, self._set_ready)

        # ── ROS ──────────────────────────────────────────────────────────
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info('Bug1 started — waiting 3 s for lidar to stabilise…')

    def _set_ready(self):
        if not self.ready:
            self.ready = True
            self.get_logger().info('Lidar ready. Starting Bug 1.')

    # ── lidar helpers ────────────────────────────────────────────────────

    def _cone_min(self, ranges: np.ndarray, msg: LaserScan, center_rad: float) -> float:
        """Minimum valid range in a small cone around center_rad. Returns inf if empty."""
        lo = int(((center_rad - self.cone_half) - msg.angle_min) / msg.angle_increment)
        hi = int(((center_rad + self.cone_half) - msg.angle_min) / msg.angle_increment)
        n  = len(ranges)
        lo, hi = max(0, lo), min(n - 1, hi)
        cone  = ranges[lo:hi + 1]
        valid = cone[(cone > msg.range_min) & np.isfinite(cone)]
        return float(np.min(valid)) if valid.size > 0 else float('inf')

    def _quadrant_is_open(self, ranges: np.ndarray, msg: LaserScan) -> bool:
        """
        True if the MAJORITY of beams between 0 deg and -90 deg read above
        quadrant_open_thresh (open space / no wall in that sweep).
        """
        lo = int((math.radians(-90.0) - msg.angle_min) / msg.angle_increment)
        hi = int((math.radians(  0.0) - msg.angle_min) / msg.angle_increment)
        n  = len(ranges)
        lo, hi = max(0, lo), min(n - 1, hi)
        quad = ranges[lo:hi + 1]
        # count beams that are either inf or above threshold
        open_count = int(np.sum((quad > self.quadrant_open_thresh) | np.isinf(quad)))
        return open_count > len(quad) // 3

    def _enter_mode(self, mode: str):
        self.mode            = mode
        self.mode_start_time = time.time()
        self.get_logger().info(f'→ {mode}')

    def nudge_right(self, now: float) -> bool:
        if self._shift_phase == 'NONE':
            self._shift_phase = 'RIGHT'
            self._shift_start = now

        if self._shift_phase == 'RIGHT':
            if now - self._shift_start < self.shift_time * 2:
                self._publish(self.forward_speed, -self.shift_speed * 2)
                return True
            self._shift_phase = 'LEFT'
            self._shift_start = now

        if self._shift_phase == 'LEFT':
            if now - self._shift_start < self.shift_time:
                self._publish(self.forward_speed, self.shift_speed)
                return True
            self._shift_phase = 'STRAIGHT'
            self._shift_start = now

        if self._shift_phase == 'STRAIGHT':
            if now - self._shift_start < self.shift_time * 2:
                self._publish(self.forward_speed, self.shift_speed * 0.5)
                return True
            self._shift_phase = 'NONE'
            self._nudge_cooldown = now  # start cooldown

        return False  # nudge complete
    
    def turn_right(self, now: float) -> bool:
        if self._shift_phase == 'NONE':
            self._shift_phase = 'RIGHT'
            self._shift_start = now

        if self._shift_phase == 'RIGHT':
            if now - self._shift_start < self.shift_time * 4:
                self._publish(self.forward_speed, -self.shift_speed * 2)
                return True
            self._shift_phase = 'LEFT'
            self._shift_start = now

        if self._shift_phase == 'LEFT':
            if now - self._shift_start < self.shift_time:
                self._publish(self.forward_speed, self.shift_speed)
                return True
            self._shift_phase = 'NONE'

        return False  # nudge complete

    def _publish(self, lin: float, ang: float):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        print(f"Publishing: lin={t.linear.x:.2f} m/s  ang={t.angular.z:.2f} rad/s")
        self.vel_pub.publish(t)

    # ── main callback ────────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        if not self.ready:
            return

        # PROCESS LIDAR READINGS
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)  # inf to max range
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        right_dist = self._cone_min(ranges, msg, math.radians(-90.0))
        front_dist = self._cone_min(ranges, msg, 0.0)
        now        = time.time()

        # ── BACKING_UP ───────────────────────────────────────────────────
        if self.mode == self.MODE_BACKING_UP:
            # set turn direction once the first time we ever back up
            if not hasattr(self, '_turn_dir'):
                quad_open = self._quadrant_is_open(ranges, msg)
                self._turn_dir = -1.0 if quad_open else 1.0  # open quad → turn right, closed → turn left
            if now - self.mode_start_time < self.backup_time:
                self._publish(-self.backup_speed, self._turn_dir * self.turn_speed)
                self.get_logger().info(
                    f'[BACKING_UP] front={front_dist:.2f}m',
                    throttle_duration_sec=0.5)
                return
            self._enter_mode(self.MODE_TURNING)
            # fall through to TURNING this cycle

        # ── TURNING ──────────────────────────────────────────────────────
        if self.mode == self.MODE_TURNING:
            if front_dist >= self.front_clear_dist:
                self._enter_mode(self.MODE_FOLLOW)
                self._publish(0.0, 0.0) # stop before moving forward
                return
            if front_dist <= self.front_stop_dist and (now - self.mode_start_time) >= 0.5:
                self.get_logger().warn(f'[TURNING] front wall at {front_dist:.2f}m — backing up again.')
                self._publish(0.0, 0.0)
                self._enter_mode(self.MODE_BACKING_UP)
                return
            self._publish(self.forward_speed, self._turn_dir * self.sharp_turn_speed)
            self.get_logger().info(
                f'[TURNING {"R" if self._turn_dir < 0 else "L"}] front={front_dist:.2f}m',
                throttle_duration_sec=0.5)
            return
        
        # ── hard stop: short front arm → BACKING_UP (preempts everything) ──
        if front_dist <= self.front_stop_dist:
            self.get_logger().warn(f'Front wall at {front_dist:.2f}m — backing up.')
            self._publish(0.0, 0.0) # stop
            self._enter_mode(self.MODE_BACKING_UP)
            return

        # ── FIND_WALL ────────────────────────────────────────────────────
        if self.mode == self.MODE_FIND_WALL:
            if right_dist <= self.right_wall_dist:
                self._shift_phase = 'NONE'
                self._enter_mode(self.MODE_FOLLOW)
                # fall through to FOLLOW this cycle
            else:
                # wall is nearby but not close enough — nudge right to close the gap
                if right_dist < 3: 
                    if self.nudge_right(now):
                        self.get_logger().info(
                            f'[FIND_WALL] nudging right  right={right_dist:.2f}m  phase={self._shift_phase}',
                            throttle_duration_sec=0.3)
                        return
                else:
                    if self.turn_right(now):
                        self.get_logger().info(
                            f'[FIND_WALL] turning right  right={right_dist:.2f}m  phase={self._shift_phase}',
                            throttle_duration_sec=0.3)
                self.get_logger().info(
                    f'[FIND_WALL] right={right_dist:.2f}m — spinning right',
                    throttle_duration_sec=0.5)
                return

        # ── FOLLOW ───────────────────────────────────────────────────────
        if self.mode == self.MODE_FOLLOW:
            if right_dist > self.right_wall_dist:
                # lost the wall
                self._enter_mode(self.MODE_FIND_WALL)
                self._publish(self.forward_speed, -self.turn_speed)
                return

            # long front arm warning — steer before hitting the stop threshold
            if front_dist <= self.front_warn_dist:
                # front is also in wall — check quadrant to decide turn direction
                quad_open = self._quadrant_is_open(ranges, msg)
                if quad_open:
                    # open space between front and right → turn right (into opening)
                    self._publish(self.forward_speed, -self.turn_speed)
                    self.get_logger().info(
                        '[FOLLOW] front wall + open quadrant → turning right',
                        throttle_duration_sec=0.5)
                else:
                    # no opening → turn left
                    self._publish(self.forward_speed, self.turn_speed)
                    self.get_logger().info(
                        '[FOLLOW] front wall + closed quadrant → turning left',
                        throttle_duration_sec=0.5)
                return

            # normal follow: slope-based angle correction to stay parallel to right wall
            # get all valid beams in the wall FOV
            fov_mask    = (angles >= math.radians(self.wall_fov_lo)) & \
                          (angles <= math.radians(self.wall_fov_hi))
            fov_r       = ranges[fov_mask]
            fov_a       = angles[fov_mask]
            valid_fov   = fov_r > msg.range_min
            fov_r_valid = fov_r[valid_fov]
            fov_a_valid = fov_a[valid_fov]

            if fov_r_valid.size > 0:
                # average N closest beams
                n           = min(self.n_min_avg, fov_r_valid.size)
                closest_idx = np.argpartition(fov_r_valid, n - 1)[:n]
                min_dist_fov    = float(np.mean(fov_r_valid[closest_idx]))
                min_angle_deg   = float(np.degrees(np.mean(fov_a_valid[closest_idx])))

                # perpendicular beam at -90 deg
                idx_90     = int(np.argmin(np.abs(fov_a - math.radians(-90.0))))
                dist_at_90 = float(fov_r[idx_90])

                # slope error: 0 when parallel
                # min_angle > -90 (nose angled toward wall) → turn left  (+z)
                # min_angle < -90 (nose angled away)        → turn right (-z)
                slope_mag   = dist_at_90 - min_dist_fov
                angle_sign  = math.copysign(1.0, min_angle_deg - (-90.0))
                angle_error = angle_sign * slope_mag
                kp = self.kp_angle_right if angle_error < 0 else self.kp_angle
                turn = float(np.clip(kp * angle_error,
                                     -self.turn_speed, self.turn_speed))
            else:
                turn = 0.0
                min_dist_fov = float('inf')
                dist_at_90   = float('inf')
                angle_error  = 0.0

            self._publish(self.forward_speed, turn)
            self.get_logger().info(
                f'[FOLLOW] d90={dist_at_90:.2f}m  min={min_dist_fov:.2f}m  '
                f'slope_err={angle_error:+.3f}m  turn={turn:+.3f}',
                throttle_duration_sec=0.2)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bug1()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
