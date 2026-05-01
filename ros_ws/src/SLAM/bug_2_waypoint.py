#!/usr/bin/env python3
"""
Bug 2 waypoint follower — same control structure as bug_2.py, but the
"keep right arm on the wall" steady-state is replaced with waypoint pursuit
driven by /slam/pose.

State machine
─────────────
  FOLLOW      : drive toward the current goal waypoint with P control on heading
  BACKING_UP  : front arm in wall — reverse for backup_time seconds
  TURNING     : recovery turn (direction picked from the -90..0 deg quadrant)

Transitions
  any FOLLOW : front_dist <= front_stop_dist          → BACKING_UP
  BACKING_UP : after backup_time seconds              → TURNING
  TURNING    : front_dist >= front_clear_dist         → FOLLOW
  TURNING    : front_dist <= front_stop_dist (>0.5s)  → BACKING_UP

Waypoint logic mirrors feedback_waypoints.py:
  - read x, y, yaw from /slam/pose (PoseWithCovarianceStamped)
  - look ahead from current_idx and pick the *furthest* waypoint still inside
    detection_radius — that becomes the goal, and current_idx advances to it
  - if no waypoint is in radius, fall back to current_idx + 1
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped


class Bug2Waypoint(Node):

    MODE_FOLLOW     = 'FOLLOW'
    MODE_BACKING_UP = 'BACKING_UP'
    MODE_TURNING    = 'TURNING'

    def __init__(self, waypoint_file='pose_history_save.txt', every_n=4, time_start=0):
        super().__init__('bug_2_waypoint')

        # ── tunable params (lidar / recovery — same as bug_2.py) ─────────
        self.front_warn_dist  = 1.5
        self.front_stop_dist  = 0.75
        self.front_clear_dist = 1.0
        self.quadrant_open_thresh = 2.0

        self.forward_speed     = 0.25
        self.turn_speed        = 0.3
        self.sharp_turn_speed  = 0.75
        self.backup_speed      = 0.25
        self.backup_time       = 1.0

        # ── tunable params (waypoint pursuit) ────────────────────────────
        self.detection_radius = 1.2    # m — lookahead window for picking goal
        self.goal_tolerance   = 0.5    # m — final-waypoint stop distance
        self.kp_heading       = 1.5    # P gain on heading error (rad → rad/s)
        self.max_turn         = 1.0    # rad/s cap on heading P controller

        self.cone_half = math.radians(5.0)

        # ── state ────────────────────────────────────────────────────────
        self.mode            = self.MODE_FOLLOW
        self.mode_start_time = time.time()
        self._turn_dir       = -1.0

        self.have_pose = False
        self.pose_x = self.pose_y = self.pose_th = 0.0
        self.current_idx = 0

        self.waypoints = self._load_waypoints(waypoint_file, every_n, time_start)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {waypoint_file}')

        # startup settling delay (lidar + slam)
        self.ready = False
        self.create_timer(3.0, self._set_ready)

        # ── ROS ──────────────────────────────────────────────────────────
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self._pose_cb, 10)
        self.get_logger().info('Bug2Waypoint started — waiting 3 s for sensors…')

    def _load_waypoints(self, filename, every_n, time_start):
        data = np.loadtxt(filename)
        data = data[::every_n]
        if data.ndim == 2 and data.shape[1] >= 4:
            data = data[data[:, 3] >= time_start]
        return data

    def _set_ready(self):
        if not self.ready:
            self.ready = True
            self.get_logger().info('Sensors ready. Starting waypoint follow.')

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.pose_th = 2.0 * math.atan2(qz, qw)
        self.have_pose = True

    # ── lidar helpers (same as bug_2.py) ─────────────────────────────────

    def _cone_min(self, ranges: np.ndarray, msg: LaserScan, center_rad: float) -> float:
        lo = int(((center_rad - self.cone_half) - msg.angle_min) / msg.angle_increment)
        hi = int(((center_rad + self.cone_half) - msg.angle_min) / msg.angle_increment)
        n  = len(ranges)
        lo, hi = max(0, lo), min(n - 1, hi)
        cone  = ranges[lo:hi + 1]
        valid = cone[(cone > msg.range_min) & np.isfinite(cone)]
        return float(np.min(valid)) if valid.size > 0 else float('inf')

    def _quadrant_is_open(self, ranges: np.ndarray, msg: LaserScan) -> bool:
        lo = int((math.radians(-90.0) - msg.angle_min) / msg.angle_increment)
        hi = int((math.radians(  0.0) - msg.angle_min) / msg.angle_increment)
        n  = len(ranges)
        lo, hi = max(0, lo), min(n - 1, hi)
        quad = ranges[lo:hi + 1]
        open_count = int(np.sum((quad > self.quadrant_open_thresh) | np.isinf(quad)))
        return open_count > len(quad) // 3

    # ── waypoint helpers ─────────────────────────────────────────────────

    def _get_goal_waypoint(self):
        """Furthest waypoint still in detection_radius; falls back to current_idx+1."""
        last_idx, last_wp = None, None
        for idx in range(self.current_idx, len(self.waypoints)):
            wp = self.waypoints[idx]
            d  = math.hypot(wp[0] - self.pose_x, wp[1] - self.pose_y)
            if d <= self.detection_radius:
                last_idx, last_wp = idx, wp
            else:
                break
        if last_idx is None:
            nxt = self.current_idx + 1
            if nxt < len(self.waypoints):
                return nxt, self.waypoints[nxt]
            return None, None
        return last_idx, last_wp

    @staticmethod
    def _normalize_angle(a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    # ── publishing ───────────────────────────────────────────────────────

    def _enter_mode(self, mode: str):
        self.mode            = mode
        self.mode_start_time = time.time()
        self.get_logger().info(f'→ {mode}')

    def _publish(self, lin: float, ang: float):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.vel_pub.publish(t)

    # ── main loop (driven by /scan) ──────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        if not self.ready or not self.have_pose:
            return

        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        front_dist = self._cone_min(ranges, msg, 0.0)
        now        = time.time()

        print(f'[DEBUG] mode={self.mode}  front={front_dist:.2f}m  '
              f'pose=({self.pose_x:.2f},{self.pose_y:.2f},{math.degrees(self.pose_th):.1f}°)  '
              f'idx={self.current_idx}/{len(self.waypoints)-1}')

        # ── BACKING_UP ───────────────────────────────────────────────────
        if self.mode == self.MODE_BACKING_UP:
            if now - self.mode_start_time < self.backup_time:
                self._publish(-self.backup_speed, 0.0)
                self.get_logger().info(f'[BACKING_UP] front={front_dist:.2f}m',
                                       throttle_duration_sec=0.5)
                return
            quad_open = self._quadrant_is_open(ranges, msg)
            self._turn_dir = -1.0 if quad_open else 1.0
            self._enter_mode(self.MODE_TURNING)
            # fall through

        # ── TURNING ──────────────────────────────────────────────────────
        if self.mode == self.MODE_TURNING:
            if front_dist >= self.front_clear_dist:
                self._enter_mode(self.MODE_FOLLOW)
                self._publish(0.0, 0.0)
                return
            if front_dist <= self.front_stop_dist and (now - self.mode_start_time) >= 0.5:
                self.get_logger().warn(f'[TURNING] front wall {front_dist:.2f}m — backing up again')
                self._publish(0.0, 0.0)
                self._enter_mode(self.MODE_BACKING_UP)
                return
            self._publish(self.forward_speed, self._turn_dir * self.sharp_turn_speed * 2)
            self.get_logger().info(
                f'[TURNING {"R" if self._turn_dir < 0 else "L"}] front={front_dist:.2f}m',
                throttle_duration_sec=0.5)
            return

        # ── front-stop preempts FOLLOW ───────────────────────────────────
        if front_dist <= self.front_stop_dist:
            self.get_logger().warn(f'Front wall {front_dist:.2f}m — backing up.')
            self._publish(0.0, 0.0)
            self._enter_mode(self.MODE_BACKING_UP)
            return

        # ── FOLLOW (waypoint pursuit) ────────────────────────────────────
        if self.current_idx >= len(self.waypoints):
            self._publish(0.0, 0.0)
            self.get_logger().info('All waypoints reached.', throttle_duration_sec=2.0)
            return

        idx, goal = self._get_goal_waypoint()
        if goal is None:
            self._publish(0.0, 0.0)
            self.get_logger().info('No reachable waypoint.', throttle_duration_sec=2.0)
            return
        self.current_idx = idx

        dx = goal[0] - self.pose_x
        dy = goal[1] - self.pose_y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        heading_err = self._normalize_angle(bearing - self.pose_th)

        # final-waypoint stop
        if idx >= len(self.waypoints) - 1 and dist <= self.goal_tolerance:
            self._publish(0.0, 0.0)
            self.get_logger().info('Final waypoint reached.')
            self.current_idx = len(self.waypoints)
            return

        # front-warn: same quadrant decision as bug_2.py FOLLOW front-warn,
        # overrides the heading P controller for this cycle to dodge the obstacle.
        if front_dist <= self.front_warn_dist:
            quad_open = self._quadrant_is_open(ranges, msg)
            if quad_open:
                self._publish(self.forward_speed, -self.turn_speed)
                self.get_logger().info('[FOLLOW] front warn + open quadrant → turning right',
                                       throttle_duration_sec=0.5)
            else:
                self._publish(self.forward_speed, self.turn_speed)
                self.get_logger().info('[FOLLOW] front warn + closed quadrant → turning left',
                                       throttle_duration_sec=0.5)
            return

        # heading P controller — slow forward speed when heading error is large
        turn = float(np.clip(self.kp_heading * heading_err,
                             -self.max_turn, self.max_turn))
        speed_scale = max(0.0, math.cos(heading_err))
        lin = self.forward_speed * speed_scale

        self._publish(lin, turn)
        self.get_logger().info(
            f'[FOLLOW] idx={idx}/{len(self.waypoints)-1}  d={dist:.2f}m  '
            f'h_err={math.degrees(heading_err):+.1f}°  lin={lin:.2f}  turn={turn:+.2f}',
            throttle_duration_sec=0.3)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bug2Waypoint(waypoint_file='pose_history_save.txt', every_n=4)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
