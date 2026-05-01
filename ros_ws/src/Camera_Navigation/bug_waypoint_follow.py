#!/usr/bin/env python3
"""
Combined Bug 1 wall follower and waypoint follower.

Primary behavior:
- follow waypoints loaded from Camera_Navigation/pose_history.txt
- use pose updates from /slam/pose for goal seeking
- monitor /scan and run Bug 1 style backup/turn recovery when a front obstacle blocks the path
"""

import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

base_dir = os.path.dirname(__file__)
pose_file = os.path.join(base_dir, "pose_history.txt")

with open(pose_file, "r") as f:
    lines = f.readlines()

waypoints = [list(map(float, line.split()[:3])) for line in lines if line.strip()]


class BugWaypointFollower(Node):
    MODE_WAYPOINT = 'WAYPOINT'
    MODE_BACKING_UP = 'BACKING_UP'
    MODE_TURNING = 'TURNING'

    def __init__(self):
        super().__init__('bug_waypoint_follower')

        self.waypoints = waypoints
        self.current_idx = 0
        self.goal_tolerance = 0.5

        self.forward_speed = 0.25
        self.angular_speed = 0.3
        self.max_angular_speed = 1.5
        self.sharp_turn_speed = 0.75
        self.backup_speed = 0.25
        self.backup_time = 1.0

        self.front_warn_dist = 1.5
        self.front_stop_dist = 0.75
        self.front_clear_dist = 1.0

        self.cone_half = math.radians(5.0)

        self.state = self.MODE_WAYPOINT
        self.state_start_time = time.time()
        self.turn_direction = 1.0

        self.latest_pose = None
        self.latest_scan = None
        self.front_dist = np.inf
        self.ranges = None
        self.angles = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self.pose_callback, 10)

        self.get_logger().info('BugWaypointFollower started.')

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = msg.pose.pose
        self.evaluate_navigation()

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        self.ranges = ranges
        self.angles = np.arange(len(ranges), dtype=np.float32) * msg.angle_increment + msg.angle_min
        self.front_dist = self._cone_min(ranges, msg, 0.0)
        self.latest_scan = msg
        self.evaluate_navigation()

    def evaluate_navigation(self):
        if self.latest_pose is None or self.latest_scan is None:
            return

        now = time.time()

        if self.state == self.MODE_BACKING_UP:
            if now - self.state_start_time < self.backup_time:
                self._publish(-self.backup_speed, 0.0)
                self.get_logger().info('[BACKING_UP] reversing to clear obstacle.')
                return

            self.turn_direction = -1.0 if self._quadrant_is_open(self.ranges, self.latest_scan) else 1.0
            self._enter_state(self.MODE_TURNING)
            return

        if self.state == self.MODE_TURNING:
            if self.front_dist >= self.front_clear_dist:
                self.get_logger().info('Front path clear. Returning to waypoint mode.')
                self._enter_state(self.MODE_WAYPOINT)
                self._publish(0.0, 0.0)
                return

            if self.front_dist <= self.front_stop_dist and now - self.state_start_time >= 0.5:
                self.get_logger().warn('Still blocked during TURNING; backing up again.')
                self._enter_state(self.MODE_BACKING_UP)
                return

            self._publish(self.forward_speed, self.turn_direction * self.sharp_turn_speed * 2)
            self.get_logger().info(f'[TURNING] direction={"R" if self.turn_direction < 0 else "L"}')
            return

        if self.front_dist <= self.front_stop_dist:
            self.get_logger().warn(f'Obstacle too close (front={self.front_dist:.2f}m). Starting recovery.')
            self._enter_state(self.MODE_BACKING_UP)
            return

        self._follow_waypoints()

    def _follow_waypoints(self):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping.')
            self._publish(0.0, 0.0)
            return

        goal_x, goal_y, goal_theta = self.waypoints[self.current_idx][:3]
        current_pose = self.latest_pose

        dx = goal_x - current_pose.position.x
        dy = goal_y - current_pose.position.y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance:
            self.get_logger().info(f'Waypoint {self.current_idx} reached.')
            self.current_idx += 1
            self._publish(0.0, 0.0)
            return

        if self.front_dist < self.front_warn_dist:
            quad_open = self._quadrant_is_open(self.ranges, self.latest_scan)
            if quad_open:
                self._publish(self.forward_speed, -self.angular_speed)
                self.get_logger().info('[WAYPOINT] obstacle ahead; turning right into open space.')
            else:
                self._publish(self.forward_speed, self.angular_speed)
                self.get_logger().info('[WAYPOINT] obstacle ahead; turning left.')
            return

        desired_angle = math.atan2(dy, dx)
        yaw = self._get_yaw_from_quaternion(current_pose.orientation)
        angle_diff = self._normalize_angle(desired_angle - yaw)

        cmd_msg = Twist()
        cmd_msg.linear.x = self.forward_speed
        cmd_msg.angular.z = float(max(-self.max_angular_speed,
                                     min(self.angular_speed * angle_diff, self.max_angular_speed)))
        self.cmd_pub.publish(cmd_msg)

    def _cone_min(self, ranges: np.ndarray, msg: LaserScan, center_rad: float) -> float:
        lo = int(((center_rad - self.cone_half) - msg.angle_min) / msg.angle_increment)
        hi = int(((center_rad + self.cone_half) - msg.angle_min) / msg.angle_increment)
        lo = max(0, lo)
        hi = min(len(ranges) - 1, hi)
        cone = ranges[lo:hi + 1]
        valid = cone[(cone > msg.range_min) & np.isfinite(cone)]
        return float(np.min(valid)) if valid.size > 0 else float('inf')

    def _quadrant_is_open(self, ranges: np.ndarray, msg: LaserScan) -> bool:
        lo = int((math.radians(-90.0) - msg.angle_min) / msg.angle_increment)
        hi = int((math.radians(0.0) - msg.angle_min) / msg.angle_increment)
        lo = max(0, lo)
        hi = min(len(ranges) - 1, hi)
        quad = ranges[lo:hi + 1]
        open_count = int(np.sum((quad > self.front_warn_dist) | np.isinf(quad)))
        return open_count > len(quad) // 3

    def _publish(self, linear: float, angular: float):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)

    def _enter_state(self, new_state: str):
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f'→ {new_state}')

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def _get_yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = BugWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
