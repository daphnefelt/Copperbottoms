#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import time


class WallFollower(Node):

    MODE_CENTER = 0
    MODE_BACKING_UP = 1
    MODE_HALLWAY_TURN_RIGHT = 2

    def __init__(self):
        super().__init__('wall_follower')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- Parameters ----
        self.forward_speed = 0.3
        self.max_angular_z = 1.0

        self.kp = 1.2
        self.kd = 0.4

        self.wall_target = 1.2   # RIGHT WALL TARGET
        self.divot_threshold = 0.7   # readings > wall_target + this are ignored (divots)

        self.stop_dis = 0.3
        self.clear_dis = 0.7

        self.side_half_cone = math.radians(20)

        # ---- State ----
        self.mode = self.MODE_CENTER

        self.prev_error = 0.0
        self.filtered_error = 0.0
        self.prev_time = None
        self.filter_alpha = 0.5

        self.backup_start = None

        # turn detection
        self.hallway_turn_trigger = 4.5
        self.suppress_right_turn = False

    # ------------------------------------------------------------
    # Utility: get median distance in angular window
    # ------------------------------------------------------------
    def get_range_window(self, ranges, angle_min, angle_inc,
                         center_angle, half_width):

        vals = []
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_inc
            if abs(angle - center_angle) <= half_width:
                if not math.isinf(r) and not math.isnan(r):
                    vals.append(r)

        if len(vals) == 0:
            return float('nan')

        return float(np.median(vals))

    # ------------------------------------------------------------
    def scan_callback(self, msg):

        ranges = msg.ranges
        now = time.time()

        # ---- Front distance ----
        front_min = self.get_range_window(
            ranges, msg.angle_min, msg.angle_increment,
            0.0, math.radians(15))

        # ---- Right distance ----
        right_dist = self.get_range_window(
            ranges, msg.angle_min, msg.angle_increment,
            -math.pi / 2, self.side_half_cone)

        # =========================================================
        # MODE: BACKING UP
        # =========================================================
        if self.mode == self.MODE_BACKING_UP:
            if self.backup_start is None:
                self.backup_start = now

            if now - self.backup_start < 0.5:
                twist = Twist()
                twist.linear.x = -0.1
                self.vel_pub.publish(twist)
                return
            else:
                self.mode = self.MODE_CENTER
                self.backup_start = None

        # =========================================================
        # MODE: SERVO TURN (placeholder - unchanged behavior)
        # =========================================================
        if self.mode == self.MODE_HALLWAY_TURN_RIGHT:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = -0.8
            self.vel_pub.publish(twist)

            # simple exit condition
            if not math.isnan(right_dist) and right_dist < 1.5:
                self.mode = self.MODE_CENTER

            return

        # =========================================================
        # MODE: NORMAL (RIGHT WALL FOLLOW)
        # =========================================================

        # ---- Emergency stop ----
        if not math.isnan(front_min) and front_min < self.stop_dis:
            self.get_logger().warn(f'Obstacle at {front_min:.2f} m → backing up')
            self.mode = self.MODE_BACKING_UP
            return

        # ---- Right opening detection ----
        if (not self.suppress_right_turn
                and not math.isnan(right_dist)
                and right_dist > self.hallway_turn_trigger):

            self.get_logger().info(f'Right opening detected ({right_dist:.2f} m)')
            self.mode = self.MODE_HALLWAY_TURN_RIGHT
            self.suppress_right_turn = True
            return

        if not math.isnan(right_dist) and right_dist < 1.5:
            self.suppress_right_turn = False

        # =========================================================
        # RIGHT-WALL-ONLY PD CONTROL
        # =========================================================

        if math.isnan(right_dist):
            error = 0.0
        elif right_dist > self.wall_target + self.divot_threshold:
            # Divot: reading far past target, treat as wall gap and coast straight
            error = 0.0
        else:
            error = self.wall_target - right_dist

        # ---- Filtering ----
        self.filtered_error = (
            self.filter_alpha * error +
            (1 - self.filter_alpha) * self.filtered_error
        )

        # ---- Derivative ----
        if self.prev_time is None:
            d_error = 0.0
        else:
            dt = max(now - self.prev_time, 1e-3)
            d_error = (self.filtered_error - self.prev_error) / dt

        self.prev_error = self.filtered_error
        self.prev_time = now

        # ---- PD output ----
        angular_z = self.kp * self.filtered_error + self.kd * d_error
        angular_z = float(np.clip(angular_z,
                                  -self.max_angular_z,
                                  self.max_angular_z))

        # ---- Command ----
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = angular_z
        self.vel_pub.publish(twist)

        # ---- Debug ----
        self.get_logger().info(
            f'right={right_dist:.2f} | front={front_min:.2f} | '
            f'err={error:+.2f} | ang={angular_z:+.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
