#!/usr/bin/env python3
"""

Commands
NUDGE RIGHT
NUDGE LEFT

"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

import os

base_dir = os.path.dirname(__file__)
pose_file = os.path.join(base_dir, "pose_history.txt")

with open(pose_file, "r") as f:
    lines = f.readlines()

# Example: parse numeric values
poses = [line.strip() for line in lines if line.strip()]


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_update, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self.pose_cb, 10)
        self.waypoints = [list(map(float, pose.split())) for pose in poses]
        self.current_idx = 0
        self.goal_tolerance = 0.5
        self.linear_speed = 0.3
        self.angular_speed = 0.3
        self.max_angular_speed = 1.5
        self.front_dist = np.inf

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!")
            return
        
        current_pose = msg.pose.pose
        goal_x, goal_y, goal_theta = self.waypoints[self.current_idx][:3]
        
        # Compute the distance and angle to the goal
        dx = goal_x - current_pose.position.x
        dy = goal_y - current_pose.position.y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        
        # Check if we are within the tolerance of the goal
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info(f"Waypoint {self.current_idx} reached!")
            self.current_idx += 1
            return
        
        # Compute the desired angle to the goal
        desired_angle = math.atan2(dy, dx)
        
        # Compute the angle difference
        angle_diff = desired_angle - self.get_yaw_from_quaternion(current_pose.orientation)
        
        # Normalize the angle difference to [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        # Create a Twist message to command the robot
        cmd_msg = Twist()
        cmd_msg.linear.x = self.linear_speed
        cmd_msg.angular.z = max(-self.max_angular_speed, min(self.angular_speed * angle_diff, self.max_angular_speed))
        
        # Publish the command
        self.cmd_pub.publish(cmd_msg)
    
    def lidar_update(self, msg: LaserScan):
        # Get the distance to the closest obstacle in front of the robot
        front_index = len(msg.ranges) // 2
        self.front_dist = msg.ranges[front_index]
        # Implement obstacle avoidance logic here if needed
        if self.front_dist < 1.0:  # Adjust threshold as needed
            self.get_logger().info("Obstacle detected! Reversing.")
            cmd_msg = Twist()
            cmd_msg.linear.x = -0.1  # Reverse speed
            self.cmd_pub.publish(cmd_msg)
            # Turn left backwards
            cmd_msg.angular.z = self.angular_speed
            self.cmd_pub.publish(cmd_msg)
        
    def get_yaw_from_quaternion(self, q):
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

