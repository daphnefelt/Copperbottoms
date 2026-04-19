#!/usr/bin/env python3
"""
Simple terminal-based odometry monitor
Displays position, orientation, and velocity in real-time
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import os


class OdometryMonitor(Node):
    def __init__(self):
        super().__init__('odometry_monitor')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom/lio',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odometry Monitor Started - Press Ctrl+C to exit')
        
    def odom_callback(self, msg):
        # Clear screen (works on Linux/Mac)
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Extract data
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        ang_vel = msg.twist.twist.angular
        
        # Convert quaternion to euler angles (yaw)
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw)
        
        # Display
        print("=" * 60)
        print("           LIDAR-INERTIAL ODOMETRY MONITOR")
        print("=" * 60)
        print()
        print("POSITION:")
        print(f"  X: {pos.x:8.3f} m")
        print(f"  Y: {pos.y:8.3f} m")
        print(f"  Z: {pos.z:8.3f} m")
        print()
        print("ORIENTATION:")
        print(f"  Yaw:   {yaw_deg:8.2f}°")
        print(f"  Quat:  [{orient.w:.3f}, {orient.x:.3f}, {orient.y:.3f}, {orient.z:.3f}]")
        print()
        print("LINEAR VELOCITY:")
        print(f"  X: {vel.x:8.3f} m/s")
        print(f"  Y: {vel.y:8.3f} m/s")
        print(f"  Z: {vel.z:8.3f} m/s")
        print(f"  Speed: {math.sqrt(vel.x**2 + vel.y**2 + vel.z**2):8.3f} m/s")
        print()
        print("ANGULAR VELOCITY:")
        print(f"  Roll:  {math.degrees(ang_vel.x):8.2f}°/s")
        print(f"  Pitch: {math.degrees(ang_vel.y):8.2f}°/s")
        print(f"  Yaw:   {math.degrees(ang_vel.z):8.2f}°/s")
        print()
        print("=" * 60)
        print("Press Ctrl+C to exit")


def main(args=None):
    rclpy.init(args=args)
    node = OdometryMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nOdometry Monitor Stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
