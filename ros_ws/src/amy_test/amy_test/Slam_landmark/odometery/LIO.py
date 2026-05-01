#!/usr/bin/env python3
"""
LiDAR-Inertial Odometry (LIO) Node
Combines LiDAR, depth camera, and IMU data for accurate odometry estimation

cd ~/code/Copperbottoms/ros_ws
colcon build --packages-select realsense_publisher amy_test
source install/setup.bash

ros2 launch amy_test odometry_launch.py


LIO Node - LIO.py - Fuses IMU, LiDAR, and depth data for odometry
Enhanced RealSense Node - realsense_depth_node.py - Publishes color, depth, and point cloud
Odometry Launch File - odometry_launch.py - Launches all sensors + LIO node


# Watch odometry position update in real-time
ros2 topic echo /odom/lio --field pose.pose.position

monitor 
ros2 run amy_test odometry_monitor
Position (X, Y, Z)
Orientation (Yaw angle)
Linear velocity
Angular velocity

# Monitor all topics with rates
watch -n 1 'ros2 topic list'
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.spatial.transform import Rotation


class LIONode(Node):
    def __init__(self):
        super().__init__('lio_odometry')
        
        # QoS profile for sensor data (BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.imu_accel_sub = self.create_subscription(
            Vector3,
            '/imu/accel',
            self.accel_callback,
            sensor_qos
        )
        
        self.imu_gyro_sub = self.create_subscription(
            Vector3,
            '/imu/gyro',
            self.gyro_callback,
            sensor_qos
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            sensor_qos
        )
        
        self.depth_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.depth_callback,
            sensor_qos
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom/lio',
            10
        )
        
        # State variables
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion [w, x, y, z]
        
        self.accel = np.array([0.0, 0.0, 0.0])
        self.gyro = np.array([0.0, 0.0, 0.0])
        
        self.last_time = self.get_clock().now()
        
        # Timer for odometry update
        self.timer = self.create_timer(0.05, self.update_odometry)  # 20 Hz
        
        self.get_logger().info('LIO Odometry Node Started')
        
    def accel_callback(self, msg):
        """Store accelerometer data"""
        self.accel = np.array([msg.x, msg.y, msg.z])
        
    def gyro_callback(self, msg):
        """Store gyroscope data"""
        self.gyro = np.array([msg.x, msg.y, msg.z])
        
    def lidar_callback(self, msg):
        """Process LiDAR scan for odometry"""
        # TODO: Implement scan matching for odometry correction
        # For now, just log that we're receiving data
        pass
        
    def depth_callback(self, msg):
        """Process depth point cloud for odometry"""
        # TODO: Implement visual odometry from depth data
        # For now, just log that we're receiving data
        pass
        
    def update_odometry(self):
        """Update odometry estimate using IMU integration"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt > 0.5 or dt <= 0:  # Skip if dt is too large or invalid
            return
            
        # Update orientation using gyroscope
        rotation = Rotation.from_quat([
            self.orientation[1], 
            self.orientation[2], 
            self.orientation[3], 
            self.orientation[0]
        ])
        
        # Angular velocity integration
        angle_delta = self.gyro * dt
        delta_rotation = Rotation.from_rotvec(angle_delta)
        new_rotation = rotation * delta_rotation
        
        quat = new_rotation.as_quat()  # [x, y, z, w]
        self.orientation = np.array([quat[3], quat[0], quat[1], quat[2]])  # [w, x, y, z]
        
        # Update velocity and position using accelerometer
        # Rotate acceleration to world frame
        accel_world = rotation.apply(self.accel)
        
        # Remove gravity (assuming z-up)
        accel_world[2] -= 9.81
        
        # Integrate acceleration to velocity
        self.velocity += accel_world * dt
        
        # Integrate velocity to position
        self.position += self.velocity * dt
        
        # Publish odometry
        self.publish_odometry()
        
    def publish_odometry(self):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        
        # Orientation
        odom.pose.pose.orientation.w = self.orientation[0]
        odom.pose.pose.orientation.x = self.orientation[1]
        odom.pose.pose.orientation.y = self.orientation[2]
        odom.pose.pose.orientation.z = self.orientation[3]
        
        # Velocity
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]
        
        # Angular velocity
        odom.twist.twist.angular.x = self.gyro[0]
        odom.twist.twist.angular.y = self.gyro[1]
        odom.twist.twist.angular.z = self.gyro[2]
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = LIONode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 

