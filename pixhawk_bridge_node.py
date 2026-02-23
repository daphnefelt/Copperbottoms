#!/usr/bin/env python3
"""
ROS2 node to bridge Pixhawk data via pymavlink
Publishes IMU, attitude, GPS, and other sensor data to ROS2 topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32
from pymavlink import mavutil
import math
import threading

class PixhawkBridgeNode(Node):
    def __init__(self):
        super().__init__('pixhawk_bridge')
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/pixhawk/imu', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pixhawk/pose', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/pixhawk/gps', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/pixhawk/velocity', 10)
        self.altitude_pub = self.create_publisher(Float32, '/pixhawk/altitude', 10)
        
        # Connect to Pixhawk
        self.get_logger().info('Connecting to Pixhawk on /dev/ttyACM0...')
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
        
        # Wait for heartbeat
        self.master.wait_heartbeat()
        self.get_logger().info(f'Connected! System ID: {self.master.target_system}')
        
        # Request data streams
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10, 1)
        
        # Start MAVLink reading thread
        self.running = True
        self.mavlink_thread = threading.Thread(target=self.read_mavlink)
        self.mavlink_thread.start()
        
        self.get_logger().info('Pixhawk bridge node started')
    
    def read_mavlink(self):
        """Read MAVLink messages in a separate thread"""
        while self.running and rclpy.ok():
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                self.process_message(msg)
    
    def process_message(self, msg):
        """Process MAVLink messages and publish to ROS topics"""
        msg_type = msg.get_type()
        
        if msg_type == 'RAW_IMU':
            self.publish_imu(msg)
        elif msg_type == 'ATTITUDE':
            self.publish_attitude(msg)
        elif msg_type == 'GPS_RAW_INT':
            self.publish_gps(msg)
        elif msg_type == 'GLOBAL_POSITION_INT':
            self.publish_velocity(msg)
        elif msg_type == 'VFR_HUD':
            self.publish_altitude(msg)
    
    def publish_imu(self, msg):
        """Publish IMU data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'pixhawk_imu'
        
        # Convert from millig to m/s^2
        imu_msg.linear_acceleration.x = msg.xacc * 9.81 / 1000.0
        imu_msg.linear_acceleration.y = msg.yacc * 9.81 / 1000.0
        imu_msg.linear_acceleration.z = msg.zacc * 9.81 / 1000.0
        
        # Convert from millirad/s to rad/s
        imu_msg.angular_velocity.x = msg.xgyro / 1000.0
        imu_msg.angular_velocity.y = msg.ygyro / 1000.0
        imu_msg.angular_velocity.z = msg.zgyro / 1000.0
        
        self.imu_pub.publish(imu_msg)
    
    def publish_attitude(self, msg):
        """Publish attitude/pose data"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'pixhawk_base'
        
        # Convert Euler angles to quaternion
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        self.pose_pub.publish(pose_msg)
    
    def publish_gps(self, msg):
        """Publish GPS data"""
        if msg.fix_type < 2:  # No fix
            return
            
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'
        
        gps_msg.latitude = msg.lat / 1e7
        gps_msg.longitude = msg.lon / 1e7
        gps_msg.altitude = msg.alt / 1000.0
        
        # Set status
        gps_msg.status.status = 0 if msg.fix_type >= 3 else -1
        gps_msg.status.service = 1  # GPS
        
        self.gps_pub.publish(gps_msg)
    
    def publish_velocity(self, msg):
        """Publish velocity data"""
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'pixhawk_base'
        
        # Convert from cm/s to m/s
        vel_msg.twist.linear.x = msg.vx / 100.0
        vel_msg.twist.linear.y = msg.vy / 100.0
        vel_msg.twist.linear.z = msg.vz / 100.0
        
        self.velocity_pub.publish(vel_msg)
    
    def publish_altitude(self, msg):
        """Publish altitude data"""
        alt_msg = Float32()
        alt_msg.data = msg.alt
        self.altitude_pub.publish(alt_msg)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if self.mavlink_thread.is_alive():
            self.mavlink_thread.join(timeout=2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
