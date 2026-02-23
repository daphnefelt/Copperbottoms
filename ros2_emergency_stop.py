#!/usr/bin/env python3
"""
ROS2 Emergency Stop Node for Pixhawk Rover
Listens to /emergency_stop topic and immediately stops the rover

Usage:
  python3 ros2_emergency_stop.py

Trigger emergency stop from another terminal:
  ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from pymavlink import mavutil
import threading

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        
        # Connect to Pixhawk
        self.get_logger().info('Connecting to Pixhawk on /dev/ttyACM0...')
        try:
            self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
            self.master.wait_heartbeat()
            self.get_logger().info(f'✓ Connected to Pixhawk (System ID: {self.master.target_system})')
        except Exception as e:
            self.get_logger().error(f'✗ Failed to connect to Pixhawk: {e}')
            self.master = None
        
        # Subscribe to emergency stop topic
        self.subscription = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10)
        
        # Publisher for status updates
        self.status_pub = self.create_publisher(String, '/emergency_stop/status', 10)
        
        self.get_logger().info('🛑 Emergency Stop Node ready')
        self.get_logger().info('   Listening on /emergency_stop topic')
        self.get_logger().info('   Trigger: ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"')

    def emergency_stop_callback(self, msg):
        """Handle emergency stop trigger"""
        if msg.data and self.master:
            self.get_logger().warn('🛑 EMERGENCY STOP TRIGGERED!')
            self.execute_emergency_stop()
        
    def execute_emergency_stop(self):
        """Execute emergency stop sequence"""
        try:
            # Step 1: Switch to HOLD mode (locks wheels immediately)
            self.get_logger().info('Setting HOLD mode...')
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4,  # HOLD mode
                0, 0, 0, 0, 0)
            
            # Step 2: Force disarm
            self.get_logger().info('Disarming...')
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # Disarm
                21196,  # Force disarm (magic number)
                0, 0, 0, 0, 0)
            
            # Publish status
            status_msg = String()
            status_msg.data = 'STOPPED - Mode: HOLD, Motors: DISARMED'
            self.status_pub.publish(status_msg)
            
            self.get_logger().info('✓ Emergency stop complete')
            self.get_logger().info('  - Mode: HOLD')
            self.get_logger().info('  - Motors: DISARMED')
            
        except Exception as e:
            self.get_logger().error(f'✗ Emergency stop failed: {e}')
            status_msg = String()
            status_msg.data = f'ERROR: {e}'
            self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down emergency stop node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
