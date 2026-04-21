#!/usr/bin/env python3
"""
Tape Recovery Node: Subscribes to /tape_contour and /cmd_vel, manages advanced recovery, overrides /cmd_vel if needed
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from amy_test.msg import TapeContour
import time

class TapeRecoveryNode(Node):
    def __init__(self):
        super().__init__('tape_recovery_node')
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('max_turn', 1.0)
        self.declare_parameter('enable_motor_control', False)
        self.forward_speed = self.get_parameter('forward_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.enable_motor_control = self.get_parameter('enable_motor_control').value
        self.lost_tape_frames = 0
        self.max_coast_frames = 5
        self.max_search_frames = 20
        self.recovery_active = False
        self.recovery_phase = 'NONE'
        self.recovery_start_time = None
        self.tape_sub = self.create_subscription(TapeContour, '/tape_contour', self.tape_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_cmd = None
        self.get_logger().info('TapeRecoveryNode started')

    def tape_callback(self, msg):
        if msg.found:
            self.lost_tape_frames = 0
            self.recovery_active = False
            self.recovery_phase = 'NONE'
            return
        self.lost_tape_frames += 1
        if self.lost_tape_frames > self.max_coast_frames:
            self.recovery_active = True
            self.recovery_phase = 'SEARCH'
            self.recovery_start_time = time.perf_counter()
            self.get_logger().warn('Tape lost: entering recovery/search mode')
            self.publish_recovery_cmd()

    def cmd_callback(self, msg):
        self.last_cmd = msg
        # If not in recovery, let normal cmd_vel pass
        if not self.recovery_active and self.enable_motor_control:
            self.vel_pub.publish(msg)

    def publish_recovery_cmd(self):
        if self.enable_motor_control:
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = self.max_turn  # Turn right aggressively
            self.vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TapeRecoveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
