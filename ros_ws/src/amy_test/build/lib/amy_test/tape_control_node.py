#!/usr/bin/env python3
"""
Tape Control Node: Subscribes to /tape_contour, runs PID, publishes /cmd_vel
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from amy_test.msg import TapeContour
import time
import numpy as np

class TapeControlNode(Node):
    def __init__(self):
        super().__init__('tape_control_node')
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('max_turn', 1.0)
        self.declare_parameter('kp', 0.6)
        self.declare_parameter('ki', 0.03)
        self.declare_parameter('kd', 0.20)
        self.declare_parameter('steering_deadband', 0.10)
        self.declare_parameter('enable_motor_control', False)

        self.forward_speed = self.get_parameter('forward_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.steering_deadband = self.get_parameter('steering_deadband').value
        self.enable_motor_control = self.get_parameter('enable_motor_control').value

        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0

        self.tape_sub = self.create_subscription(TapeContour, '/tape_contour', self.tape_callback, 10)
        self.turn_right_sub = self.create_subscription(Bool, '/turn_right', self.turn_right_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.turn_right = False
        self.get_logger().info('TapeControlNode started')

    def tape_callback(self, msg):
        if not msg.found:
            # Optionally: stop or coast
            return
        # Lateral error: normalized [-1, 1] (center_x relative to image center)
        center_x = msg.center_x
        img_width = 320  # Should match vision node
        error = (center_x - img_width / 2) / (img_width / 2)
        if abs(error) < self.steering_deadband:
            error = 0.0
        current_time = time.perf_counter()
        if self.last_time is None:
            dt = 0.035
        else:
            dt = current_time - self.last_time
            dt = max(0.001, min(dt, 0.1))
        self.last_time = current_time
        p_term = self.kp * error
        self.error_integral += error * dt
        i_term = self.ki * self.error_integral
        error_derivative = (error - self.last_error) / dt
        alpha = 0.08
        self.filtered_derivative = alpha * error_derivative + (1 - alpha) * self.filtered_derivative
        d_term = self.kd * self.filtered_derivative
        self.last_error = error
        turn = -(p_term + i_term + d_term)
        turn = float(np.clip(turn, -self.max_turn, self.max_turn))
        # LIDAR right turn bias
        if self.turn_right:
            turn += 0.3
            turn = float(np.clip(turn, -self.max_turn, self.max_turn))
        if self.enable_motor_control:
            twist = Twist()
            twist.linear.x = self.forward_speed
            twist.angular.z = turn
            self.vel_pub.publish(twist)

    def turn_right_cb(self, msg):
        self.turn_right = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = TapeControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
