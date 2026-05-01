#!/usr/bin/env python3

# Test 1: straight-line — linear.x=0.5, angular.z=0.0 for 1 second
# m/s is dist in m

# Test 2: arc turn — linear.x=0.5, angular.z=0.5 for 5 second
# turn radius = linear_speed / angular_speed

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import argparse
import sys

PUBLISH_HZ = 20 # how often to re-publish during test
TEST = 2 # which test to run (1 or 2)

class CalibrationNode(Node):
    def __init__(self, test: int):
        super().__init__('calibrate_speed')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.test = test

    def send_command(self, linear_x: float, angular_z: float, duration: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        stop = Twist()  # all zeros

        rate = self.create_rate(PUBLISH_HZ)
        deadline = time.time() + duration

        self.get_logger().info(
            f'Sending linear.x={linear_x}, angular.z={angular_z} '
            f'for {duration}s'
        )

        while time.time() < deadline:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=1.0 / PUBLISH_HZ)

        # explicit stop
        self.pub.publish(stop)
        rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        # Let the publisher connect before sending anything
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)

        if self.test == 1:
            self.get_logger().info(
                'TEST 1: Straight line\n'
                'Mark robot start position, then press Enter.'
            )
            input()
            self.send_command(linear_x=0.3, angular_z=0.0, duration=1.0)
            self.get_logger().info('Done. Measure')

        if self.test == 2:
            self.get_logger().info(
                f'TEST 2: Arc turn\n'
            )
            input()
            self.send_command(linear_x=0.0, angular_z=0.75, duration=10.0)
            self.get_logger().info('Done. Measure')

        self.get_logger().info('Calibration complete.')

def main():
    rclpy.init()
    node = CalibrationNode(test=TEST)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()