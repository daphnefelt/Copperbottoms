#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios, select

class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.1
        self.angular_speed = 1.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        print("Arrow keys to drive, q to quit")
        while True:
            key = self.get_key()
            twist = Twist()
            if key == '\x1b[A':    # up
                twist.linear.x = -self.linear_speed * 2
            elif key == '\x1b[B':  # down
                twist.linear.x = self.linear_speed
            elif key == '\x1b[C':  # right
                twist.angular.z = -self.angular_speed
                twist.linear.x = -self.linear_speed * 2
            elif key == '\x1b[D':  # left
                twist.angular.z = self.angular_speed
                twist.linear.x = -self.linear_speed * 2
            elif key == 'q':
                break
            self.pub.publish(twist)

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ArrowTeleop()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
