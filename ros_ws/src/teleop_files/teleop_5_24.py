#!/usr/bin/env python3
"""
ROS2 arrow key teleoperator for rover
Bot only moves while keys are actively pressed.
If keys are released, it smoothly slows to a stop.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import time
from datetime import datetime


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Motion state
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Speeds
        self.max_linear = 0.6
        self.max_angular = 1.2

        # Smoothing
        self.accel_step = 0.08
        self.turn_step = 0.15
        self.decel_step = 0.05

        # Track currently pressed keys
        self.keys_held = {
            'forward': False,
            'reverse': False,
            'left': False,
            'right': False,
        }

        # If no key event received recently, assume released
        self.last_key_time = time.time()
        self.key_timeout = 0.12

        self.settings = termios.tcgetattr(sys.stdin)

        # Logging
        start_stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_filename = f"teleop_log_{start_stamp}.txt"
        self.log_file = open(self.log_filename, "w")
        self.write_log("===== PROGRAM STARTED =====")

    def write_log(self, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.log_file.write(f"[{timestamp}] {message}\n")
        self.log_file.flush()

    def get_key_nonblocking(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            key = sys.stdin.read(1)
            if key == '\x1b':
                key += sys.stdin.read(2)
            return key
        return None

    def smooth_toward(self, current, target, step):
        if current < target:
            return min(current + step, target)
        elif current > target:
            return max(current - step, target)
        return current

    def update_motion(self):
        now = time.time()

        # If no key seen recently, assume released
        if now - self.last_key_time > self.key_timeout:
            for k in self.keys_held:
                self.keys_held[k] = False

        # Target linear velocity
        target_linear = 0.0
        if self.keys_held['forward']:
            target_linear = self.max_linear
        elif self.keys_held['reverse']:
            target_linear = -self.max_linear

        # Target angular velocity
        target_angular = 0.0
        if self.keys_held['left']:
            target_angular = self.max_angular
        elif self.keys_held['right']:
            target_angular = -self.max_angular

        # Smooth acceleration/deceleration
        lin_step = self.accel_step if target_linear != 0 else self.decel_step
        ang_step = self.turn_step if target_angular != 0 else self.decel_step

        self.current_linear = self.smooth_toward(
            self.current_linear, target_linear, lin_step
        )

        self.current_angular = self.smooth_toward(
            self.current_angular, target_angular, ang_step
        )

    def run(self):
        print("Arrow keys to drive:")
        print("UP = forward")
        print("DOWN = reverse")
        print("LEFT = turn left")
        print("RIGHT = turn right")
        print("Release keys = slow to stop")
        print("Q = quit\n")

        tty.setraw(sys.stdin.fileno())

        try:
            while True:
                key = self.get_key_nonblocking()

                if key:
                    self.last_key_time = time.time()

                    # Reset movement keys each press
                    self.keys_held['forward'] = False
                    self.keys_held['reverse'] = False
                    self.keys_held['left'] = False
                    self.keys_held['right'] = False

                    if key == '\x1b[A':
                        self.keys_held['forward'] = True
                    elif key == '\x1b[B':
                        self.keys_held['reverse'] = True
                    elif key == '\x1b[D':
                        self.keys_held['left'] = True
                    elif key == '\x1b[C':
                        self.keys_held['right'] = True
                    elif key in ['q', 'Q']:
                        print("\nQuitting...")
                        break

                self.update_motion()

                twist = Twist()
                twist.linear.x = self.current_linear
                twist.angular.z = self.current_angular

                self.pub.publish(twist)

                self.write_log(
                    f"PUBLISH -> linear.x={twist.linear.x:.2f}, "
                    f"angular.z={twist.angular.z:.2f}"
                )

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\nInterrupted!")

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

            stop = Twist()
            self.pub.publish(stop)

            self.write_log("STOP command published")
            self.write_log("===== PROGRAM ENDED =====")
            self.log_file.close()

            print("Stopped")


def main():
    rclpy.init()
    node = ArrowTeleop()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()