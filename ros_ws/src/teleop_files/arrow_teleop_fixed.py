#!/usr/bin/env python3
"""
ROS2 arrow key teleoperator for rover
Supports simultaneous key presses for turn-while-moving
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
        
        # Servo ranges from test_servo_manual.py:
        # Steering (ch1): 1100-1750, center 1425
        # Throttle (ch3): 1100-1900, neutral 1500
        self.steering_center = 1425
        self.steering_left = 1100
        self.steering_right = 1750
        
        self.throttle_neutral = 1500
        self.throttle_forward = 1750
        self.throttle_reverse = 1250
        
        # Track which keys are currently pressed
        self.keys_held = {
            'forward': False,
            'reverse': False,
            'left': False,
            'right': False,
        }
        
        self.settings = termios.tcgetattr(sys.stdin)

        # Create log file
        start_stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_filename = f"teleop_log_{start_stamp}.txt"
        self.log_file = open(self.log_filename, "w")

        self.write_log("===== PROGRAM STARTED =====")
    
    def write_log(self, message):
        """Write timestamped message to file"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.log_file.write(f"[{timestamp}] {message}\n")
        self.log_file.flush()

    def get_key_nonblocking(self):
        """Get next key without blocking if none available"""
        select.select([sys.stdin], [], [], 0)
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == '\x1b':  # Escape sequence for arrow keys
                key += sys.stdin.read(2)
            return key
        return None

    def calculate_servo_values(self):
        """Calculate servo values based on held keys"""
        steering = self.steering_center
        throttle = self.throttle_neutral
        
        # Steering: left/right (independent of throttle)
        if self.keys_held['left']:
            steering = self.steering_left
        elif self.keys_held['right']:
            steering = self.steering_right
        
        # Throttle: forward or reverse (independent of steering)
        if self.keys_held['forward']:
            throttle = self.throttle_forward
        elif self.keys_held['reverse']:
            throttle = self.throttle_reverse
        
        return steering, throttle

    def map_to_twist(self, steering, throttle):
        """Convert servo PWM values to Twist message"""
        twist = Twist()
        
        # Forward/backward from throttle (neutral=1500)
        # rover_node uses linear.x for throttle/forward-backward
        # Keep sign positive so UP maps to forward and DOWN maps to reverse.
        throttle_normalized = (throttle - self.throttle_neutral) / 250.0
        # twist.linear.x = throttle_normalized * 0.251
        twist.linear.x = throttle_normalized * 0.35
        
        # Turning from steering
        # rover_node uses angular.z for steering/turning
        # Negate because steering is inverted
        steering_offset = (steering - self.steering_center) / 325.0
        twist.angular.z = -steering_offset * 0.50
        
        return twist

    def run(self):
        """Main control loop"""
        print("Arrow keys to drive (hold keys for continuous movement):")
        print("  UP    = Forward")
        print("  DOWN  = Reverse")
        print("  LEFT  = Turn left")
        print("  RIGHT = Turn right")
        print("  Q     = Quit\n")
        print(" spacebar = Stop (center steering and throttle)\n")
        print("Hold UP + LEFT to go forward and turn left!\n")
        
        # Set terminal to non-blocking input
        tty.setraw(sys.stdin.fileno())
        
        try:
            while True:
                # Check for new key presses (non-blocking)
                key = self.get_key_nonblocking()
                
                if key:
                    if key == '\x1b[A':    # UP - forward
                        self.keys_held['forward'] = True
                        self.keys_held['reverse'] = False
                        self.write_log("KEY PRESS: UP (Forward)")
                    elif key == '\x1b[B':  # DOWN - reverse
                        self.keys_held['reverse'] = True
                        self.keys_held['forward'] = False
                        self.write_log("KEY PRESS: DOWN (Reverse)")
                    elif key == '\x1b[D':  # LEFT
                        self.keys_held['left'] = True
                        self.keys_held['right'] = False
                        self.write_log("KEY PRESS: LEFT")
                    elif key == '\x1b[C':  # RIGHT
                        self.keys_held['right'] = True
                        self.keys_held['left'] = False
                        self.write_log("KEY PRESS: RIGHT")
                    elif key == ' ':  # SPACE - center
                        self.keys_held['forward'] = False
                        self.keys_held['reverse'] = False
                        self.keys_held['left'] = False
                        self.keys_held['right'] = False
                        self.write_log("KEY PRESS: SPACE (STOP)")
                    elif key == 'q' or key == 'Q':
                        self.write_log("KEY PRESS: Q (QUIT)")
                        print("\nQuitting...")
                        break
                
                # Calculate and publish based on currently held keys
                steering, throttle = self.calculate_servo_values()
                twist = self.map_to_twist(steering, throttle)
                # print(f"Publishing: linear.x={twist.linear.x:.2f} m/s  angular.z={twist.angular.z:.2f} rad/s\n")
                self.pub.publish(twist)

                log_msg = (
                    f"PUBLISH -> linear.x={twist.linear.x:.2f}, "
                    f"angular.z={twist.angular.z:.2f}"
                )
                self.write_log(log_msg)
                
                # Small delay to prevent CPU spinning
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("\nInterrupted!")
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # Send stop command
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
