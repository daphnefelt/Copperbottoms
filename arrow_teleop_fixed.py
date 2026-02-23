#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class ArrowTeleop(Node):
    def __init__(self):
        super().__init__("arrow_teleop")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("Arrow Teleop Node Started - Use arrow keys to control rover")
        self.get_logger().info("UP/DOWN: forward/backward, LEFT/RIGHT: turn left/right")
        self.get_logger().info("SPACE: stop, q: quit")
        
        # Check if stdin is a terminal
        if not sys.stdin.isatty():
            self.get_logger().error("This script requires a proper terminal!")
            raise RuntimeError("Not running in a terminal")
        
        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def cleanup(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except:
            pass

    def get_key(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            
            # Handle escape sequences for arrow keys
            if key == "\x1b":
                # Read the next characters to form the full arrow key sequence
                if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                    key += sys.stdin.read(1)
                    if key == "\x1b[":
                        if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                            key += sys.stdin.read(1)
            return key
        return ""

    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher_.publish(msg)
        
    def run(self):
        print("Ready! Use arrow keys:")
        print("UP/DOWN: forward/backward")  
        print("LEFT/RIGHT: turn left/right")
        print("SPACE: stop")
        print("q: quit")
        print("Controls active...")
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == "q":  # q to quit
                    print("\nQuitting...")
                    break
                elif key == "\x1b[A":  # UP arrow
                    print("Forward")
                    self.publish_velocity(linear_x=0.5)
                elif key == "\x1b[B":  # DOWN arrow
                    print("Backward") 
                    self.publish_velocity(linear_x=-0.5)
                elif key == "\x1b[C":  # RIGHT arrow
                    print("Turn right")
                    self.publish_velocity(angular_z=-0.5)
                elif key == "\x1b[D":  # LEFT arrow
                    print("Turn left")
                    self.publish_velocity(angular_z=0.5)
                elif key == " ":  # SPACE
                    print("Stop")
                    self.publish_velocity()
                    
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            print("\nReceived Ctrl+C, stopping...")
        finally:
            self.cleanup()
            self.publish_velocity()  # Stop rover

def main():
    rclpy.init()
    
    try:
        teleop = ArrowTeleop()
        teleop.run()
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        try:
            rclpy.shutdown()
        except:
            pass
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
