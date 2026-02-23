"""
Fixed Arrow Key Teleoperator for Rover Control
Publishes to /cmd_vel when arrow keys are pressed
Fixed the double select() bug that was causing issues
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.cbreak(sys.stdin.fileno())
        
        print("Arrow keys to drive (hold keys for continuous movement):")
        print("  UP    = Forward")
        print("  DOWN  = Reverse") 
        print("  LEFT  = Turn left")
        print("  RIGHT = Turn right")
        print("  Q     = Quit")
        print()
        print("Hold UP + LEFT to go forward and turn left!")
        print()
        
        self.run()
    
    def get_key(self):
        """Non-blocking key input - Fixed version"""
        if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            return key
        return None
    
    def run(self):
        """Main teleop loop"""
        try:
            while True:
                key = self.get_key()
                
                if key:
                    if key.lower() == 'q':
                        break
                    
                    # Handle escape sequences (arrow keys)
                    if key == '\x1b':  # ESC
                        # Read the next two characters for arrow keys
                        seq1 = self.get_key()
                        seq2 = self.get_key() 
                        
                        if seq1 == '[':
                            if seq2 == 'A':    # UP
                                self.publish_twist(0.2, 0.0)  # Forward
                            elif seq2 == 'B':  # DOWN
                                self.publish_twist(-0.2, 0.0) # Reverse
                            elif seq2 == 'D':  # LEFT
                                self.publish_twist(0.0, 0.5)  # Turn left
                            elif seq2 == 'C':  # RIGHT
                                self.publish_twist(0.0, -0.5) # Turn right
                else:
                    # No key pressed - send stop command
                    self.publish_twist(0.0, 0.0)
                
                # Small delay to prevent CPU spinning
                rclpy.spin_once(self, timeout_sec=0.01)
                        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            # Send final stop command
            self.publish_twist(0.0, 0.0)
    
    def publish_twist(self, linear, angular):
        """Publish Twist message"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        
        # Show what we're publishing
        if linear != 0.0 or angular != 0.0:
            direction = ""
            if linear > 0: direction += "Forward "
            elif linear < 0: direction += "Reverse "
            if angular > 0: direction += "Left"
            elif angular < 0: direction += "Right"
            print(f"Publishing: {direction} (linear={linear:.1f}, angular={angular:.1f})")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop = ArrowTeleop()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
