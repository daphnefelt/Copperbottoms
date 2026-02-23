#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time

class RoverHybridControl(Node):
    def __init__(self):
        super().__init__('rover_hybrid_control')
        
        # ROS2 subscription
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # MAVLink connection
        self.connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
        self.get_logger().info("Waiting for MAVLink heartbeat...")
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to flight controller")
        
        # Set mode to MANUAL
        self.set_manual_mode()
        
        # Arm the vehicle
        self.arm_vehicle()
        
        # Initialize last command values
        self.last_throttle = 0.0
        self.last_steering = 0.0
        
        # Create timer for continuous RC override commands (throttle only)
        self.timer = self.create_timer(0.05, self.send_continuous_commands)  # 20Hz
        
        self.get_logger().info("Hybrid rover control initialized - RC for throttle, manual_control for steering")

    def set_manual_mode(self):
        """Set the vehicle to MANUAL mode"""
        try:
            # Send mode change command
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,  # confirmation
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0,  # MANUAL mode for rover
                0, 0, 0, 0, 0
            )
            
            # Wait for mode confirmation
            time.sleep(0.5)
            self.get_logger().info("Set mode to MANUAL")
            
        except Exception as e:
            self.get_logger().error(f"Failed to set MANUAL mode: {e}")

    def arm_vehicle(self):
        """Arm the vehicle"""
        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                1,  # arm
                0, 0, 0, 0, 0, 0
            )
            
            time.sleep(1)
            self.get_logger().info("Vehicle armed")
            
        except Exception as e:
            self.get_logger().error(f"Failed to arm vehicle: {e}")

    def cmd_vel_callback(self, msg):
        """Handle incoming Twist messages from /cmd_vel topic"""
        # Store the commands for continuous sending
        self.last_throttle = msg.linear.x
        self.last_steering = msg.angular.z
        
        # Send steering immediately via manual_control (this works!)
        self.send_steering_manual_control(self.last_steering)
        
        self.get_logger().debug(f"Received: throttle={self.last_throttle:.2f}, steering={self.last_steering:.2f}")

    def send_steering_manual_control(self, steering_cmd):
        """Send steering command via manual_control (proven to work)"""
        try:
            # Convert steering command (-1.0 to 1.0) to PWM range (1000-2000)
            # Steering uses Y parameter in manual_control
            steering_scaled = int(1500 + (steering_cmd * 500))
            steering_scaled = max(1000, min(2000, steering_scaled))
            
            # Send manual_control with only steering (Y parameter)
            # X=neutral, Y=steering, Z=neutral, R=neutral
            self.connection.mav.manual_control_send(
                self.connection.target_system,
                1500,  # X (throttle) - neutral for manual_control
                steering_scaled,  # Y (steering) - this works!
                500,   # Z (not used)
                500,   # R (not used)
                0      # Buttons
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to send steering manual_control: {e}")

    def send_continuous_commands(self):
        """Send continuous RC override commands for throttle only"""
        try:
            # Convert throttle command to PWM
            throttle_scaled = int(1500 + (self.last_throttle * 500))
            throttle_scaled = max(1000, min(2000, throttle_scaled))
            
            # Send RC override for throttle channel only (typically channel 3)
            # Channel 1 = Roll/Steering (leave neutral - handled by manual_control)
            # Channel 3 = Throttle (use RC override)
            rc_channels = [
                65535,  # Channel 1 (steering) - ignore, using manual_control
                65535,  # Channel 2 - ignore  
                throttle_scaled,  # Channel 3 (throttle) - RC override
                65535,  # Channel 4 - ignore
                65535,  # Channel 5 - ignore
                65535,  # Channel 6 - ignore
                65535,  # Channel 7 - ignore
                65535   # Channel 8 - ignore
            ]
            
            self.connection.mav.rc_channels_override_send(
                self.connection.target_system,
                self.connection.target_component,
                *rc_channels
            )
            
            if abs(self.last_throttle) > 0.1 or abs(self.last_steering) > 0.1:
                self.get_logger().debug(f"Sent - RC throttle: {throttle_scaled}, manual_control steering: active")
                
        except Exception as e:
            self.get_logger().error(f"Failed to send RC override: {e}")

    def stop_rover(self):
        """Stop the rover by sending neutral commands"""
        try:
            # Send neutral manual_control for steering
            self.connection.mav.manual_control_send(
                self.connection.target_system,
                1500,  # X (neutral)
                1500,  # Y (neutral steering)
                500,   # Z
                500,   # R
                0      # Buttons
            )
            
            # Send neutral RC override for throttle
            rc_channels = [65535, 65535, 1500, 65535, 65535, 65535, 65535, 65535]
            self.connection.mav.rc_channels_override_send(
                self.connection.target_system,
                self.connection.target_component,
                *rc_channels
            )
            
            self.get_logger().info("Rover stopped - sent neutral commands")
            
        except Exception as e:
            self.get_logger().error(f"Failed to stop rover: {e}")

    def cleanup(self):
        """Cleanup when shutting down"""
        self.get_logger().info("Cleaning up...")
        self.stop_rover()
        time.sleep(0.5)
        
        try:
            # Clear RC overrides
            rc_channels = [65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535]
            self.connection.mav.rc_channels_override_send(
                self.connection.target_system,
                self.connection.target_component,
                *rc_channels
            )
            self.get_logger().info("Cleared RC overrides")
            
        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        rover = RoverHybridControl()
        rclpy.spin(rover)
    except KeyboardInterrupt:
        print("\nReceived Ctrl+C, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            rover.cleanup()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
