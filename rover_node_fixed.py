import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time

class RoverNodeManual(Node):
    def __init__(self):
        super().__init__("rover_node_manual")

        # Connect to flight controller
        self.get_logger().info("Connecting to flight controller...")
        self.master = mavutil.mavlink_connection("/dev/ttyACM0", baud=921600)
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to flight controller!")

        # Set MANUAL mode and ARM
        self.get_logger().info("Setting MANUAL mode...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0,  # MANUAL mode
            0, 0, 0, 0, 0)

        time.sleep(1)

        # Arm
        self.get_logger().info("Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 21196, 0, 0, 0, 0, 0)

        time.sleep(1)

        # ROS2 setup
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.get_logger().info("Rover Node Fixed - swapped X/Y parameters!")

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS2"""
        linear_x = msg.linear.x    # Forward/backward (-1 to +1)
        angular_z = msg.angular.z  # Turn left/right (-1 to +1)
        print("linear x: " +str(linear_x))

        # Convert to manual control scaling (1000-2000 range for ESC)
        throttle_scaled = int(1500 + (linear_x * 500))  # 1000-2000 full range
        steering_scaled = int(angular_z * 1000)

        self.get_logger().info(f"CMD: linear={linear_x:.2f}, angular={angular_z:.2f}")
        self.get_logger().info(f"Sending: throttle={throttle_scaled}, steering={steering_scaled}")

        # FIXED: Swap X and Y parameters for this rover setup
        self.master.mav.manual_control_send(
            self.master.target_system,
            throttle_scaled,  # x = throttle (was steering)
            steering_scaled,  # y = steering (was throttle)
            0,                # z = unused
            0,                # r = yaw (unused for skid-steer)
            0)                # buttons

def main(args=None):
    rclpy.init(args=args)
    rover_node = RoverNodeManual()
    rclpy.spin(rover_node)

if __name__ == "__main__":
    main()
