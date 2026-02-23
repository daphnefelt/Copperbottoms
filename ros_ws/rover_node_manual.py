"""
Rover Node using MANUAL_CONTROL messages (the working approach)
This bypasses RCMAP and sends direct vehicle control commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from pymavlink import mavutil
import threading
import time

class RoverNodeManual(Node):
    def __init__(self):
        super().__init__('rover_node_manual')
        
        # Connect to flight controller
        self.get_logger().info("Connecting to flight controller...")
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to flight controller!")
        
        # ROS2 setup
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.imu_accel_pub = self.create_publisher(Imu, '/imu/accel', 10)
        self.imu_gyro_pub = self.create_publisher(Imu, '/imu/gyro', 10)  
        self.armed_pub = self.create_publisher(Bool, '/rover/armed', 10)
        
        # State
        self.armed = False
        self.last_cmd_time = time.time()
        self.timeout = 1.0  # 1 second timeout
        
        # Start telemetry thread
        self.telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        
        # Start control timeout checking
        self.timer = self.create_timer(0.1, self.check_timeout)
        
        # Initialize flight controller
        self.initialize_fc()
        
        self.get_logger().info("Rover Node Manual started - using MANUAL_CONTROL messages!")
    
    def initialize_fc(self):
        """Initialize flight controller to MANUAL mode and arm"""
        # Set MANUAL mode
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
        self.armed = True
    
    def send_manual_control(self, throttle, steering):
        """Send manual control message (bypasses RCMAP completely)"""
        # Convert -1.0 to +1.0 range to -1000 to +1000
        throttle_scaled = int(throttle * 1000)
        steering_scaled = int(steering * 1000)
        
        # Clamp to valid range
        throttle_scaled = max(-1000, min(1000, throttle_scaled))
        steering_scaled = max(-1000, min(1000, steering_scaled))
        
        self.master.mav.manual_control_send(
            self.master.target_system,
            steering_scaled,  # x = steering/roll
            throttle_scaled,  # y = pitch/throttle for rovers 
            0,                # z = unused
            0,                # r = yaw (unused for skid-steer)
            0)                # buttons
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS2"""
        if not self.armed:
            return
            
        self.last_cmd_time = time.time()
        
        # Extract linear and angular velocity
        linear_x = msg.linear.x    # Forward/backward (-1 to +1)
        angular_z = msg.angular.z  # Turn left/right (-1 to +1)
        print("linear x: " + str(linear_x))
        
        # Send manual control commands directly
        # For rovers: x=steering, y=throttle
        self.send_manual_control(linear_x, angular_z)
        
        # Log commands
        self.get_logger().debug(
            f"Manual Control: throttle={linear_x:.2f}, steering={angular_z:.2f}")
    
    def check_timeout(self):
        """Check for command timeout and stop rover if needed"""
        if not self.armed:
            return
            
        if time.time() - self.last_cmd_time > self.timeout:
            # Stop rover - send neutral manual control
            self.send_manual_control(0.0, 0.0)
    
    def telemetry_loop(self):
        """Background thread to handle telemetry"""
        while True:
            try:
                # Process multiple messages per loop for better responsiveness
                for _ in range(5):
                    msg = self.master.recv_match(blocking=False, timeout=0.01)
                    if msg:
                        self.handle_telemetry(msg)
                    else:
                        break
                time.sleep(0.02)  # 50 Hz loop
            except Exception as e:
                self.get_logger().error(f"Telemetry error: {e}")
                time.sleep(0.1)
    
    def handle_telemetry(self, msg):
        """Process incoming telemetry messages"""
        if msg.get_type() == 'RAW_IMU':
            # Publish accelerometer data
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Convert from mG to m/s²
            imu_msg.linear_acceleration.x = msg.xacc / 1000.0 * 9.81
            imu_msg.linear_acceleration.y = msg.yacc / 1000.0 * 9.81  
            imu_msg.linear_acceleration.z = msg.zacc / 1000.0 * 9.81
            
            # Convert from mrad/s to rad/s
            imu_msg.angular_velocity.x = msg.xgyro / 1000.0
            imu_msg.angular_velocity.y = msg.ygyro / 1000.0
            imu_msg.angular_velocity.z = msg.zgyro / 1000.0
            
            self.imu_accel_pub.publish(imu_msg)
            
        elif msg.get_type() == 'HEARTBEAT':
            # Update armed status
            was_armed = self.armed
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            
            if was_armed != self.armed:
                self.get_logger().info(f"Armed status changed: {self.armed}")
            
            # Publish armed status
            armed_msg = Bool()
            armed_msg.data = self.armed
            self.armed_pub.publish(armed_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        rover_node = RoverNodeManual()
        rclpy.spin(rover_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop on shutdown
        try:
            rover_node.get_logger().info("Shutting down - sending stop commands...")
            rover_node.send_manual_control(0.0, 0.0)
            time.sleep(0.5)
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
