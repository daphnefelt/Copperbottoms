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
        self.declare_parameter('forward_speed', 0.22)  # been hanging this between .22 and .25 
        self.declare_parameter('max_turn', 1.8)
        self.declare_parameter('kp', 0.6)
        self.declare_parameter('ki', 0.03)
        self.declare_parameter('kd', 0.20)
        self.declare_parameter('steering_deadband', 0.12)
        self.declare_parameter('enable_motor_control', True)
        self.declare_parameter('sharp_turn_boost', 1.8)  # Multiply turn by this during sharp turns
        self.declare_parameter('sharp_turn_speed_factor', 0.6)  # Reduce speed to this fraction during sharp turns
        self.declare_parameter('sharp_turn_stop_frames', 10)  # Frames to stop before turning
        self.declare_parameter('sharp_turn_execute_speed', 0.15)  # Forward speed during turn execution
        self.declare_parameter('sharp_turn_execute_rate', 0.8)  # Angular rate during turn execution

        self.forward_speed = self.get_parameter('forward_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.steering_deadband = self.get_parameter('steering_deadband').value
        self.enable_motor_control = self.get_parameter('enable_motor_control').value
        self.sharp_turn_boost = self.get_parameter('sharp_turn_boost').value
        self.sharp_turn_speed_factor = self.get_parameter('sharp_turn_speed_factor').value
        self.sharp_turn_stop_frames = self.get_parameter('sharp_turn_stop_frames').value
        self.sharp_turn_execute_speed = self.get_parameter('sharp_turn_execute_speed').value
        self.sharp_turn_execute_rate = self.get_parameter('sharp_turn_execute_rate').value

        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0

        self.tape_sub = self.create_subscription(TapeContour, '/tape_contour', self.tape_callback, 10)
        self.turn_right_sub = self.create_subscription(Bool, '/turn_right', self.turn_right_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.turn_right = False
        
        # Sharp turn state machine
        self.sharp_turn_state = 'NORMAL'  # States: NORMAL, STOPPING, TURNING
        self.sharp_turn_detected = False
        self.sharp_turn_frame_count = 0
        self.sharp_turn_direction = 0  # -1 for left, +1 for right
        self.total_frame_count = 0
        
        self.get_logger().info('TapeControlNode started')

    def tape_callback(self, msg):
        self.total_frame_count += 1
        
        if not msg.found:
            # Optionally: stop or coast
            return
        
        # Sharp turn state machine
        if self.sharp_turn_state == 'NORMAL':
            # Normal following mode
            if msg.sharp_turn:
                # Sharp turn detected, enter STOPPING state
                self.sharp_turn_state = 'STOPPING'
                self.sharp_turn_frame_count = 0
                # Determine turn direction from angle
                # Negative angle = left turn, Positive angle = right turn
                self.sharp_turn_direction = 1 if msg.angle > 0 else -1
                self.get_logger().warn(
                    f'[SHARP TURN START] Frame {self.total_frame_count} - '
                    f'Angle: {msg.angle:.1f}°, Direction: {"RIGHT" if self.sharp_turn_direction > 0 else "LEFT"}'
                )
            else:
                # Normal line following
                self._execute_normal_following(msg)
                return
        
        if self.sharp_turn_state == 'STOPPING':
            self.sharp_turn_frame_count += 1
            # Stop for a few frames to settle
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
            
            if self.sharp_turn_frame_count >= self.sharp_turn_stop_frames:
                # Done stopping, start turning
                self.sharp_turn_state = 'TURNING'
                self.sharp_turn_frame_count = 0
                self.get_logger().warn('[SHARP TURN EXECUTE] Starting turn maneuver')
            else:
                self.get_logger().info(
                    f'[STOPPING] Frame {self.sharp_turn_frame_count}/{self.sharp_turn_stop_frames}',
                    throttle_duration_sec=0.5
                )
            return
        
        if self.sharp_turn_state == 'TURNING':
            self.sharp_turn_frame_count += 1
            
            # Execute turn: slow forward + aggressive turn in detected direction
            twist = Twist()
            twist.linear.x = self.sharp_turn_execute_speed
            twist.angular.z = self.sharp_turn_direction * self.sharp_turn_execute_rate
            self.vel_pub.publish(twist)
            
            # Check exit condition: tape angle more reasonable and reasonably centered
            angle_from_vertical = abs(msg.angle - (-90.0))  # How far from vertical (-90° = straight ahead)
            center_x = msg.center_x
            img_width = 320
            center_error = abs(center_x - img_width / 2)
            
            # Exit when tape is < 30° from vertical AND within 80 pixels of center
            if angle_from_vertical < 30 and center_error < 80:
                self.sharp_turn_state = 'NORMAL'
                self.get_logger().warn(
                    f'[SHARP TURN COMPLETE] Frames: {self.sharp_turn_frame_count}, '
                    f'Final angle: {msg.angle:.1f}°, Center: {msg.center_x:.1f}'
                )
                # Resume normal following on next frame
            else:
                self.get_logger().info(
                    f'[TURNING] Frame {self.sharp_turn_frame_count}, '
                    f'Angle: {msg.angle:.1f}°, Center: {msg.center_x:.1f}',
                    throttle_duration_sec=0.5
                )
            return
    
    def _execute_normal_following(self, msg):
        """Execute normal line following control"""
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
        
        # Normal operation (no sharp turn detected)
        speed = self.forward_speed
            
        # LIDAR right turn bias
        if self.turn_right:
            turn += 0.3
            turn = float(np.clip(turn, -self.max_turn, self.max_turn))
        
        # Always publish control commands (motor control enable/disable handled at rover level)
        twist = Twist()
        twist.linear.x = speed
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
