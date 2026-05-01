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
import csv
from datetime import datetime
import os

class TapeControlNode(Node):
    def __init__(self):
        super().__init__('tape_control_node')
        self.declare_parameter('forward_speed', 0.25)  # been hanging this between .22 and .25 
        self.declare_parameter('max_turn', 5)  # Increased for faster turns
        self.declare_parameter('kp', 0.75)  # Increased for faster response (was 0.65)
        self.declare_parameter('ki', 0.005)  # Reduced to prevent late weaving
        self.declare_parameter('kd', 0.15)  # Reduced to prevent D-term fighting (was 0.25)
        self.declare_parameter('steering_deadband', 0.05)  # Larger deadband reduces weaving
        self.declare_parameter('enable_motor_control', True)
        self.declare_parameter('debug', True)  # Enable debug output
        self.declare_parameter('camera_center_offset', 72.0)  # Camera horizontal offset (pixels)
        self.declare_parameter('camera_angle_offset', -24.0)  # Camera angle offset (degrees)
        self.declare_parameter('angle_control_weight', 0.2)  # Weight for angle-based correction (was 0.05, increased for better turn response)
        self.declare_parameter('sharp_turn_boost', 1.8)  # Multiply turn by this during sharp turns
        self.declare_parameter('sharp_turn_speed_factor', 0.6)  # Reduce speed to this fraction during sharp turns
        self.declare_parameter('sharp_turn_stop_frames', 5)  # Frames to stop before turning (reduced)
        self.declare_parameter('sharp_turn_execute_speed', 0.2)  # Forward speed during turn execution
        self.declare_parameter('sharp_turn_execute_rate', 1.5)  # Angular rate during turn execution (increased)
        self.declare_parameter('adaptive_speed', True)  # Slow down for large errors
        self.declare_parameter('min_speed_ratio', 0.8)  # Minimum speed (80% of forward_speed)
        self.declare_parameter('error_threshold_slow', 0.25)  # Start slowing at 25% error
        self.declare_parameter('max_turn_rate', 10.0)  # Max steering change per update (was 6.0, increased for rapid response)
        self.declare_parameter('max_integral', 0.15)  # Tighter anti-windup to prevent late weaving

        self.forward_speed = self.get_parameter('forward_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.steering_deadband = self.get_parameter('steering_deadband').value
        self.enable_motor_control = self.get_parameter('enable_motor_control').value
        self.debug = self.get_parameter('debug').value
        self.camera_center_offset = self.get_parameter('camera_center_offset').value
        self.camera_angle_offset = self.get_parameter('camera_angle_offset').value
        self.angle_control_weight = self.get_parameter('angle_control_weight').value
        self.sharp_turn_boost = self.get_parameter('sharp_turn_boost').value
        self.sharp_turn_speed_factor = self.get_parameter('sharp_turn_speed_factor').value
        self.sharp_turn_stop_frames = self.get_parameter('sharp_turn_stop_frames').value
        self.sharp_turn_execute_speed = self.get_parameter('sharp_turn_execute_speed').value
        self.sharp_turn_execute_rate = self.get_parameter('sharp_turn_execute_rate').value
        self.adaptive_speed = self.get_parameter('adaptive_speed').value
        self.min_speed_ratio = self.get_parameter('min_speed_ratio').value
        self.error_threshold_slow = self.get_parameter('error_threshold_slow').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.max_integral = self.get_parameter('max_integral').value

        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0
        self.last_turn = 0.0  # For steering rate limiting

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
        
        # CSV logging setup
        self.csv_file = None
        self.csv_writer = None
        if self.debug:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_filename = os.path.expanduser(f'~/performance_logs/tape_control_debug_{timestamp}.csv')
            os.makedirs(os.path.dirname(csv_filename), exist_ok=True)
            self.csv_file = open(csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header
            self.csv_writer.writerow([
                'timestamp', 'frame', 'speed', 'turn', 'error', 'lateral_error', 'angle_error',
                'p_term', 'i_term', 'd_term', 'center_x', 'img_center', 'angle', 'angle_corrected',
                'sharp_turn_state', 'tape_found'
            ])
            self.get_logger().info(f'CSV logging to: {csv_filename}')
        
        self.get_logger().info(f'TapeControlNode started (debug={self.debug})')

    def tape_callback(self, msg):
        self.total_frame_count += 1
        
        if not msg.found and self.sharp_turn_state == 'NORMAL':
            # Stop only if not executing sharp turn maneuver
            return
        
        # Sharp turn state machine
        if self.sharp_turn_state == 'NORMAL':
            # Normal following mode
            if msg.sharp_turn:
                # Sharp turn detected, enter STOPPING state
                self.sharp_turn_state = 'STOPPING'
                self.sharp_turn_frame_count = 0
                # Determine turn direction from tape angle
                # -90° = straight up, 0° = horizontal right (RIGHT turn), ±180° = horizontal left (LEFT turn)
                if -20 <= msg.angle <= 20:
                    self.sharp_turn_direction = 1  # RIGHT turn (tape horizontal right)
                elif msg.angle >= 160 or msg.angle <= -160:
                    self.sharp_turn_direction = -1  # LEFT turn (tape horizontal left)
                else:
                    # Fallback for intermediate angles (shouldn't happen with 85° threshold)
                    self.sharp_turn_direction = 1 if abs(msg.angle) < 90 else -1
                self.get_logger().warn(
                    f'[SHARP TURN START] Frame {self.total_frame_count} - '
                    f'Angle: {msg.angle:.1f}°, Direction: {"RIGHT" if self.sharp_turn_direction > 0 else "LEFT"}'
                )
                # Log to CSV
                if self.csv_writer:
                    self.csv_writer.writerow([
                        time.time(), self.total_frame_count, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, msg.center_x, 0.0, msg.angle, 0.0,
                        'SHARP_TURN_DETECTED', msg.found
                    ])
                    self.csv_file.flush()
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
            
            # Log to CSV
            if self.csv_writer:
                self.csv_writer.writerow([
                    time.time(), self.total_frame_count, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, msg.center_x, 0.0, msg.angle, 0.0,
                    'STOPPING', msg.found
                ])
                self.csv_file.flush()
            
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
            # ROS convention: negative angular.z = clockwise (RIGHT), positive = counter-clockwise (LEFT)
            twist = Twist()
            twist.linear.x = self.sharp_turn_execute_speed
            twist.angular.z = -self.sharp_turn_direction * self.sharp_turn_execute_rate
            self.vel_pub.publish(twist)
            
            # Check exit condition: tape angle more reasonable and reasonably centered
            angle_from_vertical = abs(msg.angle - (-90.0))  # How far from vertical (-90° = straight ahead)
            center_x = msg.center_x
            img_width = 320
            center_error = abs(center_x - img_width / 2)
            
            # Log to CSV
            if self.csv_writer:
                self.csv_writer.writerow([
                    time.time(), self.total_frame_count, self.sharp_turn_execute_speed, 
                    -self.sharp_turn_direction * self.sharp_turn_execute_rate, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, center_x, 0.0, msg.angle, 0.0,
                    'TURNING', msg.found
                ])
                self.csv_file.flush()
            
            # Exit when tape is detected and reasonably visible, let normal PID handle alignment
            # Require minimum turn duration to avoid exiting on fake straight paths at junction
            min_turn_frames = 60  # Must turn at least 22 frames (~0.8s) to clear fork junction
            angle_from_vertical = abs(msg.angle - (-90.0))
            is_angle_reasonable = angle_from_vertical < 45  # Within 45° of vertical (tighter for fake path rejection)
            is_timeout = self.sharp_turn_frame_count > 60  # Max 35 frames (~1.2s at 28 fps)
            
            if msg.found and self.sharp_turn_frame_count >= min_turn_frames and (is_angle_reasonable or is_timeout):
                self.sharp_turn_state = 'NORMAL'
                reason = 'aligned' if is_angle_reasonable else 'timeout'
                self.get_logger().warn(
                    f'[SHARP TURN COMPLETE] {reason} - Frames: {self.sharp_turn_frame_count}, '
                    f'Angle: {msg.angle:.1f}° (from vertical: {angle_from_vertical:.1f}°), '
                    f'Center: {msg.center_x:.1f}'
                )
                # Resume normal following on next frame
            else:
                self.get_logger().info(
                    f'[TURNING] Frame {self.sharp_turn_frame_count}, '
                    f'Angle: {msg.angle:.1f}° (from vert: {angle_from_vertical:.1f}°), Center: {msg.center_x:.1f}',
                    throttle_duration_sec=0.5
                )
            return
    
    def _execute_normal_following(self, msg):
        """Execute normal line following control"""
        # Lateral error: normalized [-1, 1] with camera offset correction
        center_x = msg.center_x
        img_width = 320  # Should match vision node
        img_center = img_width / 2 + self.camera_center_offset  # Corrected center
        lateral_error = (center_x - img_center) / (img_width / 2)
        
        # Angle error: normalized deviation from straight ahead (-90° expected)
        angle_corrected = msg.angle - self.camera_angle_offset  # Should be near -90° when straight
        angle_error = (angle_corrected - (-90.0)) / 90.0  # Normalize: 0 = straight, ±1 = 90° off
        
        # Apply deadband to LATERAL error only (prevents drift accumulation)
        # Angle correction is still applied but doesn't trigger deadband
        if abs(lateral_error) < self.steering_deadband:
            lateral_error_filtered = 0.0
        else:
            lateral_error_filtered = lateral_error
        
        # Combined error with weighted angle correction
        error = (1.0 - self.angle_control_weight) * lateral_error_filtered + self.angle_control_weight * angle_error
        
        current_time = time.perf_counter()
        if self.last_time is None:
            dt = 0.035
        else:
            dt = current_time - self.last_time
            dt = max(0.001, min(dt, 0.1))
        self.last_time = current_time
        p_term = self.kp * error
        
        # Only accumulate integral when outside deadband (prevents windup during deadband)
        if abs(lateral_error) >= self.steering_deadband:
            self.error_integral += error * dt
            # Anti-windup: clamp integral to prevent excessive buildup
            self.error_integral = np.clip(self.error_integral, -self.max_integral, self.max_integral)
        else:
            # Decay I-term when inside deadband to prevent long-term accumulation
            self.error_integral *= 0.95  # 5% decay per frame when centered
        i_term = self.ki * self.error_integral
        error_derivative = (error - self.last_error) / dt
        alpha = 0.08
        self.filtered_derivative = alpha * error_derivative + (1 - alpha) * self.filtered_derivative
        d_term = self.kd * self.filtered_derivative
        self.last_error = error
        turn = -(p_term + i_term + d_term)
        
        # Steering rate limiter to prevent mechanical oscillation
        turn_delta = turn - self.last_turn
        turn_delta_limited = np.clip(turn_delta, -self.max_turn_rate, self.max_turn_rate)
        turn = self.last_turn + turn_delta_limited
        turn = float(np.clip(turn, -self.max_turn, self.max_turn))
        self.last_turn = turn
        
        # Adaptive speed control: slow down for large errors to reduce weaving
        speed = self.forward_speed
        if self.adaptive_speed:
            error_magnitude = abs(lateral_error)
            if error_magnitude > self.error_threshold_slow:
                # Ensure minimum speed is always at least 0.2 m/s absolute
                min_absolute_speed = 0.2
                effective_min_ratio = max(self.min_speed_ratio, min_absolute_speed / self.forward_speed)
                
                # Linear interpolation between min and max speed
                speed_ratio = 1.0 - ((error_magnitude - self.error_threshold_slow) / (1.0 - self.error_threshold_slow))
                speed_ratio = np.clip(speed_ratio, effective_min_ratio, 1.0)
                speed = self.forward_speed * speed_ratio
        
        # Final safety check: ensure speed is always at least 0.2 m/s
        speed = max(0.2, speed)
            
        # LIDAR right turn bias
        if self.turn_right:
            turn += 0.3
            turn = float(np.clip(turn, -self.max_turn, self.max_turn))
        
        # Debug output
        if self.debug:
            self.get_logger().info(
                f'[DEBUG] Speed: {speed:.3f} | Turn: {turn:.3f} | '
                f'Error: {error:.3f} (Lat: {lateral_error:.3f}, Ang: {angle_error:.3f}) | '
                f'P: {p_term:.3f} | I: {i_term:.3f} | D: {d_term:.3f} | '
                f'Center: {center_x:.1f}/{img_center:.1f} | Angle: {msg.angle:.1f}° ({angle_corrected:.1f}°)',
                throttle_duration_sec=0.5
            )
            # Write to CSV
            if self.csv_writer:
                self.csv_writer.writerow([
                    time.time(), self.total_frame_count, speed, turn, error, lateral_error, angle_error,
                    p_term, i_term, d_term, center_x, img_center, msg.angle, angle_corrected,
                    self.sharp_turn_state, msg.found
                ])
                self.csv_file.flush()  # Ensure data is written immediately
        
        # Always publish control commands (motor control enable/disable handled at rover level)
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = turn
        self.vel_pub.publish(twist)

    def turn_right_cb(self, msg):
        self.turn_right = msg.data
    
    def __del__(self):
        """Cleanup: close CSV file on shutdown"""
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info('CSV file closed')

def main(args=None):
    rclpy.init(args=args)
    node = TapeControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
