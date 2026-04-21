#!/usr/bin/env python3
"""
Real-Time Tape Contour Detection Node

to test 

source ~/code/Copperbottoms/ros_ws/install/setup.bash
ros2 launch amy_test tape_contour_rt.launch.py enable_motor_control:=true

Based on FIFO_Hough.c template - Tests color filtering + contour detection
Meets real-time deadlines for motor command input at 320x240
Publishes visualization for RViz 2
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os
import sys


class TapeContourRT(Node):
    def __init__(self):
        super().__init__('tape_contour_rt')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('debug_image_topic', '/tape_debug/image')
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('max_turn', 1.0)
        self.declare_parameter('deadline_ms', 70.0)  # 70ms deadline (matching FIFO_Hough)
        self.declare_parameter('target_width', 320)
        self.declare_parameter('target_height', 240)
        self.declare_parameter('min_contour_area', 100)
        self.declare_parameter('publish_timing', True)
        self.declare_parameter('enable_motor_control', False)  # Safety: disabled by default
        
        # PID tuning parameters for better straight-line following
        self.declare_parameter('kp', 0.6)  # Proportional gain (reduced from default)
        self.declare_parameter('ki', 0.03)  # Integral gain (low to prevent windup)
        self.declare_parameter('kd', 0.20)  # Derivative gain (increased for damping)
        
        # Weave reduction parameters
        self.declare_parameter('steering_deadband', 0.05)  # Don't steer for errors < 5%
        self.declare_parameter('adaptive_speed', True)  # Slow down for large errors
        self.declare_parameter('min_speed_ratio', 0.6)  # Minimum speed (60% of forward_speed)
        self.declare_parameter('error_threshold_slow', 0.25)  # Start slowing at 25% error
        self.declare_parameter('max_integral', 0.25)  # Anti-windup limit (reduced)
        
        # Get parameters
        image_topic = self.get_parameter('image_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.deadline_ms = self.get_parameter('deadline_ms').value
        self.target_width = self.get_parameter('target_width').value
        self.target_height = self.get_parameter('target_height').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.publish_timing = self.get_parameter('publish_timing').value
        self.enable_motor_control = self.get_parameter('enable_motor_control').value
        
        # PID tuning
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        # Weave reduction
        self.steering_deadband = self.get_parameter('steering_deadband').value
        self.adaptive_speed = self.get_parameter('adaptive_speed').value
        self.min_speed_ratio = self.get_parameter('min_speed_ratio').value
        self.error_threshold_slow = self.get_parameter('error_threshold_slow').value
        self.max_integral = self.get_parameter('max_integral').value
        
        # Tape color detection parameters (BGR format - note: user says blue but values look brown)
        self.tape_color = np.array([164, 108, 7])  # BGR
        self.color_tolerance = np.array([50, 50, 90])
        
        # State tracking
        self.frame_count = 0
        self.miss_count = 0
        self.total_process_time_ms = 0.0
        self.max_process_time_ms = 0.0
        self.min_process_time_ms = float('inf')
        
        # Lost tape handling state
        self.lost_tape_frames = 0
        self.max_coast_frames = 5      # Coast for ~170ms @ 30Hz
        self.max_search_frames = 20    # Search for ~700ms @ 30Hz
        self.last_contour_center = None
        self.last_turn = 0.0
        self.search_mode = False
        
        # Advanced recovery state (for sharp turns)
        self.recovery_phase = 'NONE'  # NONE, BACKING_UP, TURN_RIGHT, TURN_LEFT
        self.recovery_start_time = None
        self.backup_duration = 1.5  # Reduced from 3.0 seconds
        self.skip_backup_on_sharp_turn = True  # Skip backup for sharp turns
        self.turn_search_angle = 45.0  # Increased from 30 degrees
        self.turn_accumulated = 0.0
        self.sharp_turn_detected = False
        self.last_angle = None
        self.angle_change_threshold = 35.0  # Reduced from 45 - more sensitive
        
        # PID control state
        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0
        
        # Real-time priority setup (requires sudo/root)
        self.setup_realtime_priority()
        
        # CV Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.timing_pub = self.create_publisher(String, '/tape_debug/timing', 10)
        
        self.get_logger().info('='*50)
        self.get_logger().info('TapeContourRT Node Started')
        self.get_logger().info('='*50)
        self.get_logger().info(f'Image Topic: {image_topic}')
        self.get_logger().info(f'Debug Topic: {debug_image_topic}')
        self.get_logger().info(f'Cmd Vel Topic: {cmd_vel_topic}')
        self.get_logger().info(f'Target Resolution: {self.target_width}x{self.target_height}')
        self.get_logger().info(f'Deadline: {self.deadline_ms} ms')
        self.get_logger().info(f'Motor Control: {"ENABLED" if self.enable_motor_control else "DISABLED (testing only)"}')
        self.get_logger().info(f'PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, MaxI={self.max_integral}')
        self.get_logger().info(f'Weave Reduction: Deadband={self.steering_deadband}, AdaptiveSpeed={self.adaptive_speed}')
        self.get_logger().info(f'Speed Control: Min={self.min_speed_ratio:.0%}, SlowThreshold={self.error_threshold_slow}')
        self.get_logger().info(f'Tape Color (BGR): {self.tape_color}')
        self.get_logger().info(f'Color Tolerance: {self.color_tolerance}')
        self.get_logger().info('='*50)
    
    def setup_realtime_priority(self):
        """Attempt to set SCHED_FIFO real-time priority (like FIFO_Hough.c)"""
        try:
            # Try to set SCHED_FIFO with priority 90 (requires root/sudo)
            import os
            param = os.sched_param(90)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            self.get_logger().info('✓ Successfully set SCHED_FIFO scheduler (priority 90)')
        except PermissionError:
            self.get_logger().warn('⚠ Could not set SCHED_FIFO (need sudo/root) - using default scheduler')
        except Exception as e:
            self.get_logger().warn(f'⚠ Real-time scheduler setup failed: {e}')
    
    def apply_color_filter(self, img):
        """
        Apply color threshold to detect tape
        Returns: binary mask
        """
        # Create color mask
        lower_bound = self.tape_color - self.color_tolerance
        upper_bound = self.tape_color + self.color_tolerance
        mask = cv2.inRange(img, lower_bound, upper_bound)
        
        # Morphological operations to clean up noise
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        return mask
    
    def find_best_contour(self, mask):
        """
        Find contours and select the best one (largest area)
        Returns: (best_contour, contour_center, contour_angle) or (None, None, None)
        """
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None, None, None
        
        # Filter by minimum area and select largest
        valid_contours = [c for c in contours if cv2.contourArea(c) >= self.min_contour_area]
        
        if len(valid_contours) == 0:
            return None, None, None
        
        # Get largest contour
        best_contour = max(valid_contours, key=cv2.contourArea)
        
        # Calculate contour center (centroid)
        M = cv2.moments(best_contour)
        if M['m00'] == 0:
            return None, None, None
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        contour_center = (cx, cy)
        
        # Fit line to contour to get orientation angle
        # Use minAreaRect for better angle estimation
        rect = cv2.minAreaRect(best_contour)
        angle = rect[2]  # Angle in degrees
        
        # Alternatively, use fitLine for direction vector
        [vx, vy, x, y] = cv2.fitLine(best_contour, cv2.DIST_L2, 0, 0.01, 0.01)
        angle_from_line = np.degrees(np.arctan2(vy[0], vx[0]))
        
        return best_contour, contour_center, angle_from_line
    
    def compute_steering_command(self, contour_center, contour_angle, img_width, img_height):
        """
        Compute steering based on contour position using PID control
        Goal: Keep tape centered in frame
        Args:
            contour_center: (x, y) center of contour
            contour_angle: angle of contour in degrees
            img_width: image width
            img_height: image height
        Returns: (turn_rate, lateral_error, p_term, i_term, d_term)
        """
        if contour_center is None:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        
        cx, cy = contour_center
        
        # Calculate lateral error (shift from image center)
        # This is our primary control objective: keep tape centered
        center_x = img_width / 2
        lateral_error = (cx - center_x) / center_x  # Normalized to [-1, 1]
        
        # Use lateral error as our control error for PID
        error = lateral_error
        
        # Apply steering deadband to reduce weaving on straight lines
        if abs(error) < self.steering_deadband:
            error = 0.0  # Ignore small errors - drives straighter
        
        # Calculate time delta for I and D terms
        current_time = time.perf_counter()
        if self.last_time is None:
            dt = 0.035  # Assume ~28 Hz camera rate for first iteration
        else:
            dt = current_time - self.last_time
            dt = max(0.001, min(dt, 0.1))  # Clamp to reasonable range (1-100ms)
        
        self.last_time = current_time
        
        # P term: Proportional to current error
        p_term = self.kp * error
        
        # I term: Accumulated error over time (with anti-windup)
        self.error_integral += error * dt
        # Anti-windup: clamp integral to prevent excessive buildup
        self.error_integral = np.clip(self.error_integral, -self.max_integral, self.max_integral)
        i_term = self.ki * self.error_integral
        
        # D term: Rate of change of error (filtered to reduce noise)
        error_derivative = (error - self.last_error) / dt
        # Stronger low-pass filter on derivative to reduce oscillation/weaving
        alpha = 0.15  # Was 0.3 - much more smoothing to prevent over-reaction
        self.filtered_derivative = alpha * error_derivative + (1 - alpha) * self.filtered_derivative
        d_term = self.kd * self.filtered_derivative
        
        self.last_error = error
        
        # PID control law: turn to center the tape
        # Negative sign: positive error (tape right of center) -> turn right (negative)
        turn = -(p_term + i_term + d_term)
        turn = float(np.clip(turn, -self.max_turn, self.max_turn))
        
        return turn, lateral_error, p_term, i_term, d_term
    
    def reset_pid_state(self):
        """Reset PID controller state (call when tape is lost)"""
        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0
    
    def image_callback(self, msg: Image):
        """Main image processing callback - timed for real-time performance"""
        start_time = time.perf_counter()
        self.frame_count += 1
        
        # Convert ROS Image to OpenCV format
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Resize to target resolution (320x240 for optimal performance)
        if img.shape[1] != self.target_width or img.shape[0] != self.target_height:
            img = cv2.resize(img, (self.target_width, self.target_height))
        
        height, width = img.shape[:2]
        
        # STEP 1: Apply color filter to get binary mask
        mask = self.apply_color_filter(img)
        
        # STEP 2: Find contours and select best one
        best_contour, contour_center, contour_angle = self.find_best_contour(mask)
        
        # Detect sharp turns (sudden angle changes)
        if contour_angle is not None:
            if self.last_angle is not None:
                angle_change = abs(contour_angle - self.last_angle)
                if angle_change > self.angle_change_threshold:
                    self.sharp_turn_detected = True
                    self.get_logger().info(f'Sharp turn detected! Angle change: {angle_change:.1f}°')
            self.last_angle = contour_angle
        
        # Also detect sharp turns from current position: large error + tape at edge
        if contour_center is not None:
            cx, cy = contour_center
            lateral_err_magnitude = abs((cx - width/2) / (width/2))
            # If tape is near edge with high error, it's likely a sharp turn
            if lateral_err_magnitude > 0.4:
                self.sharp_turn_detected = True
                if self.frame_count % 30 == 0:  # Log occasionally
                    self.get_logger().info(f'Sharp turn detected from position! Error: {lateral_err_magnitude:.2f}')
        
        # STEP 3: Handle tape loss with temporal filtering and search
        turn = 0.0
        lateral_error = 0.0
        p_term = 0.0
        i_term = 0.0
        d_term = 0.0
        tape_detected = best_contour is not None
        
        if not tape_detected:
            self.lost_tape_frames += 1
            
            # Phase 1: Coast mode - maintain last command for a few frames
            if self.lost_tape_frames <= self.max_coast_frames:
                if self.last_contour_center is not None:
                    # Use last known position with reduced turn rate
                    turn = self.last_turn * 0.8
                    lateral_error = 0.0  # Unknown
                    
                    twist = Twist()
                    twist.linear.x = self.forward_speed * 0.7  # Slow down while coasting
                    twist.angular.z = turn
                    if self.enable_motor_control:
                        self.vel_pub.publish(twist)
                    
                    if self.lost_tape_frames == 1:
                        self.get_logger().warn(
                            f'Tape lost - coasting (frame {self.lost_tape_frames}/{self.max_coast_frames})'
                        )
                    
                    # Continue to visualization (skip rest of callback)
                    end_time = time.perf_counter()
                    process_time_ms = (end_time - start_time) * 1000.0
                    self.update_statistics(process_time_ms)
                    debug_img = self.create_debug_visualization(
                        img, mask, None, None, None,
                        turn, lateral_error, 0.0, 0.0, 0.0, process_time_ms, False, 'COASTING'
                    )
                    self.publish_debug_image(debug_img, msg.header)
                    return
            
            # Phase 2: Active search mode - turn in place to find tape
            elif self.lost_tape_frames <= self.max_coast_frames + self.max_search_frames:
                if not self.search_mode:
                    self.reset_pid_state()  # Reset PID when entering search
                    self.search_mode = True
                    self.get_logger().warn('Entering search mode - turning to find tape')
                
                # For sharp turns, use more aggressive search
                if self.sharp_turn_detected:
                    search_turn = 1.0  # Very aggressive turn for sharp corners (was 0.8)
                    # Strong right turn bias for sharp turns (track has 4 right turns)
                    if self.last_contour_center is not None:
                        last_x = self.last_contour_center[0]
                        if last_x > width // 2:  # Tape was on right
                            search_turn = 1.0  # Maximum right turn
                        else:
                            search_turn = -0.7  # Less aggressive left (was -0.6)
                else:
                    # Normal search - determine direction based on last known position
                    search_turn = 0.5
                    if self.last_contour_center is not None:
                        last_x = self.last_contour_center[0]
                        if last_x < width // 2:
                            search_turn = -0.5  # Turn left (tape was on left)
                        else:
                            search_turn = 0.5   # Turn right (tape was on right)
                
                twist = Twist()
                twist.linear.x = 0.0  # Stop forward motion during search
                twist.angular.z = search_turn
                if self.enable_motor_control:
                    self.vel_pub.publish(twist)
                
                # Continue to visualization
                end_time = time.perf_counter()
                process_time_ms = (end_time - start_time) * 1000.0
                self.update_statistics(process_time_ms)
                debug_img = self.create_debug_visualization(
                    img, mask, None, None, None,
                    search_turn, 0.0, 0.0, 0.0, 0.0, process_time_ms, False, 'SEARCHING'
                )
                self.publish_debug_image(debug_img, msg.header)
                return
            
            # Phase 3: Advanced recovery (backup, then systematic turn search)
            else:
                # Initialize recovery phase
                if self.recovery_phase == 'NONE':
                    # Skip backup if sharp turn detected - go straight to turning
                    if self.skip_backup_on_sharp_turn and self.sharp_turn_detected:
                        self.recovery_phase = 'TURN_RIGHT'
                        self.recovery_start_time = time.perf_counter()
                        self.turn_accumulated = 0.0
                        self.get_logger().warn('Sharp turn recovery: Skipping backup, going to TURN_RIGHT')
                    else:
                        self.recovery_phase = 'BACKING_UP'
                        self.recovery_start_time = time.perf_counter()
                        self.turn_accumulated = 0.0
                        self.get_logger().warn('Starting advanced recovery: BACKING UP')
                
                current_time = time.perf_counter()
                
                # Sub-phase 3a: Back up for 3 seconds
                if self.recovery_phase == 'BACKING_UP':
                    elapsed = current_time - self.recovery_start_time
                    
                    if elapsed < self.backup_duration:
                        twist = Twist()
                        twist.linear.x = -self.forward_speed * 0.7  # Reverse at coasting speed
                        twist.angular.z = 0.0
                        if self.enable_motor_control:
                            self.vel_pub.publish(twist)
                        
                        status_text = f'BACKING_UP ({elapsed:.1f}s/{self.backup_duration:.1f}s)'
                    else:
                        # Backup complete, move to right turn search
                        self.recovery_phase = 'TURN_RIGHT'
                        self.recovery_start_time = current_time
                        self.turn_accumulated = 0.0
                        self.get_logger().warn('Backup complete. Starting RIGHT turn search')
                        status_text = 'BACKING_UP'
                    
                    end_time = time.perf_counter()
                    process_time_ms = (end_time - start_time) * 1000.0
                    self.update_statistics(process_time_ms)
                    debug_img = self.create_debug_visualization(
                        img, mask, None, None, None,
                        0.0, 0.0, 0.0, 0.0, 0.0, process_time_ms, False, status_text
                    )
                    self.publish_debug_image(debug_img, msg.header)
                    return
                
                # Sub-phase 3b: Turn right 45 degrees and look for tape
                elif self.recovery_phase == 'TURN_RIGHT':
                    # More aggressive turn rate for recovery
                    turn_rate = 0.8 if self.sharp_turn_detected else 0.5  # Faster for sharp turns
                    dt = 0.033  # Assume ~30Hz
                    turn_degrees = np.degrees(turn_rate * dt)
                    
                    if self.turn_accumulated < self.turn_search_angle:
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = turn_rate  # Turn right (positive)
                        if self.enable_motor_control:
                            self.vel_pub.publish(twist)
                        
                        self.turn_accumulated += turn_degrees
                        status_text = f'TURN_RIGHT ({self.turn_accumulated:.1f}°/{self.turn_search_angle:.1f}°)'
                    else:
                        # Right turn complete, move to left turn search
                        self.recovery_phase = 'TURN_LEFT'
                        self.recovery_start_time = current_time
                        self.turn_accumulated = 0.0
                        self.get_logger().warn('Right turn complete. Starting LEFT turn search')
                        status_text = 'TURN_RIGHT'
                    
                    end_time = time.perf_counter()
                    process_time_ms = (end_time - start_time) * 1000.0
                    self.update_statistics(process_time_ms)
                    debug_img = self.create_debug_visualization(
                        img, mask, None, None, None,
                        turn_rate, 0.0, 0.0, 0.0, 0.0, process_time_ms, False, status_text
                    )
                    self.publish_debug_image(debug_img, msg.header)
                    return
                
                # Sub-phase 3c: Return to center, then turn left 45 degrees
                elif self.recovery_phase == 'TURN_LEFT':
                    turn_rate = -0.6 if self.sharp_turn_detected else -0.4  # Faster for sharp turns
                    dt = 0.033
                    turn_degrees = np.degrees(abs(turn_rate * dt))
                    
                    # First return to center (90 degrees total: 45 back + 45 left)
                    total_turn_needed = self.turn_search_angle * 2
                    
                    if self.turn_accumulated < total_turn_needed:
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = turn_rate  # Turn left (negative)
                        if self.enable_motor_control:
                            self.vel_pub.publish(twist)
                        
                        self.turn_accumulated += turn_degrees
                        status_text = f'TURN_LEFT ({self.turn_accumulated:.1f}°/{total_turn_needed:.1f}°)'
                    else:
                        # All recovery attempts exhausted - give up and stop
                        self.recovery_phase = 'STOPPED'
                        self.get_logger().error('All recovery attempts failed - STOPPING')
                        status_text = 'TURN_LEFT'
                    
                    end_time = time.perf_counter()
                    process_time_ms = (end_time - start_time) * 1000.0
                    self.update_statistics(process_time_ms)
                    debug_img = self.create_debug_visualization(
                        img, mask, None, None, None,
                        turn_rate, 0.0, 0.0, 0.0, 0.0, process_time_ms, False, status_text
                    )
                    self.publish_debug_image(debug_img, msg.header)
                    return
                
                # Sub-phase 3d: Final stop
                else:  # recovery_phase == 'STOPPED'
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    if self.enable_motor_control:
                        self.vel_pub.publish(twist)
                    
                    end_time = time.perf_counter()
                    process_time_ms = (end_time - start_time) * 1000.0
                    self.update_statistics(process_time_ms)
                    debug_img = self.create_debug_visualization(
                        img, mask, None, None, None,
                        0.0, 0.0, 0.0, 0.0, 0.0, process_time_ms, False, 'STOPPED'
                    )
                    self.publish_debug_image(debug_img, msg.header)
                    return
        
        # Tape found - reset lost counter and search mode
        if self.lost_tape_frames > self.max_coast_frames:
            # Was in search mode, reset PID
            self.reset_pid_state()
        if self.lost_tape_frames > 0:
            recovery_msg = f'Tape reacquired after {self.lost_tape_frames} frames'
            if self.recovery_phase != 'NONE':
                recovery_msg += f' (Recovery phase: {self.recovery_phase})'
            self.get_logger().info(recovery_msg)
        
        tape_detected = True
        self.lost_tape_frames = 0
        self.search_mode = False
        self.recovery_phase = 'NONE'
        self.sharp_turn_detected = False  # Reset sharp turn flag
        self.last_contour_center = contour_center
        
        # Compute steering command
        turn, lateral_error, p_term, i_term, d_term = self.compute_steering_command(
            contour_center, contour_angle, width, height
        )
        
        # Boost turn rate for sharp turns during normal tracking
        if self.sharp_turn_detected and abs(lateral_error) > 0.25:
            # Increase turn command by 50% for sharp turns
            turn_boost = 1.5
            turn = turn * turn_boost
            turn = float(np.clip(turn, -self.max_turn, self.max_turn))
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Sharp turn boost applied! Turn: {turn:.2f}')
        
        self.last_turn = turn  # Remember for coasting
        
        # Adaptive speed control: slow down for large errors to reduce weaving
        speed = self.forward_speed
        if self.adaptive_speed:
            error_magnitude = abs(lateral_error)
            if error_magnitude > self.error_threshold_slow:
                # Linear interpolation between min and max speed
                speed_ratio = 1.0 - ((error_magnitude - self.error_threshold_slow) / (1.0 - self.error_threshold_slow))
                speed_ratio = np.clip(speed_ratio, self.min_speed_ratio, 1.0)
                speed = self.forward_speed * speed_ratio
        
        # End timing (core processing complete)
        end_time = time.perf_counter()
        process_time_ms = (end_time - start_time) * 1000.0
        
        # Update statistics (includes deadline miss check)
        self.update_statistics(process_time_ms)
        deadline_miss = process_time_ms > self.deadline_ms  # For visualization
        
        # STEP 4: Publish motor command (if enabled)
        if self.enable_motor_control:
            twist = Twist()
            twist.linear.x = speed  # Use adaptive speed
            twist.angular.z = turn
            self.vel_pub.publish(twist)
        
        # STEP 5: Create debug visualization (after timing)
        debug_img = self.create_debug_visualization(
            img, mask, best_contour, contour_center, contour_angle, 
            turn, lateral_error, p_term, i_term, d_term, process_time_ms, deadline_miss, 'TRACKING'
        )
        
        # Publish debug image for RViz 2
        self.publish_debug_image(debug_img, msg.header)
        
        # Publish timing info
        if self.publish_timing and self.frame_count % 10 == 0:
            avg_time_ms = self.total_process_time_ms / self.frame_count
            miss_rate = (self.miss_count / self.frame_count) * 100.0
            
            timing_msg = String()
            timing_msg.data = (
                f'Frame {self.frame_count}: '
                f'proc={process_time_ms:.2f}ms, '
                f'avg={avg_time_ms:.2f}ms, '
                f'miss_rate={miss_rate:.1f}%, '
                f'tape={tape_detected}, '
                f'speed={speed:.2f}, '
                f'turn={turn:.2f}, '
                f'err={lateral_error:.2f}, '
                f'P={p_term:.2f}, I={i_term:.2f}, D={d_term:.2f}'
            )
            self.timing_pub.publish(timing_msg)
            
            self.get_logger().info(
                f'Frame {self.frame_count}: '
                f'{process_time_ms:.2f}ms '
                f'(avg={avg_time_ms:.2f}, min={self.min_process_time_ms:.2f}, max={self.max_process_time_ms:.2f}) | '
                f'Misses: {self.miss_count}/{self.frame_count} ({miss_rate:.1f}%) | '
                f'Tape: {tape_detected} | Speed: {speed:.2f} | Turn: {turn:.2f} | '
                f'Error: {lateral_error:.2f} [P={p_term:.2f} I={i_term:.2f} D={d_term:.2f}]'
            )
    
    def update_statistics(self, process_time_ms):
        """Update timing statistics"""
        deadline_miss = process_time_ms > self.deadline_ms
        if deadline_miss:
            self.miss_count += 1
        
        self.total_process_time_ms += process_time_ms
        self.max_process_time_ms = max(self.max_process_time_ms, process_time_ms)
        self.min_process_time_ms = min(self.min_process_time_ms, process_time_ms)
    
    def publish_debug_image(self, debug_img, header):
        """Publish debug visualization image"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')
    
    def create_debug_visualization(self, img, mask, contour, center, angle, 
                                   turn, lateral_err, p_term, i_term, d_term, 
                                   proc_time_ms, deadline_miss, status='TRACKING'):
        """
        Create debug visualization showing:
        - Original image with contour overlay
        - Binary mask
        - Direction vector
        - Steering info
        - Status (TRACKING, COASTING, SEARCHING, STOPPED)
        """
        height, width = img.shape[:2]
        
        # Create side-by-side visualization
        debug_img = np.zeros((height, width * 2, 3), dtype=np.uint8)
        
        # Left side: Original with overlays
        img_overlay = img.copy()
        
        if contour is not None:
            # Draw contour
            cv2.drawContours(img_overlay, [contour], -1, (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(img_overlay, center, 8, (0, 0, 255), -1)
            
            # Draw direction vector
            cx, cy = center
            vector_length = 60
            angle_rad = np.radians(angle)
            end_x = int(cx + vector_length * np.cos(angle_rad))
            end_y = int(cy + vector_length * np.sin(angle_rad))
            cv2.arrowedLine(img_overlay, center, (end_x, end_y), (255, 0, 255), 3, tipLength=0.3)
        
        # Draw center line
        cv2.line(img_overlay, (width // 2, 0), (width // 2, height), (255, 255, 0), 1)
        
        # Add text overlay with status
        status_colors = {
            'TRACKING': (0, 255, 0),
            'COASTING': (0, 255, 255),
            'SEARCHING': (0, 165, 255),
            'STOPPED': (0, 0, 255)
        }
        # Handle recovery status text (may include progress)
        base_status = status.split('(')[0].strip() if '(' in status else status
        status_color = status_colors.get(base_status, (255, 128, 0))  # Orange for recovery phases
        
        # Smaller font if status text is long
        font_scale = 0.5 if len(status) > 20 else 0.7
        cv2.putText(img_overlay, status, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, status_color, 2)
        
        cv2.putText(img_overlay, f'Turn: {turn:.2f}', (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(img_overlay, f'Error: {lateral_err:.2f}', (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if contour is not None:
            cv2.putText(img_overlay, f'P:{p_term:.2f} I:{i_term:.2f} D:{d_term:.2f}', (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Timing info
        timing_color = (0, 0, 255) if deadline_miss else (0, 255, 0)
        cv2.putText(img_overlay, f'Time: {proc_time_ms:.1f}ms', (10, 180),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, timing_color, 2)
        
        cv2.putText(img_overlay, f'Deadline: {self.deadline_ms:.0f}ms', (10, 210),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Right side: Binary mask
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # Combine
        debug_img[:, :width] = img_overlay
        debug_img[:, width:] = mask_color
        
        # Add separator line
        cv2.line(debug_img, (width, 0), (width, height), (255, 255, 255), 2)
        
        return debug_img
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.frame_count > 0:
            avg_time = self.total_process_time_ms / self.frame_count
            miss_rate = (self.miss_count / self.frame_count) * 100.0
            
            self.get_logger().info('='*50)
            self.get_logger().info('Final Statistics:')
            self.get_logger().info(f'  Total Frames: {self.frame_count}')
            self.get_logger().info(f'  Avg Time: {avg_time:.2f} ms')
            self.get_logger().info(f'  Min Time: {self.min_process_time_ms:.2f} ms')
            self.get_logger().info(f'  Max Time: {self.max_process_time_ms:.2f} ms')
            self.get_logger().info(f'  Deadline Misses: {self.miss_count}/{self.frame_count} ({miss_rate:.1f}%)')
            
            if miss_rate < 1.0:
                self.get_logger().info('  ✓ Excellent real-time performance')
            elif miss_rate < 5.0:
                self.get_logger().info('  ✓ Good real-time performance')
            elif miss_rate < 10.0:
                self.get_logger().info('  ⚠ Marginal real-time performance')
            else:
                self.get_logger().info('  ✗ Poor real-time performance')
            self.get_logger().info('='*50)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TapeContourRT()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
