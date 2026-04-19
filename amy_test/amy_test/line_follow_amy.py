#!/usr/bin/env python3
"""
Line Following Node - Amy Test Version
Clean implementation for RTES latency testing and monitoring
"""
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import time


class LineFollowerAmy(Node):
    def __init__(self):
        super().__init__('line_follower_amy')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.1)  # Integral gain
        self.declare_parameter('kd', 0.05)  # Derivative gain
        self.declare_parameter('max_turn', 1.0)
        self.declare_parameter('min_pixels', 50)
        self.declare_parameter('debug_plot', False)
        self.declare_parameter('debug_interval', 1)  # Update debug view every N frames
        self.declare_parameter('publish_timing', True)

        # Get parameters
        image_topic = self.get_parameter('image_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_turn = self.get_parameter('max_turn').value
        self.min_pixels = self.get_parameter('min_pixels').value
        self.debug_plot = self.get_parameter('debug_plot').value
        self.debug_interval = self.get_parameter('debug_interval').value
        self.publish_timing = self.get_parameter('publish_timing').value

        # State
        self.frame_count = 0
        self.see_line = False
        self.error_offset = 0.0
        self.lost_line_frames = 0  # Track consecutive frames with no line
        self.max_lost_frames = 5   # Allow 5 frames (~170ms) before active search
        self.max_search_frames = 20  # Search for 20 more frames (~700ms) before stopping
        self.last_tape_x = None    # Remember last known position
        self.last_turn = 0.0       # Remember last turn command
        self.search_mode = False   # Active search mode flag
        
        # PID state
        self.error_integral = 0.0      # Accumulated error for I term
        self.last_error = 0.0          # Previous error for D term
        self.last_time = None          # Previous callback time for dt calculation
        self.max_integral = 0.5        # Anti-windup limit for integral term
        
        # Color threshold for brown tape (RGB)
        self.tape_color = np.array([164, 108, 7])
        self.color_tolerance = np.array([50, 50, 90])

        # Perspective warp parameters
        self.top_shift = 40
        self.bottom_shift = -225

        # Timing instrumentation
        self.last_process_time_ms = 0.0
        self.avg_process_time_ms = 0.0
        self.process_time_alpha = 0.1  # EMA smoothing
        self.last_pixel_count = 0.0  # For diagnostics

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            f'LineFollowerAmy started: {image_topic} -> {cmd_vel_topic}'
        )
        self.get_logger().info(
            f'Params: speed={self.forward_speed}, kp={self.kp}, ki={self.ki}, kd={self.kd}, debug={self.debug_plot}'
        )

    def apply_perspective_warp(self, img):
        """Apply perspective warp to correct for camera angle"""
        h, w = img.shape[:2]
        
        src = np.float32([
            [0, 0],                         # top-left
            [w - self.top_shift, 0],        # top-right
            [0, h],                         # bottom-left
            [w - self.bottom_shift, h]      # bottom-right
        ])
        
        dst = np.float32([
            [0, 0],
            [w, 0],
            [0, h],
            [w, h]
        ])
        
        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(img, M, (w, h))

    def detect_line_hough(self, img, relaxed=False):
        """
        Detect line using color mask + Hough line detection
        Args:
            img: Input image
            relaxed: If True, use more permissive detection for search mode
        Returns: (tape_x, tape_y, lines) or (None, None, None) if not found
        """
        height, width = img.shape[:2]
        
        # Color mask for brown tape (wider tolerance in search mode)
        tolerance = self.color_tolerance * 1.3 if relaxed else self.color_tolerance
        mask = cv2.inRange(
            img,
            self.tape_color - tolerance,
            self.tape_color + tolerance
        )
        
        # Check if enough pixels found (lower threshold in search mode)
        min_pix = self.min_pixels * 0.5 if relaxed else self.min_pixels
        pixel_count = np.sum(mask) / 255
        
        if pixel_count < min_pix:
            if self.frame_count % 10 == 0:
                self.get_logger().debug(
                    f"Insufficient pixels: {pixel_count:.0f} < {min_pix} (relaxed={relaxed})"
                )
            return None, None, None
        
        # Morphological operations to clean up mask
        # Remove noise and connect nearby tape regions
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Edge detection
        edges = cv2.Canny(mask, 50, 150, apertureSize=3)
        
        # Store pixel count for diagnostics
        self.last_pixel_count = pixel_count
        
        # Hough line detection (more permissive in search mode)
        threshold = 30 if relaxed else 50
        min_line_len = 30 if relaxed else 50
        max_gap = 20 if relaxed else 10
        
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=threshold,
            minLineLength=min_line_len,
            maxLineGap=max_gap
        )
        
        if lines is None:
            if self.frame_count % 10 == 0:
                self.get_logger().debug(
                    f"Hough found no lines (pixels={pixel_count:.0f}, relaxed={relaxed})"
                )
            return None, None, None
        
        # IMPROVED: Filter lines by orientation and position
        filtered_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate line angle
            dx = x2 - x1
            dy = y2 - y1
            length = np.sqrt(dx**2 + dy**2)
            
            if length < 10:  # Skip very short lines
                continue
            
            # Angle in degrees (-90 to 90, where 0 is horizontal)
            angle = np.abs(np.degrees(np.arctan2(dy, dx)))
            
            # Accept lines that are mostly vertical or horizontal (tape orientation)
            # Tape should be relatively straight, not diagonal
            is_horizontal = angle < 30 or angle > 150  # Within 30° of horizontal
            is_vertical = 60 < angle < 120  # Within 30° of vertical
            
            if is_horizontal or is_vertical:
                # Check if line is in bottom 70% of image (where tape should be)
                avg_y = (y1 + y2) / 2
                if avg_y > height * 0.3:  # Skip lines in top 30%
                    filtered_lines.append(line)
        
        if len(filtered_lines) == 0:
            if self.frame_count % 10 == 0:
                self.get_logger().debug(
                    f"All {len(lines)} Hough lines filtered out (wrong orientation/position)"
                )
            return None, None, None
        
        # Score function: prefer points that are low (high y for bottom of image)
        # and biased toward last known position if available
        def score_point(x, y):
            # Base score: prefer bottom of image
            score = y
            
            # Bias toward last known position in search mode
            if self.last_tape_x is not None and relaxed:
                distance_from_last = abs(x - self.last_tape_x)
                score -= distance_from_last * 0.3  # Penalize far from last known
            
            return score
        
        # Find best point across all FILTERED line segments
        best_x = width // 2
        best_y = height // 2
        best_score = -float('inf')
        
        for line in filtered_lines:
            x1, y1, x2, y2 = line[0]
            
            # Score both endpoints
            score1 = score_point(x1, y1)
            if score1 > best_score:
                best_score = score1
                best_x = x1
                best_y = y1
            
            score2 = score_point(x2, y2)
            if score2 > best_score:
                best_score = score2
                best_x = x2
                best_y = y2
        
        return best_x, best_y, np.array(filtered_lines)

    def detect_line_blob(self, img):
        """
        Fallback: Detect line using color mask centroid in bottom ROI
        Returns: tape_x or None
        """
        height, width = img.shape[:2]
        roi_start = int(height * 0.70)  # bottom 30%
        roi = img[roi_start:height, :, :]
        
        # Use same brown tape color mask
        mask = cv2.inRange(
            roi,
            self.tape_color - self.color_tolerance,
            self.tape_color + self.color_tolerance
        )
        
        # Check if enough pixels found
        pixel_count = np.sum(mask) / 255
        if pixel_count < self.min_pixels:
            return None
        
        # Find horizontal center of mass
        rows, cols = np.where(mask > 0)
        if len(cols) == 0:
            return None
        
        tape_x = int(np.mean(cols))
        return tape_x

    def compute_control(self, tape_x, width):
        """
        Compute steering command from tape position using PID control
        Args:
            tape_x: horizontal position of tape
            width: image width
        Returns: (turn rate, error, p_term, i_term, d_term)
        """
        center_x = width / 2
        error = (tape_x - center_x) / center_x  # normalize to [-1, 1]
        error += self.error_offset
        
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
        # Low-pass filter on derivative to reduce noise amplification
        alpha = 0.3  # Filter coefficient (0=no filtering, 1=no smoothing)
        if not hasattr(self, 'filtered_derivative'):
            self.filtered_derivative = 0.0
        self.filtered_derivative = alpha * error_derivative + (1 - alpha) * self.filtered_derivative
        d_term = self.kd * self.filtered_derivative
        
        self.last_error = error
        
        # PID control law
        turn = -(p_term + i_term + d_term)
        turn = float(np.clip(turn, -self.max_turn, self.max_turn))
        
        return turn, error, p_term, i_term, d_term
    
    def reset_pid_state(self):
        """Reset PID controller state (call when line is lost or reacquired)"""
        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        if hasattr(self, 'filtered_derivative'):
            self.filtered_derivative = 0.0

    def publish_stop_command(self):
        """Publish zero velocity to stop the rover"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)

    def image_callback(self, msg: Image):
        """Main image processing callback"""
        start_time = time.perf_counter()
        self.frame_count += 1
        
        # Validate encoding
        if msg.encoding.lower() != "bgr8":
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
            return
        
        # Convert ROS Image to numpy array
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)
        )
        
        # Apply perspective warp
        img = self.apply_perspective_warp(img)
        height, width = img.shape[:2]
        
        # Detect line using Hough method (relaxed mode if searching)
        tape_x, tape_y, lines = self.detect_line_hough(img, relaxed=self.search_mode)
        
        # If Hough fails, try blob method as fallback
        if tape_x is None:
            blob_x = self.detect_line_blob(img)
            if blob_x is not None:
                tape_x = blob_x
                tape_y = height // 2  # default y position
                lines = None
        
        # Handle line loss with temporal filtering and active search
        if tape_x is None:
            self.lost_line_frames += 1
            
            # Phase 1: Coast for a few frames (maintain last command)
            if self.lost_line_frames <= self.max_lost_frames:
                if self.last_tape_x is not None:
                    tape_x = self.last_tape_x
                    turn = self.last_turn * 0.8  # Reduce turn rate while coasting
                    
                    twist = Twist()
                    twist.linear.x = self.forward_speed * 0.7  # Slow down while coasting
                    twist.angular.z = turn
                    self.vel_pub.publish(twist)
                    
                    if self.lost_line_frames == 1:
                        self.get_logger().warn(f"Line lost - coasting (frame {self.lost_line_frames}/{self.max_lost_frames})")
                    return
            
            # Phase 2: Active search (turn in place to find line)
            elif self.lost_line_frames <= self.max_lost_frames + self.max_search_frames:
                if not self.search_mode:
                    self.reset_pid_state()  # Reset PID when entering search
                    self.search_mode = True
                    self.get_logger().warn(f"Entering search mode - turning to find line")
                
                # Determine search direction based on last known position
                search_turn = 0.5  # Default turn rate for search
                if self.last_tape_x is not None:
                    # Turn toward side where line was last seen
                    if self.last_tape_x < width // 2:
                        search_turn = -0.5  # Turn left (line was on left)
                    else:
                        search_turn = 0.5   # Turn right (line was on right)
                
                twist = Twist()
                twist.linear.x = 0.0  # Stop forward motion during search
                twist.angular.z = search_turn
                self.vel_pub.publish(twist)
                return
            
            # Phase 3: Give up and stop
            else:
                if self.see_line or self.lost_line_frames == self.max_lost_frames + self.max_search_frames + 1:
                    self.get_logger().error(f"Line not found after search - stopping")
                self.see_line = False
                self.search_mode = False
                self.publish_stop_command()
                return
        
        # Line found - reset lost counter and search mode
            if self.lost_line_frames > self.max_lost_frames:
                # Was in search mode, reset PID
                self.reset_pid_state()
        if self.lost_line_frames > 0:
            self.get_logger().info(f"Line reacquired after {self.lost_line_frames} frames")
        self.see_line = True
        self.lost_line_frames = 0
        self.search_mode = False
        self.last_tape_x = tape_x
        
        # Safety check (should never happen, but prevents crashes)
        if tape_x is None:
            self.get_logger().error("Internal error: tape_x is None after detection - stopping")
            self.publish_stop_command()
            return
        
        # Compute control command
        turn, error, p_term, i_term, d_term = self.compute_control(tape_x, width)
        self.last_turn = turn  # Remember for coasting
        
        # Publish velocity command
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = turn
        self.vel_pub.publish(twist)
        
        # Timing instrumentation
        end_time = time.perf_counter()
        process_time_ms = (end_time - start_time) * 1000.0
        self.last_process_time_ms = process_time_ms
        
        # Exponential moving average
        if self.avg_process_time_ms == 0.0:
            self.avg_process_time_ms = process_time_ms
        else:
            self.avg_process_time_ms = (
                self.process_time_alpha * process_time_ms +
                (1 - self.process_time_alpha) * self.avg_process_time_ms
            )
        
        # Logging
        if self.frame_count % 10 == 0:
            position_label = "CENTER"
            third = width // 3
            if tape_x < third:
                position_label = "LEFT"
            elif tape_x > 2 * third:
                position_label = "RIGHT"
            self.get_logger().info(
                f"Frame {self.frame_count}: tape_x={tape_x} ({position_label}), err={error:.3f}, "
                f"turn={turn:.3f} [P={p_term:.3f} I={i_term:.3f} D={d_term:.3f}], "
                f"proc_time={process_time_ms:.2f}ms (avg={self.avg_process_time_ms:.2f}ms)"
            )
        
        # Debug visualization
        if self.debug_plot and self.frame_count % self.debug_interval == 0:
            self.draw_debug_view(img, tape_x, tape_y, turn, lines)

    def draw_debug_view(self, img, tape_x, tape_y, turn, lines):
        """Draw debug visualization with detected lines and target point"""
        height, width = img.shape[:2]
        img_draw = img.copy()
        
        # Draw detected lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img_draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw target point
        cv2.circle(img_draw, (tape_x, tape_y), 10, (0, 0, 255), -1)
        
        # Draw center thirds boundaries
        third = width // 3
        cv2.line(img_draw, (third, 0), (third, height), (0, 255, 255), 1)
        cv2.line(img_draw, (2 * third, 0), (2 * third, height), (0, 255, 255), 1)
        
        # Draw center line
        cv2.line(img_draw, (width // 2, 0), (width // 2, height), (255, 0, 0), 1)
        
        # Add text overlay
        cv2.putText(
            img_draw,
            f'turn={turn:.2f}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )
        cv2.putText(
            img_draw,
            f'proc={self.last_process_time_ms:.1f}ms',
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )
        
        # Show combined view
        cv2.imshow('LineFollowerAmy Debug', img_draw)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineFollowerAmy()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
