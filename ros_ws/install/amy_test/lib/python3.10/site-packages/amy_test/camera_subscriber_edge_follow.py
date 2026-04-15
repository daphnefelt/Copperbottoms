#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import Twist

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        
        # Create subscribers with approximate time synchronization
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        
        # Publisher for velocity commands (for line following)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Synchronize the two streams (max 100ms difference)
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        # Line following parameters
        self.declare_parameter('line_following_enabled', True)
        self.declare_parameter('base_speed', 0.2)  # m/s
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        
        self.get_logger().info('Camera subscriber initialized with line following')
        self.frame_count = 0

    def synchronized_callback(self, color_msg, depth_msg):
        """Process synchronized color and depth frames"""
        try:
            # Convert ROS messages to OpenCV format
            color_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            self.frame_count += 1
            
            # Log frame info
            if self.frame_count % 10 == 0:  # Print every 10 frames
                self.get_logger().info(
                    f'Frame {self.frame_count}: '
                    f'Color {color_frame.shape}, '
                    f'Depth {depth_frame.shape}, '
                    f'Timestamp: {color_msg.header.stamp.sec}'
                )
            
            # TODO: Add your processing here
            # Example: terrain classification, obstacle detection, etc.
            self.process_frames(color_frame, depth_frame)
            
        except Exception as e:
            self.get_logger().warn(f'Error processing frames: {e}')

    def process_frames(self, color_frame, depth_frame):
        """
        Process color and depth frames with Canny edge detection and line following
        """
        # Get parameters
        canny_low = self.get_parameter('canny_low').value
        canny_high = self.get_parameter('canny_high').value
        line_following = self.get_parameter('line_following_enabled').value
        
        # Convert to grayscale for edge detection
        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blurred, canny_low, canny_high)
        
        # Region of interest (focus on lower portion of image where path is)
        roi_edges = self.region_of_interest(edges)
        
        # Line following logic
        line_vis = color_frame.copy()
        if line_following:
            steering_angle = self.detect_and_follow_lines(roi_edges, line_vis)
            if steering_angle is not None:
                self.publish_velocity_command(steering_angle)
        
        # Convert edges to BGR for visualization
        edges_colored = cv2.cvtColor(roi_edges, cv2.COLOR_GRAY2BGR)
        
        # Normalize depth for visualization (0-255)
        depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        
        # Create visualization: Color+Lines | Edges | Depth
        combined = np.hstack([line_vis, edges_colored, depth_colored])
        
        # Display (only works if GUI available)
        try:
            cv2.imshow('Line Following | Canny Edges | Depth', combined)
            cv2.waitKey(1)
        except:
            pass  # No GUI available (e.g., on headless Pi)
    
    def region_of_interest(self, edges):
        """
        Apply region of interest mask to focus on the area where path/lines are expected
        """
        height, width = edges.shape
        mask = np.zeros_like(edges)
        
        # Define trapezoidal region of interest (lower half, narrowing upward)
        polygon = np.array([[
            (int(width * 0.1), height),  # bottom left
            (int(width * 0.4), int(height * 0.6)),  # top left
            (int(width * 0.6), int(height * 0.6)),  # top right
            (int(width * 0.9), height)   # bottom right
        ]], np.int32)
        
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        return masked_edges
    
    def detect_and_follow_lines(self, edges, visualization_frame):
        """
        Detect lines using Hough transform and calculate steering angle
        """
        # Detect lines using Hough Line Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1,              # Distance resolution in pixels
            theta=np.pi/180,    # Angular resolution in radians
            threshold=50,       # Minimum votes to consider a line
            minLineLength=30,   # Minimum line length in pixels
            maxLineGap=20       # Maximum gap between line segments
        )
        
        if lines is None:
            self.get_logger().warn('No lines detected', throttle_duration_sec=2.0)
            return None
        
        # Separate lines into left and right based on slope
        left_lines = []
        right_lines = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            if x2 == x1:  # Avoid division by zero
                continue
                
            slope = (y2 - y1) / (x2 - x1)
            
            # Filter out horizontal lines (slope too close to 0)
            if abs(slope) < 0.5:
                continue
            
            # Categorize by slope
            if slope < 0:  # Left line (negative slope)
                left_lines.append(line[0])
            else:  # Right line (positive slope)
                right_lines.append(line[0])
        
        # Average the lines
        left_line = self.average_lines(left_lines) if left_lines else None
        right_line = self.average_lines(right_lines) if right_lines else None
        
        # Draw detected lines
        height = visualization_frame.shape[0]
        if left_line is not None:
            x1, y1, x2, y2 = self.extrapolate_line(left_line, height)
            cv2.line(visualization_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        if right_line is not None:
            x1, y1, x2, y2 = self.extrapolate_line(right_line, height)
            cv2.line(visualization_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(
            visualization_frame, left_line, right_line
        )
        
        return steering_angle
    
    def average_lines(self, lines):
        """
        Average multiple line segments into a single line
        """
        x1s, y1s, x2s, y2s = [], [], [], []
        
        for line in lines:
            x1, y1, x2, y2 = line
            x1s.append(x1)
            y1s.append(y1)
            x2s.append(x2)
            y2s.append(y2)
        
        return [int(np.mean(x1s)), int(np.mean(y1s)), 
                int(np.mean(x2s)), int(np.mean(y2s))]
    
    def extrapolate_line(self, line, height):
        """
        Extrapolate line to span the full ROI height
        """
        x1, y1, x2, y2 = line
        
        if x2 == x1:
            return x1, height, x2, int(height * 0.6)
        
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
        
        # Bottom of image
        y_bottom = height
        x_bottom = int((y_bottom - intercept) / slope)
        
        # Top of ROI
        y_top = int(height * 0.6)
        x_top = int((y_top - intercept) / slope)
        
        return x_bottom, y_bottom, x_top, y_top
    
    def calculate_steering_angle(self, frame, left_line, right_line):
        """
        Calculate steering angle based on detected lines
        Returns angle in radians (negative = turn left, positive = turn right)
        """
        height, width = frame.shape[:2]
        
        # Image center
        center_x = width // 2
        
        # Calculate lane center
        if left_line is not None and right_line is not None:
            # Both lines detected - use midpoint
            left_x = left_line[0]  # x1 of left line
            right_x = right_line[0]  # x1 of right line
            lane_center_x = (left_x + right_x) // 2
        elif left_line is not None:
            # Only left line - offset to the right
            lane_center_x = left_line[0] + int(width * 0.2)
        elif right_line is not None:
            # Only right line - offset to the left
            lane_center_x = right_line[0] - int(width * 0.2)
        else:
            return None
        
        # Draw center line and lane center
        cv2.line(frame, (center_x, 0), (center_x, height), (255, 0, 0), 2)  # Blue: image center
        cv2.line(frame, (lane_center_x, 0), (lane_center_x, height), (0, 0, 255), 2)  # Red: lane center
        
        # Calculate steering angle (proportional control)
        offset = lane_center_x - center_x
        steering_angle = float(offset) / (width / 2) * (np.pi / 4)  # Max ±45 degrees
        
        # Display steering info
        cv2.putText(frame, f'Offset: {offset}px', (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f'Angle: {np.degrees(steering_angle):.1f}deg', (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return steering_angle
    
    def publish_velocity_command(self, steering_angle):
        """
        Publish velocity command based on steering angle
        """
        twist = Twist()
        
        # Get parameters
        base_speed = self.get_parameter('base_speed').value
        max_angular = self.get_parameter('max_angular_speed').value
        
        # Linear velocity (reduce speed when turning sharply)
        speed_factor = 1.0 - (abs(steering_angle) / (np.pi / 2))
        twist.linear.x = base_speed * max(0.3, speed_factor)
        
        # Angular velocity (proportional to steering angle)
        twist.angular.z = -np.clip(steering_angle, -max_angular, max_angular)
        
        self.cmd_vel_pub.publish(twist)
        
        # Log occasionally
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'vel: {twist.linear.x:.2f} m/s, '
                f'ang: {twist.angular.z:.2f} rad/s'
            )

def main():
    rclpy.init()
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robot on shutdown
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_vel_pub.publish(twist)
        node.get_logger().info('Stopping robot...')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
