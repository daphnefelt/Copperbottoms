#!/usr/bin/env python3
"""
Tape Vision Node: Subscribes to camera, detects tape, publishes TapeContour message
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from amy_test.msg import TapeContour

class TapeVisionNode(Node):
    def __init__(self):
        super().__init__('tape_vision_node')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('debug_image_topic', '/tape_debug/image')
        self.declare_parameter('target_width', 320)
        self.declare_parameter('target_height', 240)
        self.declare_parameter('min_contour_area', 100)
        # HSV color params (more robust to lighting than BGR)
        # For cyan/teal tape: H~85-100, S~90-255, V~70-255 (high saturation to reject walls)
        self.declare_parameter('hsv_lower', [75, 90, 70])  # Require high saturation to reject dull walls
        self.declare_parameter('hsv_upper', [105, 255, 255])  # Upper HSV bound
        # Adaptive tolerance parameters
        self.declare_parameter('adaptive_tolerance', False)  # Enable adaptive adjustment
        self.declare_parameter('h_tolerance_step', 3)  # Hue adjustment step
        self.declare_parameter('sv_tolerance_step', 10)  # Saturation/Value step
        self.declare_parameter('hsv_lower_min', [70, 70, 50])  # Tighter minimum to prevent wall detection
        # testing theseparameter s[]
        self.declare_parameter('hsv_upper_max', [110, 255, 255])  # Maximum (loosest)
        self.declare_parameter('max_detection_percent', 10.0)  # Reset if detecting >10% of image

        image_topic = self.get_parameter('image_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        self.target_width = self.get_parameter('target_width').value
        self.target_height = self.get_parameter('target_height').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.hsv_lower = np.array(self.get_parameter('hsv_lower').value)
        self.hsv_upper = np.array(self.get_parameter('hsv_upper').value)
        
        # Adaptive tolerance settings
        self.adaptive_tolerance = self.get_parameter('adaptive_tolerance').value
        self.h_tolerance_step = self.get_parameter('h_tolerance_step').value
        self.sv_tolerance_step = self.get_parameter('sv_tolerance_step').value
        self.hsv_lower_min = np.array(self.get_parameter('hsv_lower_min').value)
        self.hsv_upper_max = np.array(self.get_parameter('hsv_upper_max').value)
        self.max_detection_percent = self.get_parameter('max_detection_percent').value
        
        # Tracking for adaptive adjustment
        self.frames_without_detection = 0
        self.max_frames_before_adapt = 3  # Adjust after 3 failed frames
        
        # Spatial filtering parameters to avoid false detections (walls, etc.)
        self.declare_parameter('roi_top_ignore_ratio', 0.35)  # Ignore top 35% where walls appear
        self.declare_parameter('center_bias_weight', 200.0)  # Penalty for distance from center
        self.declare_parameter('continuity_weight', 300.0)  # Penalty for distance from last position
        self.declare_parameter('y_position_weight', 50.0)  # Prefer lower (closer) tape
        
        self.roi_top_ignore_ratio = self.get_parameter('roi_top_ignore_ratio').value
        self.center_bias_weight = self.get_parameter('center_bias_weight').value
        self.continuity_weight = self.get_parameter('continuity_weight').value
        self.y_position_weight = self.get_parameter('y_position_weight').value
        
        # Track last known tape position for continuity
        self.last_tape_center = None
        
        # Sharp turn detection parameters
        self.declare_parameter('sharp_turn_angle_threshold', 85.0)  # Degrees from vertical  # changed to 85 
        self.declare_parameter('sharp_turn_min_area', 500)  # Minimum contour area for valid sharp turn
        self.declare_parameter('sharp_turn_position_threshold', 0.6)  # Fraction of width for position check
        
        self.sharp_turn_angle_threshold = self.get_parameter('sharp_turn_angle_threshold').value
        self.sharp_turn_min_area = self.get_parameter('sharp_turn_min_area').value
        self.sharp_turn_position_threshold = self.get_parameter('sharp_turn_position_threshold').value
        
        # Sharp turn tracking
        self.sharp_turn_frames = 0  # Consecutive frames with sharp turn detected
        self.is_sharp_turn = False  # Current sharp turn state

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.tape_pub = self.create_publisher(TapeContour, '/tape_contour', 10)
        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)
        self.get_logger().info('TapeVisionNode started')

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return
        if img.shape[1] != self.target_width or img.shape[0] != self.target_height:
            img = cv2.resize(img, (self.target_width, self.target_height))
        mask = self.apply_color_filter(img)
        contour, center, angle, area = self.find_best_contour(mask)
        
        # Sharp turn detection with 2-frame validation
        tape_msg = TapeContour()
        if contour is not None:
            tape_msg.found = True
            tape_msg.center_x = float(center[0])
            tape_msg.center_y = float(center[1])
            tape_msg.angle = float(angle)
            tape_msg.area = float(area)
            
            # Check if this frame has sharp turn conditions
            angle_from_vertical = abs(angle - (-90.0))  # -90 = straight up, 0/180 = horizontal
            is_sharp_angle = angle_from_vertical > self.sharp_turn_angle_threshold
            has_enough_area = area >= self.sharp_turn_min_area
            
            # Position check: sharp turns push tape to edge in turn direction
            width = self.target_width
            cx = center[0]
            position_threshold = width * self.sharp_turn_position_threshold
            
            # Right turn: angle near 0° (horizontal right), tape on right side
            # Left turn: angle near ±180° (horizontal left), tape on left side
            if -20 <= angle <= 20:  # Horizontal right turn
                is_correct_position = cx > position_threshold
            elif angle >= 160 or angle <= -160:  # Horizontal left turn
                is_correct_position = cx < (width - position_threshold)
            else:
                is_correct_position = False
            
            # Validate sharp turn
            if is_sharp_angle and has_enough_area and is_correct_position:
                self.sharp_turn_frames += 1
            else:
                self.sharp_turn_frames = 0
            
            # Require 2 consecutive frames for sharp turn confirmation
            was_sharp_turn = self.is_sharp_turn
            self.is_sharp_turn = (self.sharp_turn_frames >= 2)
            tape_msg.sharp_turn = self.is_sharp_turn
            
            # Log when sharp turn is first confirmed
            if self.is_sharp_turn and not was_sharp_turn:
                self.get_logger().warn(
                    f'[VISION] Sharp turn confirmed! Angle: {angle:.1f}°, '
                    f'Area: {area:.0f}, Position: ({center[0]:.1f}, {center[1]:.1f})'
                )
        else:
            tape_msg.found = False
            tape_msg.center_x = 0.0
            tape_msg.center_y = 0.0
            tape_msg.angle = 0.0
            tape_msg.area = 0.0
            tape_msg.sharp_turn = False
            self.sharp_turn_frames = 0
            self.is_sharp_turn = False
        
        self.tape_pub.publish(tape_msg)
        
        # Track detection for adaptive tolerance
        if not tape_msg.found:
            self.frames_without_detection += 1
        else:
            self.frames_without_detection = 0
        
        # Publish debug image
        debug_img = self.create_debug_visualization(img, mask, contour, center, angle)
        self.publish_debug_image(debug_img, msg.header)

    def apply_color_filter(self, img):
        """Apply HSV color threshold with ROI masking and adaptive tolerance"""
        height, width = img.shape[:2]
        
        # ROI mask: black out top portion where walls typically appear
        roi_mask = np.ones((height, width), dtype=np.uint8) * 255
        ignore_rows = int(height * self.roi_top_ignore_ratio)
        roi_mask[0:ignore_rows, :] = 0
        
        # Convert BGR to HSV (more robust to lighting)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Apply HSV color filter
        color_mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # Combine ROI mask with color mask
        mask = cv2.bitwise_and(color_mask, roi_mask)
        
        # Clean up noise
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Adaptive tolerance
        if self.adaptive_tolerance:
            white_pixels = cv2.countNonZero(mask)
            total_pixels = mask.shape[0] * mask.shape[1]
            white_percent = (white_pixels / total_pixels) * 100
            
            # Over-detection check: if detecting too much, reset to defaults
            if white_percent > self.max_detection_percent:
                default_lower = np.array(self.get_parameter('hsv_lower').value)
                self.hsv_lower = default_lower.copy()
                self.hsv_upper = np.array(self.get_parameter('hsv_upper').value)
                self.get_logger().warn(
                    f'Over-detection ({white_percent:.1f}% > {self.max_detection_percent}%), '
                    f'resetting HSV to defaults: {self.hsv_lower}',
                    throttle_duration_sec=2.0
                )
                self.frames_without_detection = 0
            # Too little detection - widen HSV range
            elif white_percent < 0.5:
                self.frames_without_detection += 1
                if self.frames_without_detection >= self.max_frames_before_adapt:
                    # Widen Hue range
                    self.hsv_lower[0] = max(self.hsv_lower[0] - self.h_tolerance_step, 
                                           self.hsv_lower_min[0])
                    self.hsv_upper[0] = min(self.hsv_upper[0] + self.h_tolerance_step, 
                                           self.hsv_upper_max[0])
                    # Widen S/V ranges
                    self.hsv_lower[1] = max(self.hsv_lower[1] - self.sv_tolerance_step, 
                                           self.hsv_lower_min[1])
                    self.hsv_lower[2] = max(self.hsv_lower[2] - self.sv_tolerance_step, 
                                           self.hsv_lower_min[2])
                    self.get_logger().info(
                        f'Adapting: Low detection ({white_percent:.2f}%), '
                        f'HSV range: {self.hsv_lower} to {self.hsv_upper}',
                        throttle_duration_sec=2.0
                    )
            # Good detection - gradually tighten
            elif white_percent > 1.0:
                self.frames_without_detection = 0
                # Slowly restore default ranges
                default_lower = np.array(self.get_parameter('hsv_lower').value)
                default_upper = np.array(self.get_parameter('hsv_upper').value)
                if self.hsv_lower[0] < default_lower[0]:
                    self.hsv_lower[0] += 1
                if self.hsv_upper[0] > default_upper[0]:
                    self.hsv_upper[0] -= 1
            else:
                self.frames_without_detection = 0
        
        return mask

    def find_best_contour(self, mask):
        """Find best contour using spatial scoring (not just largest area)"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, None, 0.0
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_contour_area]
        if not valid:
            return None, None, None, 0.0
        
        # Score contours by multiple factors instead of just area
        height, width = mask.shape[:2]
        center_x = width / 2
        center_y = height / 2
        
        best_score = -float('inf')
        best = None
        
        for contour in valid:
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
            area = cv2.contourArea(contour)
            
            # Base score: area (larger is better)
            score = area
            
            # Penalty 1: Distance from image center (prefer centered tape)
            # Reduce penalty during sharp turns (tape expected to be off-center)
            dx_center = cx - center_x
            dy_center = cy - center_y
            dist_from_center = np.sqrt(dx_center**2 + dy_center**2)
            center_bias = self.center_bias_weight * (0.3 if self.is_sharp_turn else 1.0)
            score -= dist_from_center * center_bias
            
            # Penalty 2: Distance from last known position (continuity)
            # Reduce penalty during sharp turns (tape position changes rapidly)
            if self.last_tape_center is not None:
                last_cx, last_cy = self.last_tape_center
                dx_last = cx - last_cx
                dy_last = cy - last_cy
                dist_from_last = np.sqrt(dx_last**2 + dy_last**2)
                continuity_bias = self.continuity_weight * (0.3 if self.is_sharp_turn else 1.0)
                score -= dist_from_last * continuity_bias
            
            # Penalty 3: Y-position (prefer bottom/ground over top/walls)
            # Higher cy = closer to bottom (better), lower cy = top (worse)
            y_factor = (center_y - cy) / center_y  # Negative when below center (good)
            score -= y_factor * self.y_position_weight
            
            if score > best_score:
                best_score = score
                best = contour
        
        if best is None:
            return None, None, None, 0.0
        
        M = cv2.moments(best)
        if M['m00'] == 0:
            return None, None, None, 0.0
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        area = cv2.contourArea(best)
        [vx, vy, x, y] = cv2.fitLine(best, cv2.DIST_L2, 0, 0.01, 0.01)
        
        # Ensure angle points forward (toward top of image, y=0)
        # If vector points downward (vy > 0), flip it to point upward
        if vy[0] > 0:
            vx[0] = -vx[0]
            vy[0] = -vy[0]
        
        angle = np.degrees(np.arctan2(vy[0], vx[0])) 
        
        # Update last known position for continuity tracking
        self.last_tape_center = (cx, cy)
        
        return best, (cx, cy), angle, area

    def create_debug_visualization(self, img, mask, contour, center, angle):
        height, width = img.shape[:2]
        debug_img = np.zeros((height, width * 2, 3), dtype=np.uint8)
        img_overlay = img.copy()
        if contour is not None:
            cv2.drawContours(img_overlay, [contour], -1, (0, 255, 0), 2)
            cv2.circle(img_overlay, center, 8, (0, 0, 255), -1)
            cx, cy = center
            vector_length = 60
            angle_rad = np.radians(angle)
            end_x = int(cx + vector_length * np.cos(angle_rad))
            end_y = int(cy + vector_length * np.sin(angle_rad))
            cv2.arrowedLine(img_overlay, center, (end_x, end_y), (255, 0, 255), 3, tipLength=0.3)
        cv2.line(img_overlay, (width // 2, 0), (width // 2, height), (255, 255, 0), 1)
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        debug_img[:, :width] = img_overlay
        debug_img[:, width:] = mask_color
        cv2.line(debug_img, (width, 0), (width, height), (255, 255, 255), 2)
        
        # Add HSV range info overlay
        if self.adaptive_tolerance:
            hsv_text = f'HSV: [{self.hsv_lower[0]:.0f}-{self.hsv_upper[0]:.0f}] S:{self.hsv_lower[1]:.0f} V:{self.hsv_lower[2]:.0f}'
            cv2.putText(debug_img, hsv_text, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (0, 255, 255), 1, cv2.LINE_AA)
            status = 'FOUND' if contour is not None else 'LOST'
            cv2.putText(debug_img, status, (5, 40), cv2.FONT_HERSHEY_SIMPLEX,
                       0.5, (0, 255, 0) if contour is not None else (0, 0, 255), 1, cv2.LINE_AA)
        
        return debug_img

    def publish_debug_image(self, debug_img, header):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TapeVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
