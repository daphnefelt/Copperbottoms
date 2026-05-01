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
        # Tape color params (BGR)
        self.declare_parameter('tape_color', [164, 108, 7])
        self.declare_parameter('color_tolerance', [50, 50, 90])

        image_topic = self.get_parameter('image_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        self.target_width = self.get_parameter('target_width').value
        self.target_height = self.get_parameter('target_height').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.tape_color = np.array(self.get_parameter('tape_color').value)
        self.color_tolerance = np.array(self.get_parameter('color_tolerance').value)

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
        tape_msg = TapeContour()
        if contour is not None:
            tape_msg.found = True
            tape_msg.center_x = float(center[0])
            tape_msg.center_y = float(center[1])
            tape_msg.angle = float(angle)
            tape_msg.area = float(area)
        else:
            tape_msg.found = False
            tape_msg.center_x = 0.0
            tape_msg.center_y = 0.0
            tape_msg.angle = 0.0
            tape_msg.area = 0.0
        self.tape_pub.publish(tape_msg)
        # Publish debug image
        debug_img = self.create_debug_visualization(img, mask, contour, center, angle)
        self.publish_debug_image(debug_img, msg.header)

    def apply_color_filter(self, img):
        lower = self.tape_color - self.color_tolerance
        upper = self.tape_color + self.color_tolerance
        mask = cv2.inRange(img, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    def find_best_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, None, 0.0
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_contour_area]
        if not valid:
            return None, None, None, 0.0
        best = max(valid, key=cv2.contourArea)
        M = cv2.moments(best)
        if M['m00'] == 0:
            return None, None, None, 0.0
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        area = cv2.contourArea(best)
        [vx, vy, x, y] = cv2.fitLine(best, cv2.DIST_L2, 0, 0.01, 0.01)
        angle = np.degrees(np.arctan2(vy[0], vx[0]))
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
