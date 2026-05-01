#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        
        # Create subscribers with approximate time synchronization
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        
        # Synchronize the two streams (max 100ms difference)
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        self.get_logger().info('Camera subscriber initialized')
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
        Process color and depth frames
        Override this method with your custom logic
        """
        # Example: Display frames with OpenCV
        # Normalize depth for visualization (0-255)
        depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        
        # Combine side-by-side for visualization
        combined = np.hstack([color_frame, depth_colored])
        
        # Display (only works if GUI available)
        try:
            cv2.imshow('Color and Depth', combined)
            cv2.waitKey(1)
        except:
            pass  # No GUI available (e.g., on headless Pi)

def main():
    rclpy.init()
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
