                                                                                               
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealsenseColorDepthPublisher(Node):
    def __init__(self, width=424, height=240, fps=6):
        super().__init__('realsense_color_depth_publisher')
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.bridge = CvBridge()

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.get_logger().info(f'Starting RealSense pipeline: {width}x{height} @ {fps} FPS')
        self.pipe.start(cfg)

        # timer at ~fps
        self.timer = self.create_timer(1.0 / fps, self.capture_and_publish)

    def capture_and_publish(self):
        try:
            frames = self.pipe.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return
            
            # Get current timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Process and publish color image (BGR)
            color_img = np.asanyarray(color_frame.get_data())
            color_msg = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = 'camera_color_optical_frame'
            self.color_pub.publish(color_msg)
            
            # Process and publish depth image (uint16, millimeters)
            depth_img = np.asanyarray(depth_frame.get_data())
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
            depth_msg.header.stamp = timestamp
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            self.depth_pub.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Frame skip: {e}')

    def destroy_node(self):
        try:
            self.pipe.stop()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = RealsenseColorDepthPublisher(width=424, height=240, fps=6)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
