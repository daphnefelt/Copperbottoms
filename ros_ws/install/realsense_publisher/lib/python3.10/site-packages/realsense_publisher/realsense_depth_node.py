#!/usr/bin/env python3
"""
Enhanced RealSense Publisher with Depth Support
Publishes both color images and depth point cloud for odometry
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import struct


class RealSenseDepthPublisher(Node):
    def __init__(self):
        super().__init__('realsense_depth_publisher')
        
        # Publishers
        self.color_pub = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            10
        )
        
        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth/image_raw',
            10
        )
        
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/camera/depth/points',
            10
        )
        
        self.bridge = CvBridge()
        
        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Start pipeline
        profile = self.pipeline.start(config)
        
        # Get depth sensor
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # Alignment object to align depth to color
        self.align = rs.align(rs.stream.color)
        
        # Timer
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("RealSense Depth Publisher Started")
        
    def timer_callback(self):
        # Wait for frames
        frames = self.pipeline.wait_for_frames()
        
        # Align depth to color
        aligned_frames = self.align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return
        
        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # Publish color image
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        color_msg.header.stamp = self.get_clock().now().to_msg()
        color_msg.header.frame_id = 'camera_color_optical_frame'
        self.color_pub.publish(color_msg)
        
        # Publish depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_pub.publish(depth_msg)
        
        # Generate and publish point cloud
        self.publish_pointcloud(color_frame, depth_frame)
        
    def publish_pointcloud(self, color_frame, depth_frame):
        """Generate and publish point cloud from depth and color"""
        # Get intrinsics
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        
        # Convert depth image to numpy
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        height, width = depth_image.shape
        
        # Create point cloud
        points = []
        
        # Sample every nth pixel for performance (adjust as needed)
        step = 4
        
        for v in range(0, height, step):
            for u in range(0, width, step):
                depth = depth_image[v, u] * self.depth_scale
                
                if depth == 0 or depth > 10.0:  # Skip invalid or too far points
                    continue
                
                # Deproject pixel to 3D point
                x = (u - depth_intrin.ppx) / depth_intrin.fx * depth
                y = (v - depth_intrin.ppy) / depth_intrin.fy * depth
                z = depth
                
                # Get color
                b, g, r = color_image[v, u]
                
                # Pack RGB into single float
                rgb = struct.unpack('f', struct.pack('I', 
                    (int(r) << 16) | (int(g) << 8) | int(b)))[0]
                
                points.append([x, y, z, rgb])
        
        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        header = color_frame.profile.as_video_stream_profile().get_extrinsics_to(
            depth_frame.profile.as_video_stream_profile())
        
        pc2_msg = point_cloud2.create_cloud(
            point_cloud2.Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='camera_depth_optical_frame'
            ),
            fields,
            points
        )
        
        self.pointcloud_pub.publish(pc2_msg)
        
    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseDepthPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
