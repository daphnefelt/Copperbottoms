#!/usr/bin/env python3

# Subscribes to color + depth images, detects AprilTags, publishes results as JSON on /landmarks.

import sys
import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import message_filters

# Function for detecting and measuring landmarks
sys.path.insert(0, os.path.dirname(__file__))
from find_landmarks import detect_landmark

class LandmarkSensorNode(Node):
    def __init__(self):
        super().__init__('landmark_sensor')
        self.bridge = CvBridge()

        # Synchronize color and depth streams
        color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=10,
            slop=0.05, # seconds of allowed timestamp difference
        )
        self.sync.registerCallback(self.image_callback)

        self.landmark_pub = self.create_publisher(String, '/landmarks', 10)
        self.get_logger().info('Landmark Sensor Node started')

    def image_callback(self, color_msg: Image, depth_msg: Image):
        print('running img callback')
        color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        landmarks = detect_landmark(color_img, depth_img)

        if landmarks:
            print(f"Detected {len(landmarks)} landmarks")
            payload = json.dumps(landmarks)
            self.landmark_pub.publish(String(data=payload))
            for lm in landmarks:
                self.get_logger().info(
                    f"Tag {lm['id']}: depth={lm['depth']:.2f}m  angle={lm['angle']:.3f}rad"
                )

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()