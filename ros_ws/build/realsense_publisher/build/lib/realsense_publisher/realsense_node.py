import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

class RealSensePublisher(Node):
	def __init__(self):

		super().__init__('realsense_publisher')

		self.publisher_ = self.create_publisher(
			Image,
			'/camera/color/image_raw',
			10
		)

		self.bridge = CvBridge()

		self.pipeline = rs.pipeline()
		config = rs.config()
		config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
		self.pipeline.start(config)

		self.timer = self.create_timer(1.0/ 30.0, self.timer_callback)
		self.get_logger().info("RealSense Publisher Started")

	def timer_callback(self):
		frames = self.pipeline.wait_for_frames()
		color_frame = frames.get_color_frame()

		if not color_frame:
			return

		color_image = np.asanyarray(color_frame.get_data())

		msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

		self.publisher_.publish(msg)

	def destroy_node(self):
		self.pipeline.stop()
		super().destroy_node()

def main():
	rclpy.init()
	node = RealSensePublisher()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
