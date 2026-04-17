#!/usr/bin/env python3

from pathlib import Path

import numpy as np
import cv2
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class OneShotImageSaver(Node):
	def __init__(self, output_path: Path, topic: str):
		super().__init__('one_shot_image_saver')
		self.output_path = output_path
		self.bridge = CvBridge()
		self.saved = False
		self.error = None
		self.subscription = self.create_subscription(Image, topic, self._callback, 10)

	def _callback(self, msg: Image) -> None:
		if self.saved:
			return
		try:
			frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
			if cv2.imwrite(str(self.output_path), frame):
				self.saved = True
				self.get_logger().info(f'Saved image to {self.output_path.resolve()}')
			else:
				self.error = f'failed to save image to {self.output_path}'
		except Exception as exc:
			self.error = str(exc)


def save_from_robo_realsense_topic(output_path: Path, timeout_sec: float = 8.0) -> tuple[bool, str]:
	topic = '/camera/color/image_raw'
	rclpy.init(args=None)
	node = OneShotImageSaver(output_path, topic)
	end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)

	try:
		while rclpy.ok() and not node.saved and node.error is None:
			rclpy.spin_once(node, timeout_sec=0.2)
			if node.get_clock().now().nanoseconds >= end_time:
				break

		if node.saved:
			return True, f'Saved image to {output_path.resolve()} from topic {topic}'

		if node.error:
			return False, f'Error processing topic frame: {node.error}'

		return False, (
			f'No image received on {topic} within {timeout_sec:.1f}s. '
			'Start robo_realsense publisher first (python3 robo_realsense/ros_stream.py).'
		)
	finally:
		node.destroy_node()
		rclpy.shutdown()


def main() -> int:
	output_path = Path("img3.jpg")
	width, height, fps = 640, 480, 30

	ok, message = save_from_robo_realsense_topic(output_path)
	if ok:
		print(message)
		return 0
	print(f"Warning: {message}")

	pipeline = rs.pipeline()
	config = rs.config()
	config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

	try:
		pipeline.start(config)
		rgb_frame = None

		# Grab a few frames so auto-exposure can settle and we get a valid image.
		for _ in range(30):
			frames = pipeline.wait_for_frames(timeout_ms=2000)
			color = frames.get_color_frame()
			if color:
				rgb_frame = np.asanyarray(color.get_data())
				break

		if rgb_frame is None:
			print("Error: no color frame received from RealSense camera.")
			return 1

		if not cv2.imwrite(str(output_path), rgb_frame):
			print(f"Error: failed to save image to {output_path}.")
			return 1

		print(f"Saved image to {output_path.resolve()}")
		return 0
	except Exception as exc:
		print(
			"Error: RealSense capture failed: "
			f"{exc}\n"
			"Tip: if camera is busy, run 'python3 robo_realsense/ros_stream.py' and try again."
		)
		return 1
	finally:
		try:
			pipeline.stop()
		except Exception:
			pass


if __name__ == "__main__":
	raise SystemExit(main())
