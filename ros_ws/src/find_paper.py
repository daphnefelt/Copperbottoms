#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2 as cv
from std_msgs.msg import Float64

class FindPaperNode(Node):
    def __init__(self) -> None:
        super().__init__("find_paper")

        self.color_topic = "/camera/color/image_raw"
        self.depth_topic = "/camera/depth/image_raw"
        # self.actual_bgr = np.array([20, 255, 57]) # bgr for rgb (57, 255, 20)
        self.actual_bgr = np.array([74, 94, 15]) # bgr for rgb 15, 94, 74
        self.lower_bgr = self.actual_bgr - np.array([20, 20, 20])
        self.upper_bgr = self.actual_bgr + np.array([20, 20, 20])
        self.min_depth_mm = 1
        self.max_depth_mm = 10000

        self.latest_depth = None
        self.latest_depth_stamp = None
        self.frame_count = 0

        self.color_sub = self.create_subscription(Image, self.color_topic, self.color_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.angle_pub = self.create_publisher(Float64, "/angle_goal", 10)

        self.get_logger().info(f"Subscribed color: {self.color_topic}")
        self.get_logger().info(f"Subscribed depth: {self.depth_topic}")
        self.get_logger().info(f"Mask BGR range: lower={self.lower_bgr} upper={self.upper_bgr}")

    @staticmethod
    def image_to_bgr(msg: Image) -> np.ndarray:
        if msg.encoding.lower() != "bgr8":
            raise ValueError(f"Expected bgr8, got {msg.encoding}")
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return arr.reshape((msg.height, msg.width, 3))

    @staticmethod
    def image_to_depth_mm(msg: Image) -> np.ndarray:
        if msg.encoding.upper() != "16UC1":
            raise ValueError(f"Expected 16UC1, got {msg.encoding}")
        arr = np.frombuffer(msg.data, dtype=np.uint16)
        return arr.reshape((msg.height, msg.width))

    def depth_callback(self, msg: Image) -> None:
        try:
            self.latest_depth = self.image_to_depth_mm(msg)
            self.latest_depth_stamp = msg.header.stamp
        except Exception as exc:
            self.get_logger().error(f"Depth parse error: {exc}")

    def color_callback(self, msg: Image) -> None:
        self.frame_count += 1

        if self.latest_depth is None:
            self.get_logger().warn("No depth frame received yet.")
            return

        try:
            color = self.image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Color parse error: {exc}")
            return

        depth = self.latest_depth

        if color.shape[:2] != depth.shape:
            self.get_logger().warn(
                f"Color/depth size mismatch: color={color.shape[:2]} depth={depth.shape}. Skipping frame."
            )
            return

        lower = self.lower_bgr.reshape((1, 1, 3))
        upper = self.upper_bgr.reshape((1, 1, 3))

        color_mask = np.all((color >= lower) & (color <= upper), axis=2)

        # display = color.copy()
        # cv.imshow("image", display)
        # cv.waitKey(1)

        ys, xs = np.where(color_mask)
        if xs.size == 0:
            self.get_logger().info(f"Frame {self.frame_count}: no pixels in mask.")
            return

        centroid_x = float(xs.mean())
        image_center_x = (msg.width - 1) / 2.0
        x_offset = centroid_x - image_center_x  # left is negative, right is positive

        masked_depth = depth[color_mask]
        valid_depth = masked_depth[
            (masked_depth >= self.min_depth_mm) & (masked_depth <= self.max_depth_mm)
        ]

        if valid_depth.size == 0:
            self.get_logger().info(
                f"Frame {self.frame_count}: x_offset={x_offset:.2f}px, no valid depth in mask."
            )
            return

        avg_depth_mm = float(valid_depth.mean())

        self.get_logger().info(
            "Frame %d | stamp=%d.%09d | x_offset=%.2f px | avg_depth=%.1f mm | mask_pixels=%d valid_depth_pixels=%d"
            % (
                self.frame_count,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                x_offset,
                avg_depth_mm,
                xs.size,
                valid_depth.size,
            )
        )

        # send angle goal as proportional to x_offset
        angle_goal = Float64()
        # mapping from -200,200 to -45,45
        angle_goal.data = 0.225 * x_offset  # negative because left offset should
        # publish
        self.angle_pub.publish(angle_goal)
        self.get_logger().info(f"Published angle_goal: {angle_goal.data:.2f} degrees")

        # plot img with mask and centroid
        # display = color.copy()
        # display[color_mask] = [0, 255, 255]  # highlight mask
        # cv.circle(display, (int(centroid_x), int(ys.mean())), 5, (0, 0, 255), -1)  # centroid
        # cv.imshow("Mask", display)
        # cv.waitKey(1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FindPaperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()