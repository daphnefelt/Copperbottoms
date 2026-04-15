### Dark Lane Following ###

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # parameters
        self.frame_count = 0
        image_topic = '/camera/color/image_raw'
        cmd_vel_topic = '/cmd_vel'

        # tuning
        self.kp = 0.8
        self.col_threshold = 30
        self.min_pixels = 50
        self.forward_speed = 0.2

        # subscribers
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # publishers
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            f'Line follower node started. image_topic={image_topic}, cmd_vel_topic={cmd_vel_topic}'
        )

    def image_callback(self, msg: Image):
        # frame count
        self.frame_count += 1
        if self.frame_count % 10 == 0:
        self.get_logger().info(
                    f"tape_x={tape_x}, err={error:.3f}, blue_px={blue_count}, turn={turn:.3f}"
                )

        if msg.encoding.lower() != "bgr8":
            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
            return
        
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

        # focus on ROI (bottom half)
        height = img.shape[0]
        width = img.shape[1]
        
        roi_strt = int(height * 0.70) # bottom 30%
        roi = img[roi_strt:height, :, :]

        # blue color thresholding
        b = roi[:, :, 0].astype(np.float32)
        g = roi[:, :, 1].astype(np.float32)
        r = roi[:, :, 2].astype(np.float32)

        blue_mask = (b > self.col_threshold) & (b > g) & (b > r)
        blue_count = np.sum(blue_mask)

        if blue_count < self.min_pixels:
            self.get_logger().info("No tape detected, stopping.")
            self.publish_velocity(0.0, 0.0)
            return

        


        pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


    


