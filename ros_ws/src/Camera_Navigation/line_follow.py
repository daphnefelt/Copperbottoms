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
        self.error_offset = -0.4 # to account for location of the camera lens
        self.kp = 0.8
        self.color_threshold = 30
        self.min_pixels = 50
        self.forward_speed = 0.3
        self.max_turn = 1.0
        self.see_line = False # by default we assume we don't see the line until we do, to avoid spurious turns at startup

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


        blue_score = b - 0.5 * (g + r)  # simple blue score
        blue_mask = (blue_score > self.color_threshold)
        blue_count = np.sum(blue_mask)

        if blue_count >= self.min_pixels:
            self.see_line = True
            
        if blue_count < self.min_pixels:
            self.get_logger().info("No tape detected, stopping.")
            self.see_line = False
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)

            return

        # find tape position
        weighted = blue_score * blue_mask
        column_strength = weighted.mean(axis=0)  # average over rows
        tape_x = np.argmax(column_strength)

        # error
        center_x = width / 2
        error = (tape_x - center_x) / center_x  # normalize error
        error += self.error_offset

        # control
        turn = float(np.clip(-self.kp * error, -self.max_turn, self.max_turn))

        # speed + publish

        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = turn
        self.vel_pub.publish(twist)
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f"tape_x={tape_x}, err={error:.3f}, blue_px={blue_count}, turn={turn:.3f}"
            )

    def turn_to_find_line(self):
        # rotate in place to try to find the line when it is lost
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # turn right
        self.vel_pub.publish(twist)
        self.get_logger().info("Line lost - turning to try to find it.")



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


    

