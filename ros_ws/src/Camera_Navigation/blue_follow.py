import pyrealsense2 as rs
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from collections import deque


class PaperFollower(Node):
    def __init__(self):
        super().__init__('paper_follower')

        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.angle_goal_sub = self.create_subscription(
            Float64,
            '/angle_goal',
            self.angle_goal_callback,
            10
        )

        self.previous_errors = deque(maxlen=10)
        self.update_rate = 10.0

        # SEARCH / LOST TARGET
        self.search_mode = False
        self.search_direction = "LEFT"
        self.no_blue_count = 0

        # REVERSE MODE
        self.reverse_mode = False
        self.reverse_count = 0
        self.reverse_frames = 15

        # RealSense setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.timer = self.create_timer(0.1, self.camera_callback)

        self.get_logger().info("Paper follower started.")

    # ---------------- CAMERA LOOP ----------------
    def camera_callback(self):

        try:
            frames = self.pipeline.wait_for_frames()
            frame = frames.get_color_frame()

            if not frame:
                return

            image = np.asanyarray(frame.get_data())
            h, w, _ = image.shape

            # BGR threshold for blue
            blue_mask = (
                (image[:, :, 0] > 120) &
                (image[:, :, 1] < 100) &
                (image[:, :, 2] < 100)
            )

            # ---------------- NO BLUE FOUND ----------------
            if not np.any(blue_mask):
                self.no_blue_count += 1

                if self.no_blue_count > 3:
                    self.reverse_mode = True

                if self.reverse_mode:
                    self.reverse_until_found()
                else:
                    self.search_for_blue()

                return

            # ---------------- BLUE FOUND ----------------
            self.no_blue_count = 0
            self.reverse_mode = False
            self.reverse_count = 0
            self.search_mode = False

            # Get all blue pixel x locations
            y_pixels, x_pixels = np.where(blue_mask)

            mean_x = np.mean(x_pixels)
            median_x = np.median(x_pixels)

            # normalize center error from -1 to +1
            error = (mean_x - (w / 2)) / (w / 2)

            self.get_logger().info(
                f"Blue Found | MeanX={mean_x:.1f}  MedianX={median_x:.1f}  Error={error:.3f}"
            )

            self.follow_blue(error)

        except Exception as e:
            self.get_logger().error(str(e))

    # ---------------- FOLLOW USING MEAN X ----------------
    def follow_blue(self, error):
        twist = Twist()

        # forward speed
        twist.linear.x = 0.22

        # steering
        twist.angular.z = -1.8 * error

        self.drive_pub.publish(twist)

    # ---------------- REVERSE ----------------
    def reverse_until_found(self):
        self.go_backward()
        self.reverse_count += 1

        self.get_logger().info("REVERSING - BLUE NOT FOUND")

        if self.reverse_count >= self.reverse_frames:
            self.reverse_mode = False
            self.reverse_count = 0
            self.search_for_blue()

    # ---------------- SEARCH ----------------
    def search_for_blue(self):
        twist = Twist()
        twist.linear.x = 0.0

        if self.search_direction == "LEFT":
            twist.angular.z = 0.6
            self.get_logger().info("SEARCHING LEFT")
        else:
            twist.angular.z = -0.6
            self.get_logger().info("SEARCHING RIGHT")

        self.search_direction = (
            "RIGHT" if self.search_direction == "LEFT" else "LEFT"
        )

        self.drive_pub.publish(twist)

    # ---------------- OPTIONAL ANGLE TOPIC ----------------
    def angle_goal_callback(self, msg):
        # disabled while using camera tracking
        return

    # ---------------- BASIC MOTION ----------------
    def go_backward(self):
        twist = Twist()
        twist.linear.x = -0.15
        twist.angular.z = 0.0
        self.drive_pub.publish(twist)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PaperFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()