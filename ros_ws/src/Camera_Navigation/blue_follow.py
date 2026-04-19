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

        # SEARCH STATE
        self.search_mode = False
        self.search_direction = "LEFT"
        self.no_blue_count = 0

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

            left_end = w // 3
            mid_end = 2 * w // 3

            blue_mask = (
                (image[:, :, 0] > 120) &
                (image[:, :, 1] < 100) &
                (image[:, :, 2] < 100)
            )

            # ---------------- NO BLUE DETECTED ----------------
            if not np.any(blue_mask):
                self.no_blue_count += 1

                # only enter search after a few missed frames (prevents flicker)
                if self.no_blue_count > 3:
                    self.search_mode = True

                self.search_for_blue()
                return

            # ---------------- BLUE FOUND ----------------
            self.no_blue_count = 0
            self.search_mode = False

            cols = np.where(blue_mask.any(axis=0))[0]

            left_count = np.sum(cols < left_end)
            middle_count = np.sum((cols >= left_end) & (cols < mid_end))
            right_count = np.sum(cols >= mid_end)

            if right_count > max(left_count, middle_count):
                self.turn_right()

            elif middle_count > max(left_count, right_count):
                self.go_straight()

            else:
                self.go_left()

        except Exception as e:
            self.get_logger().error(str(e))

    # ---------------- SEARCH BEHAVIOR ----------------
    def search_for_blue(self):
        twist = Twist()
        twist.linear.x = 0.15

        if self.search_direction == "LEFT":
            twist.angular.z = 0.6
            self.get_logger().info("SEARCHING LEFT")

        else:
            twist.angular.z = -0.6
            self.get_logger().info("SEARCHING RIGHT")

        # flip direction every cycle
        self.search_direction = "RIGHT" if self.search_direction == "LEFT" else "LEFT"

        self.drive_pub.publish(twist)

    # ---------------- ANGLE CONTROL ----------------
    def angle_goal_callback(self, msg):

        if self.search_mode:
            return

        error = -msg.data
        self.previous_errors.append(error)

        integral = sum(self.previous_errors)
        derivative = 0.0

        if len(self.previous_errors) > 1:
            derivative = (self.previous_errors[-1] - self.previous_errors[-2]) * self.update_rate

        turn = 0.2 * error + 0.0 * integral + 0.0 * derivative

        twist = Twist()
        twist.linear.x = 0.25
        twist.angular.z = turn
        self.drive_pub.publish(twist)

    # ---------------- MOTION ----------------
    def turn_right(self):
        twist = Twist()
        twist.linear.x = 0.20
        twist.angular.z = -2
        self.drive_pub.publish(twist)

    def go_straight(self):
        twist = Twist()
        twist.linear.x = 0.25
        twist.angular.z = 0.0
        self.drive_pub.publish(twist)

    def go_left(self):
        twist = Twist()
        twist.linear.x = 0.20
        twist.angular.z = 0.5
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
