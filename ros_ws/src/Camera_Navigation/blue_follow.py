import pyrealsense2 as rs
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
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

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.timer = self.create_timer(0.1, self.camera_callback)

        self.blue_override = False

        self.get_logger().info("Paper follower with blue override started.")

    def camera_callback(self):
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                return

            image = np.asanyarray(color_frame.get_data())
            height, width, _ = image.shape

            right = 0
            middle = 0

            for i in range(height):
                for j in range(width):
                    b, g, r = image[i, j]

                    # Blue detection
                    if b > 100 and 100 < g < 150 and r < 100:

                        # RIGHT THIRD = highest priority
                        if j >= 2 * width // 3:
                            right += 1

                        # MIDDLE THIRD
                        elif j >= width // 3:
                            middle += 1

            if right > 0:
                self.blue_override = True
                self.turn_right()

            elif middle > 0:
                self.blue_override = True
                self.go_straight()

            else:
                self.blue_override = False

        except Exception as e:
            self.get_logger().error(str(e))

    def angle_goal_callback(self, msg):

        if self.blue_override:
            return

        angle_goal = -msg.data

        p = 0.2
        i = 0.0
        d = 0.0

        self.previous_errors.append(angle_goal)

        integral_error = sum(self.previous_errors)

        if len(self.previous_errors) > 1:
            derivative_error = (
                angle_goal - self.previous_errors[-2]
            ) / (1.0 / self.update_rate)
        else:
            derivative_error = 0.0

        turn_amount = (
            p * angle_goal +
            i * integral_error +
            d * derivative_error
        )

        twist = Twist()
        twist.linear.x = 0.25
        twist.angular.z = turn_amount
        self.drive_pub.publish(twist)

    def turn_right(self):
        twist = Twist()
        twist.linear.x = 0.20
        twist.angular.z = -0.5
        self.drive_pub.publish(twist)

    def go_straight(self):
        twist = Twist()
        twist.linear.x = 0.25
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