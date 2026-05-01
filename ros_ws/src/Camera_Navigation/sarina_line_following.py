import pyrealsense2 as rs
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from CannyEdgeDetection import CannyEdgeDetection


class LineFollower(Node):

    def __init__(self):
        super().__init__('paper_follower')

        # Publishers / Subscribers
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.angle_pub = self.create_publisher(Float64, '/line_angle', 10)

        # If you still want to listen to /angle_goal
        self.angle_goal_sub = self.create_subscription(
            Float64,
            '/angle_goal',
            self.angle_goal_callback,
            10
        )

        # PID gains
        self.kp = 0.20
        self.ki = 0.01
        self.kd = 0.05

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 0.1

        # RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        self.get_logger().info("Starting RealSense pipeline...")
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.get_logger().info("RealSense pipeline started.")

        # Main timer loop
        self.timer = self.create_timer(0.1, self.loop)

    # -------------------------------------------------
    # Optional external angle topic
    # -------------------------------------------------
    def angle_goal_callback(self, msg):
        self.get_logger().info(f"Received /angle_goal: {msg.data:.2f}")

    # -------------------------------------------------
    # Manual Hough Transform
    # -------------------------------------------------
    def hough(self, edges):

        rows, cols = edges.shape

        thetas = np.deg2rad(np.arange(-90, 90))
        diag = int(np.sqrt(rows**2 + cols**2))

        acc = np.zeros((2 * diag, len(thetas)))

        y_idxs, x_idxs = np.nonzero(edges)

        for i in range(len(x_idxs)):
            x = x_idxs[i]
            y = y_idxs[i]

            for t in range(len(thetas)):
                rho = int(x * np.cos(thetas[t]) + y * np.sin(thetas[t])) + diag

                if 0 <= rho < 2 * diag:
                    acc[rho, t] += 1

        return acc, thetas

    # -------------------------------------------------
    # Vision angle extraction
    # -------------------------------------------------
    def get_angle(self, image):

        edges = CannyEdgeDetection.process(image)

        acc, thetas = self.hough(edges)

        idx = np.unravel_index(np.argmax(acc), acc.shape)
        _, theta_idx = idx

        theta = thetas[theta_idx]

        angle = np.degrees(theta) - 90

        # Print / log angle
        self.get_logger().info(f"Detected Angle: {angle:.2f}")

        # Publish angle topic
        msg = Float64()
        msg.data = float(angle)
        self.angle_pub.publish(msg)

        return angle

    # -------------------------------------------------
    # PID Controller
    # -------------------------------------------------
    def pid(self, error):

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -50.0, 50.0)

        derivative = (error - self.prev_error) / self.dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error

        return np.clip(output, -1.0, 1.0)

    # -------------------------------------------------
    # Main Loop
    # -------------------------------------------------
    def loop(self):

        try:
            frames = self.pipeline.wait_for_frames()
            frame = frames.get_color_frame()

            if not frame:
                return

            image = np.asanyarray(frame.get_data())

            angle_error = self.get_angle(image)

            turn = self.pid(angle_error)

            twist = Twist()
            twist.linear.x = 0.30
            twist.angular.z = float(turn)

            self.pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Loop error: {e}")


def main():
    rclpy.init()
    node = LineFollower()

    try:
        rclpy.spin(node)

    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()