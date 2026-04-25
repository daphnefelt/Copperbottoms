import pyrealsense2 as rs
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from CannyEdgeDetection import CannyEdgeDetection


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0

        self.kp = 0.25
        self.ki = 0.01
        self.kd = 0.05

        self.dt = 0.1

        # RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.timer = self.create_timer(0.1, self.loop)

    # ---------------- Canny + Hough ----------------
    def hough(self, edges):

        rows, cols = edges.shape

        thetas = np.deg2rad(np.arange(-90, 90))
        diag = int(np.sqrt(rows**2 + cols**2))
        rhos = np.arange(-diag, diag)

        acc = np.zeros((len(rhos), len(thetas)))

        y_idxs, x_idxs = np.nonzero(edges)

        for i in range(len(x_idxs)):
            x = x_idxs[i]
            y = y_idxs[i]

            for t in range(len(thetas)):
                rho = int(x*np.cos(thetas[t]) + y*np.sin(thetas[t])) + diag
                acc[rho, t] += 1

        return acc, thetas

    def get_angle(self, image):

        edges = CannyEdgeDetection.process(image)

        acc, thetas = self.hough(edges)

        idx = np.unravel_index(np.argmax(acc), acc.shape)
        _, theta_idx = idx

        theta = thetas[theta_idx]

        # convert to robot error
        angle = np.degrees(theta) - 90

        return angle

    # ---------------- PID ----------------
    def pid(self, error):

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -50, 50)

        derivative = (error - self.prev_error) / self.dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error

        return np.clip(output, -1.0, 1.0)

    # ---------------- Main loop ----------------
    def loop(self):

        frames = self.pipeline.wait_for_frames()
        frame = frames.get_color_frame()

        if not frame:
            return

        image = np.asanyarray(frame.get_data())

        angle_error = self.get_angle(image)

        turn = self.pid(angle_error)

        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = turn

        self.pub.publish(twist)


def main():
    rclpy.init()
    node = LineFollower()

    try:
        rclpy.spin(node)
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()