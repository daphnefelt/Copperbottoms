## Brings in the canny edge detection to convert to an angle and magnitude for the line following node.

import pyrealsense2 as rs
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from collections import deque
from CannyEdgeDetection import CannyEdgeDetection

# RealSense setup
class Angle(Node):
    def __init__(self):
        super().__init__('angle_calculator')

        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.angle_pub = self.create_publisher(Float64, '/line_angle', 10)

        # RealSense setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.timer = self.create_timer(0.1, self.camera_callback)
    
    def get_angle(self, image):
        x, y = CannyEdgeDetection.angle_x, CannyEdgeDetection.angle_y
        angle = np.arctan2(y, x) * (180. / np.pi)
        return angle
    
    def hough_transform(self, edges):
        rows, cols = edges.shape

        thetas = np.deg2rad(np.arange(-90, 90))
        diag_len = int(np.sqrt(rows**2 + cols**2))
        rhos = np.arange(-diag_len, diag_len)

        accumulator = np.zeros((len(rhos), len(thetas)), dtype=np.uint64)

        y_idxs, x_idxs = np.nonzero(edges)

        for i in range(len(x_idxs)):
            x = x_idxs[i]
            y = y_idxs[i]

            for t_idx in range(len(thetas)):
                theta = thetas[t_idx]
                rho = int(x*np.cos(theta) + y*np.sin(theta)) + diag_len
                accumulator[rho, t_idx] += 1

        return accumulator, rhos, thetas
    
    def get_angle_from_hough(self, edges):
        acc, rhos, thetas = self.hough_transform(edges)

        # Find strongest vote
        idx = np.unravel_index(np.argmax(acc), acc.shape)
        rho_idx, theta_idx = idx

        theta = thetas[theta_idx]

        angle = np.degrees(theta)

        return angle
    
    def follow_line(self, angle):

        twist = Twist()
        twist.linear.x = 0.2

        if angle is None:
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # search turn
            return twist

        kp = 0.01
        twist.angular.z = -kp * angle

        # clamp
        twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)

        return twist
    
    def camera_callback(self):

        try:
            frames = self.pipeline.wait_for_frames()
            frame = frames.get_color_frame()

            if not frame:
                return

            image = np.asanyarray(frame.get_data())

            # 1. Canny (your custom module)
            edges = CannyEdgeDetection.process(image)

            # 2. Hough → angle
            angle = self.get_angle_from_hough(edges)

            # 3. Control
            twist = self.follow_line(angle)

            self.drive_pub.publish(twist)

            msg = Float64()
            msg.data = float(angle)
            self.angle_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

def main(args=None):
    rclpy.init(args=args)
    angle_calculator = Angle()

    try:
        rclpy.spin(angle_calculator)

    finally:
        angle_calculator.pipeline.stop()
        angle_calculator.destroy_node()
        rclpy.shutdown()
