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
    
    def follow_line(self, angle):
        # Simple proportional controller for line following
        twist = Twist()
        twist.linear.x = 0.2  # Constant forward speed
        twist.angular.z = -0.01 * angle  # Proportional control for steering
        return twist
    
    def camera_callback(self):
        try:
            frames = self.pipeline.wait_for_frames()
            frame = frames.get_color_frame()

            if not frame:
                return

            image = np.asanyarray(frame.get_data())
            angle = self.get_angle(image)
            twist = self.follow_line(angle)
            self.drive_pub.publish(twist.linear.x, twist.angular.z)
            msg = Float64()
            msg.data = angle
            self.angle_pub.publish(msg)

        finally:
            self.pipeline.stop()


def main(args=None):
    rclpy.init(args=args)
    angle_calculator = Angle()
    rclpy.spin(angle_calculator)
    angle_calculator.destroy_node()
    rclpy.shutdown()
