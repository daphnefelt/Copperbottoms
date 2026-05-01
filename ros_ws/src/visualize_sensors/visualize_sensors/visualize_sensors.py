import rclpy
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import threading
"""
How to use:
Run this node to visualize sensor data.
Requires SLAM to be publishing /slam/pose.

Launch this node:
ros2 run visualize_sensors visualize_sensors

"""

class SensorVisualizationNode(Node):

    def __init__(self):
        super().__init__('Visualize_Sensors_node')

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam/pose',
            self._pose_cb,
            10)

        # Store latest values
        self.latest_heading = 0.0
        self.latest_velocity = 0.0
        self._prev_x = None
        self._prev_y = None
        self._prev_time = None

        # Setup matplotlib for live updating
        plt.ion()
        self.fig, (self.ax_compass, self.ax_vel) = plt.subplots(1, 2, figsize=(8, 4))
        self.arrow = None
        self.bar = None
        self._init_plots()

        # # Timer to update plots
        # self.timer = self.create_timer(0.1, self.update_plots)  # 10 Hz

    def _init_plots(self):
        # Compass plot
        self.ax_compass.clear()
        self.ax_compass.set_title('Heading (Compass)')
        self.ax_compass.set_xlim(-1.2, 1.2)
        self.ax_compass.set_ylim(-1.2, 1.2)
        self.ax_compass.set_aspect('equal')
        self.ax_compass.axis('off')
        # Draw compass circle
        circle = plt.Circle((0, 0), 1.0, color='black', fill=False)
        self.ax_compass.add_patch(circle)
        # Draw N/E/S/W
        self.ax_compass.text(0, 1.1, 'N', ha='center', va='center')
        self.ax_compass.text(1.1, 0, 'E', ha='center', va='center')
        self.ax_compass.text(0, -1.1, 'S', ha='center', va='center')
        self.ax_compass.text(-1.1, 0, 'W', ha='center', va='center')
        # Initial arrow
        self.arrow = self.ax_compass.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.15, fc='r', ec='r')

        # Velocity bar plot
        self.ax_vel.clear()
        self.ax_vel.set_title('Velocity (m/s)')
        self.ax_vel.set_ylim(0, 5)
        self.bar = self.ax_vel.bar(['Velocity'], [0], color='blue')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.latest_heading = 2.0 * math.atan2(q.z, q.w)

        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self._prev_x is not None and self._prev_time is not None:
            dt = t - self._prev_time
            if dt > 0:
                dist = math.hypot(x - self._prev_x, y - self._prev_y)
                self.latest_velocity = dist / dt
        self._prev_x = x
        self._prev_y = y
        self._prev_time = t

    def update_plots(self):
        x = np.sin(self.latest_heading)
        y = np.cos(self.latest_heading)
        if self.arrow:
            try:
                self.arrow.remove()
            except Exception:
                pass
        self.arrow = self.ax_compass.arrow(0, 0, x, y, head_width=0.1, head_length=0.15, fc='r', ec='r')
        self.bar[0].set_height(self.latest_velocity)
        self.fig.canvas.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SensorVisualizationNode()
        node.get_logger().info('Launching visualize_sensors node...')

        # Spin ROS in background thread
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        # All matplotlib drawing happens here on the main thread
        while rclpy.ok():
            node.update_plots()
            plt.pause(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
