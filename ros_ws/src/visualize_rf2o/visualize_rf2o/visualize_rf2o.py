# Matplot lib live graphics to detect speed #
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np

from geometry_msgs.msg import Twist
class SpeedVisualizer(Node):
    def __init__(self):
        super().__init__('speed_visualizer')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.speeds = []
        plt.ion()  # Interactive mode on
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.speeds, 'b-')
        self.ax.set_ylim(0, 2)  # Set y-axis limits for better visualization
        self.ax.set_xlabel('Time Steps')
        self.ax.set_ylabel('Linear Speed (m/s)')
        self.ax.set_title('Real-time Linear Speed Visualization')

    def cmd_vel_callback(self, msg: Twist):
        speed = msg.linear.x
        self.speeds.append(speed)
        if len(self.speeds) > 100:  # Keep only the last 100 speeds for visualization
            self.speeds.pop(0)
        self.update_plot()

    def update_plot(self):
        self.line.set_ydata(self.speeds)
        self.line.set_xdata(range(len(self.speeds)))
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    speed_visualizer = SpeedVisualizer()
    rclpy.spin(speed_visualizer)
    speed_visualizer.destroy_node()
    rclpy.shutdown()