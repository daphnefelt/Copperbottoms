import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import matplotlib.pyplot as plt

class SensorVisualizationNode(Node):
    
    def __init__(self):
        super().__init__('sensor_visualization_node')

        # imu_heading estimate
        self.heading_sub = self.create_subscription(
            Float32, 
            'imu/heading_estimate',      
            self.heading_estimate_callback, 
            10)  

        # imu_velocity estimate
        self.velocity_sub = self.create_subscription(
            Float32, 
            'imu/velocity_estimate',      
            self.velocity_estimate_callback, 
            10)  

        # Store latest values
        self.latest_heading = 0.0
        self.latest_velocity = 0.0

        # Setup matplotlib for live updating
        plt.ion()
        self.fig, (self.ax_compass, self.ax_vel) = plt.subplots(1, 2, figsize=(8, 4))
        self.arrow = None
        self.bar = None
        self._init_plots()

        # Timer to update plots
        self.timer = self.create_timer(0.1, self.update_plots)  # 10 Hz

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

    def heading_estimate_callback(self, msg):
        self.latest_heading = msg.data
        self.get_logger().info(f"Heading Estimate: {msg.data:.2f} radians")

    def velocity_estimate_callback(self, msg):
        self.latest_velocity = msg.data
        self.get_logger().info(f"Velocity Estimate: {msg.data:.2f} m/s")

    def update_plots(self):
        # Update compass arrow
        x = np.sin(self.latest_heading)
        y = np.cos(self.latest_heading)
        if self.arrow:
            try:
                self.arrow.remove()
            except Exception:
                pass
        self.arrow = self.ax_compass.arrow(0, 0, x, y, head_width=0.1, head_length=0.15, fc='r', ec='r')

        # Update velocity bar
        self.bar[0].set_height(self.latest_velocity)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SensorVisualizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
