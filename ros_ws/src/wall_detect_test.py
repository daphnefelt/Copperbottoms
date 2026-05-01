import rclpy
fom rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

    def scan_callback(self, msg):
        min_distance = float('inf')

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Only check front ±15 degrees
            cone = math.radians(15)
            if -cone < angle < cone:
                if not math.isinf(distance):
                    min_distance = min(min_distance, distance)
        if min_distance < 1.0:
            self.get_logger().warn("Stopping: Obstacle within 1 meter")
