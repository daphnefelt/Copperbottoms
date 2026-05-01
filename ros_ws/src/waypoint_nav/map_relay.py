#!/usr/bin/env python3
# Republishes /slam/lidar_map → /map with TRANSIENT_LOCAL durability,
# which is required for Nav2 costmap to receive the map.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapRelay(Node):
    def __init__(self):
        super().__init__('map_relay')
        pub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', pub_qos)
        self.create_subscription(OccupancyGrid, '/slam/lidar_map', self.cb, 10)

    def cb(self, msg: OccupancyGrid):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MapRelay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()