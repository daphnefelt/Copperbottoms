#!/usr/bin/env python3
"""
TurnRightNode

Reads LiDAR and publishes a Bool to /turn_right.

  True  — average distance in the right cone (-90 ± 10 deg) > open_thresh
           → opening on the right, robot should turn right
  False — wall is present on the right
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class TurnRightNode(Node):

    def __init__(self):
        super().__init__('turn_right_node')

        # ── params ───────────────────────────────────────────────────────
        self.cone_center   = math.radians(-90.0)   # right side
        self.cone_half     = math.radians(10.0)    # ±10 deg
        self.open_thresh   = 6.0                   # m — avg above this → True

        # ── ROS ──────────────────────────────────────────────────────────
        self.pub = self.create_publisher(Bool, '/turn_right', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info('TurnRightNode started — publishing to /turn_right')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)  # inf → max range

        # cone indices
        lo = int(((self.cone_center - self.cone_half) - msg.angle_min) / msg.angle_increment)
        hi = int(((self.cone_center + self.cone_half) - msg.angle_min) / msg.angle_increment)
        n  = len(ranges)
        lo = max(0, min(lo, n - 1))
        hi = max(0, min(hi, n - 1))

        cone  = ranges[lo:hi + 1]
        valid = cone[np.isfinite(cone) & (cone > msg.range_min)]

        if valid.size == 0:
            avg       = 0.0
            turn_right = False  # no valid readings → assume wall (be conservative)
        else:
            avg        = float(np.mean(valid))
            turn_right = avg > self.open_thresh

        out = Bool()
        out.data = turn_right
        self.pub.publish(out)

        self.get_logger().info(
            f'right cone avg={avg:.2f}m  turn_right={turn_right}',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TurnRightNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
