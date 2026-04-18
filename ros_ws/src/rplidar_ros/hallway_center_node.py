#!/usr/bin/env python3
"""
Hallway centering node with integrated obstacle stop.

Three lidar cones:
  FRONT  ±15°   around   0°  → stop if anything within 0.5 m
  LEFT   ±22.5° around  90°  → left wall distance for centering
  RIGHT  ±22.5° around -90°  → right wall distance for centering

Published topics:
  /cmd_vel    geometry_msgs/Twist
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class HallwayCenterNode(Node):

    def __init__(self):
        super().__init__('hallway_center_node')

        # ── Cone half-widths ─────────────────────────────────────────────────
        self.front_half_cone = math.radians(15)    # ±15°  front
        self.side_half_cone  = math.radians(22.5)  # ±22.5° each side (45° total)

        # ── Distances ────────────────────────────────────────────────────────
        self.stop_dis    = 1   # stop if anything in front cone is within 0.5 m
        self.wall_target = 0.1   # desired distance from a single visible wall (m)

        # ── Centering gains ───────────────────────────────────────────────────
        self.forward_speed = 0.25   # m/s
        self.kp            = 0.6  # proportional steering gain
        self.max_angular_z = 10   # rad/s clamp

        # ── Startup delay ────────────────────────────────────────────────────
        self.ready = False
        self.create_timer(3.0, self.set_ready)

        # ── Publisher / subscriber ───────────────────────────────────────────
        self.vel_pub      = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info('Hallway center node started. Waiting 3 seconds for lidar to stabilize...')

    def set_ready(self):
        self.ready = True
        self.get_logger().info('Lidar stabilized. Starting hallway centering.')

    # ── Index helpers ─────────────────────────────────────────────────────────

    def _cone_indices(self, msg: LaserScan, center_rad: float, half_cone_rad: float):
        start_idx = int(((center_rad - half_cone_rad) - msg.angle_min) / msg.angle_increment)
        end_idx   = int(((center_rad + half_cone_rad) - msg.angle_min) / msg.angle_increment)
        n = len(msg.ranges)
        start_idx = max(0, min(start_idx, n - 1))
        end_idx   = max(0, min(end_idx,   n - 1))
        return start_idx, end_idx

    def _valid_min(self, ranges_np: np.ndarray, msg: LaserScan,
                   center_rad: float, half_cone_rad: float) -> float:
        """Minimum valid range in a cone. Returns inf when no valid reading."""
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & (np.isfinite(cone))]
        return float(np.min(valid)) if valid.size > 0 else float('inf')

    def _valid_median(self, ranges_np: np.ndarray, msg: LaserScan,
                      center_rad: float, half_cone_rad: float) -> float:
        """Median valid range in a cone. Returns nan when no valid reading."""
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & (np.isfinite(cone))]
        return float(np.median(valid)) if valid.size > 0 else float('nan')

    # ── Main callback ─────────────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):

        # ── Startup guard ─────────────────────────────────────────────────────
        if not self.ready:
            return

        ranges = np.array(msg.ranges)
        twist  = Twist()

        # ── 1. Front obstacle check ───────────────────────────────────────────
        front_min = self._valid_min(ranges, msg, 0.0, self.front_half_cone)

        if front_min <= self.stop_dis:
            self.get_logger().warn(
                f'Obstacle at {front_min:.2f} m — stopping.',
                throttle_duration_sec=1.0)
            self.vel_pub.publish(twist)   # all-zero Twist = full stop
            return

        # ── 2. Side wall sampling ─────────────────────────────────────────────
        left_dist  = self._valid_median(ranges, msg,  math.pi / 2, self.side_half_cone)
        right_dist = self._valid_median(ranges, msg, -math.pi / 2, self.side_half_cone)

        # ── 3. Steering correction ────────────────────────────────────────────
        if math.isnan(left_dist) and math.isnan(right_dist):
            error = 0.0
        elif math.isnan(left_dist):
            error = -(right_dist - self.wall_target)
        elif math.isnan(right_dist):
            error = left_dist - self.wall_target
        else:
            error = (left_dist - right_dist) / 2.0

        # positive error → too close to left → steer right (negative angular.z)
        angular_z = float(np.clip(self.kp * error, -self.max_angular_z, self.max_angular_z))

        # ── Readout ───────────────────────────────────────────────────────────
        left_str  = f'{left_dist:.2f} m'  if not math.isnan(left_dist)  else '  --  '
        right_str = f'{right_dist:.2f} m' if not math.isnan(right_dist) else '  --  '

        if angular_z > -0.05:
            direction = f'turning left  (ω={angular_z:+.2f})'
        elif angular_z < 0.05:
            direction = f'turning right (ω={angular_z:+.2f})'
        else:
            direction = 'straight'

        self.get_logger().info(
            f'left: {left_str} | front: {front_min:.2f} m | right: {right_str} | {direction}',
            throttle_duration_sec=0.5)

        twist.linear.x  = self.forward_speed
        twist.angular.z = angular_z
        self.vel_pub.publish(twist)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    try:
        node = HallwayCenterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.vel_pub.publish(Twist())   # zero-stop on shutdown
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
