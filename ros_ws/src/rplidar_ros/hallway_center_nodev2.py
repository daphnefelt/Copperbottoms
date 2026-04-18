#!/usr/bin/env python3
"""
Hallway centering node with integrated obstacle stop + recovery.

Three lidar cones:
  FRONT  ±15°   around   0°  -> stop if anything within 0.5 m
  LEFT   ±22.5° around  90°  -> left wall distance for centering
  RIGHT  ±22.5° around -90°  -> right wall distance for centering

Behaviour is a small state machine:
  CENTERING   -> drive forward, keep equidistant from walls
  BACKING_UP  -> obstacle hit, reverse for self.backup_time seconds
  TURNING     -> rotate toward the more open side until front is clear

Published topics:
  /cmd_vel    geometry_msgs/Twist
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class HallwayCenterNode(Node):

    def __init__(self):
        super().__init__('hallway_center_node')

        # -- Cone half-widths -------------------------------------------------
        self.front_half_cone = math.radians(15)    # +/-15  front
        self.side_half_cone  = math.radians(22.5)  # +/-22.5 each side

        # -- Distances --------------------------------------------------------
        self.stop_dis        = 0.5   # stop if front cone reading <= this (m)
        self.clear_dis       = 0.7   # must exceed this to resume centering (m)
        self.wall_target     = 0.8   # desired distance from a single wall (m)

        # -- Motion parameters ------------------------------------------------
        self.forward_speed   = 0.2   # m/s
        self.backup_speed    = 0.15  # m/s (magnitude; applied as negative)
        self.backup_time     = 1.0   # seconds to reverse after a hit
        self.kp              = 0.6   # proportional steering gain
        self.max_angular_z   = 1.2   # rad/s clamp (was 10 -- way too fast)
        self.turn_speed      = 0.8   # rad/s while actively turning away

        # -- State machine ----------------------------------------------------
        self.MODE_CENTERING  = 'CENTERING'
        self.MODE_BACKING_UP = 'BACKING_UP'
        self.MODE_TURNING    = 'TURNING'
        self.mode            = self.MODE_CENTERING
        self.mode_start_time = 0.0
        self.turn_dir        = 1.0   # +1 = left, -1 = right

        # -- Startup delay ----------------------------------------------------
        self.ready = False
        self.create_timer(3.0, self.set_ready)

        # -- Pub / sub --------------------------------------------------------
        self.vel_pub      = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info(
            'Hallway center node started. Waiting 3 seconds for lidar to stabilize...')

    def set_ready(self):
        self.ready = True
        self.get_logger().info('Lidar stabilized. Starting hallway centering.')

    # -- Index / range helpers ------------------------------------------------

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

    # -- Mode transitions -----------------------------------------------------

    def _enter_mode(self, new_mode: str):
        self.mode = new_mode
        self.mode_start_time = time.time()

    def _pick_turn_direction(self, ranges: np.ndarray, msg: LaserScan) -> float:
        """Return +1 (left) or -1 (right) based on which side is more open.
        Treats NaN (nothing seen) as 'very open' so we prefer unseen sides."""
        left_dist  = self._valid_median(ranges, msg,  math.pi / 2, self.side_half_cone)
        right_dist = self._valid_median(ranges, msg, -math.pi / 2, self.side_half_cone)
        l = left_dist  if not math.isnan(left_dist)  else float('inf')
        r = right_dist if not math.isnan(right_dist) else float('inf')
        return 1.0 if l >= r else -1.0

    # -- Main callback --------------------------------------------------------

    def scan_callback(self, msg: LaserScan):
        if not self.ready:
            return

        ranges    = np.array(msg.ranges)
        front_min = self._valid_min(ranges, msg, 0.0, self.front_half_cone)
        now       = time.time()

        # ------------------------------------------------------------------
        # BACKING_UP: reverse in place for a fixed duration, then pick a
        # turn direction and switch to TURNING.
        # ------------------------------------------------------------------
        if self.mode == self.MODE_BACKING_UP:
            if now - self.mode_start_time < self.backup_time:
                twist = Twist()
                twist.linear.x = -self.backup_speed
                self.vel_pub.publish(twist)
                self.get_logger().info(
                    f'[BACKING_UP] front: {front_min:.2f} m',
                    throttle_duration_sec=0.5)
                return

            # Done reversing -> choose turn direction
            self.turn_dir = self._pick_turn_direction(ranges, msg)
            self._enter_mode(self.MODE_TURNING)
            self.get_logger().info(
                f'Backup complete. Turning {"left" if self.turn_dir > 0 else "right"}.')
            # fall through to TURNING this cycle

        # ------------------------------------------------------------------
        # TURNING: rotate in place toward the open side until the front
        # cone is clearly clear (hysteresis: clear_dis > stop_dis).
        # ------------------------------------------------------------------
        if self.mode == self.MODE_TURNING:
            if front_min >= self.clear_dis:
                self._enter_mode(self.MODE_CENTERING)
                self.vel_pub.publish(Twist())   # brief stop before resuming
                self.get_logger().info('Front clear. Resuming centering.')
                return

            twist = Twist()
            twist.angular.z = self.turn_dir * self.turn_speed
            self.vel_pub.publish(twist)
            self.get_logger().info(
                f'[TURNING {("L" if self.turn_dir > 0 else "R")}] '
                f'front: {front_min:.2f} m',
                throttle_duration_sec=0.5)
            return

        # ------------------------------------------------------------------
        # CENTERING: normal wall-following. If we see an obstacle, switch
        # to BACKING_UP on the next cycle.
        # ------------------------------------------------------------------
        if front_min <= self.stop_dis:
            self.get_logger().warn(
                f'Obstacle at {front_min:.2f} m -- backing up.')
            self.vel_pub.publish(Twist())   # immediate stop this cycle
            self._enter_mode(self.MODE_BACKING_UP)
            return

        # -- Side wall sampling -----------------------------------------------
        left_dist  = self._valid_median(ranges, msg,  math.pi / 2, self.side_half_cone)
        right_dist = self._valid_median(ranges, msg, -math.pi / 2, self.side_half_cone)

        # -- Steering correction ----------------------------------------------
        if math.isnan(left_dist) and math.isnan(right_dist):
            error = 0.0
        elif math.isnan(left_dist):
            error = -(right_dist - self.wall_target)
        elif math.isnan(right_dist):
            error = left_dist - self.wall_target
        else:
            error = (left_dist - right_dist) / 2.0

        # positive error -> too close to left wall -> steer right (negative w)
        # Note: sign follows from how error is defined above; if your lidar
        # frame is flipped, invert the sign here.
        angular_z = float(np.clip(self.kp * error,
                                  -self.max_angular_z, self.max_angular_z))

        # -- Readout ----------------------------------------------------------
        left_str  = f'{left_dist:.2f} m'  if not math.isnan(left_dist)  else '  --  '
        right_str = f'{right_dist:.2f} m' if not math.isnan(right_dist) else '  --  '

        if angular_z > -0.05:
            direction = f'turning left  (w={angular_z:+.2f})'
        elif angular_z < 0.05:
            direction = f'turning right (w={angular_z:+.2f})'
        else:
            direction = 'straight'

        self.get_logger().info(
            f'left: {left_str} | front: {front_min:.2f} m | '
            f'right: {right_str} | {direction}',
            throttle_duration_sec=0.5)

        twist = Twist()
        twist.linear.x  = self.forward_speed
        twist.angular.z = angular_z
        self.vel_pub.publish(twist)


# -- Entry point --------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HallwayCenterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.vel_pub.publish(Twist())   # zero-stop on shutdown
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
