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
        self.stop_dis    = 0.5   # stop if anything in front cone is within 0.5 m
        self.wall_target = 0.1   # desired distance from a single visible wall (m)
        self.hallway_dist_threshold_m = 10

        self.saw_blue_on_right = False

        # ── Centering gains ───────────────────────────────────────────────────
        self.forward_speed = 0.2  # m/s
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

        """

            Returns the angle indexes that contain center_rad
            
            Shouldn't happen but if the index lies outside valid indexes, maps to 0 or n-1
            
        """
    def _cone_indices(self, msg: LaserScan, center_rad: float, half_cone_rad: float):
        start_idx = int(((center_rad - half_cone_rad) - msg.angle_min) / msg.angle_increment)
        end_idx   = int(((center_rad + half_cone_rad) - msg.angle_min) / msg.angle_increment)
        n = len(msg.ranges)
        start_idx = max(0, min(start_idx, n - 1))
        end_idx   = max(0, min(end_idx,   n - 1))
        return start_idx, end_idx


        """
             Given a center_rad checks the endpoints of the cone that contains that 
             angle. Gets the valid range values for those endpoints. Valid meaning it's within 
             the rplidar scope. Returns the smallest range value. Otherwise just sends inf
        """
    def _valid_min(self, ranges_np: np.ndarray, msg: LaserScan,
                   center_rad: float, half_cone_rad: float) -> float:
        """Minimum valid range in a cone. Returns inf when no valid reading."""

        # get 
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & (np.isfinite(cone))]
        return float(np.min(valid)) if valid.size > 0 else float('inf')


        """ 
            From the endpoints of the cone that contains center_rad. Return the 
            median range. If there are other issues, return nan
        """
    def _valid_median(self, ranges_np: np.ndarray, msg: LaserScan,
                      center_rad: float, half_cone_rad: float) -> float:
        """Median valid range in a cone. Returns nan when no valid reading."""
        s, e  = self._cone_indices(msg, center_rad, half_cone_rad)
        cone  = ranges_np[s:e + 1]
        valid = cone[(cone > msg.range_min) & (np.isfinite(cone))]
        return float(np.median(valid)) if valid.size > 0 else float('nan')

    # ── Main callback ─────────────────────────────────────────────────────────



    def get_angle_from_indexes(self, msg: LaserScan, idxs):
        return msg.angle_min + msg.angle_increment*idxs
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
        left_boundary  = self._cone_indices(msg, -math.pi / 2, math.radians(22.5))
        left_boundary = range(left_boundary[0], left_boundary[1]+1)
        right_boundary = self._cone_indices(msg, math.pi / 2, math.radians(22.5))
        right_boundary = range(right_boundary[0], right_boundary[1]+1)


        # fit a line 
        left_angles = self.get_angle_from_indexes(msg, left_boundary)
        a = np.ones(len(left_angles), 3)
        a[:,0] = np.cos(left_angles)*ranges[left_boundary]
        a[:,1] = np.sin(left_angles)*ranges[left_boundary]
        left_c, left_residuals, _, _ = np.lstsq(a, np.zeros(len(left_angles)))


        if np.any(ranges(right_boundary) == np.inf):
            # time to move right
            print("Big distance turning")
            dir = 1
            twist.linear.x  = self.forward_speed
            twist.angular.z = dir
            self.vel_pub.publish(twist)
            return
        
        right_angles = self.get_angle_from_indexes(msg, right_boundary)
        a = np.ones(len(right_angles), 3)
        a[:,0] = np.cos(right_angles)*ranges[right_boundary]
        a[:,1] = np.sin(right_angles)*ranges[right_boundary]
        right_c, right_residuals, _, _ = np.lstsq(a, np.zeros(len(right_angles)))

        # significant error in mapping to a line

        error_vector = np.zeros(len(right_angles)) - np.dot(a, right_c)
        # the right is not mapping as well to a line - maybe time to move right
        if np.any(np.abs(error_vector) > 7):
            print("Error exceed max: turning")
            dir = 0.2
            twist.linear.x  = self.forward_speed
            twist.angular.z = dir
            self.vel_pub.publish(twist)
            return
        

        # otherwise we can model as between two lines




        # get vector oriented towards lines
        
        towards_left = np.arctan2(np.sign(left_c[2])*left_c[1], np.sign(left_c[2])*left_c[0]) + np.pi*2
        shortest_dist_left = np.abs(left_c[2])/np.sqrt(left_c[0]*left_c[0] + left_c[1]*left_c[1])

        towards_right = np.arctan2(np.sign(right_c[2])*right_c[1], np.sign(right_c[2])*right_c[0])
        shortest_dist_right = np.sqrt(right_c[0]*right_c[0] + right_c[1]*right_c[1])


        # the line is not that bad we want to align our heading with the right wall at some distance away
        # head to some point in the future


        # if our alignment is good and our distance is good

        if np.abs(towards_right) < math.radians(3) and shortest_dist_right > 1.5 and shortest_dist_left > 1.5:
            dir = 0.0
            twist.linear.x  = self.forward_speed
            twist.angular.z = dir
            self.vel_pub.publish(twist)
            return



        # try to get to 2.5 meters away along wall
        goal_pt = np.array([shortest_dist_right*np.cos(towards_right), shortest_dist_left*np.sin(towards_right)]) + 2.5*np.array([np.cos(towards_right+np.pi/2), np.sin(towards_right+np.pi/2)])
        if goal_pt[0] > 0:
            dir = 0.3
            if np.pi - np.arctan2(goal_pt[1], goal_pt[0]) > math.radians(25):
                dir = 1
        else:
            dir = -0.3
            if np.arctan2(goal_pt[1], goal_pt[0]) - np.pi > math.radians(25):
                dir = -1

        twist.linear.x  = self.forward_speed
        twist.angular.z = dir
        self.vel_pub.publish(twist)
        return


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
