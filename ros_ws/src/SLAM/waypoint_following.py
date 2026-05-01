#!/usr/bin/env python3

# state:  [x, y, θ, lx_0, ly_0, lx_1, ly_1, ...], where each (lx_j, ly_j) is the world position of that apriltag ID
# motion: differential drive (v, ω from /cmd_vel)
# obs: range + bearing to AprilTag landmarks from /landmarks
# Output: /slam/pose (PoseWithCovarianceStamped)
#         /slam/landmarks (JSON: [{id, x, y}, ...]) # locations of all known landmarks in world frame

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String
import cv2
import os
os.environ.setdefault('QT_LOGGING_RULES', '*.warning=false') # stupid font warnings
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from enum import Enum


class HALL(Enum):
    ONE = 1
    TWO = 2
    THREE = 3


# MARY - maybe need to change this later
CENTER = np.array([16, 1.5])
HALLWAY_WIDTH = 3.4
MAX_X_DIST = 40 
MAX_Y_DIST = 40
# measured good estimate
HALLWAY_THREE_WIDTH = 2.79
HALLWAY_TWO_DIST = 25

# not measured
HALLWAY_FOUR_WIDTH = HALLWAY_THREE_WIDTH
HALLWAY_THREE_DIST = MAX_X_DIST - HALLWAY_THREE_WIDTH

def wrap(a: float) -> float: # wraps to -pi, +pi
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('ekf_slam')

        # state
        #self.mu: np.ndarray = np.array([16.0, 1.5, math.pi]) # No landmarks yet, so just robot pose (approximating the start here)
        self.mu: np.ndarray = np.array([0.0, 0.0, math.pi]) # No landmarks yet, so just robot pose (approximating the start here)
        self.landmarks = {}

        self.Sigma: np.ndarray = np.zeros((3, 3)) # Pose known at start, so zero covariance
        # maps tag_id (int) to index j so landmark is at mu[3+2j : 3+2j+2]
        self.lm_index: dict[int, int] = {}

        # last velocity cmds
        self.v = 0.0
        self.delta = 0.0
        self.last_time = self.get_clock().now()

        # lidar occupancy map
        self.lidar_grid = np.zeros((LIDAR_GRID_SIZE, LIDAR_GRID_SIZE), dtype=np.int32)

        self.pose_pub = self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self._update_pos, 10)
        self.map_pub = self.create_subscription(MarkerArray, '/slam/landmarks', self._update_landmark_pos, 10)
        self.lidar_map_pub = self.create_subscription(OccupancyGrid, '/slam/lidar_map', self._update_occupancy_grid, 10)
        self.create_publisher(Twist, '/cmd_vel',   self._cmd_vel_cb,   10)
        self.create_subscription(LaserScan, '/scan', self._check_for_recovery, 10)
        self.create_timer(0.02, self.loop) # make a decision at 50hz

        # ordered
        self.waypoints = np.array([[]])
        self.idx = 0
        self.get_logger().info('SLAM node is up')

    def _check_for_recovery(self, msg: LaserScan):
        pass
    
    # MARY - will need to get accurate measurements
    def _get_current_hallway(self):
        x = self.mu[0] - CENTER[0]
        y = self.mu[1] - CENTER[1] 
        LEFT_NEAR = -MAX_X_DIST/2 + HALLWAY_WIDTH
        TOP_NEAR = -HALLWAY_WIDTH/2 + HALLWAY_TWO_DIST
        if x > LEFT_NEAR  and y < HALLWAY_WIDTH/2:
            return HALL.ONE 
        
        elif x < LEFT_NEAR and y < HALLWAY_TWO_DIST:
            return HALL.TWO 
        
        elif y >= HALLWAY_TWO_DIST and x < HALLWAY_THREE_DIST:

        
        pass
        

    def _update_pos(self, msg: PoseWithCovarianceStamped):
        self.mu[0] = msg.pose.pose.position.x 
        self.mu[1] = msg.pose.pose.position.y
        # unpack quarternion value
        self.mu[2] = math.asin(msg.pose.pose.orientation.z) * 2.0

    def _update_landmark_pos(self, msg: MarkerArray):
        for marker in MarkerArray:
            id = marker.id
            self.landmarks[id] = np.array([marker.pose.position.x, marker.pos.position.y])
    def _update_occupancy_grid(self, msg: OccupancyGrid):
        pass

    def loop(self):
        # check whether we can get to point without excessive turning
        # otherwise go to next point
        # if there is a turn to the next point see whether turn is feasible
        waypoint = self.waypoints[self.idx]
        
        # at max turning
        SMALLEST_TURN_RADIUS = 1.05/2


        
        
        pass




def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node     )
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()