#!/usr/bin/env python3

# state:  [x, y, θ, lx_0, ly_0, lx_1, ly_1, ...], where each (lx_j, ly_j) is the world position of that apriltag ID
# motion: ackermann steering (v, steering_angle from /cmd_vel)
# obs: range + bearing to AprilTag landmarks from /landmarks
#      pose from rf2o odometry
# Output: /slam/pose (PoseWithCovarianceStamped)
#         /slam/landmarks # locations of all known landmarks in world frame
#         /slam/lidar_map # occupancy grid map built from lidar scans in world frame

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
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import time
import signal
import time
from enum import Enum

# NOISE PARAMS
MOTION_NOISE = np.diag([100000**2, 100000**2, np.deg2rad(100000.0)**2]) # process noise on robot pose
OBS_NOISE = np.diag([0.1**2, np.deg2rad(5.0)**2]) # measurement noise for range (m) and bearing (rad)
# OBS_NOISE = np.diag([100000**2, np.deg2rad(100000.0)**2])
INIT_LM_COV = 1000.0 # init covariance for a new landmark

# RF2O_NOISE = np.diag([1e-8, 1e-8, 1e-8])  # rf2o scan-match uncertainty
RF2O_NOISE = np.diag([0.05**2, 0.05**2, 1e-8])  # rf2o scan-match uncertainty

# Params for global lidar occupancy grid mapping
LIDAR_GRID_RES = 0.05 # m per cell
LIDAR_GRID_SIZE = 1600 # cells per side (so 40m x 40m)
LIDAR_GRID_ORIGIN = (-40.0, -40.0) # world coords of cell (0, 0)

def wrap(a: float) -> float: # wraps to -pi, +pi
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class HALL(Enum):
    ONE = 1
    TWO = 2 
    THREE = 3
    FOUR = 4

class EKFSlamNode(Node):
    def __init__(self):
        super().__init__('ekf_slam')

        # state
        self.mu: np.ndarray = np.array([16.0, 1.5, math.pi]) # No landmarks yet, so just robot pose (approximating the start here)
        self.mu_prev: np.ndarray = self.mu.copy()
        #self.mu: np.ndarray = np.array([0.0, 0.0, math.pi]) # No landmarks yet, so just robot pose (approximating the start here)

        self.Sigma: np.ndarray = np.zeros((3, 3)) # Pose known at start, so zero covariance
        # maps tag_id (int) to index j so landmark is at mu[3+2j : 3+2j+2]
        self.lm_index: dict[int, int] = {}

        # last velocity cmds
        self.v = 0.0
        self.delta = 0.0
        self.last_time = self.get_clock().now()

        # track last rf2o pose so we can get relative change
        self._rf2o_prev = None
        self.last_rf2o_time = self.get_clock().now()
        
        # track times between rf2o updates to see if it's running at a reasonable rate and not stalling
        self.rf20_update_times = []

        # lidar occupancy grid
        self.lidar_grid = np.zeros((LIDAR_GRID_SIZE, LIDAR_GRID_SIZE), dtype=np.int32)

        # pose history for trace visualization
        self.pose_history = []
        self.pose_marker_id = 100  # avoid collision with landmark marker IDs (they should only go up to 20)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb,  10)
        self.create_subscription(String, '/landmarks', self._landmark_cb, 10)
        self.create_subscription(Odometry, '/odom_rf2o', self._rf2o_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._lidar_cb, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/slam/pose', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/slam/landmarks', 10)
        self.lidar_map_pub = self.create_publisher(OccupancyGrid, '/slam/lidar_map', 10)
        self.pose_history_pub = self.create_publisher(MarkerArray, '/slam/pose_history', 10)
        self.create_timer(0.01, self._predict_step) # predict at 20hz

        self.get_logger().info('SLAM node is up')

        self.hallway = HALL.ONE

        # cv2.namedWindow('Lines', cv2.WINDOW_AUTOSIZE)
        # self.debug_img_counter = 0

    @property
    def n_lm(self) -> int:
        return len(self.lm_index)

    @property
    def state_size(self) -> int:
        return 3 + 2 * self.n_lm

    def _lm_slice(self, tag_id: int) -> slice: # index slice for just that landmark in the state vector
        j = self.lm_index[tag_id]
        return slice(3 + 2 * j, 3 + 2 * j + 2)

    # MOTION MODEL

    # tuning for the meaning of linear.x and angular.z
    def V_NOMINAL(self, linear_x):
        return 2.0555*linear_x - 0.1072 # m/s for a given linear.x cmd
    def DELTA_NOMINAL(self, angular_z):
        abs_angle = abs(angular_z)
        pos_angle = math.radians(5.3505*abs_angle**2 + 7.2598*abs_angle - 0.2384) * 2 # DOUBLE THAT SHIT
        return pos_angle if angular_z >= 0 else -pos_angle

    def _cmd_vel_cb(self, msg: Twist):
        self._predict_step() # predict up to now before changing velocity
        self.v = self.V_NOMINAL(msg.linear.x)
        self.delta = self.DELTA_NOMINAL(msg.angular.z)
        print(f"self. delta is {self.delta}")
        if abs(self.v) < 0.15: # doesn't turn or do anything below this speed
            self.v = 0.0
            self.delta = 0.0

    # vehicle measurements in meters
    FRONT_WIDTH = 0.165
    REAR_WIDTH = 0.165
    BACK_TO_FRONT = 0.165
    def _predict_step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9 # time that command has been running for
        self.last_time = now

        if dt <= 0.0 or dt > 1.0: # startup might be over a second, don't count. Otherwise this runs every 50ms
            return

        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        v = self.v
        w = v * math.tan(self.delta) / self.BACK_TO_FRONT # Ackermann yaw rate

        # Differential drive motion model w const velocity over dt
        th_mid = th + 0.5 * w * dt # midpoint heading for x and y update
        self.mu[0] = x + v * dt * math.cos(th_mid)
        self.mu[1] = y + v * dt * math.sin(th_mid)
        self.mu[2] = wrap(th + w * dt)

        # How uncertainty propagates, this is the partial derivative of the motion model
        Gx = np.array([
            [1.0, 0.0, -v * dt * math.sin(th_mid)],
            [0.0, 1.0,  v * dt * math.cos(th_mid)],
            [0.0, 0.0,  1.0],
        ])
        n = self.state_size
        G = np.eye(n) # Full G is Gx for robot pose and I for landmarks
        G[:3, :3] = Gx
        Q = np.zeros((n, n)) # Q is the motion noise covar on robot pose, no noise on landmarks (since they're not moving)
        Q[:3, :3] = MOTION_NOISE
        self.Sigma = G @ self.Sigma @ G.T + Q

        self._publish_pose() # Best estimate of robot pose
        self.get_logger().info(f'ROBOT POSE: x={self.mu[0]:.2f} y={self.mu[1]:.2f} th={math.degrees(self.mu[2]):.1f} deg')

    # MEASUREMENT MODEL

    def _landmark_cb(self, msg: String):
        detections = json.loads(msg.data)
        for det in detections: # each apriltag seen
            tag_id = int(det['id'])
            r_obs = float(det['depth'])
            phi_obs = -float(det['angle']) # ccw + sign notation

            if r_obs <= 0.0: # something wrong w the depth
                continue
            if tag_id not in self.lm_index: # never before seen landmark
                self._init_landmark(tag_id, r_obs, phi_obs)
            self._ekf_update(tag_id, r_obs, phi_obs)
        self._publish_map() # Publishes all the landmark locations

    def _init_landmark(self, tag_id: int, r: float, phi: float):
        n_old = self.state_size

        # calc where landmark is in world frame
        print(f"Initialising landmark {tag_id} at range {r:.2f} and bearing {math.degrees(phi):.1f} deg")
        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        lx = x + r * math.cos(th + phi)
        ly = y + r * math.sin(th + phi)

        # next available landmark index
        j = self.n_lm
        self.lm_index[tag_id] = j # map tag_id to this index

        # add landmark position to state mean
        self.mu = np.append(self.mu, [lx, ly])

        # add landmark to covariance
        n_new = self.state_size
        Sigma_new = np.zeros((n_new, n_new))
        Sigma_new[:n_old, :n_old] = self.Sigma
        Sigma_new[n_old:, n_old:] = np.eye(2) * INIT_LM_COV # no correlation w existing state so off-diagonal blocks are zero
        self.Sigma = Sigma_new

        self.get_logger().info(f'Initialised landmark tag={tag_id}  world=({lx:.2f}, {ly:.2f})')

    def _ekf_update(self, tag_id: int, r_obs: float, phi_obs: float):
        n = self.state_size
        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        sl = self._lm_slice(tag_id)
        lx, ly = self.mu[sl]

        # Where we think the landmark should have been seen
        dx = lx - x
        dy = ly - y
        q  = dx**2 + dy**2
        r_pred = math.sqrt(q)
        phi_pred = wrap(math.atan2(dy, dx) - th)

        # diff in predicted vs observed
        z_diff = np.array([r_obs - r_pred, wrap(phi_obs - phi_pred)])

        # Observation Jacobian  H
        H = np.zeros((2, n))
        # pose part of H
        H[0, 0] = -dx / r_pred
        H[0, 1] = -dy / r_pred
        H[0, 2] = 0.0
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0
        # landmark part of H
        j0 = sl.start
        H[0, j0] = dx / r_pred
        H[0, j0 + 1] = dy / r_pred
        H[1, j0] = -dy / q
        H[1, j0 + 1] = dx / q

        # Kalman gain
        S = H @ self.Sigma @ H.T + OBS_NOISE
        K = self.Sigma @ H.T @ np.linalg.inv(S)

        # Update
        self.mu += K @ z_diff
        self.mu[2] = wrap(self.mu[2])
        self.Sigma = (np.eye(n) - K @ H) @ self.Sigma

        self._publish_pose() # Best estimate of robot pose
        self.get_logger().info(f'LANDMARK UPDATE: tag={tag_id}  x={self.mu[0]:.2f} y={self.mu[1]:.2f} th={math.degrees(self.mu[2]):.1f} deg')

    def _rf2o_cb(self, msg: Odometry):
        print(f"time since last rf2o update: {(self.get_clock().now() - self.last_rf2o_time).nanoseconds * 1e-9:.2f} seconds")
        self.rf20_update_times.append((self.get_clock().now() - self.last_rf2o_time).nanoseconds * 1e-9)

        # get pose from rf2o odometry message (it's in mf quaternions)
        x_rf2o = msg.pose.pose.position.x
        y_rf2o = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        th_rf2o = 2.0 * math.atan2(qz, qw)

        if self._rf2o_prev is None: # first time running this
            self._rf2o_prev = (x_rf2o, y_rf2o, th_rf2o)
            return

        px, py, pth = self._rf2o_prev
        self._rf2o_prev = (x_rf2o, y_rf2o, th_rf2o)

        # get local change from rf2o since last update
        dx_odom = x_rf2o - px
        dy_odom = y_rf2o - py
        dth = wrap(th_rf2o - pth)
        c, s = math.cos(pth), math.sin(pth)
        dx_local =  c * dx_odom + s * dy_odom
        dy_local = -s * dx_odom + c * dy_odom

        # Project local change into world frame using EKF heading estimate
        th_ekf = self.mu[2]
        c2, s2 = math.cos(th_ekf), math.sin(th_ekf)
        x_sm  = self.mu[0] + c2 * dx_local - s2 * dy_local
        y_sm  = self.mu[1] + s2 * dx_local + c2 * dy_local
        th_sm = wrap(th_ekf + dth)

        # EKF update on direct pose observation
        n = self.state_size
        H = np.zeros((3, n))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0

        z_obs = np.array([x_sm, y_sm, th_sm])
        z_pred = np.array([self.mu[0], self.mu[1], self.mu[2]])
        z_diff = z_obs - z_pred
        z_diff[2] = wrap(z_diff[2])

        S = H @ self.Sigma @ H.T + RF2O_NOISE
        K = self.Sigma @ H.T @ np.linalg.inv(S)

        self.mu += K @ z_diff
        self.mu[2] = wrap(self.mu[2])
        self.Sigma = (np.eye(n) - K @ H) @ self.Sigma

        self._publish_pose() # Best estimate of robot pose
        self.last_rf2o_time = self.get_clock().now()
        self.get_logger().info(f'RF2O UPDATE: x={self.mu[0]:.2f} y={self.mu[1]:.2f} th={math.degrees(self.mu[2]):.1f} deg')

    def _world_to_cell(self, wx, wy):
        ox, oy = LIDAR_GRID_ORIGIN
        ci = int((wx - ox) / LIDAR_GRID_RES)
        cj = int((wy - oy) / LIDAR_GRID_RES)
        return ci, cj # (col, row)

    def _lidar_cb(self, msg: LaserScan):
        # put scan into world frame using the EKF pose
        x, y, th = self.mu[0], self.mu[1], self.mu[2]


        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            wx = x + r * math.cos(th + angle)
            wy = y + r * math.sin(th + angle)
            ci, cj = self._world_to_cell(wx, wy)
            if 0 <= ci < LIDAR_GRID_SIZE and 0 <= cj < LIDAR_GRID_SIZE:
                self.lidar_grid[cj, ci] += 1

        
        self._publish_lidar_map()

        # generate numpy array on lidar data
        max_range = 6.0
        grid_res = .05
        num_x_grid = int((max_range*2)/grid_res)
        num_y_grid = num_x_grid

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(msg.ranges))*msg.angle_increment
        angles = angles[ranges < 6.0]
        ranges = ranges[ranges < 6.0]

        detected_pts_img = np.zeros((num_y_grid, num_x_grid), dtype=np.uint8)


        bottom_left_corner = np.array([-max_range, -max_range])
        
        # convert to pixel position
        x_idx = np.clip(((np.cos(angles)*ranges - bottom_left_corner[0])/grid_res).astype(int), 0, num_x_grid-1)
        y_idx = np.clip(((np.sin(angles)* ranges - bottom_left_corner[1]) / grid_res).astype(int), 0, num_y_grid-1)

        detected_pts_img[y_idx, x_idx] = 255

        lines = cv2.HoughLinesP(
            detected_pts_img, 
            rho=1, 
            theta=np.pi / 180, 
            threshold=50,    # Adjust based on how 'solid' your 255 lines are
            minLineLength=20, 
            maxLineGap=5
        )
        orientation_strength = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0] 
                cur_orientation = np.arctan2(y2-y1, x2-x1)
                cur_dist = np.hypot(x2-x1, y2-y1)
                found_orientation = False
                for i in range(len(orientation_strength)):
                    orientation, votes = orientation_strength[i]
                    diff = ((cur_orientation - orientation) % (2 * np.pi)) - np.pi
                    if diff > np.pi:
                        diff = diff - 2*np.pi
                    if np.abs(diff) < np.radians(1):
                        orientation_strength[i][1] += cur_dist
                        found_orientation = True
                        break
                         
                if not found_orientation:
                    orientation_strength.append([cur_orientation, cur_dist])

            orientation_strength = np.array(orientation_strength).reshape((-1, 2))
            idx = np.argmax(orientation_strength[:, 1])
            self.get_logger().info(f"Orientation of longests length lines: {orientation_strength[idx,0]}")

        # if self.debug_img_counter % 5 == 0:
        #     # 2. Show the image. Using the name 'Mapping' consistently 
        #     # ensures it updates the same window rather than opening a new one.
        #     cv2.imshow('Lines', detected_pts_img)
        
        #     # 3. CRITICAL: You must include cv2.waitKey()
        #     # This allows OpenCV to process window events. 
        #     # 1ms delay is enough to refresh the image.
        #     cv2.waitKey(1)

        # self.debug_img_counter += 1
        
        






        

        

    # PUBLISHING


    def _publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(th / 2.0)
        msg.pose.pose.orientation.w = math.cos(th / 2.0)

        # [x, y, z, roll, pitch, yaw]
        cov = [0.0] * 36
        cov[0] = self.Sigma[0, 0] # x–x
        cov[1] = self.Sigma[0, 1] # x–y
        cov[5] = self.Sigma[0, 2] # x–yaw
        cov[6] = self.Sigma[1, 0] # y–x
        cov[7] = self.Sigma[1, 1] # y–y
        cov[11] = self.Sigma[1, 2] # y–yaw
        cov[30] = self.Sigma[2, 0] # yaw–x
        cov[31] = self.Sigma[2, 1] # yaw–y
        cov[35] = self.Sigma[2, 2] # yaw–yaw
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)
        self._publish_pose_history()

    def get_marker(self, id, x, y, namespace="landmarks"):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = id
        radius = 0.1
        marker.ns = namespace
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        return marker

    def _publish_map(self):
        landmarks = []
        markerArr = MarkerArray()
        for tag_id, j in self.lm_index.items():
            sl = slice(3 + 2 * j, 3 + 2 * j + 2)
            markerArr.markers.append(self.get_marker(tag_id, float(self.mu[sl][0]), float(self.mu[sl][1]), namespace="landmarks"))
        self.map_pub.publish(markerArr)

    def _publish_pose_history(self):
        self.pose_history.append((float(self.mu[0]), float(self.mu[1]), float(self.mu[2]), time.time()))

        # Only publish every 10 poses
        if len(self.pose_history) % 10 != 0:
            return

        markerArr = MarkerArray()
        for idx, (x, y, th, _) in enumerate(self.pose_history[::10]):
            marker = self.get_marker(self.pose_marker_id + idx, x, y, namespace="pose_history")
            markerArr.markers.append(marker)
        self.pose_history_pub.publish(markerArr)

    def _publish_lidar_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = LIDAR_GRID_RES
        msg.info.width = LIDAR_GRID_SIZE
        msg.info.height = LIDAR_GRID_SIZE
        msg.info.origin.position.x = LIDAR_GRID_ORIGIN[0]
        msg.info.origin.position.y = LIDAR_GRID_ORIGIN[1]
        msg.info.origin.orientation.w = 1.0

        multiplier = 50 # need 2 hits to be sure there's something there
        lidar_grid_mult = self.lidar_grid * multiplier
        clipped = np.clip(lidar_grid_mult, 0, 100).astype(np.int8)
        msg.data = clipped.flatten().tolist()

        self.lidar_map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFSlamNode()

    # start time
    start_time = node.get_clock().now()

    def save_and_exit(signum, frame):
        binary_grid = np.where(node.lidar_grid > 0, 100, 0).astype(np.int8)
        np.save("last_lidar_grid.npy", binary_grid)
        print("Lidar grid saved to last_lidar_grid.npy")

        # Save lidar occupancy grid as map.jpg, only marking cells with enough hits
        img = np.ones((LIDAR_GRID_SIZE, LIDAR_GRID_SIZE, 3), dtype=np.uint8) * 255
        hits = node.lidar_grid >= 3
        idxs = np.argwhere(hits)
        for (cj, ci) in idxs:
            img[cj, ci] = (0, 0, 0)  # black point
        img = np.flipud(img).copy() # origin bottom left

        # Draw grid lines every 5m
        grid_spacing_m = 5.0
        grid_spacing_px = int(grid_spacing_m / LIDAR_GRID_RES)
        color_grid = (200, 200, 200)
        for i in range(0, LIDAR_GRID_SIZE, grid_spacing_px):
            cv2.line(img, (0, i), (LIDAR_GRID_SIZE-1, i), color_grid, 1)
            cv2.line(img, (i, 0), (i, LIDAR_GRID_SIZE-1), color_grid, 1)

        # Draw axes at origin
        ox, oy = LIDAR_GRID_ORIGIN
        x0 = int((0.0 - ox) / LIDAR_GRID_RES)
        y0 = int((0.0 - oy) / LIDAR_GRID_RES)
        y0_flipped = LIDAR_GRID_SIZE - 1 - y0
        color_axis = (0, 0, 255)
        if 0 <= x0 < LIDAR_GRID_SIZE:
            cv2.line(img, (x0, 0), (x0, LIDAR_GRID_SIZE-1), color_axis, 2)
        if 0 <= y0_flipped < LIDAR_GRID_SIZE:
            cv2.line(img, (0, y0_flipped), (LIDAR_GRID_SIZE-1, y0_flipped), color_axis, 2)

        # Add axis labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_color = (0, 0, 0)
        thickness = 1
        label_offset = 18  # pixels from the bottom
        for i in range(0, LIDAR_GRID_SIZE, int(10.0 / LIDAR_GRID_RES)):
            x_m = ox + i * LIDAR_GRID_RES
            cv2.putText(img, f"{x_m:.0f}", (i+2, LIDAR_GRID_SIZE-2), font, font_scale, font_color, thickness)
            y_m = oy + i * LIDAR_GRID_RES
            cv2.putText(img, f"{y_m:.0f}", (2, LIDAR_GRID_SIZE-1-i-2), font, font_scale, font_color, thickness)

        # add pose history
        N = len(node.pose_history)
        for idx, (x, y, th, _) in enumerate(node.pose_history):
            ci = int((x - LIDAR_GRID_ORIGIN[0]) / LIDAR_GRID_RES)
            cj = int((y - LIDAR_GRID_ORIGIN[1]) / LIDAR_GRID_RES)
            if 0 <= ci < LIDAR_GRID_SIZE and 0 <= cj < LIDAR_GRID_SIZE:
                # fade blue to red over the history
                r = int(255 * idx / max(N-1, 1))
                g = 0
                b = int(255 * (1 - idx / max(N-1, 1)))
                cv2.circle(img, (ci, LIDAR_GRID_SIZE - 1 - cj), 2, (b, g, r), -1)

        # ADD LANDMARKS
        N_landmarks = len(node.lm_index)
        for tag_id, j in node.lm_index.items():
            sl = slice(3 + 2 * j, 3 + 2 * j + 2)
            lx, ly = node.mu[sl]
            ci = int((lx - LIDAR_GRID_ORIGIN[0]) / LIDAR_GRID_RES)
            cj = int((ly - LIDAR_GRID_ORIGIN[1]) / LIDAR_GRID_RES)
            if 0 <= ci < LIDAR_GRID_SIZE and 0 <= cj < LIDAR_GRID_SIZE:
                cv2.circle(img, (ci, LIDAR_GRID_SIZE - 1 - cj), 5, (0, 255, 0), -1)
                cv2.putText(img, f"Tag {tag_id}", (ci+5, LIDAR_GRID_SIZE - 1 - cj - 5), font, font_scale, (0, 128, 0), thickness)

        # Get max value and average value of rf2o update times
        if node.rf20_update_times:
            max_rf2o_time = max(node.rf20_update_times)
            avg_rf2o_time = sum(node.rf20_update_times) / len(node.rf20_update_times)
            print(f"RF2O update times: max={max_rf2o_time:.4f} s, avg={avg_rf2o_time:.4f} s")

        cv2.imwrite("map_pose_history.jpg", img)
        print("Map and pose history image saved to map_pose_history.jpg")

        current_time = int(time.time())
        with open(f"pose_history_{current_time}.txt", "w") as f:
            for i in range(len(node.pose_history)):
                x, y, th, time_stamp = node.pose_history[i]
                f.write(f"{x:.4f} {y:.4f} {th:.4f} {time_stamp}\n")
        print("Pose history saved to pose_history.txt")


        with open(f"landmark_positions_{current_time}.txt", "w") as f:

            landmark_positions = {}
            for tag_id, j in node.lm_index.items():
                sl = slice(3 + 2 * j, 3 + 2 * j + 2)
                lx, ly = node.mu[sl]
                landmark_positions[tag_id] = {}
                landmark_positions[tag_id]["x"] = lx 
                landmark_positions[tag_id]["y"] = ly 
            json.dump(node.lm_index, f, indent=4)
            



        node.destroy_node()
        rclpy.shutdown()
        exit(0)

    signal.signal(signal.SIGINT, save_and_exit)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
