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
from nav_msgs.msg import Odometry, Odometry, OccupancyGrid
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

# for mapping
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

# NOISE PARAMS
MOTION_NOISE = np.diag([0.05**2, 0.05**2, np.deg2rad(2.0)**2]) # process noise on robot pose
OBS_NOISE = np.diag([0.1**2, np.deg2rad(5.0)**2]) # measurement noise for range (m) and bearing (rad)
INIT_LM_COV = 1000.0 # init covariance for a new landmark

RF2O_NOISE = np.diag([0.05**2, 0.05**2, np.deg2rad(3.0)**2])  # rf2o scan-match uncertainty

# Params for global lidar occupancy grid mapping
LIDAR_GRID_RES = 0.05 # m per cell
LIDAR_GRID_SIZE = 1600 # cells per side (so 80m x 80m)
LIDAR_GRID_ORIGIN = (0.0, 0.0) # world coords of cell (0, 0)

def wrap(a: float) -> float: # wraps to -pi, +pi
    return (a + math.pi) % (2.0 * math.pi) - math.pi

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

        # lidar occupancy grid
        self.lidar_grid = np.zeros((LIDAR_GRID_SIZE, LIDAR_GRID_SIZE), dtype=np.int32)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb,  10)
        self.create_subscription(String, '/landmarks', self._landmark_cb, 10)
        self.create_subscription(Odometry, '/odom_rf2o', self._rf2o_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._lidar_cb, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/slam/pose', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/slam/landmarks', 10)
        self.lidar_map_pub = self.create_publisher(OccupancyGrid, '/slam/lidar_map', 10)
        self.create_timer(0.05, self._predict_step) # predict at 20hz
        # map
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('SLAM node is up')

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
        pos_angle = math.radians(5.3505*abs_angle**2 + 7.2598*abs_angle - 0.2384)
        return pos_angle if angular_z >= 0 else -pos_angle

    def _cmd_vel_cb(self, msg: Twist):
        self._predict_step() # predict up to now before changing velocity
        self.v = self.V_NOMINAL(msg.linear.x)
        self.delta = self.DELTA_NOMINAL(msg.angular.z)
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


        # map stuff
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # map->odom is the correction between EKF pose and odometry
        t.transform.translation.x = self.mu[0] - self._rf2o_prev[0] if self._rf2o_prev else self.mu[0]
        t.transform.translation.y = self.mu[1] - self._rf2o_prev[1] if self._rf2o_prev else self.mu[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.mu[2] / 2.0)
        t.transform.rotation.w = math.cos(self.mu[2] / 2.0)
        self.tf_broadcaster.sendTransform(t)
    # MEASUREMENT MODEL

    def _landmark_cb(self, msg: String):
        detections = json.loads(msg.data)
        for det in detections: # each apriltag seen
            tag_id = int(det['id'])
            r_obs = float(det['depth'])
            phi_obs = float(det['angle'])

            if r_obs <= 0.0: # something wrong w the depth
                continue
            if tag_id not in self.lm_index: # never before seen landmark
                self._init_landmark(tag_id, r_obs, phi_obs)
            self._ekf_update(tag_id, r_obs, phi_obs)
        self._publish_map() # Publishes all the landmark locations

    def _init_landmark(self, tag_id: int, r: float, phi: float):
        n_old = self.state_size

        # calc where landmark is in world frame
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

        self.get_logger().info(f'LANDMARK UPDATE: tag={tag_id}  x={self.mu[0]:.2f} y={self.mu[1]:.2f} th={math.degrees(self.mu[2]):.1f} deg')

    def _rf2o_cb(self, msg: Odometry):
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

    def get_marker(self, id, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = id
        radius = 0.1
        marker.ns = "landmarks"
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
        marker.pose.position.z = 0
        return marker

    def _publish_map(self):
        landmarks = []
        markerArr = MarkerArray()
        for tag_id, j in self.lm_index.items():
            sl = slice(3 + 2 * j, 3 + 2 * j + 2)
            markerArr.append(self.get_marker(tag_id, float(self.mu[sl][0]), float(self.mu[sl][1])))
        self.map_pub.publish(markerArr)

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

        max_hits = max(int(self.lidar_grid.max()), 1)
        normalized = (self.lidar_grid * 100 // max_hits).astype(np.int8)
        msg.data = normalized.flatten().tolist()

        self.lidar_map_pub.publish(msg)

    # PLOTTING
    PLOT_PIXELS = 800
    PLOT_METERS = 40.0
    PLOT_ORIGIN = (0, 0) # pixels for 0,0 in world frame

    def _world_to_px(self, x, y):
        scale = self.PLOT_PIXELS / self.PLOT_METERS
        px = int(self.PLOT_ORIGIN[0] + x * scale)
        py = int(self.PLOT_ORIGIN[1] - y * scale)
        return px, py

def main(args=None):
    rclpy.init(args=args)
    node = EKFSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
