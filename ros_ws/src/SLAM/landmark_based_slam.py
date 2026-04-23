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

# NOISE PARAMS
MOTION_NOISE = np.diag([0.05**2, 0.05**2, np.deg2rad(2.0)**2]) # process noise on robot pose
OBS_NOISE = np.diag([0.1**2, np.deg2rad(5.0)**2]) # measurement noise for range (m) and bearing (rad)
INIT_LM_COV = 1000.0 # init covariance for a new landmark

def wrap(a: float) -> float: # wraps to -pi, +pi
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class EKFSlamNode(Node):
    def __init__(self):
        super().__init__('ekf_slam')

        # state
        self.mu: np.ndarray = np.zeros(3) # No landmarks yet, so just robot pose
        self.Sigma: np.ndarray = np.zeros((3, 3)) # Pose known at start, so zero covariance
        # maps tag_id (int) to index j so landmark is at mu[3+2j : 3+2j+2]
        self.lm_index: dict[int, int] = {}

        # last velocity cmds
        self.v = 0.0
        self.delta = 0.0
        self.last_time = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel_temp',   self._cmd_vel_cb,   10)
        self.create_subscription(String, '/landmarks', self._landmark_cb,  10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/slam/pose', 10)
        self.map_pub = self.create_publisher(String, '/slam/landmarks', 10)
        self.create_timer(0.05, self._predict_step) # predict at 20hz

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
    V_NOMINAL = 0.2 # m/s for linear.x = 1.0
    DELTA_NOMINAL = 0.35 # rad steering angle for angular.z = 1.0
    def _cmd_vel_cb(self, msg: Twist):
        self._predict_step() # predict up to now before changing velocity
        self.v = msg.linear.x * self.V_NOMINAL
        self.delta = msg.angular.z * self.DELTA_NOMINAL
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
        self._plot_pose() # Plot pose and landmarks

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

    def _publish_map(self):
        landmarks = []
        for tag_id, j in self.lm_index.items():
            sl = slice(3 + 2 * j, 3 + 2 * j + 2)
            landmarks.append({
                'id': tag_id,
                'x': float(self.mu[sl][0]),
                'y': float(self.mu[sl][1]),
            })
        self.map_pub.publish(String(data=json.dumps(landmarks)))

    # PLOTTING
    PLOT_PIXELS = 800
    PLOT_METERS = 10.0
    PLOT_ORIGIN = (400, 400) # pixels for 0,0 in world frame

    def _world_to_px(self, x, y):
        scale = self.PLOT_PIXELS / self.PLOT_METERS
        px = int(self.PLOT_ORIGIN[0] + x * scale)
        py = int(self.PLOT_ORIGIN[1] - y * scale)
        return px, py
    
    def _plot_pose(self):
        img = np.ones((self.PLOT_PIXELS, self.PLOT_PIXELS, 3), dtype=np.uint8) * 255
        # landmarks
        for tag_id, j in self.lm_index.items():
            sl = slice(3 + 2 * j, 3 + 2 * j + 2)
            lx, ly = self.mu[sl]
            px, py = self._world_to_px(lx, ly)
            cv2.circle(img, (px, py), 5, (0, 0, 255), -1)
            cv2.putText(img, str(tag_id), (px + 5, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        # robot pose
        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        px, py = self._world_to_px(x, y)
        cv2.circle(img, (px, py), 5, (255, 0, 0), -1)
        arrow_length = 20 # pixels
        arrow_length_m = arrow_length * self.PLOT_METERS / self.PLOT_PIXELS
        arrow_dx = arrow_length_m * math.cos(th)
        arrow_dy = arrow_length_m * math.sin(th)
        px2, py2 = self._world_to_px(x + arrow_dx, y + arrow_dy)
        cv2.arrowedLine(img, (px, py), (px2, py2), (255, 0, 0), 2)

        cv2.imshow('SLAM Pose', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = EKFSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()