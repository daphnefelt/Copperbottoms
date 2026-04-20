#!/usr/bin/env python3
"""
CombinedFollower — tape following + LiDAR wall following

State machine
─────────────
  TAPE  : drive by camera tape detection (PID)
  LIDAR : drive by LiDAR right-wall following (P)

Transitions
  TAPE  → LIDAR : >inf_thresh inf readings in right wall FOV
  LIDAR → TAPE  : lidar_min_dwell seconds elapsed AND tape currently visible
"""

import os
import time

import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge


class CombinedFollower(Node):

    MODE_TAPE  = 'TAPE'
    MODE_LIDAR = 'LIDAR'

    def __init__(self):
        super().__init__('combined_follower')

        # ── shared ──────────────────────────────────────────────────────
        self.forward_speed = 0.25
        self.max_turn      = 0.5

        # ── lidar params ────────────────────────────────────────────────
        self.kp_dist         = 0.1
        self.kp_angle        = 0.25
        self.n_min_avg       = 3
        self.target_dist     = 1.524             # desired distance to right wall (m)
        self.wall_fov_deg    = (-105.0, -75.0)   # right side window
        self.inf_thresh      = 4                 # inf readings that trigger LIDAR mode
        self.lidar_min_dwell = 4.0               # minimum seconds to stay in LIDAR mode

        # ── tape params ─────────────────────────────────────────────────
        self.tape_color        = np.array([164, 108,  7])   # BGR
        self.color_tolerance   = np.array([ 50,  50, 90])
        self.min_contour_area  = 100
        self.target_width      = 320
        self.target_height     = 240
        # PID gains
        self.kp_tape           = 0.6
        self.ki_tape           = 0.03
        self.kd_tape           = 0.20
        self.steering_deadband = 0.1
        self.max_integral      = 0.25
        # adaptive speed
        self.adaptive_speed       = True
        self.min_speed_ratio      = 0.8
        self.error_threshold_slow = 0.25

        # ── state machine ────────────────────────────────────────────────
        self.mode             = self.MODE_TAPE
        self.lidar_mode_start = None   # time.monotonic() when LIDAR mode began
        self.tape_visible     = False  # updated every image_callback

        # ── tape PID state ───────────────────────────────────────────────
        self.error_integral      = 0.0
        self.last_error          = 0.0
        self.last_pid_time       = None
        self.filtered_derivative = 0.0

        # ── misc ─────────────────────────────────────────────────────────
        self.bridge       = CvBridge()
        self.have_display = bool(os.environ.get('DISPLAY'))

        # ── ROS I/O ──────────────────────────────────────────────────────
        self.scan_sub  = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('CombinedFollower started — initial mode: TAPE')

    # ════════════════════════════════════════════════════════════════════
    # LiDAR helpers
    # ════════════════════════════════════════════════════════════════════

    def _count_inf_in_fov(self, msg: LaserScan) -> int:
        """Return number of raw inf readings inside the wall FOV."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        fov_mask = (angles >= np.radians(self.wall_fov_deg[0])) & \
                   (angles <= np.radians(self.wall_fov_deg[1]))
        return int(np.sum(np.isinf(ranges[fov_mask])))

    def _lidar_control(self, msg: LaserScan):
        """
        Wall-following P control (right wall).
        Returns (linear_speed, angular_speed).
        """
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        fov_mask   = (angles >= np.radians(self.wall_fov_deg[0])) & \
                     (angles <= np.radians(self.wall_fov_deg[1]))
        fov_ranges = ranges[fov_mask].copy()
        fov_angles = angles[fov_mask]

        # replace inf with range_max (already-cleaned FOV)
        fov_ranges = np.where(np.isinf(fov_ranges), msg.range_max, fov_ranges)
        valid             = np.isfinite(fov_ranges)
        fov_ranges_valid  = fov_ranges[valid]

        if fov_ranges_valid.size == 0:
            return self.forward_speed, 0.0

        # average N closest beams → min cluster
        valid_angles  = fov_angles[valid]
        n             = min(self.n_min_avg, fov_ranges_valid.size)
        closest_idx   = np.argpartition(fov_ranges_valid, n - 1)[:n]
        min_dist      = float(np.mean(fov_ranges_valid[closest_idx]))
        min_angle_deg = float(np.degrees(np.mean(valid_angles[closest_idx])))

        # perpendicular beam at -90 deg
        idx_90     = int(np.argmin(np.abs(fov_angles - np.radians(-90.0))))
        dist_at_90 = float(fov_ranges[idx_90])

        # P distance control (uses d90 as feedback)
        dist_error = dist_at_90 - self.target_dist
        dist_turn  = -self.kp_dist * dist_error

        # P angle/slope control
        slope_mag   = dist_at_90 - min_dist          # 0 when parallel
        angle_sign  = np.sign(min_angle_deg - (-90.0))
        angle_error = float(angle_sign * slope_mag)
        angle_turn  = self.kp_angle * angle_error

        turn = float(np.clip(dist_turn + angle_turn, -self.max_turn, self.max_turn))
        return self.forward_speed, turn

    # ════════════════════════════════════════════════════════════════════
    # Tape helpers
    # ════════════════════════════════════════════════════════════════════

    def _apply_color_filter(self, img):
        lower = self.tape_color - self.color_tolerance
        upper = self.tape_color + self.color_tolerance
        mask = cv2.inRange(img, lower, upper)
        k    = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
        return mask

    def _find_best_contour(self, mask):
        """Returns (contour, center) or (None, None)."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_contour_area]
        if not valid:
            return None, None
        best = max(valid, key=cv2.contourArea)
        M = cv2.moments(best)
        if M['m00'] == 0:
            return None, None
        return best, (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

    def _tape_pid(self, center, img_width):
        """PID lateral error → turn. Returns (turn, abs_error)."""
        cx, _ = center
        error = (cx - img_width / 2.0) / (img_width / 2.0)
        if abs(error) < self.steering_deadband:
            error = 0.0

        now = time.monotonic()
        dt  = (now - self.last_pid_time) if self.last_pid_time is not None else 0.035
        dt  = max(0.001, min(dt, 0.1))
        self.last_pid_time = now

        p_term = self.kp_tape * error

        self.error_integral += error * dt
        self.error_integral  = float(np.clip(self.error_integral,
                                             -self.max_integral, self.max_integral))
        i_term = self.ki_tape * self.error_integral

        deriv = (error - self.last_error) / dt
        self.filtered_derivative = 0.15 * deriv + 0.85 * self.filtered_derivative
        d_term = self.kd_tape * self.filtered_derivative
        self.last_error = error

        turn = float(np.clip(-(p_term + i_term + d_term), -self.max_turn, self.max_turn))
        return turn, abs(error)

    def _reset_pid(self):
        self.error_integral      = 0.0
        self.last_error          = 0.0
        self.last_pid_time       = None
        self.filtered_derivative = 0.0

    # ════════════════════════════════════════════════════════════════════
    # Callbacks
    # ════════════════════════════════════════════════════════════════════

    def scan_callback(self, msg: LaserScan):
        inf_count = self._count_inf_in_fov(msg)

        # ── TAPE → LIDAR transition ────────────────────────────────────
        if self.mode == self.MODE_TAPE and inf_count > self.inf_thresh:
            self.mode             = self.MODE_LIDAR
            self.lidar_mode_start = time.monotonic()
            self.get_logger().info(
                f'→ LIDAR mode  ({inf_count} inf readings in FOV)')

        # ── LIDAR → TAPE transition ────────────────────────────────────
        if self.mode == self.MODE_LIDAR:
            elapsed = time.monotonic() - self.lidar_mode_start
            if elapsed >= self.lidar_min_dwell and self.tape_visible:
                self.mode = self.MODE_TAPE
                self._reset_pid()
                self.get_logger().info(
                    f'→ TAPE mode  (lidar dwell {elapsed:.1f}s, tape visible)')

        # ── drive in LIDAR mode ────────────────────────────────────────
        if self.mode == self.MODE_LIDAR:
            lin, ang = self._lidar_control(msg)
            self._publish(lin, ang)
            # self.get_logger().info(
            #     f'[LIDAR] inf={inf_count}  lin={lin:.2f}  ang={ang:+.3f}')

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image convert failed: {e}')
            return

        if img.shape[1] != self.target_width or img.shape[0] != self.target_height:
            img = cv2.resize(img, (self.target_width, self.target_height))

        mask          = self._apply_color_filter(img)
        _, center     = self._find_best_contour(mask)
        self.tape_visible = center is not None

        # only drive from camera when in TAPE mode
        if self.mode != self.MODE_TAPE:
            return

        if not self.tape_visible:
            self._publish(self.forward_speed * 0.5, 0.0)   # coast slowly
            return

        turn, err_mag = self._tape_pid(center, img.shape[1])

        speed = self.forward_speed
        if self.adaptive_speed and err_mag > self.error_threshold_slow:
            ratio = 1.0 - (err_mag - self.error_threshold_slow) / \
                          (1.0 - self.error_threshold_slow)
            speed = self.forward_speed * float(np.clip(ratio, self.min_speed_ratio, 1.0))

        self._publish(speed, turn)

        # ── debug window ───────────────────────────────────────────────
        if self.have_display:
            dbg = img.copy()
            cx, cy = center
            cv2.circle(dbg, (cx, cy), 8, (0, 0, 255), -1)
            cv2.line(dbg, (img.shape[1] // 2, 0),
                         (img.shape[1] // 2, img.shape[0]), (255, 255, 0), 1)
            cv2.putText(dbg,
                        f'TAPE  turn={turn:+.3f}  err={err_mag:.2f}',
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            try:
                cv2.imshow('Tape Debug', dbg)
                cv2.waitKey(1)
            except Exception:
                self.have_display = False

    # ════════════════════════════════════════════════════════════════════

    def _publish(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self.vel_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CombinedFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
