### Follow the Gap w/ Lidar

import os
from pathlib import Path
import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FollowTheGap(Node):

    def __init__(self):
        super().__init__('follow_the_gap')

        # params
        self.forward_speed = 0.25
        self.max_turn = 0.5
        self.kp = 0.25 # p gain on heading error
        self.safety_dist = 0.25 # m - readings below this treated as 0
        self.fov_range_deg = (-90.0, 45.0) # [min, max] angle range to consider (degrees)
        self.window_deg = 10.0 # sliding window half-width used when scoring each angle
        self.debug_show_cv = True
        self.debug_save_jpg = False
        self.debug_jpg_path = Path('/tmp/follow_the_gap_debug.jpg')
        self.cv_window_name = 'Follow The Gap Debug'
        self.have_display = bool(os.environ.get('DISPLAY'))

        scan_topic = '/scan'
        cmd_vel_topic = '/cmd_vel'

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            f'Follow The Gap node started')

        self.get_logger().info(
            f'Debug output: show_cv={self.debug_show_cv and self.have_display}, '
            f'save_jpg={self.debug_save_jpg}, jpg_path={self.debug_jpg_path}')

    # helpers

    def _draw_debug_frame(self, fov_angles, fov_ranges, scores, goal_angle, status_text):
        canvas = np.zeros((700, 700, 3), dtype=np.uint8)
        cx, cy = 350, 650

        max_range = float(np.max(fov_ranges)) if fov_ranges.size > 0 else 1.0
        max_range = max(max_range, 1e-3)
        scale = 560.0 / max_range

        # Colour each beam by its normalised score (blue=low, green=high).
        max_score = float(np.max(scores)) if scores.size > 0 else 1.0
        max_score = max(max_score, 1e-3)
        for a, r, s in zip(fov_angles, fov_ranges, scores):
            if r <= 0:
                continue
            x = int(cx - (r * np.sin(a) * scale))
            y = int(cy - (r * np.cos(a) * scale))
            if 0 <= x < canvas.shape[1] and 0 <= y < canvas.shape[0]:
                t = s / max_score  # 0..1
                canvas[y, x] = (int(255 * (1 - t)), int(255 * t), 40)

        # Goal direction in red.
        goal_len = int(0.85 * 560)
        gx = int(cx - goal_len * np.sin(goal_angle))
        gy = int(cy - goal_len * np.cos(goal_angle))
        cv2.arrowedLine(canvas, (cx, cy), (gx, gy), (30, 30, 255), 2, tipLength=0.05)

        cv2.putText(canvas, f'goal={np.degrees(goal_angle):.1f} deg', (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2, cv2.LINE_AA)
        cv2.putText(canvas, status_text, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2, cv2.LINE_AA)
        cv2.putText(canvas, 'Blue=low score  Green=high score  Red=goal', (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1, cv2.LINE_AA)

        return canvas

    def _emit_debug(self, frame: np.ndarray):
        if self.debug_save_jpg:
            cv2.imwrite(str(self.debug_jpg_path), frame)

        if self.debug_show_cv and self.have_display:
            try:
                cv2.imshow(self.cv_window_name, frame)
                cv2.waitKey(1)
            except Exception as exc:
                self.have_display = False
                self.get_logger().warn(f'CV window unavailable, continuing with JPG only: {exc}')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        # Restrict to configured angle range.
        fov_min = np.radians(self.fov_range_deg[0])
        fov_max = np.radians(self.fov_range_deg[1])
        fov_mask = (angles >= fov_min) & (angles <= fov_max)
        fov_ranges = ranges[fov_mask].copy()
        fov_angles = angles[fov_mask]

        # Replace inf readings with the sensor's reported max range.
        fov_ranges = np.where(np.isinf(fov_ranges), msg.range_max, fov_ranges)
        fov_ranges = np.where(np.isfinite(fov_ranges) & (fov_ranges > self.safety_dist), fov_ranges, 0.0)

        if fov_ranges.max() == 0.0:
            self._publish(0.0, 0.0)
            scores = np.zeros_like(fov_ranges)
            frame = self._draw_debug_frame(fov_angles, fov_ranges, scores, 0.0, 'No safe range, stopping')
            self._emit_debug(frame)
            self.get_logger().warn('No valid range readings, stopping.')
            return

        # Sliding-window score: for every beam, sum the distances of all beams
        # within ±(window_deg/2) of that beam's angle.
        half_window = np.radians(self.window_deg / 2.0)
        scores = np.array([
            float(np.sum(fov_ranges[np.abs(fov_angles - a) <= half_window]))
            for a in fov_angles
        ])

        best_idx = int(np.argmax(scores))
        goal_angle = float(fov_angles[best_idx])

        turn = float(np.clip(self.kp * goal_angle, -self.max_turn, self.max_turn))

        frame = self._draw_debug_frame(
            fov_angles, fov_ranges, scores, goal_angle,
            f'best_idx={best_idx} score={scores[best_idx]:.2f} turn={turn:.3f}',
        )
        self._emit_debug(frame)

        self._publish(self.forward_speed, turn)
        self.get_logger().info(
            f'goal={np.degrees(goal_angle):.1f}°, score={scores[best_idx]:.2f}, turn={turn:.3f}')
            
    def _publish(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x  = linear
        twist.angular.z = angular
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FollowTheGap()
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