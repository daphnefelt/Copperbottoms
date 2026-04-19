### Wall Following w/ Lidar (maintain distance to right wall)

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
        self.kp = 0.5 # P gain: turn per metre of distance error
        self.target_dist = 1.524   # desired distance to right wall (m) — 5 ft
        self.wall_fov_deg = (-90.0 - 45.0, -90.0 + 45.0)  # right side: -135 to -45 deg
        self.safety_dist = 0.0    # m - ignore readings below this
        self.debug_show_cv = True
        self.debug_save_jpg = False
        self.debug_jpg_path = Path('/tmp/follow_the_gap_debug.jpg')
        self.cv_window_name = 'Wall Follow Debug'
        self.have_display = bool(os.environ.get('DISPLAY'))

        scan_topic = '/scan'
        cmd_vel_topic = '/cmd_vel'

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info('Wall follow node started — maintaining {:.2f} m to the right'.format(self.target_dist))

    # helpers

    def _draw_debug_frame(self, fov_angles, fov_ranges, avg_dist, turn, status_text):
        canvas = np.zeros((700, 700, 3), dtype=np.uint8)
        cx, cy = 350, 350  # centre of canvas

        scale = 280.0 / max(self.target_dist * 2, 1e-3)

        for a, r in zip(fov_angles, fov_ranges):
            if r <= 0:
                continue
            x = int(cx - r * np.sin(a) * scale)
            y = int(cy - r * np.cos(a) * scale)
            if 0 <= x < canvas.shape[1] and 0 <= y < canvas.shape[0]:
                canvas[y, x] = (255, 180, 40)

        # target distance ring to the right
        target_px = int(self.target_dist * scale)
        cv2.circle(canvas, (cx, cy), target_px, (0, 200, 200), 1)

        cv2.putText(canvas, f'avg={avg_dist:.2f}m  target={self.target_dist:.2f}m', (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2, cv2.LINE_AA)
        cv2.putText(canvas, f'turn={turn:.3f}', (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2, cv2.LINE_AA)
        cv2.putText(canvas, status_text, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1, cv2.LINE_AA)

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
                self.get_logger().warn(f'CV window unavailable: {exc}')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        # Restrict to right-side window (-135 to -45 deg).
        fov_min = np.radians(self.wall_fov_deg[0])
        fov_max = np.radians(self.wall_fov_deg[1])
        fov_mask = (angles >= fov_min) & (angles <= fov_max)
        fov_ranges = ranges[fov_mask].copy()
        fov_angles = angles[fov_mask]

        # Replace inf with sensor max, then zero out invalid/too-close readings.
        fov_ranges = np.where(np.isinf(fov_ranges), msg.range_max, fov_ranges)
        valid = np.isfinite(fov_ranges) & (fov_ranges > self.safety_dist)
        fov_ranges_valid = fov_ranges[valid]

        if fov_ranges_valid.size == 0:
            self._publish(self.forward_speed, 0.0)
            self.get_logger().warn('No valid wall readings — going straight.')
            return

        avg_dist = float(np.mean(fov_ranges_valid))

        # P control: positive error (too far) → turn right (negative angular.z)
        error = avg_dist - self.target_dist
        turn = float(np.clip(-self.kp * error, -self.max_turn, self.max_turn))

        frame = self._draw_debug_frame(fov_angles, fov_ranges, avg_dist, turn,
                                       f'n={fov_ranges_valid.size} beams  err={error:.3f}m')
        self._emit_debug(frame)

        self._publish(self.forward_speed, turn)
        self.get_logger().info(f'avg_dist={avg_dist:.2f}m, err={error:.3f}m, turn={turn:.3f}')

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