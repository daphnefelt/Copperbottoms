### Bug 1

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

        # range of dist errors possible is like 3, range of angle errors is like 30
        self.kp_dist  = 0.05  # P gain: turn per metre of distance error
        self.kp_angle = 0.1 / 10  # P gain: turn per degree of heading error
        
        self.target_dist = 1.524   # desired distance to right wall (m) — 5 ft
        self.wall_fov_deg = (-90.0 - 15.0, -90.0 + 15.0)  # right side: -105 to -75 deg
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

    def _draw_debug_frame(self, fov_angles, fov_ranges, min_angle_rad, min_dist, dist_turn, angle_turn):
        canvas = np.zeros((700, 700, 3), dtype=np.uint8)
        cx, cy = 350, 350
        scale = 280.0 / max(self.target_dist * 2, 1e-3)

        # All FOV points — dim blue
        for a, r in zip(fov_angles, fov_ranges):
            if r <= 0:
                continue
            x = int(cx - r * np.sin(a) * scale)
            y = int(cy - r * np.cos(a) * scale)
            if 0 <= x < canvas.shape[1] and 0 <= y < canvas.shape[0]:
                canvas[y, x] = (80, 120, 200)

        # Target distance ring
        cv2.circle(canvas, (cx, cy), int(self.target_dist * scale), (0, 200, 200), 1)

        # Min-distance point — bright cyan dot + line showing angle to wall
        mx = int(cx - min_dist * np.sin(min_angle_rad) * scale)
        my = int(cy - min_dist * np.cos(min_angle_rad) * scale)
        if 0 <= mx < canvas.shape[1] and 0 <= my < canvas.shape[0]:
            cv2.line(canvas, (cx, cy), (mx, my), (0, 200, 200), 1)
            cv2.circle(canvas, (mx, my), 7, (0, 255, 255), -1)
            label_x = mx + 10 if mx <= cx else mx - 95
            cv2.putText(canvas, f'{np.degrees(min_angle_rad):.1f}deg',
                        (label_x, my - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 255), 1, cv2.LINE_AA)

        # Robot + forward arrow
        cv2.circle(canvas, (cx, cy), 8, (255, 255, 255), -1)
        cv2.arrowedLine(canvas, (cx, cy), (cx, cy - 45), (255, 255, 255), 2, tipLength=0.25)

        # Turn component arrows — positive z = left = negative x in image
        def _turn_arrow(y_off, turn_val, color, label):
            origin = (cx, cy + y_off)
            if abs(turn_val) < 1e-4:
                cv2.circle(canvas, origin, 4, color, -1)
            else:
                px_len = int(np.clip(abs(turn_val) / self.max_turn * 80, 6, 80))
                sign = -1 if turn_val > 0 else 1  # positive z = left = -x
                end = (cx + sign * px_len, cy + y_off)
                cv2.arrowedLine(canvas, origin, end, color, 2, tipLength=0.3)
            cv2.putText(canvas, f'{label}: {turn_val:+.3f}',
                        (20, cy + y_off + 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 1, cv2.LINE_AA)

        _turn_arrow(55, dist_turn,  (30, 130, 255), 'dist ')   # orange
        _turn_arrow(85, angle_turn, (80, 220,  80), 'angle')   # green

        # Status overlay
        dist_err  = min_dist - self.target_dist
        angle_err = np.degrees(min_angle_rad) - (-90.0)
        cv2.putText(canvas, f'min_d={min_dist:.2f}m  err={dist_err:+.3f}m',
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(canvas, f'min_ang={np.degrees(min_angle_rad):.1f}deg  err={angle_err:+.1f}deg',
                    (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(canvas, f'total turn={dist_turn + angle_turn:+.3f}',
                    (20, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1, cv2.LINE_AA)

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

        # Replace inf with sensor max, then zero out invalid readings.
        fov_ranges = np.where(np.isinf(fov_ranges), msg.range_max, fov_ranges)
        valid = np.isfinite(fov_ranges)
        fov_ranges_valid = fov_ranges[valid]

        if fov_ranges_valid.size == 0:
            self._publish(self.forward_speed, 0.0)
            self.get_logger().warn('No valid wall readings — going straight.')
            return

        # Find closest point in FOV — this is our wall-distance and heading feedback
        valid_angles = fov_angles[valid]
        min_idx  = int(np.argmin(fov_ranges_valid))
        min_dist = float(fov_ranges_valid[min_idx])
        min_angle_deg = float(np.degrees(valid_angles[min_idx]))

        # Distance error: positive = too far, turn right (negative z)
        dist_error = min_dist - self.target_dist
        dist_turn = -self.kp_dist * dist_error

        # Angle error: target is -90 deg (perpendicular to right wall)
        # > -90 → angled towards wall, turn left (positive z)
        # < -90 → angled away from wall, turn right (negative z)
        angle_error = min_angle_deg - (-90.0)   # 0 when parallel
        angle_turn  = self.kp_angle * angle_error

        turn = float(np.clip(dist_turn + angle_turn, -self.max_turn, self.max_turn))

        frame = self._draw_debug_frame(fov_angles, fov_ranges,
                               valid_angles[min_idx], min_dist,
                               dist_turn, angle_turn)
        self._emit_debug(frame)

        self._publish(self.forward_speed, turn)
        self.get_logger().info(
            f'min_dist={min_dist:.2f}m  min_ang={min_angle_deg:.1f}°  '
            f'dist_err={dist_error:.3f}m  angle_err={angle_error:.1f}°  turn={turn:.3f}'
        )

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