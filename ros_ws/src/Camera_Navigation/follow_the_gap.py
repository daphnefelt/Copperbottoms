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

        self.kp_dist  = 0.05
        self.kp_angle = 0.1
        
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

    def _draw_debug_frame(self, fov_angles, fov_ranges, min_angle_rad, min_dist, dist_at_90, dist_turn, angle_turn):
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

        # -90 deg reference beam — grey dot showing dist_at_90
        ref_angle = np.radians(-90.0)
        rx = int(cx - dist_at_90 * np.sin(ref_angle) * scale)
        ry = int(cy - dist_at_90 * np.cos(ref_angle) * scale)
        cv2.line(canvas, (cx, cy), (rx, ry), (100, 100, 100), 1)
        cv2.circle(canvas, (rx, ry), 5, (160, 160, 160), 2)
        cv2.putText(canvas, f'd90={dist_at_90:.2f}m', (rx + 8, ry + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (140, 140, 140), 1, cv2.LINE_AA)

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

        # Resultant velocity arrow — forward + both corrections combined
        total_turn = dist_turn + angle_turn
        fwd_px  = 80  # fixed length for forward speed (visual reference)
        turn_px = int(np.clip(abs(total_turn) / self.max_turn * 80, 0, 80))
        turn_sign = -1 if total_turn > 0 else 1  # positive z = left = -x in image
        res_end = (cx + turn_sign * turn_px, cy - fwd_px)
        cv2.arrowedLine(canvas, (cx, cy), res_end, (255, 255, 0), 3, tipLength=0.2)  # yellow
        cv2.putText(canvas, 'resultant', (res_end[0] + 5, res_end[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)

        # Status overlay
        dist_err  = min_dist - self.target_dist
        slope_mag = dist_at_90 - min_dist
        angle_sign = np.sign(np.degrees(min_angle_rad) - (-90.0))
        slope_err = float(angle_sign * slope_mag)
        cv2.putText(canvas, f'min_d={min_dist:.2f}m  d90={dist_at_90:.2f}m  dist_err={dist_err:+.3f}m',
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(canvas, f'min_ang={np.degrees(min_angle_rad):.1f}deg  slope_err={slope_err:+.3f}m',
                    (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(canvas, f'total turn={dist_turn + angle_turn:+.3f}',
                    (20, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)

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

        # Find closest point in FOV
        valid_angles = fov_angles[valid]
        min_idx  = int(np.argmin(fov_ranges_valid))
        min_dist = float(fov_ranges_valid[min_idx])
        min_angle_deg = float(np.degrees(valid_angles[min_idx]))

        # dist_at_90: range of the beam closest to exactly -90 deg (use cleaned FOV arrays)
        idx_90 = int(np.argmin(np.abs(fov_angles - np.radians(-90.0))))
        dist_at_90 = float(fov_ranges[idx_90])

        # Distance error: use d90 as feedback (true perpendicular distance to wall)
        dist_error = dist_at_90 - self.target_dist
        dist_turn = -self.kp_dist * dist_error

        # Slope error: how much closer the min point is vs straight right
        # Always >= 0; sign set by which side of -90 the min falls on
        #   min_angle > -90 (nose angled toward wall) → turn left  (+z)
        #   min_angle < -90 (nose angled away from wall) → turn right (-z)
        slope_mag   = dist_at_90 - min_dist   # 0 when perfectly parallel
        angle_sign  = np.sign(min_angle_deg - (-90.0))  # +1, 0, or -1
        angle_error = float(angle_sign * slope_mag)
        angle_turn  = self.kp_angle * angle_error

        turn = float(np.clip(dist_turn + angle_turn, -self.max_turn, self.max_turn))
        self._publish(self.forward_speed, turn)

        frame = self._draw_debug_frame(fov_angles, fov_ranges,
                               valid_angles[min_idx], min_dist, dist_at_90,
                               dist_turn, angle_turn)
        self._emit_debug(frame)

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