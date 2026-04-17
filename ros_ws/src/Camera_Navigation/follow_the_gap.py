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
        self.forward_speed = 0.3
        self.max_turn = 0.8
        self.kp = 1.0 # p gain on heading error
        self.bubble_radius = 0.5 # m - zeroed out around closest point
        self.safety_dist = 0.25 # m - readings below this treated as 0
        self.fov_deg = 180.0 # degrees to consider in front of the robot
        self.min_gap_width  = 5 # min free beams in a row for a valid gap
        self.range_group_delta = 0.5 # m - start a new gap when adjacent beams jump by more than this
        self.debug_show_cv = True
        self.debug_save_jpg = True
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

    def _draw_debug_frame(self, fov_angles, fov_ranges_raw, fov_ranges_bubbled, gap, goal_angle, status_text):
        canvas = np.zeros((700, 700, 3), dtype=np.uint8)
        cx, cy = 350, 650

        max_range = float(np.max(fov_ranges_raw)) if fov_ranges_raw.size > 0 else 1.0
        max_range = max(max_range, 1e-3)
        scale = 560.0 / max_range

        # Raw scan in blue.
        for a, r in zip(fov_angles, fov_ranges_raw):
            if r <= 0:
                continue
            x = int(cx - (r * np.sin(a) * scale))
            y = int(cy - (r * np.cos(a) * scale))
            if 0 <= x < canvas.shape[1] and 0 <= y < canvas.shape[0]:
                canvas[y, x] = (255, 100, 40)

        # Best gap in green.
        if gap is not None:
            g_start, g_end = gap
            for a, r in zip(fov_angles[g_start:g_end + 1], fov_ranges_bubbled[g_start:g_end + 1]):
                if r <= 0:
                    continue
                x = int(cx - (r * np.sin(a) * scale))
                y = int(cy - (r * np.cos(a) * scale))
                if 0 <= x < canvas.shape[1] and 0 <= y < canvas.shape[0]:
                    canvas[y, x] = (40, 255, 40)

        # Goal direction in red.
        goal_len = int(0.85 * 560)
        gx = int(cx - goal_len * np.sin(goal_angle))
        gy = int(cy - goal_len * np.cos(goal_angle))
        cv2.arrowedLine(canvas, (cx, cy), (gx, gy), (30, 30, 255), 2, tipLength=0.05)

        cv2.putText(canvas, f'goal={np.degrees(goal_angle):.1f} deg', (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2, cv2.LINE_AA)
        cv2.putText(canvas, status_text, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2, cv2.LINE_AA)
        cv2.putText(canvas, 'Blue: scan  Green: best gap  Red: steering goal', (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1, cv2.LINE_AA)

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

    def _apply_safety_bubble(self, ranges: np.ndarray, angles: np.ndarray) -> np.ndarray:
        # Take out the bubble area around closest obstacle
        working = ranges.copy()
        closest_idx = int(np.argmin(np.where(working > 0, working, np.inf)))
        closest_dist = working[closest_idx]

        half_angle = np.arctan2(self.bubble_radius, closest_dist)
        bubble_mask = np.abs(angles - angles[closest_idx]) <= half_angle
        working[bubble_mask] = 0.0
        return working

    def _find_best_gap(self, ranges: np.ndarray):
        # Split free-space runs into groups with similar adjacent distances,
        # then pick the group with the highest free-space score.
        in_gap = False
        cur_start = 0
        best_start = best_end = 0
        best_score = -1.0
        best_width = 0

        def consider_gap(start_idx: int, end_idx: int):
            nonlocal best_start, best_end, best_score, best_width
            width = end_idx - start_idx + 1
            if width < self.min_gap_width:
                return

            segment = ranges[start_idx:end_idx + 1]
            score = float(np.sum(segment))

            # Higher score wins; if tied, use wider gap.
            if score > best_score or (score == best_score and width > best_width):
                best_score = score
                best_width = width
                best_start, best_end = start_idx, end_idx

        for i, r in enumerate(ranges):
            if r > 0:
                if not in_gap:
                    cur_start = i
                    in_gap = True
                elif abs(float(r) - float(ranges[i - 1])) > self.range_group_delta:
                    consider_gap(cur_start, i - 1)
                    cur_start = i
            elif in_gap:
                consider_gap(cur_start, i - 1)
                in_gap = False

        if in_gap:
            consider_gap(cur_start, len(ranges) - 1)

        return (best_start, best_end) if best_score >= 0.0 else None

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        # cut to front FOV
        half_fov = np.radians(self.fov_deg / 2.0)
        fov_mask = np.abs(angles) <= half_fov
        fov_ranges = ranges[fov_mask].copy()
        fov_angles = angles[fov_mask]

        # Ignore invalid readings (including inf/nan) and too-close returns.
        fov_ranges = np.where(np.isfinite(fov_ranges) & (fov_ranges > self.safety_dist), fov_ranges, 0.0)

        # safety bubble
        fov_ranges_pre_bubble = fov_ranges.copy()
        fov_ranges = self._apply_safety_bubble(fov_ranges, fov_angles)

        if fov_ranges.max() == 0.0:
            self._publish(0.0, 0.0)
            frame = self._draw_debug_frame(fov_angles, fov_ranges_pre_bubble, fov_ranges, None, 0.0, 'No safe range, stopping')
            self._emit_debug(frame)
            self.get_logger().warn('Too close to obstacles, stopping.')
            return

        # find widest gap
        gap = self._find_best_gap(fov_ranges)

        if gap is None:
            self._publish(0.0, 0.0)
            frame = self._draw_debug_frame(fov_angles, fov_ranges_pre_bubble, fov_ranges, None, 0.0, 'No valid gap, stopping')
            self._emit_debug(frame)
            self.get_logger().warn('No valid gap found, stopping.')
            return

        gap_start, gap_end = gap

        # aim for deepest point in gap
        gap_ranges = fov_ranges[gap_start : gap_end + 1]
        best_in_gap = gap_start + int(np.argmax(gap_ranges))
        goal_angle  = float(fov_angles[best_in_gap])

        # proportional steering (positive angle is left, positive angular.z)
        turn = float(np.clip(-self.kp * goal_angle, -self.max_turn, self.max_turn))

        frame = self._draw_debug_frame(
            fov_angles,
            fov_ranges_pre_bubble,
            fov_ranges,
            gap,
            goal_angle,
            f'gap=[{gap_start},{gap_end}] turn={turn:.3f}',
        )
        self._emit_debug(frame)

        self._publish(self.forward_speed, turn)
        self.get_logger().info(
            f'gap=[{gap_start},{gap_end}], '
            f'goal={np.degrees(goal_angle):.1f}°, turn={turn:.3f}')
            
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