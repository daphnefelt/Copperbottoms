import rclpy
import numpy as np
import os
from numpy.lib.stride_tricks import as_strided
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

try:
    import cv2
    CV2_AVAILABLE = True
except Exception:
    cv2 = None
    CV2_AVAILABLE = False

try:
    import matplotlib.pyplot as plt
    MPL_AVAILABLE = True
except Exception:
    plt = None
    MPL_AVAILABLE = False


class LineFollowerV2(Node):

    def __init__(self):
        super().__init__('line_follower_v2')

        image_topic = '/camera/color/image_raw'
        cmd_vel_topic = '/cmd_vel'
        debug_image_topic = '/line_follower_v2/debug_image'

        # parameters (tuned from proven Dark Lane Following code)
        self.forward_speed = 0.25  # Reduced from 0.30 for more stable camera processing during corners
        self.min_speed = 0.20
        # Safety default: rotate in place while searching so the rover does not drift away.
        self.search_speed = 0.0
        self.max_turn = 1.0
        self.search_turn = 0.35
        self.kp = 0.80  # Proportional gain (P-only steering like proven old code)
        self.kd = 0.0   # Derivative disabled to match simple old code behavior
        self.error_offset = float(os.environ.get('LFV2_ERROR_OFFSET', '-0.40'))
        self.error_filter_alpha = 0.18
        self.track_x_filter_alpha = 0.22
        self.max_turn_step = 0.05
        self.normal_max_turn = 1.0
        self.min_edge_track_pixels = 120
        # Corner detection thresholds
        self.corner_near_vertical_min_deg = 62.0
        self.corner_far_horizontal_max_deg = 38.0
        self.corner_near_min_pixels = 36
        self.corner_far_min_pixels = 28
        self.corner_confirm_frames = 2
        # Turn mode parameters
        self.corner_turn_speed = 0.10  # Forward speed during corner (slow for camera processing)
        self.corner_turn_rate = 0.65   # Angular velocity during corner turn (slightly reduced for deliberate turn)
        self.corner_turn_hold_frames = 5  # Frames to commit to turn (reduced for faster reacquire check)
        self.corner_reacquire_frames_required = 4  # Stable frames with line before exit
        self.corner_reacquire_min_pixels = 80  # Minimum track pixels to consider line reacquired
        self.corner_scan_full_image = True
        self.corner_edge_support_min_pixels = 45

        # OpenCV GUI can hard-crash process over SSH when DISPLAY/X11 is unavailable.
        # Keep window view opt-in; ROS debug topic remains enabled by default.
        self.enable_debug_view = os.environ.get('LFV2_DEBUG_VIEW', '0') == '1'
        self.enable_debug_topic = os.environ.get('LFV2_DEBUG_TOPIC', '0') == '1'
        self.debug_view_backend = os.environ.get('LFV2_DEBUG_BACKEND', 'cv2').strip().lower()
        if self.debug_view_backend not in ('cv2', 'mpl'):
            self.debug_view_backend = 'cv2'
        self.debug_max_fps = max(float(os.environ.get('LFV2_DEBUG_FPS', '5.0')), 0.5)
        self.debug_view_scale = min(max(float(os.environ.get('LFV2_DEBUG_SCALE', '1.0')), 0.2), 1.0)
        self.debug_publish_every_n_frames = max(int(os.environ.get('LFV2_DEBUG_TOPIC_EVERY', '1')), 1)
        self.debug_window_name = 'LineFollowerV2 Debug'
        self.enable_debug_snapshot = os.environ.get('LFV2_DEBUG_SNAPSHOT', '0') == '1'
        self.debug_snapshot_path = '/tmp/line_follower_v2_debug.jpg'
        self.debug_snapshot_every_n_frames = int(os.environ.get('LFV2_DEBUG_SNAPSHOT_EVERY', '10'))
        self.display_env = os.environ.get('DISPLAY', '')
        self.last_debug_time_sec = 0.0
        self._mpl_fig = None
        self._mpl_ax = None
        self._mpl_im = None

        # blue-tape masking around target BGR
        self.target_bgr = np.array([204.0, 146.0, 39.0], dtype=np.float32)
        self.color_tolerance = np.array([62.0, 58.0, 55.0], dtype=np.float32)
        self.blue_score_threshold = 22.0

        # Hybrid tracker tuning (legacy + edge-boost)
        self.simple_blue_threshold = 30.0
        self.edge_follow_boost = 1.6
        self.min_column_peak = 2.5

        self.min_track_pixels = 90
        self.track_lock_required = 3

        # Adaptive Canny thresholds
        self.canny_high_percentile = 86.0
        self.canny_low_ratio = 0.45

        # focus tracking mostly on lower image region
        self.roi_top_ratio = 0.55
        self.roi_bottom_ratio = 1.0

        # runtime state
        self.rover_armed = False
        self.seen_first_frame = False
        self.frame_count = 0
        self.last_error = 0.0
        self.last_turn = 0.0
        self.last_speed = 0.0
        self.lost_frames = 0
        self.track_lock_frames = 0
        self.last_unbiased_error = 0.0
        self.last_biased_error = 0.0
        self.track_x_filtered = None
        self.drive_state = 'follow'
        self.corner_turn_sign = 1.0
        self.corner_streak = 0
        self.turn_hold_frames_left = 0
        self.turn_reacquire_frames = 0
        self.last_corner_near_angle = -1.0
        self.last_corner_far_angle = -1.0
        self.last_corner_confidence = 0.0
        self._warned_no_cv2 = False

        if self.enable_debug_view and not self.display_env:
            self.get_logger().warn('LFV2_DEBUG_VIEW=1 set but DISPLAY is empty; disabling cv2 window view.')
            self.enable_debug_view = False

        if self.enable_debug_view and self.debug_view_backend == 'mpl' and (not MPL_AVAILABLE):
            self.get_logger().warn('LFV2_DEBUG_BACKEND=mpl requested but matplotlib is unavailable; disabling debug view.')
            self.enable_debug_view = False

        if self.enable_debug_view and self.debug_view_backend == 'cv2' and (not CV2_AVAILABLE):
            self.get_logger().warn('LFV2_DEBUG_BACKEND=cv2 requested but OpenCV is unavailable; disabling debug view.')
            self.enable_debug_view = False

        # subscribers
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.armed_sub = self.create_subscription(
            Bool,
            '/rover/armed',
            self.armed_callback,
            10
        )

        # publishers
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)

        self.get_logger().info(
            (
                f'Line follower node started. image_topic={image_topic}, '
                f'cmd_vel_topic={cmd_vel_topic}, debug_image_topic={debug_image_topic}, '
                f'debug_topic={self.enable_debug_topic}, debug_snapshot={self.enable_debug_snapshot}, '
                f'debug_view={self.enable_debug_view}, debug_backend={self.debug_view_backend}, '
                f'debug_fps={self.debug_max_fps}, debug_scale={self.debug_view_scale}'
            )
        )

    """
    states
    """
    # follow
    # search
    # bridge
    # turn

    """
    callbacks
    """
    def armed_callback(self, msg: Bool):
        was_armed = self.rover_armed
        self.rover_armed = bool(msg.data)

        if self.rover_armed and not was_armed:
            self.get_logger().info('Rover armed detected. Enabling line-follow motion commands.')
        elif (not self.rover_armed) and was_armed:
            self.get_logger().warn('Rover disarmed. Holding zero cmd_vel.')

    def image_callback(self, msg):
        self.frame_count += 1

        img = self.decode_image(msg)
        if img is None:
            return

        if not self.seen_first_frame:
            self.seen_first_frame = True
            self.get_logger().info(f"First camera frame received. encoding={msg.encoding}")

        if not self.rover_armed:
            self.publish_stop()
            return

        height, width, _ = img.shape
        y0 = int(height * self.roi_top_ratio)
        y1 = int(height * self.roi_bottom_ratio)
        roi = img[y0:y1, :, :]

        blue_mask, blue_score = self.compute_blue_mask_and_score(roi)
        blue_mask = self.refine_binary_mask(blue_mask)
        edge_map = self.canny_edge_detection(roi)
        edge_mask = edge_map > 0

        # Use cleaned blue mask as primary follow signal.
        # Edge overlap remains useful as confidence/shape support.
        edge_track_mask = blue_mask & edge_mask
        edge_track_pixels = int(np.sum(edge_track_mask))
        track_mask = blue_mask
        track_pixels = int(np.sum(track_mask))

        # Corner logic scans the full frame so hard turns near the top/edges are visible,
        # while follow control still uses the lower ROI for stability.
        corner_edge_track_pixels = edge_track_pixels
        if self.corner_scan_full_image:
            blue_mask_full, _ = self.compute_blue_mask_and_score(img)
            blue_mask_full = self.refine_binary_mask(blue_mask_full)
            edge_map_full = self.canny_edge_detection(img)
            edge_mask_full = edge_map_full > 0
            corner_edge_track_mask = blue_mask_full & edge_mask_full
            corner_edge_track_pixels = int(np.sum(corner_edge_track_mask))
        else:
            corner_edge_track_mask = edge_track_mask
            blue_mask_full = blue_mask
            corner_edge_track_pixels = edge_track_pixels

        corner_blue_candidate, corner_blue_turn_sign = self.detect_corner_signature(blue_mask_full)
        corner_edge_candidate, corner_edge_turn_sign = self.detect_corner_signature(corner_edge_track_mask)
        corner_candidate = corner_blue_candidate and (
            corner_edge_candidate or (corner_edge_track_pixels >= self.corner_edge_support_min_pixels)
        )
        if corner_edge_candidate:
            candidate_turn_sign = corner_edge_turn_sign
        else:
            candidate_turn_sign = corner_blue_turn_sign
        if corner_candidate:
            self.corner_streak += 1
            if candidate_turn_sign != 0.0:
                self.corner_turn_sign = candidate_turn_sign
        else:
            self.corner_streak = 0

        corner_confirmed = self.corner_streak >= self.corner_confirm_frames

        if self.drive_state != 'turn' and corner_confirmed and track_pixels >= self.min_track_pixels:
            self.drive_state = 'turn'
            self.turn_hold_frames_left = self.corner_turn_hold_frames
            self.turn_reacquire_frames = 0
            turn_dir_str = 'LEFT' if self.corner_turn_sign > 0 else 'RIGHT'
            self.get_logger().warn(
                (
                    f"[TURN ENTER] direction={turn_dir_str}, sign={self.corner_turn_sign:+.0f}, "
                    f"near_ang={self.last_corner_near_angle:.1f}°, far_ang={self.last_corner_far_angle:.1f}°, "
                    f"confidence={self.last_corner_confidence:.2f}, hold_frames={self.corner_turn_hold_frames}, "
                    f"track_px={track_pixels}"
                )
            )

        if self.drive_state == 'turn':
            # Turn mode is edge-triggered and temporarily overrides follow to handle hard corners.
            if self.turn_hold_frames_left > 0:
                self.turn_hold_frames_left -= 1
            
            # Check for line reacquisition: enough track pixels AND no longer corner signature
            if (
                track_pixels >= self.corner_reacquire_min_pixels
                and (not corner_confirmed)
                and self.turn_hold_frames_left <= 0
            ):
                self.turn_reacquire_frames += 1
            else:
                self.turn_reacquire_frames = 0

            # Exit turn once reacquired for N stable frames
            if self.turn_reacquire_frames >= self.corner_reacquire_frames_required:
                self.drive_state = 'follow'
                self.corner_streak = 0
                self.turn_reacquire_frames = 0
                turn_dir_str = 'LEFT' if self.corner_turn_sign > 0 else 'RIGHT'
                self.get_logger().warn(
                    (
                        f"[TURN EXIT] direction={turn_dir_str}, reacquired_frames={self.corner_reacquire_frames_required}, "
                        f"track_px={track_pixels}, resuming P follow"
                    )
                )
            else:
                self.command_corner_turn()
                self.show_debug_view(
                    roi,
                    blue_mask,
                    edge_map,
                    track_mask,
                    track_pixels,
                    edge_track_pixels,
                    msg.header,
                )
                return

        if track_pixels >= self.min_track_pixels:
            self.track_lock_frames += 1
            if self.track_lock_frames < self.track_lock_required:
                self.drive_state = 'acquire'
                self.publish_stop()
                if self.frame_count % 10 == 0:
                    self.get_logger().info(
                        (
                            f"state=acquire, lock={self.track_lock_frames}/{self.track_lock_required}, "
                            f"track_px={track_pixels}, edge_track_px={edge_track_pixels}"
                        )
                    )
                self.show_debug_view(
                    roi,
                    blue_mask,
                    edge_map,
                    track_mask,
                    track_pixels,
                    edge_track_pixels,
                    msg.header,
                )
                return

            track_x = self.compute_track_center_x(
                track_mask,
                blue_score=blue_score,
                edge_mask=edge_mask
            )
            self.drive_state = 'follow'
            self.follow_track(track_x, width)

            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    (
                        f"state=follow, blue_px={int(np.sum(blue_mask))}, edge_px={int(np.sum(edge_mask))}, "
                        f"track_px={track_pixels}, edge_track_px={edge_track_pixels}, "
                        f"corner_edge_track_px={corner_edge_track_pixels}, "
                        f"corner_blue={corner_blue_candidate}, corner_edge={corner_edge_candidate}, "
                        f"lin={self.last_speed:.2f}, ang={self.last_turn:.2f}, "
                        f"err_unbiased={self.last_unbiased_error:.3f}, err_biased={self.last_biased_error:.3f}, "
                        f"offset={self.error_offset:.3f}"
                    )
                )
        else:
            self.drive_state = 'search'
            self.track_lock_frames = 0
            self.search_for_track()

        self.show_debug_view(
            roi,
            blue_mask,
            edge_map,
            track_mask,
            track_pixels,
            edge_track_pixels,
            msg.header,
        )

    """
    Helper functions
    """

    def publish_stop(self):
        cmd = Twist()
        self.vel_pub.publish(cmd)

    def show_debug_view(
        self,
        roi: np.ndarray,
        blue_mask: np.ndarray,
        edge_map: np.ndarray,
        track_mask: np.ndarray,
        track_pixels: int,
        edge_track_pixels: int,
        header,
    ):
        if (not self.enable_debug_view) and (not self.enable_debug_topic):
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        min_period = 1.0 / max(self.debug_max_fps, 0.5)
        if (now_sec - self.last_debug_time_sec) < min_period:
            return
        self.last_debug_time_sec = now_sec

        if not CV2_AVAILABLE:
            if not self._warned_no_cv2:
                self.get_logger().warn('Debug view disabled: cv2 is not available.')
                self._warned_no_cv2 = True
            return

        try:
            roi_vis = roi.copy()
            h, w, _ = roi_vis.shape
            center_x = int(w / 2)

            # Robot center line in green.
            cv2.line(roi_vis, (center_x, 0), (center_x, h - 1), (0, 255, 0), 2)

            # Tape line estimate in red.
            if self.track_x_filtered is not None:
                tape_x = int(np.clip(self.track_x_filtered, 0, w - 1))
                cv2.line(roi_vis, (tape_x, 0), (tape_x, h - 1), (0, 0, 255), 2)

            blue_u8 = (blue_mask.astype(np.uint8) * 255)
            edge_u8 = edge_map.astype(np.uint8)
            track_u8 = (track_mask.astype(np.uint8) * 255)

            blue_vis = cv2.cvtColor(blue_u8, cv2.COLOR_GRAY2BGR)
            edge_vis = cv2.cvtColor(edge_u8, cv2.COLOR_GRAY2BGR)
            track_vis = cv2.cvtColor(track_u8, cv2.COLOR_GRAY2BGR)

            cv2.putText(roi_vis, 'ROI', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(blue_vis, 'Blue Mask', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(edge_vis, 'Canny Edges', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(track_vis, 'Track Mask', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            mode = self.drive_state
            telemetry = (
                f"mode={mode}  err_u={self.last_unbiased_error:.3f}  "
                f"err_b={self.last_biased_error:.3f}  off={self.error_offset:.3f}  "
                f"ang={self.last_turn:.2f}  blue={int(np.sum(blue_mask))}  "
                f"edge={int(np.sum(edge_u8 > 0))}  edge_track={edge_track_pixels}  "
                f"near_a={self.last_corner_near_angle:.1f}  far_a={self.last_corner_far_angle:.1f}  "
                f"corner_scan={'full' if self.corner_scan_full_image else 'roi'}"
            )

            top_row = np.hstack((roi_vis, blue_vis))
            bottom_row = np.hstack((edge_vis, track_vis))
            grid = np.vstack((top_row, bottom_row))
            canvas = np.zeros((grid.shape[0] + 34, grid.shape[1], 3), dtype=np.uint8)
            canvas[:grid.shape[0], :, :] = grid
            cv2.putText(canvas, telemetry, (10, grid.shape[0] + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 255, 200), 1)

            if self.debug_view_scale < 1.0:
                target_w = max(int(canvas.shape[1] * self.debug_view_scale), 64)
                target_h = max(int(canvas.shape[0] * self.debug_view_scale), 64)
                canvas_vis = cv2.resize(canvas, (target_w, target_h), interpolation=cv2.INTER_AREA)
            else:
                canvas_vis = canvas

            if self.enable_debug_topic and (self.frame_count % self.debug_publish_every_n_frames == 0):
                self.publish_debug_image(canvas_vis, header)

            if self.enable_debug_snapshot:
                self.write_debug_snapshot(canvas_vis)

            if self.enable_debug_view:
                if self.debug_view_backend == 'mpl':
                    self.show_mpl_view(canvas_vis)
                else:
                    cv2.imshow(self.debug_window_name, canvas_vis)
                    cv2.waitKey(1)
        except Exception as exc:
            # Disable visualization if display backend is unavailable.
            if self.enable_debug_view:
                self.get_logger().warn(f'Debug window view disabled at runtime: {exc}')
                self.enable_debug_view = False

    def show_mpl_view(self, canvas_bgr: np.ndarray):
        if not MPL_AVAILABLE:
            return

        rgb = canvas_bgr[:, :, ::-1]
        if self._mpl_fig is None:
            plt.ion()
            self._mpl_fig, self._mpl_ax = plt.subplots(num=self.debug_window_name)
            self._mpl_im = self._mpl_ax.imshow(rgb)
            self._mpl_ax.set_axis_off()
            self._mpl_fig.tight_layout(pad=0.05)
            plt.show(block=False)
        else:
            self._mpl_im.set_data(rgb)

        self._mpl_fig.canvas.draw_idle()
        self._mpl_fig.canvas.flush_events()

    def publish_debug_image(self, canvas: np.ndarray, header):
        msg = Image()
        msg.header = header
        msg.height = int(canvas.shape[0])
        msg.width = int(canvas.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(canvas.shape[1] * 3)
        msg.data = canvas.tobytes()
        self.debug_image_pub.publish(msg)

    def write_debug_snapshot(self, canvas: np.ndarray):
        if (self.frame_count % max(self.debug_snapshot_every_n_frames, 1)) != 0:
            return
        if not CV2_AVAILABLE:
            return
        try:
            cv2.imwrite(self.debug_snapshot_path, canvas)
        except Exception as exc:
            if self.enable_debug_snapshot:
                self.get_logger().warn(f'Debug snapshot disabled at runtime: {exc}')
                self.enable_debug_snapshot = False

    def follow_track(self, track_x: int, width: int):
        # Smooth centroid to reduce frame-to-frame steering spikes.
        if self.track_x_filtered is None:
            self.track_x_filtered = float(track_x)
        else:
            self.track_x_filtered = (
                (1.0 - self.track_x_filter_alpha) * self.track_x_filtered
                + self.track_x_filter_alpha * float(track_x)
            )

        center_x = width / 2.0
        unbiased_error = (self.track_x_filtered - center_x) / max(center_x, 1.0)
        raw_biased_error = unbiased_error + self.error_offset

        if abs(raw_biased_error) < 0.03:
            raw_biased_error = 0.0

        biased_error = (
            (1.0 - self.error_filter_alpha) * self.last_error
            + self.error_filter_alpha * raw_biased_error
        )

        self.last_unbiased_error = float(unbiased_error)
        self.last_biased_error = float(biased_error)

        # Pure proportional steering (P-only, no derivative) - matches proven old code
        self.last_error = biased_error
        turn = -self.kp * biased_error
        turn = float(np.clip(turn, -self.max_turn, self.max_turn))

        # Rate-limit steering to prevent sudden oversteer.
        turn = float(np.clip(
            turn,
            self.last_turn - self.max_turn_step,
            self.last_turn + self.max_turn_step,
        ))

        # Constant forward speed (simple approach from proven code)
        speed = self.forward_speed

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = turn
        self.vel_pub.publish(cmd)

        self.last_speed = speed
        self.last_turn = turn
        self.lost_frames = 0

    def search_for_track(self):
        self.lost_frames += 1
        self.track_lock_frames = 0
        self.track_x_filtered = None

        # Keep searching in direction of last commanded turn.
        spin_sign = -1.0 if self.last_turn >= 0.0 else 1.0

        cmd = Twist()
        cmd.linear.x = self.search_speed
        cmd.angular.z = spin_sign * self.search_turn
        self.vel_pub.publish(cmd)

        if self.lost_frames % 10 == 0:
            self.get_logger().warn(
                (
                    f'No track detected: search mode, lost_frames={self.lost_frames}, '
                    f'ang={cmd.angular.z:.2f}, err_unbiased={self.last_unbiased_error:.3f}, '
                    f'err_biased={self.last_biased_error:.3f}, offset={self.error_offset:.3f}'
                )
            )

    def command_corner_turn(self):
        cmd = Twist()
        cmd.linear.x = self.corner_turn_speed
        cmd.angular.z = float(np.clip(self.corner_turn_sign * self.corner_turn_rate, -self.max_turn, self.max_turn))
        self.vel_pub.publish(cmd)
        self.last_speed = cmd.linear.x
        self.last_turn = cmd.angular.z

    def detect_corner_signature(self, edge_track_mask: np.ndarray):
        h, w = edge_track_mask.shape
        split = int(h * 0.42)
        split = min(max(split, 1), h - 1)

        far_mask = edge_track_mask[:split, :]
        near_mask = edge_track_mask[split:, :]

        far_px = int(np.sum(far_mask))
        near_px = int(np.sum(near_mask))

        near_angle, _ = self.estimate_orientation_deg(near_mask)
        far_angle, far_x_mean = self.estimate_orientation_deg(far_mask)

        self.last_corner_near_angle = near_angle if near_angle is not None else -1.0
        self.last_corner_far_angle = far_angle if far_angle is not None else -1.0

        if near_angle is None or far_angle is None:
            self.last_corner_confidence = 0.0
            return False, 0.0

        angle_conf = min(max((near_angle - far_angle) / 90.0, 0.0), 1.0)
        pixel_conf = min(
            near_px / max(float(self.corner_near_min_pixels), 1.0),
            far_px / max(float(self.corner_far_min_pixels), 1.0),
            1.0,
        )
        self.last_corner_confidence = 0.5 * angle_conf + 0.5 * pixel_conf

        is_corner = (
            near_px >= self.corner_near_min_pixels
            and far_px >= self.corner_far_min_pixels
            and near_angle >= self.corner_near_vertical_min_deg
            and far_angle <= self.corner_far_horizontal_max_deg
        )

        if not is_corner:
            return False, 0.0

        if far_x_mean is None:
            return True, self.corner_turn_sign

        turn_sign = 1.0 if far_x_mean < (w / 2.0) else -1.0
        return True, turn_sign

    def estimate_orientation_deg(self, mask: np.ndarray):
        ys, xs = np.where(mask)
        if xs.size < 10:
            return None, None

        x = xs.astype(np.float64)
        y = ys.astype(np.float64)
        x_mean = float(np.mean(x))
        y_mean = float(np.mean(y))
        x -= x_mean
        y -= y_mean

        cov = np.cov(np.vstack((x, y)))
        eigvals, eigvecs = np.linalg.eigh(cov)
        axis = eigvecs[:, int(np.argmax(eigvals))]

        angle = abs(np.degrees(np.arctan2(axis[1], axis[0])))
        if angle > 90.0:
            angle = 180.0 - angle
        return float(angle), x_mean

    def compute_track_center_x(
        self,
        mask: np.ndarray,
        blue_score: np.ndarray = None,
        edge_mask: np.ndarray = None,
    ) -> int:
        # Prefer legacy-style column strength (stable), boosted by edge confidence.
        if blue_score is not None:
            weighted = np.maximum(blue_score, 0.0).astype(np.float32)
            if edge_mask is not None:
                weighted *= (1.0 + self.edge_follow_boost * edge_mask.astype(np.float32))
            weighted *= mask.astype(np.float32)

            column_strength = weighted.mean(axis=0)
            if column_strength.size > 0 and float(np.max(column_strength)) >= self.min_column_peak:
                return int(np.argmax(column_strength))

        ys, xs = np.where(mask)
        if xs.size == 0:
            return mask.shape[1] // 2
        return int(np.mean(xs))

    def refine_binary_mask(self, mask: np.ndarray) -> np.ndarray:
        # Lightweight binary-mask cleanup to suppress isolated noise before tracking.
        m = mask.astype(np.uint8)
        for _ in range(2):
            p = np.pad(m, 1, mode='edge')
            neighborhood = (
                p[:-2, :-2] + p[:-2, 1:-1] + p[:-2, 2:] +
                p[1:-1, :-2] + p[1:-1, 1:-1] + p[1:-1, 2:] +
                p[2:, :-2] + p[2:, 1:-1] + p[2:, 2:]
            )
            # Majority-like filter: keep pixels with local support.
            m = (neighborhood >= 4).astype(np.uint8)
        return m.astype(bool)

    def compute_blue_mask_and_score(self, roi: np.ndarray):
        roi_f = roi.astype(np.float32)
        b = roi_f[:, :, 0]
        g = roi_f[:, :, 1]
        r = roi_f[:, :, 2]

        # Legacy blue score
        blue_score = b - 0.5 * (g + r)

        # Target-color gate
        channel_error = np.abs(roi_f - self.target_bgr)
        near_target = np.all(channel_error <= self.color_tolerance, axis=2)
        tuned_blue = near_target & (blue_score > self.blue_score_threshold)

        # Hybrid: tuned OR simple
        simple_blue = blue_score > self.simple_blue_threshold
        blue_mask = tuned_blue | simple_blue

        return blue_mask, blue_score

    # Backward-compatible helper
    def compute_blue_mask(self, roi: np.ndarray) -> np.ndarray:
        blue_mask, _ = self.compute_blue_mask_and_score(roi)
        return blue_mask

    def decode_image(self, msg: Image):
        # Support common raw camera encodings and convert to BGR.
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        try:
            if enc in ('bgr8', '8uc3'):
                return data.reshape((msg.height, msg.width, 3))

            if enc == 'rgb8':
                rgb = data.reshape((msg.height, msg.width, 3))
                return rgb[:, :, ::-1]

            if enc == 'bgra8':
                bgra = data.reshape((msg.height, msg.width, 4))
                return bgra[:, :, :3]

            if enc == 'rgba8':
                rgba = data.reshape((msg.height, msg.width, 4))
                return rgba[:, :, [2, 1, 0]]

        except ValueError:
            self.get_logger().warn(
                f"Bad image buffer size for encoding={msg.encoding}, w={msg.width}, h={msg.height}"
            )
            return None

        self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
        return None

    """ Canny Edge Detection """

    def blur_image(self, image, kernel):
        image_array = np.array(image, dtype=np.float32)
        return self.convolve2d(image_array, kernel)

    def convolve2d(self, image, kernel):
        image = np.array(image, dtype=np.float64)
        kernel = np.array(kernel, dtype=np.float64)

        k_h, k_w = kernel.shape
        pad_h = k_h // 2
        pad_w = k_w // 2

        padded_image = np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='edge')

        i_h, i_w = image.shape
        stride_window = (i_h, i_w, k_h, k_w)

        s_h, s_w = padded_image.strides
        view_strides = (s_h, s_w, s_h, s_w)

        windows = as_strided(padded_image, shape=stride_window, strides=view_strides)

        return np.sum(windows * kernel, axis=(2, 3))

    # Sobel filter
    def sobel_filters(self, image):
        image_array = np.array(image, dtype=np.float32)

        Gx = np.array([
            [-1, 0, 1],
            [-2, 0, 2],
            [-1, 0, 1],
        ], dtype=np.float32)

        Gy = np.array([
            [1, 2, 1],
            [0, 0, 0],
            [-1, -2, -1],
        ], dtype=np.float32)

        gradient_x = self.convolve2d(image_array, Gx)
        gradient_y = self.convolve2d(image_array, Gy)

        gradient_magnitude = np.hypot(gradient_x, gradient_y)
        gradient_direction = np.arctan2(gradient_y, gradient_x)

        return gradient_magnitude, gradient_direction

    def non_max_suppression(self, magnitude, direction):
        M, N = magnitude.shape
        Z = np.zeros((M, N), dtype=np.float32)

        # Normalize angles
        angle = np.rad2deg(direction) % 180

        mag = magnitude

        # Shifted images
        magN = np.pad(mag, 1, mode='edge')

        # neighbors
        d0 = (angle < 22.5) | (angle >= 157.5)
        d45 = (22.5 <= angle) & (angle < 67.5)
        d90 = (67.5 <= angle) & (angle < 112.5)
        d135 = (112.5 <= angle) & (angle < 157.5)

        # For each direction define neighbors
        West = magN[1:M + 1, 0:N]
        East = magN[1:M + 1, 2:N + 2]

        North = magN[0:M, 1:N + 1]
        South = magN[2:M + 2, 1:N + 1]

        NW = magN[0:M, 0:N]
        NE = magN[0:M, 2:N + 2]
        SW = magN[2:M + 2, 0:N]
        SE = magN[2:M + 2, 2:N + 2]

        # Performing non_max_supression
        store0 = d0 & (mag >= West) & (mag >= East)
        store90 = d90 & (mag >= North) & (mag >= South)

        store45 = d45 & (mag >= NE) & (mag >= SW)
        store135 = d135 & (mag >= NW) & (mag >= SE)

        # Combine
        store = store0 | store90 | store45 | store135
        Z[store] = mag[store]

        return Z

    # Double thresholding
    def double_thresholding(self, image):
        strong = np.int32(255)
        weak = np.int32(75)

        M, N = image.shape
        res = np.zeros((M, N), dtype=np.int32)

        nonzero = image[image > 0]
        if nonzero.size == 0:
            return res, weak, strong

        highThreshold = max(float(np.percentile(nonzero, self.canny_high_percentile)), 30.0)
        lowThreshold = max(8.0, highThreshold * self.canny_low_ratio)

        strong_x, strong_y = np.where(image >= highThreshold)
        weak_x, weak_y = np.where((image < highThreshold) & (image >= lowThreshold))

        res[strong_x, strong_y] = strong
        res[weak_x, weak_y] = weak

        return res, weak, strong

    # Edge tracking by hysteresis
    def hysteresis(self, image, weak, strong=255):
        img = image.copy()

        # Start with only strong pixels
        strong_mask = (img == strong)
        weak_mask = (img == weak)

        changed = True
        while changed:
            # Grow strong region by 1 pixel in all directions
            grown = np.pad(strong_mask, 1, mode="edge")
            grown = (
                grown[:-2, :-2] | grown[:-2, 1:-1] | grown[:-2, 2:] |
                grown[1:-1, :-2] | grown[1:-1, 1:-1] | grown[1:-1, 2:] |
                grown[2:, :-2] | grown[2:, 1:-1] | grown[2:, 2:]
            )

            # weak pixels adjacent to strong become strong
            new_strong = grown & weak_mask

            if not new_strong.any():
                changed = False
            else:
                strong_mask |= new_strong
                weak_mask &= ~new_strong

        # final image
        out = np.zeros_like(img)
        out[strong_mask] = strong
        return out

    # Complete Canny edge detection; accepts numpy BGR/RGB or grayscale frame.
    def canny_edge_detection(self, raw_image):
        raw_np = np.array(raw_image)

        # Step 1: grayscale
        if raw_np.ndim == 2:
            gray = raw_np.astype(np.float32)
        else:
            # Inputs in this node are BGR; this weighted mix is robust enough for edges.
            b = raw_np[:, :, 0].astype(np.float32)
            g = raw_np[:, :, 1].astype(np.float32)
            r = raw_np[:, :, 2].astype(np.float32)
            gray = 0.114 * b + 0.587 * g + 0.299 * r

        # Step 2: blur then gradient
        K = (1.0 / 9.0) * np.ones((3, 3), dtype=np.float32)
        blurred = self.convolve2d(gray, K)
        gradient_magnitude, gradient_direction = self.sobel_filters(blurred)

        # Step 3: non-maximum suppression
        non_max_img = self.non_max_suppression(gradient_magnitude, gradient_direction)

        # Step 4: double threshold
        thresholded_img, weak, strong = self.double_thresholding(non_max_img)

        # Step 5: hysteresis
        final_img = self.hysteresis(thresholded_img, weak, strong)

        # Return uint8 edge map (0 or 255)
        return np.uint8(final_img)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineFollowerV2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if CV2_AVAILABLE:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()