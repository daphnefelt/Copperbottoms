### Dark Lane Following ###

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # parameters
        self.frame_count = 0
        image_topic = '/camera/color/image_raw'
        cmd_vel_topic = '/cmd_vel'

        # tuning
        self.error_offset = -0.3 # to account for location of the camera lens
        self.kp = 0.5
        self.kd = 0.15  # derivative gain for damping oscillations
        self.kh = 0.30  # heading-alignment gain using near lookahead zone
        self.error_filter_alpha = 0.35
        self.max_turn_step = 0.10
        self.color_threshold = 30
        self.min_pixels = 50
        self.search_min_pixels = 30
        self.forward_speed = 0.22
        self.max_turn = 0.65
        self.corner_min_speed = 0.20
        self.corner_turn_gain = 1.10
        self.corner_max_turn = 0.55
        self.zones = (1, 2, 3, 4) # 1=bottom quarter, 4=top quarter
        self.bridge_max_frames = 6
        self.bridge_speed = 0.20
        self.search_spin_speed = 1.0
        self.search_circle_turn_speed = 0.75
        self.startup_relaxed_frames = 14
        self.relaxed_pixel_floor = 20
        self.relaxed_color_tolerance_scale = 1.30
        self.relaxed_color_threshold_delta = 10
        self.enable_adaptive_color = True
        self.color_update_period_frames = 4
        self.color_update_alpha = 0.08
        self.color_update_min_blue_pixels = 120
        self.max_target_drift = np.array([35.0, 35.0, 35.0], dtype=np.float32)
        self.color_update_log_period_frames = 30

        # Target tape color in BGR space (camera uses BGR ordering)
        self.target_bgr = np.array([204.0, 146.0, 39.0], dtype=np.float32)
        self.base_target_bgr = self.target_bgr.copy()
        self.color_tolerance = np.array([55.0, 50.0, 45.0], dtype=np.float32)

        # operation states
        """
        - follow (normal operation, we see the line and are following it)
            - Turn Left (when the line is on the left side of the image)
            - Turn Right (when the line is on the right side of the image)
        - search (when we lose the line, we can expand out search zones)
        - bridge (for discontinuities like the gap in the tape at the bridge)
        """
        self.state = 'search' # search by default at startup
        self.see_line = False
        self.lost_frames = 0
        self.last_error = 0.0
        self.filtered_error = 0.0
        self.last_turn = 0.0
        self.last_drive_turn = 0.0
        self.last_tape_x_norm = 0.0
        self.last_seen_zone = 1
        self.preferred_zone = 1  # Prefer zone 1, only escalate if needed
        self.search_mode = 'probe'  # probe zones 2->3->4 before circle drive
        self.search_probe_zones = [2, 3, 4]
        self.search_probe_index = 0
        self.last_search_mode_logged = None
        self.last_color_update_frame = -999
        self.seen_first_frame = False
        self.rover_armed = False
        self._logged_waiting_for_arm = False

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

        self.get_logger().info(
            f'Line follower node started. image_topic={image_topic}, cmd_vel_topic={cmd_vel_topic}'
        )

    def armed_callback(self, msg: Bool):
        was_armed = self.rover_armed
        self.rover_armed = bool(msg.data)

        if self.rover_armed and not was_armed:
            self.get_logger().info('Rover armed detected. Enabling line-follow motion commands.')
            self._logged_waiting_for_arm = False
        elif (not self.rover_armed) and was_armed:
            self.get_logger().warn('Rover disarmed. Holding zero cmd_vel.')

    def image_callback(self, msg: Image):
        self.frame_count += 1

        img = self.decode_image(msg)
        if img is None:
            return

        if not self.seen_first_frame:
            self.seen_first_frame = True
            self.get_logger().info(f"First camera frame received. encoding={msg.encoding}")

        _, width, _ = img.shape

        if not self.rover_armed:
            # Wait until rover node reports armed; avoid sending drive commands early.
            stop = Twist()
            self.vel_pub.publish(stop)
            if not self._logged_waiting_for_arm and self.frame_count % 30 == 0:
                self.get_logger().warn('Waiting for /rover/armed=True before driving...')
                self._logged_waiting_for_arm = True
            return

        zone_candidates = self.collect_zone_candidates(img)
        if not zone_candidates and self.should_use_relaxed_detection():
            zone_candidates = self.collect_zone_candidates(
                img,
                min_pixels=self.relaxed_pixel_floor,
                relaxed_color=True
            )
        candidate = self.select_primary_candidate(zone_candidates)

        twist = Twist()
        blue_count = 0
        zone_used = 0
        far_zone = 0
        corner_mode = False
        scan_zone = 0
        circle_drive = False

        if candidate is not None:
            self.search_mode = 'probe'
            self.search_probe_index = 0
            self.last_search_mode_logged = None

            # Use zone 4 for corner detection only if available
            if 4 in zone_candidates:
                farthest_candidate = zone_candidates[4]
            else:
                farthest_candidate = candidate  # Fall back to highest zone 1-3
            far_zone = farthest_candidate['zone']
            corner_mode = self.corner_from_farthest_zone(candidate, farthest_candidate)

            # Use near lookahead only (zone 2 preferred) to avoid over-bias on snaky tape.
            heading_error = self.compute_near_lookahead_heading_error(candidate, zone_candidates)

            # Tape found: follow it
            twist, blue_count, zone_used = self.follow_line(
                candidate,
                width,
                corner_mode,
                heading_error
            )

            # Adapt tape color slowly only during stable follow.
            self.maybe_update_target_color(img, candidate)

            # update state and tracking info for potential future use 
            # in bridging or search
            self.state = 'follow'
            self.see_line = True
            self.lost_frames = 0
            self.last_error = self.calculate_follow_error(candidate['tape_x'], width)
            self.last_turn = twist.angular.z
            self.last_tape_x_norm = self.pixel_to_norm(candidate['tape_x'], width)
            self.last_seen_zone = candidate['zone']
            
        else:
            # No tape found: bridge or search
            self.see_line = False
            self.lost_frames += 1
            if self.lost_frames == 1:
                self.get_logger().warn('Line lost. Entering recovery flow (bridge -> probe -> circle).')
            # Reset preferred zone when we lose the line, so we restart at zone 1
            self.preferred_zone = 1

            if self.lost_frames <= self.bridge_max_frames:
                # Bridge short gaps, looking for tape on left or right edges
                twist, blue_count, zone_used = self.bridge_gap(img, width)
                # update state
                self.state = 'bridge'
            else:
                search_result = self.run_recovery_search(img, width)
                twist = search_result['twist']
                blue_count = search_result['blue_count']
                zone_used = search_result['zone_used']
                scan_zone = search_result['scan_zone']
                circle_drive = search_result['circle_drive']
                self.state = search_result['state']
                self.log_search_mode(scan_zone, circle_drive)

        # send velocity command
        self.vel_pub.publish(twist)

        if self.frame_count % 10 == 0:
            self.get_logger().info(
                (
                    f"state={self.state}, zone={zone_used}, blue_px={blue_count}, "
                    f"lost_frames={self.lost_frames}, corner={corner_mode}, far_zone={far_zone}, "
                    f"lin={twist.linear.x:.2f}, ang={twist.angular.z:.2f}"
                )
            )

    def collect_zone_candidates(self, img, min_pixels=None, relaxed_color=False):
        zone_candidates = {}
        for zone in self.zones:
            detected = self.detect_candidate_in_zone(
                img,
                zone,
                min_pixels=min_pixels,
                relaxed_color=relaxed_color
            )
            if detected is not None:
                zone_candidates[zone] = detected
        return zone_candidates

    def should_use_relaxed_detection(self):
        # Camera auto-exposure/white-balance can drift in first frames.
        startup_window = self.frame_count <= self.startup_relaxed_frames
        recent_loss = 0 < self.lost_frames <= 2
        return startup_window or recent_loss

    def select_primary_candidate(self, zone_candidates):
        # Prefer near zones to keep control grounded in local geometry.
        for zone in (1, 2, 3):
            if zone in zone_candidates:
                return zone_candidates[zone]
        return None

    def compute_near_lookahead_heading_error(self, candidate, zone_candidates):
        # Estimate heading from near lookahead only to avoid overreacting to distant snake segments.
        lookahead_candidate = zone_candidates.get(2)
        if lookahead_candidate is None:
            lookahead_candidate = zone_candidates.get(3)
        if lookahead_candidate is None:
            return 0.0

        near_norm = self.pixel_to_norm(candidate['tape_x'], candidate['width'])
        lookahead_norm = self.pixel_to_norm(
            lookahead_candidate['tape_x'],
            lookahead_candidate['width']
        )
        return float(np.clip(lookahead_norm - near_norm, -0.8, 0.8))

    def run_recovery_search(self, img, width):
        if self.search_mode == 'probe':
            return self.run_probe_search(img, width)
        return self.run_circle_search(img, width)

    def run_probe_search(self, img, width):
        scan_zone = self.search_probe_zones[self.search_probe_index]
        probe_candidate = self.detect_candidate_in_zone(
            img,
            scan_zone,
            min_pixels=self.search_min_pixels
        )

        if probe_candidate is not None:
            twist, blue_count, zone_used = self.follow_line(
                probe_candidate,
                width,
                corner_mode=False,
                heading_error=0.0
            )
            self.see_line = True
            self.lost_frames = 0
            self.search_probe_index = 0
            self.search_mode = 'probe'
            self.last_turn = twist.angular.z
            self.last_search_mode_logged = None
            return {
                'twist': twist,
                'blue_count': blue_count,
                'zone_used': zone_used,
                'scan_zone': scan_zone,
                'circle_drive': False,
                'state': 'follow',
            }

        self.search_probe_index += 1
        if self.search_probe_index >= len(self.search_probe_zones):
            self.search_probe_index = 0
            self.search_mode = 'circle'

        twist = Twist()
        twist.linear.x = 0.20
        twist.angular.z = 0.0
        return {
            'twist': twist,
            'blue_count': 0,
            'zone_used': scan_zone,
            'scan_zone': scan_zone,
            'circle_drive': False,
            'state': 'search',
        }

    def run_circle_search(self, img, width):
        # In circle mode, scan every frame before commanding arc motion.
        zone_candidates = self.collect_zone_candidates(img, min_pixels=self.search_min_pixels)
        reacquired = self.select_primary_candidate(zone_candidates)

        if reacquired is not None:
            twist, blue_count, zone_used = self.follow_line(
                reacquired,
                width,
                corner_mode=False,
                heading_error=0.0
            )
            self.see_line = True
            self.lost_frames = 0
            self.search_probe_index = 0
            self.search_mode = 'probe'
            self.last_turn = twist.angular.z
            self.last_search_mode_logged = None
            return {
                'twist': twist,
                'blue_count': blue_count,
                'zone_used': zone_used,
                'scan_zone': 0,
                'circle_drive': False,
                'state': 'follow',
            }

        twist, blue_count, zone_used = self.search_for_line()
        return {
            'twist': twist,
            'blue_count': blue_count,
            'zone_used': zone_used,
            'scan_zone': 0,
            'circle_drive': True,
            'state': 'search',
        }

    def log_search_mode(self, scan_zone, circle_drive):
        mode = 'circle' if circle_drive else 'probe'
        mode_changed = mode != self.last_search_mode_logged
        should_log = mode_changed or (self.frame_count % 5 == 0)
        if not should_log:
            return

        if circle_drive:
            self.get_logger().warn('Lost line recovery: mode=circle, scanning zones=all(1-4)')
        else:
            self.get_logger().warn(f'Lost line recovery: mode=probe, scanning zone={scan_zone}')
        self.last_search_mode_logged = mode

    def maybe_update_target_color(self, img, candidate):
        # Keep adaptive color updates conservative to avoid drift onto background.
        if not self.enable_adaptive_color:
            return

        if self.lost_frames != 0:
            return

        if candidate['zone'] not in (1, 2):
            return

        if self.should_use_relaxed_detection():
            return

        frames_since_update = self.frame_count - self.last_color_update_frame
        if frames_since_update < self.color_update_period_frames:
            return

        roi = self.roi_from_zone(img, candidate['zone'])
        blue_mask, _ = self.compute_blue_mask(roi, relaxed_color=False)
        blue_count = int(np.sum(blue_mask))
        if blue_count < self.color_update_min_blue_pixels:
            return

        roi_f = roi.astype(np.float32)
        measured_bgr = roi_f[blue_mask].mean(axis=0)
        blended = (1.0 - self.color_update_alpha) * self.target_bgr + self.color_update_alpha * measured_bgr

        lower = self.base_target_bgr - self.max_target_drift
        upper = self.base_target_bgr + self.max_target_drift
        self.target_bgr = np.clip(blended, lower, upper)
        self.last_color_update_frame = self.frame_count

        if self.frame_count % self.color_update_log_period_frames == 0:
            self.get_logger().info(
                (
                    f"adaptive_color target_bgr="
                    f"({self.target_bgr[0]:.1f}, {self.target_bgr[1]:.1f}, {self.target_bgr[2]:.1f})"
                )
            )

    def detect_candidate_in_zone(self, img, zone, min_pixels=None, relaxed_color=False):
        roi = self.roi_from_zone(img, zone) # region of interest for this zone
        _, width, _ = roi.shape # width of the ROI for normalization

        blue_mask, blue_score = self.compute_blue_mask(roi, relaxed_color=relaxed_color)
        blue_count = int(np.sum(blue_mask))
        required_pixels = self.min_pixels if min_pixels is None else int(min_pixels)

        # If not enough blue, we will change zones, if no blue in any zone
        # we will entually go into search mode
        if blue_count < required_pixels:
            return None

        # Use weighted column score for x position with better noise rejection.
        weighted = np.where(blue_mask, np.maximum(blue_score, 0.0), 0.0)
        column_strength = weighted.sum(axis=0)
        tape_x = int(np.argmax(column_strength))

        # blue pixels on left half of image (for turning based systems)
        left_count = int(np.sum(blue_mask[:, :width // 2]))
        # blue pixels on right half of image (for turning based systems)
        right_count = int(np.sum(blue_mask[:, width // 2:]))

        return {
            'zone': zone,
            'tape_x': tape_x,
            'width': width,
            'blue_count': blue_count,
            'left_count': left_count,
            'right_count': right_count,
        }

    def follow_line(self, candidate, width, corner_mode, heading_error=0.0):
        # follow detected tape line with PD steering control
        raw_error = self.calculate_follow_error(candidate['tape_x'], width)
        if abs(raw_error) < 0.05:
            raw_error = 0.0

        # Low-pass filter the lateral error to reduce steering twitch/oversteer.
        self.filtered_error = (
            (1.0 - self.error_filter_alpha) * self.filtered_error
            + self.error_filter_alpha * raw_error
        )
        error = self.filtered_error

        # Compute error derivative (rate of change) for damping
        error_rate = error - self.last_error
        self.last_error = error

        if corner_mode:
            # Keep enough forward drive in corners while limiting steering saturation.
            # PD: proportional + derivative damping
            p_term = -self.corner_turn_gain * self.kp * error
            d_term = -self.corner_turn_gain * self.kd * error_rate
            h_term = -self.corner_turn_gain * self.kh * heading_error
            turn = float(np.clip(
                p_term + d_term + h_term,
                -self.corner_max_turn,
                self.corner_max_turn,
            ))
            speed = max(self.corner_min_speed, self.forward_speed * 0.65)
        else:
            # Normal follow: PD control with derivative damping
            p_term = -self.kp * error
            d_term = -self.kd * error_rate
            h_term = -self.kh * heading_error
            target_turn = p_term + d_term + h_term
            turn = float(np.clip(target_turn, -self.max_turn, self.max_turn))
            speed = self.forward_speed

        # Rate-limit steering changes to prevent sudden oversteer spikes.
        turn = float(np.clip(
            turn,
            self.last_drive_turn - self.max_turn_step,
            self.last_drive_turn + self.max_turn_step,
        ))

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = turn

        self.last_drive_turn = turn

        return twist, candidate['blue_count'], candidate['zone']

    def bridge_gap(self, img, width):
        # bridge short gaps by crawling forward and steering toward visible edges.
        # Hold last steering + crawl forward
        base_turn = float(np.clip(self.last_turn * 0.8, -self.max_turn, self.max_turn))

        # Scan bottom zone for any blue on left or right edges to guide bridging
        roi = self.roi_from_zone(img, 1)
        _, roi_width, _ = roi.shape
        blue_mask, _ = self.compute_blue_mask(roi)

        # Check left and right fifths for tape presence
        left_fifth = int(roi_width / 5)
        right_fifth = int(4 * roi_width / 5)
        left_edge_count = int(np.sum(blue_mask[:, :left_fifth]))
        right_edge_count = int(np.sum(blue_mask[:, right_fifth:]))
        center_count = int(np.sum(blue_mask[:, left_fifth:right_fifth]))

        # Bias turn toward visible tape on the edges
        edge_bias = 0.0
        if left_edge_count > center_count and left_edge_count > right_edge_count:
            edge_bias = -0.3  # Steer left
        elif right_edge_count > center_count and right_edge_count > left_edge_count:
            edge_bias = 0.3   # Steer right

        # Combine base turn with edge bias, ensuring we don't exceed max turn limits.
        turn = float(np.clip(base_turn + edge_bias, -self.max_turn, self.max_turn))

        twist = Twist()
        twist.linear.x = self.bridge_speed
        twist.angular.z = turn

        total_blue = left_edge_count + center_count + right_edge_count
        return twist, total_blue, 0

    def compute_blue_mask(self, roi, relaxed_color=False):
        # Match a tight BGR neighborhood around the known tape color.
        roi_f = roi.astype(np.float32)
        tolerance = self.color_tolerance
        threshold = self.color_threshold
        if relaxed_color:
            tolerance = self.color_tolerance * self.relaxed_color_tolerance_scale
            threshold = max(5.0, self.color_threshold - self.relaxed_color_threshold_delta)

        channel_error = np.abs(roi_f - self.target_bgr)
        near_target = np.all(channel_error <= tolerance, axis=2)

        # Keep a blue dominance term to reject similarly colored noise.
        b = roi_f[:, :, 0]
        g = roi_f[:, :, 1]
        r = roi_f[:, :, 2]
        blue_score = b - 0.45 * (g + r)
        blue_mask = near_target & (blue_score > threshold)

        return blue_mask, blue_score

    def decode_image(self, msg: Image):
        # Support common raw camera encodings and convert to BGR.
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        try:
            if enc in ("bgr8", "8uc3"):
                return data.reshape((msg.height, msg.width, 3))

            if enc == "rgb8":
                rgb = data.reshape((msg.height, msg.width, 3))
                return rgb[:, :, ::-1]

            if enc == "bgra8":
                bgra = data.reshape((msg.height, msg.width, 4))
                return bgra[:, :, :3]

            if enc == "rgba8":
                rgba = data.reshape((msg.height, msg.width, 4))
                return rgba[:, :, [2, 1, 0]]

        except ValueError:
            self.get_logger().warn(
                f"Bad image buffer size for encoding={msg.encoding}, w={msg.width}, h={msg.height}"
            )
            return None

        self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
        return None

    def search_for_line(self):
        # Circle-drive search mode; image callback still checks every frame for reacquire.
        spin_sign = -1.0 if self.last_error >= 0.0 else 1.0

        twist = Twist()
        twist.linear.x = 0.20
        twist.angular.z = spin_sign * self.search_circle_turn_speed

        return twist, 0, 0

    def corner_from_farthest_zone(self, primary_candidate, farthest_candidate):
        # Corner mode comes from the highest visible zone, not bottom-only data.
        if farthest_candidate['zone'] <= primary_candidate['zone']:
            return False

        left_count = farthest_candidate['left_count']
        right_count = farthest_candidate['right_count']
        total = max(left_count + right_count, 1)
        side_imbalance = abs(left_count - right_count) / total

        far_x_norm = self.pixel_to_norm(farthest_candidate['tape_x'], farthest_candidate['width'])
        primary_x_norm = self.pixel_to_norm(primary_candidate['tape_x'], primary_candidate['width'])
        near_edge = abs(far_x_norm) > 0.65

        # Require consistent direction between near and far zones to avoid false latches.
        same_turn_direction = (far_x_norm * primary_x_norm) >= 0.0

        return side_imbalance > 0.55 and near_edge and same_turn_direction

    def pixel_to_norm(self, x, width):
        # Convert pixel x position to normalized -1.0 (left) to 1.0 (right)
        # for consistent error scaling.
        if width <= 1:
            return 0.0
        return (2.0 * x / float(width - 1)) - 1.0

    def calculate_follow_error(self, tape_x, width):
        # Calculate error as normalized distance from tape_x to center of image, 
        # with offset for camera position.
        center_x = width / 2
        error = (tape_x - center_x) / center_x
        error += self.error_offset
        return error

    def roi_from_zone(self, img, zone):
        # calculate region of interest within image based on zone number
        height = img.shape[0]
        if zone == 1:
            return img[int(height * 0.75):height, :, :]
        elif zone == 2:
            return img[int(height * 0.5):int(height * 0.75), :, :]
        elif zone == 3:
            return img[int(height * 0.25):int(height * 0.5), :, :]
        elif zone == 4:
            return img[0:int(height * 0.25), :, :]
        else:
            raise ValueError(f"Invalid zone: {zone}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


    

