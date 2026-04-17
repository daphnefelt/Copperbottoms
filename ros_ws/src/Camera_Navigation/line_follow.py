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
        self.color_threshold = 30
        self.min_pixels = 50
        self.forward_speed = 0.3
        self.max_turn = 0.65
        self.corner_min_speed = 0.20
        self.corner_turn_gain = 1.10
        self.corner_max_turn = 0.55
        self.zones = (1, 2, 3, 4) # 1=bottom quarter, 4=top quarter
        self.bridge_max_frames = 6
        self.bridge_speed = 0.24
        self.search_spin_speed = 1.0

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
        self.last_turn = 0.0
        self.last_drive_turn = 0.0
        self.last_tape_x_norm = 0.0
        self.last_seen_zone = 1
        self.preferred_zone = 1  # Prefer zone 1, only escalate if needed
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

        # Zone preference strategy: stay in zone 1 as much as possible.
        # Only escalate to higher zones if the current zone loses the line.
        # Zones checked in order: preferred_zone (1), then 2, then 3
        zone_candidates = {}
        candidate = None
        
        # Try preferred zone first (usually 1)
        for zone in range(self.preferred_zone, 4):  # 1→2→3 starting from preferred
            detected = self.detect_candidate_in_zone(img, zone)
            if detected is not None:
                zone_candidates[zone] = detected
                if candidate is None:
                    candidate = detected
                    # Stick with this zone (only update if we had to escalate)
                    if zone <= self.preferred_zone:
                        self.preferred_zone = zone

        # If preferred zone failed, try zones 1 through preferred_zone-1 (fallback)
        if candidate is None and self.preferred_zone > 1:
            for zone in range(1, self.preferred_zone):
                detected = self.detect_candidate_in_zone(img, zone)
                if detected is not None:
                    zone_candidates[zone] = detected
                    candidate = detected
                    break  # Use first one found

        # Scan zone 4 separately for corner detection (lookahead only)
        zone4_candidate = self.detect_candidate_in_zone(img, 4)
        if zone4_candidate is not None:
            zone_candidates[4] = zone4_candidate

        twist = Twist()
        blue_count = 0
        zone_used = 0
        far_zone = 0
        corner_mode = False

        if candidate is not None:
            # Use zone 4 for corner detection only if available
            if 4 in zone_candidates:
                farthest_candidate = zone_candidates[4]
            else:
                farthest_candidate = candidate  # Fall back to highest zone 1-3
            far_zone = farthest_candidate['zone']
            corner_mode = self.corner_from_farthest_zone(candidate, farthest_candidate)

            # Tape found: follow it
            twist, blue_count, zone_used = self.follow_line(candidate, width, corner_mode)

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
            # Reset preferred zone when we lose the line, so we restart at zone 1
            self.preferred_zone = 1

            if self.lost_frames <= self.bridge_max_frames:
                # Bridge short gaps, looking for tape on left or right edges
                twist, blue_count, zone_used = self.bridge_gap(img, width)
                # update state
                self.state = 'bridge'
            else:
                # Full recovery: spin in place
                twist, blue_count, zone_used = self.search_for_line()
                # update state
                self.state = 'search'

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

    def detect_candidate_in_zone(self, img, zone):
        roi = self.roi_from_zone(img, zone) # region of interest for this zone
        _, width, _ = roi.shape # width of the ROI for normalization

        b = roi[:, :, 0].astype(np.float32)
        g = roi[:, :, 1].astype(np.float32)
        r = roi[:, :, 2].astype(np.float32)
        # Simple blue detection: blue channel minus half of red and green to 
        # reduce noise from non-blue areas.
        blue_score = b - 0.5 * (g + r)
        blue_mask = (blue_score > self.color_threshold)
        blue_count = int(np.sum(blue_mask))

        # If not enough blue, we will change zones, if no blue in any zone
        # we will entually go into search mode
        if blue_count < self.min_pixels:
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

    def follow_line(self, candidate, width, corner_mode):
        # follow detected tape line with PD steering control
        error = self.calculate_follow_error(candidate['tape_x'], width)
        if abs(error) < 0.05:
            error = 0.0

        # Compute error derivative (rate of change) for damping
        error_rate = error - self.last_error
        self.last_error = error

        if corner_mode:
            # Keep enough forward drive in corners while limiting steering saturation.
            # PD: proportional + derivative damping
            p_term = -self.corner_turn_gain * self.kp * error
            d_term = -self.corner_turn_gain * self.kd * error_rate
            turn = float(np.clip(
                p_term + d_term,
                -self.corner_max_turn,
                self.corner_max_turn,
            ))
            speed = max(self.corner_min_speed, self.forward_speed * 0.65)
        else:
            # Normal follow: PD control with derivative damping
            p_term = -self.kp * error
            d_term = -self.kd * error_rate
            turn = float(np.clip(p_term + d_term, -self.max_turn, self.max_turn))
            speed = self.forward_speed

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
        b = roi[:, :, 0].astype(np.float32)
        g = roi[:, :, 1].astype(np.float32)
        r = roi[:, :, 2].astype(np.float32)
        blue_score = b - 0.5 * (g + r)
        blue_mask = (blue_score > self.color_threshold)

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
        # Drive in a gentle arc while searching so the rover keeps moving.
        spin_sign = -1.0 if self.last_error >= 0.0 else 1.0

        twist = Twist()
        twist.linear.x = 0.12
        twist.angular.z = spin_sign * self.search_spin_speed

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


    

