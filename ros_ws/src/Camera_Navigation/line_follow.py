### Dark Lane Following ###

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # parameters
        self.frame_count = 0
        image_topic = '/camera/color/image_raw'
        cmd_vel_topic = '/cmd_vel'

        # tuning
        self.error_offset = -0.4 # to account for location of the camera lens
        self.kp = 0.8
        self.color_threshold = 30
        self.min_pixels = 50
        self.forward_speed = 0.3
        self.max_turn = 1.0
        self.zones = (1, 2, 3, 4) # 1=bottom quarter, 4=top quarter
        self.bridge_max_frames = 12
        self.bridge_speed = 0.18
        self.search_spin_speed = 0.8

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
        self.last_tape_x_norm = 0.0
        self.last_seen_zone = 1

        # subscribers
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # publishers
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            f'Line follower node started. image_topic={image_topic}, cmd_vel_topic={cmd_vel_topic}'
        )

    def image_callback(self, msg: Image):
        self.frame_count += 1

        if msg.encoding.lower() != "bgr8":
            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
            return

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        _, width, _ = img.shape

        # Priority-based zone search: try lowest zone first, step up if no detection.
        candidate = None
        for zone in self.zones:
            candidate = self.detect_candidate_in_zone(img, zone)
            if candidate is not None:
                break  # Found tape, stop searching higher zones.

        twist = Twist()
        blue_count = 0
        zone_used = 0

        if candidate is not None:
            # Tape found: follow it
            twist, blue_count, zone_used = self.follow_line(candidate, width)

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
                    f"lost_frames={self.lost_frames}, lin={twist.linear.x:.2f}, ang={twist.angular.z:.2f}"
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
            'blue_count': blue_count,
            'left_count': left_count,
            'right_count': right_count,
        }

    def follow_line(self, candidate, width):
        # follow detected tape line with steering and corner
        error = self.calculate_follow_error(candidate['tape_x'], width)
        is_corner = self.is_side_heavy_turn(candidate)

        if is_corner:
            # Slow down and turn harder at 90-degree corners
            turn = float(np.clip(-1.25 * self.kp * error, -self.max_turn, self.max_turn))
            speed = max(0.12, self.forward_speed * 0.45)
        else:
            turn = float(np.clip(-self.kp * error, -self.max_turn, self.max_turn))
            speed = self.forward_speed

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = turn

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

    def search_for_line(self):
        # Spin in place to search for the line after bridge timeout, 
        # using last error sign to determine direction.
        spin_sign = -1.0 if self.last_error >= 0.0 else 1.0

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = spin_sign * self.search_spin_speed

        return twist, 0, 0

    def is_side_heavy_turn(self, candidate):
        # looks for hard turns (like 90 degree corners) 
        # where tape is heavily skewed to one side of the image
        left_count = candidate['left_count']
        right_count = candidate['right_count']
        total = max(left_count + right_count, 1)
        side_imbalance = abs(left_count - right_count) / total
        # If more than 40% of pixels are on one side, it's a corner cue.
        return side_imbalance > 0.40 # we can tune this

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()


    

