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

        # Evaluate all zones every frame and score detections by continuity + confidence.
        candidates = []
        for zone in self.zones:
            candidate = self.detect_candidate_in_zone(img, zone)
            
            if candidate is None:
                continue

            x_norm = self.pixel_to_norm(candidate['tape_x'], width)
            continuity = 1.0 - min(abs(x_norm - self.last_tape_x_norm), 2.0) / 2.0
            count_term = min(candidate['blue_count'] / 1000.0, 1.5)
            bottom_bonus = (5 - zone) * 0.10
            candidate['score'] = 1.2 * continuity + count_term + bottom_bonus
            candidate['x_norm'] = x_norm
            candidates.append(candidate)

        twist = Twist()

        if candidates:
            best = max(candidates, key=lambda c: c['score'])
            error = self.calculate_follow_error(best['tape_x'], width)
            is_corner = self.is_hard_turn_candidate(best)

            if is_corner:
                # Slow down and allow stronger turn so 90-degree corners can be acquired.
                turn = float(np.clip(-1.25 * self.kp * error, -self.max_turn, self.max_turn))
                speed = max(0.12, self.forward_speed * 0.45)
            else:
                turn = float(np.clip(-self.kp * error, -self.max_turn, self.max_turn))
                speed = self.forward_speed

            twist.linear.x = speed
            twist.angular.z = turn

            self.state = 'follow'
            self.see_line = True
            self.lost_frames = 0
            self.last_error = error
            self.last_turn = turn
            self.last_tape_x_norm = best['x_norm']
            self.last_seen_zone = best['zone']
            blue_count = best['blue_count']
            zone_used = best['zone']
        else:
            self.see_line = False
            self.lost_frames += 1

            if self.lost_frames <= self.bridge_max_frames:
                # Bridge short gaps by holding recent steering with a slow forward crawl.
                self.state = 'bridge'
                twist.linear.x = self.bridge_speed
                twist.angular.z = float(np.clip(self.last_turn * 0.8, -self.max_turn, self.max_turn))
                blue_count = 0
                zone_used = 0
            else:
                # Full recovery search: rotate in place until a zone reacquires tape.
                self.state = 'search'
                spin_sign = -1.0 if self.last_error >= 0.0 else 1.0
                twist.linear.x = 0.0
                twist.angular.z = spin_sign * self.search_spin_speed
                blue_count = 0
                zone_used = 0

        self.vel_pub.publish(twist)

        if self.frame_count % 10 == 0:
            self.get_logger().info(
                (
                    f"state={self.state}, zone={zone_used}, blue_px={blue_count}, "
                    f"lost_frames={self.lost_frames}, lin={twist.linear.x:.2f}, ang={twist.angular.z:.2f}"
                )
            )

    def detect_candidate_in_zone(self, img, zone):
        roi = self.roi_from_zone(img, zone)
        _, width, _ = roi.shape

        b = roi[:, :, 0].astype(np.float32)
        g = roi[:, :, 1].astype(np.float32)
        r = roi[:, :, 2].astype(np.float32)
        blue_score = b - 0.5 * (g + r)
        blue_mask = (blue_score > self.color_threshold)
        blue_count = int(np.sum(blue_mask))

        if blue_count < self.min_pixels:
            return None

        # Use weighted column score for x position with better noise rejection.
        weighted = np.where(blue_mask, np.maximum(blue_score, 0.0), 0.0)
        column_strength = weighted.sum(axis=0)
        tape_x = int(np.argmax(column_strength))

        left_count = int(np.sum(blue_mask[:, :width // 2]))
        right_count = int(np.sum(blue_mask[:, width // 2:]))

        return {
            'zone': zone,
            'tape_x': tape_x,
            'blue_count': blue_count,
            'left_count': left_count,
            'right_count': right_count,
        }

    def is_hard_turn_candidate(self, candidate):
        left_count = candidate['left_count']
        right_count = candidate['right_count']
        total = max(left_count + right_count, 1)
        side_imbalance = abs(left_count - right_count) / total

        x_norm = candidate['x_norm']
        near_edge = abs(x_norm) > 0.65
        return near_edge and side_imbalance > 0.30

    def pixel_to_norm(self, x, width):
        if width <= 1:
            return 0.0
        return (2.0 * x / float(width - 1)) - 1.0

    def calculate_follow_error(self, tape_x, width):
        center_x = width / 2
        error = (tape_x - center_x) / center_x
        error += self.error_offset
        return error

    def roi_from_zone(self, img, zone):
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


    

