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
        self.zones = [1,2,3,4] # follow zones by quarter of the image height (1=bottom 25%).

        # operation states
        """
        - follow (normal operation, we see the line and are following it)
            - Turn Left (when the line is on the left side of the image)
            - Turn Right (when the line is on the right side of the image)
        - search (when we lose the line, we can expand out search zones)
        - bridge (for discontinuities like the gap in the tape at the bridge)
        """
        self.state = 'search' # Search by default (when we start we look for the line)
        self.see_line = False # by default we assume we don't see the line until we do, to avoid spurious turns at startup

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
        # frame count
        self.frame_count += 1

        if msg.encoding.lower() != "bgr8":
            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
            return
        
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

        roi = self.roi_from_zone(self.zones[0])
        height, width, _ = roi.shape

        # blue color thresholding
        b = roi[:, :, 0].astype(np.float32)
        g = roi[:, :, 1].astype(np.float32)
        r = roi[:, :, 2].astype(np.float32)


        blue_score = b - 0.5 * (g + r)  # simple blue score
        blue_mask = (blue_score > self.color_threshold)
        blue_count = np.sum(blue_mask)

        # while our roi doesn't see the line, up the search zones
        while blue_count < self.min_pixels and len(self.zones) < 4:
            self.zones.append(len(self.zones) + 1) # add the next zone
            roi = self.roi_from_zone(self.zones[-1]) # get the new roi
            height, width, _ = roi.shape

            b = roi[:, :, 0].astype(np.float32)
            g = roi[:, :, 1].astype(np.float32)
            r = roi[:, :, 2].astype(np.float32)

            blue_score = b - 0.5 * (g + r)  # simple blue score
            blue_mask = (blue_score > self.color_threshold)
            blue_count = np.sum(blue_mask)
        
        # if we still don't see the line after searching all zones, go into search mode (rotate in place to try to find it)
        if blue_count < self.min_pixels:
            self.state = 'search'
            self.turn_to_find_line()
            return
        else:
            self.state = 'follow' # if we do see the line, go into follow mode

        # while in follow mode, business as usual but also handle turns
        if self.state == 'follow':

            # first we prioritize following then turn handling
            follow_error = self.calculate_follow_error(blue_mask)
            self.follow_line(follow_error)

            # turn handling (more pixels on left = turn left, more pixels on right = turn right)
            left_half = blue_mask[:, :width//2]
            right_half = blue_mask[:, width//2:]

            # turn logic
            if left_half.sum() > right_half.sum():
                self.turn_left()
            elif right_half.sum() > left_half.sum():
                self.turn_right()
        

        

        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f"tape_x={tape_x}, err={error:.3f}, blue_px={blue_count}, turn={turn:.3f}"
            )

    def turn_to_find_line(self):
        # rotate in place to try to find the line when it is lost
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = -1.0  # turn right
        self.vel_pub.publish(twist)
        self.get_logger().info("Line lost - turning to try to find it.")

    def turn_left(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = self.max_turn
        self.vel_pub.publish(twist)
        self.get_logger().info("Turning left.")

    def turn_right(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = -self.max_turn
        self.vel_pub.publish(twist)
        self.get_logger().info("Turning right.")

    def calculate_follow_error(self, blue_mask):
        height, width = blue_mask.shape
        weighted = blue_mask.astype(np.float32) * np.arange(width)  # weight by column index
        column_strength = weighted.sum(axis=0) / (blue_mask.sum(axis=0) + 1e-5)  # avoid div by zero
        tape_x = np.argmax(column_strength)
        center_x = width / 2
        error = (tape_x - center_x) / center_x  # normalize error
        error += self.error_offset
        return error

    def follow_line(self, error):
        turn = float(np.clip(-self.kp * error, -self.max_turn, self.max_turn))
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = turn
        self.vel_pub.publish(twist)
        self.get_logger().info(f"Following line with error={error:.3f}, turn={turn:.3f}.")

    def bridge_mode(self):
       # steer towards the strongest blue signal
        # steer logic
        # find tape position
        weighted = blue_score * blue_mask
        column_strength = weighted.mean(axis=0)  # average over rows
        tape_x = np.argmax(column_strength)
        error = (tape_x - center_x) / center_x  # normalize error
        error += self.error_offset
        steer_cmd = float(np.clip(-self.kp * error, -self.max_turn, self.max_turn))

        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = steer_cmd
        self.vel_pub.publish(twist)
        self.get_logger().info("Bridge mode - going straight.")

    def roi_from_zone(self, zone):
        height = img.shape[0]
        if zone == 1:
            return img[int(height*0.75):height, :, :]
        elif zone == 2:
            return img[int(height*0.5):int(height*0.75), :, :]
        elif zone == 3:
            return img[int(height*0.25):int(height*0.5), :, :]
        elif zone == 4:
            return img[0:int(height*0.25), :, :]
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


    

