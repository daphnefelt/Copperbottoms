### Blue Line Following ###
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # parameters
        self.frame_count = 0
        image_topic = '/camera/color/image_raw'
        cmd_vel_topic = '/cmd_vel'

        # tuning
        self.error_offset = 0 # to account for location of the camera lens
        self.kp = 0.8
        self.color_threshold = 30
        self.min_pixels = 50
        self.forward_speed = 0.25
        self.max_turn = 1.0
        self.see_line = False # by default we assume we don't see the line until we do, to avoid spurious turns at startup

        # plotting
        self.debug_plot = True
        self.plot_interval = 10  # plot every N frames

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

        # WARP
        h, w = img.shape[:2]

        top_shift = 40   # shift top of image right (positive) or left (negative)
        bottom_shift = -225 # shift bottom of image right (positive) or left (negative)

        src = np.float32([
            [0, 0],           # top-left
            [w - top_shift, 0],       # top-right
            [0, h],        # bottom-left
            [w - bottom_shift, h]     # bottom-right
        ])

        dst = np.float32([
            [0, 0],
            [w, 0],
            [0, h],
            [w, h]
        ])

        M = cv2.getPerspectiveTransform(src, dst)
        img = cv2.warpPerspective(img, M, (w, h))

        height = img.shape[0]
        width = img.shape[1]
        
        ## OLD
        # roi_strt = int(height * 0.70) # bottom 30%
        # roi = img[roi_strt:height, :, :]

        # # blue color thresholding
        # b = roi[:, :, 0].astype(np.float32)
        # g = roi[:, :, 1].astype(np.float32)
        # r = roi[:, :, 2].astype(np.float32)


        # blue_score = b - 0.5 * (g + r)  # simple blue score
        # blue_mask = (blue_score > self.color_threshold)
        # blue_count = np.sum(blue_mask)

        # if blue_count >= self.min_pixels:
        #     self.see_line = True
            
        # if blue_count < self.min_pixels:
        #     self.get_logger().info("No tape detected, stopping.")
        #     self.see_line = False
        #     twist = Twist()
        #     twist.linear.x = 0.0
        #     twist.angular.z = 0.0
        #     self.vel_pub.publish(twist)

        #     return

        # # find tape position
        # weighted = blue_score * blue_mask
        # column_strength = weighted.mean(axis=0)  # average over rows
        # tape_x = np.argmax(column_strength)

        ## NEW
        # roi
        roi_strt = int(height * 0.25) # bottom 75%
        roi = img[roi_strt:height, :, :]
        width = roi.shape[1]
        height = roi.shape[0]

        rgb = np.array([164, 108, 7])
        plus_minus = np.array([50, 50, 90])
        mask = cv2.inRange(roi, rgb - plus_minus, rgb + plus_minus)

        # check if tape is visible
        if np.sum(mask) < self.min_pixels * 255:
            self.get_logger().info("No tape detected, stopping.")
            self.see_line = False
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
            return

        self.see_line = True

        # find lines w Canny and Hough
        edges = cv2.Canny(mask, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        def score_func(x, y):
            score = (-y) + (x) # prefer higher and more right
            return score

        # find highest score point to get tape_x
        in_center_third = True
        third = width / 3
        tape_x = width // 2  # default to center if no lines found
        tape_y = height // 2
        if lines is not None:
            max_score = -float('inf')
            for line in lines:
                x1, y1, x2, y2 = line[0]

                if not (third < x1 < 2 * third) or not (third < x2 < 2 * third):
                    in_center_third = False

                    score1 = score_func(x1, y1)
                    if score1 > max_score:
                        max_score = score1
                        tape_x = x1
                        tape_y = y1
                    score2 = score_func(x2, y2)
                    if score2 > max_score:
                        max_score = score2
                        tape_x = x2
                        tape_y = y2

        if in_center_third: # old method
            print("CENTER METHOD")
            roi_strt = int(height * 0.70) # bottom 30%
            roi = img[roi_strt:height, :, :]

            # blue color thresholding
            b = roi[:, :, 0].astype(np.float32)
            g = roi[:, :, 1].astype(np.float32)
            r = roi[:, :, 2].astype(np.float32)

            blue_score = b - 0.5 * (g + r)  # simple blue score
            blue_mask = (blue_score > self.color_threshold)
            blue_count = np.sum(blue_mask)
            # find tape position
            weighted = blue_score * blue_mask
            column_strength = weighted.mean(axis=0)  # average over rows
            tape_x = np.argmax(column_strength)

        # error
        center_x = width / 2
        error = (tape_x - center_x) / center_x  # normalize error
        error += self.error_offset

        # control
        turn = float(np.clip(-self.kp * error, -self.max_turn, self.max_turn))

        # speed + publish
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = turn
        self.vel_pub.publish(twist)
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f"tape_x={tape_x}, err={error:.3f}, turn={turn:.3f}"
            )

            if self.debug_plot:
                self.get_logger().info(f"Generating debug plot for frame {self.frame_count}")
                target_y = tape_y if lines is not None else img.shape[0] // 2
                img_draw = img.copy()
                if lines is not None:
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        cv2.line(img_draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(img_draw, (tape_x, target_y), 10, (0, 0, 255), -1)
                cv2.line(img_draw, (width // 3, 0), (width // 3, height), (0, 255, 255), 1)
                cv2.line(img_draw, (2 * width // 3, 0), (2 * width // 3, height), (0, 255, 255), 1)
                cv2.putText(img_draw, f'turn={turn:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                combined = np.hstack([mask_bgr, img_draw])
                cv2.imshow('debug', combined)
                cv2.waitKey(1)

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