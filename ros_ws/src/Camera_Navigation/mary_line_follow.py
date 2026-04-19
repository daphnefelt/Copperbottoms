### Dark Lane Following ###

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2, logging

class LineFollower(Node):

    def __init__(self, debug=True):
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
        self.see_line = False # by default we assume we don't see the line until we do, to avoid spurious turns at startup
        self.debug_count = 0
        self.right_angle_detected = False

        self.width = 720
        self.height = 1280
        # fraction grabbed from line_follow tuning 
        offset = 0.4*int(self.width/2)


        self.debug = debug
        logging.basicConfig(level=logging.INFO)
        if debug:
            logging.basicConfig(level=logging.DEBUG) 


        # alternatively could get the top two longest segments
        self.robot_center = np.array([int(self.width/2) + offset, 0])
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

    def display_img_lines_contours(self, mask, roi, lines, contours, frame_count):
        self.get_logger().debug(f"Generating debug plot for frame {self.frame_count}")
        img_draw = roi.copy()
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img_draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([mask_bgr, img_draw])
        cv2.imshow('debug', combined)
        cv2.waitKey(1)

    def get_lines_contours(self, mask):
        #cv2_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	
        # blur to help with noise picked up
        blurred = cv2.GaussianBlur(mask, (3, 3), 0)
	
        # threshold for the lines
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
	
        # cv2.RETR_EXTERNAL: if contours are nested - retrieves only outermost
        # cv2.CHAIN_APPROX_SIMPLE: Removes redundant points - memory efficient
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        edges = cv2.Canny(thresh, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        return (lines, contours)



    def image_callback(self, msg: Image):
        self.frame_count += 1
        # skip the next 5 calls ~ 167 miliseconds
        if self.right_angle_detected:
            print(f"Skipping frame")
            if self.frame_count == 5:
                self.frame_count = 0
                self.right_angle_detected = False
            return
        print(f"Received frame")

        if msg.encoding.lower() != "bgr8":
            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
            return
        
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))


        # focus on ROI (bottom half)
        print(f"img shape is {img.shape}")
        height = img.shape[0]
        width = img.shape[1]

	

        

        # blue color thresholding
        b = img[:, :, 0].astype(np.uint16)
        g = img[:, :, 1].astype(np.uint16)
        r = img[:, :, 2].astype(np.uint16)

        # window the output to the bottom


        blue_score = b - (0.5 * (g + r)).astype(np.uint16)  # simple blue score
        blue_mask = ((blue_score > self.color_threshold)*255).astype(np.uint8)
        start = int(height*.3)
        roi = img[start:, :]
        blue_mask = blue_mask[start:, :]

        lines, contours = self.get_lines_contours(blue_mask)

        if lines == None:
            lines = []
        if contours == None:
            contours = []

        # last part of screen

        # want the two contours closest to us and closest to each other
        # maybe want something other than a for loop, but i don't anticipate many curves
        print(f"Number of contours: {len(contours)}")


        if self.frame_count % 10 == 0:
            self.display_img_lines_contours(blue_mask, roi, lines, contours, self.frame_count)


        

        
    

        # identify point in future - higher up in the image

        # if there is a right angle it needs special handling

        # what are the orientations of directions
        # print("Look ahead")

        # max_future_pixels = 50
        # # get difference between two points and reverse order so that closest points are first
        # diff_between_pts = (right[-max_future_pixels:-1] - right[-(max_future_pixels-1):])[::-1]
        # orientations = np.arctan2(diff_between_pts[:,1], diff_between_pts[:,0])
        # diff_orientations = orientations[1:] - orientations[:-1]
        # # within 20 degrees - right angle
        # if np.abs(np.max(diff_orientations) - np.pi/2) < 0.4:
        #     self.right_angle_detected = True 
        #     print("In right angle detected")
        # else:
        #     # follow local line
        #     # can turn up to 45 degrees at -2 to 2
        #     print("Following local line")
        #     self.turn_to_point(right[-15])


            
    def turn_to_point(self, pt):
        print("Trying to command movement")
        diff_between_pts = (pt - self.robot_center)
        dir = np.arctan2(diff_between_pts[1], diff_between_pts[0])
        turn_val = np.clip(-2.0 + 4*(dir + np.pi/4)*2/np.pi, -2.0, 2.0)
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = turn_val  # turn right
        self.vel_pub.publish(twist)

	






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


    

