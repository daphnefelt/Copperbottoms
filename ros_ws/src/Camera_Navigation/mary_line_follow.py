### Dark Lane Following ###

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2, logging, math, time

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
        self.forward_speed = 0.2
        self.max_turn = 1.0
        self.see_line = False # by default we assume we don't see the line until we do, to avoid spurious turns at startup
        self.debug_count = 0
        self.right_angle_detected = False
        self.timer = 0

        # 164, 108, 7
        # 50, 50, 90

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


    def pub_vel_cmd(self, vel, dir):
        self.get_logger().debug(f"Vel Cmd: {vel} vel {dir} direction")
        twist = Twist()
        twist.linear.x = vel
        twist.angular.z = dir
        self.vel_pub.publish(twist)
        

    def display_img_lines_contours(self, mask, roi, lines, contours, frame_count):
        self.get_logger().debug(f"Generating debug plot for frame {self.frame_count}")
        img_draw = roi.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img_draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if contours is not None:
            cv2.drawContours(img_draw, contours, -1, (255, 0, 0), 3)

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


    # segment the image before getting the lines if you only want to check for right angles in a certain zone
    def detect_right_angle(self, lines):
        lines = lines.reshape((-1, 4))
        orientations = np.arctan2(lines[:,3] - lines[:,1], lines[:,2] - lines[:,0])
        degree_range = 2
        orientation_bins = np.arange(-math.radians(degree_range), math.radians(degree_range) + .0001, math.radians(1))


        pixel_dist_threshold = 600
        for i in range(0, degree_range*2):
            o_mask = np.logical_and(orientations < orientation_bins[i+1], orientations >= orientation_bins[i])
            segments = lines[o_mask]
            if(len(segments) == 0):
                continue

            # should be right of robot - might want only segments past  a certain amount - for now don't worry
            
            dist = np.sum(np.sqrt(np.power(segments[:, 3] - segments[:, 1], 2) + np.power(segments[:, 2] - segments[:, 0], 2)))     
            if dist > pixel_dist_threshold:
                return True
        return False
    

    def end_turn_state_callback(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0  # straight
        self.vel_pub.publish(twist)
        self.right_angle_detected = False 

    



    def image_callback(self, msg: Image):
        self.frame_count += 1
        # skip the next 5 calls ~ 167 miliseconds
        if self.right_angle_detected:
            print(f"Skipping frame")
            return


        if msg.encoding.lower() != "bgr8":
            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
            return
        
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))


        # focus on ROI (bottom half)
        height = img.shape[0]
        width = img.shape[1]

	

        start = int(height*.7)
        roi = img[start:, :]
        

        # blue color thresholding
        b = roi[:, :, 0].astype(np.uint16)
        g = roi[:, :, 1].astype(np.uint16)
        r = roi[:, :, 2].astype(np.uint16)

        # window the output to the bottom


        rgb = [164, 108, 7]
        plus_minus = [50, 50, 90]
        blue_mask = cv2.inRange(roi, np.array(rgb) - np.array(plus_minus), np.array(rgb) + np.array(plus_minus))

        lines, contours = self.get_lines_contours(blue_mask)

        if contours is None:
            contours = []

        # last part of screen

        # want the two contours closest to us and closest to each other
        # maybe want something other than a for loop, but i don't anticipate many curves

        if lines is not None:
            self.right_angle_detected = self.detect_right_angle(lines)

        if self.right_angle_detected:
            self.pub_vel_cmd(self.forward_speed, -2.0)
            self.get_logger().debug(f"Right angle found")
            self.timer = self.create_timer(1.5, self.end_turn_state_callback)



        if self.frame_count % 10 == 0:
            self.get_logger().debug(f"Trying to display lines and contours")
            self.display_img_lines_contours(blue_mask, roi, lines, contours, self.frame_count)


        # find tape position
        column_strength = blue_mask.mean(axis=0)  # average over rows
        tape_x = np.argmax(column_strength)

        # error
        center_x = width / 2
        error = (tape_x - center_x) / center_x  # normalize error
        error += self.error_offset

        # control
        turn = float(np.clip(-self.kp * error, -self.max_turn, self.max_turn))

        # speed + publish

        print(f"Vel Cmd {self.forward_speed} {turn}")
        self.pub_vel_cmd(self.forward_speed, turn)
        



        

        
    

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

	# b8f9f5






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


    

