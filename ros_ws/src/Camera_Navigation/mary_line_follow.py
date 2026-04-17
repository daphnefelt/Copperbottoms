### Dark Lane Following ###

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

    def image_callback(self, msg: Image):
        # skip the next 5 calls ~ 167 miliseconds
        if self.right_angle_detected:
            print(f"Skipping frame")
            self.frame_count += 1
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


        blue_score = b - (0.5 * (g + r)).astype(np.uint16)  # simple blue score
        blue_mask = ((blue_score > self.color_threshold)*255).astype(np.uint8)

        #cv2_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	
        # blur to help with noise picked up
        blurred = cv2.GaussianBlur(blue_mask, (3, 3), 0)
	
        # threshold for the lines
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
	
        # cv2.RETR_EXTERNAL: if contours are nested - retrieves only outermost
        # cv2.CHAIN_APPROX_SIMPLE: Removes redundant points - memory efficient
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # want the two contours closest to us and closest to each other
        # maybe want something other than a for loop, but i don't anticipate many curves
        print(f"Number of contours: {len(contours)}")
	
        
        closest_contour_idx = np.array([-1, -1])
        min_dist = np.array([max(height, width) + 50, 0])
        min_dist[1] = min_dist[0]
        for idx, contour in enumerate(contours):
            # try distances to the bottom 20 points on the screen in case it curves away
            idxs = contour[-30:, 0, 1] < self.height-10
            if len(idxs) == 0:
                print(f"Not enough space for idxs meet criteria")
                continue
            dist = np.min(np.power(self.robot_center - contour[-20:,0,idxs], 2))
		
            if any(closest_contour_idx==-1):
                i_dist = (closest_contour_idx==-1).argmax()
                min_dist[i_dist] = dist
                closest_contour_idx[i_dist] = idx
            elif any(dist < min_dist):
                i_dist = (dist < min_dist).argmax()
                min_dist[i_dist] = dist
                closest_contour_idx[i_dist] = idx
        
        if any(closest_contour_idx == -1):
            print(f"Not all closest indexes initialized")
            return
        # got the two closest blue contours
        print(f"Distances recorded: {min_dist}")
        # identify rightmost and leftmost - might not be two if there is a closed loop - MARY
        right_idx = closest_contour_idx[0]
        left_idx = closest_contour_idx[1]
        if closest_contour_idx[1][-1][0] > closest_contour_idx[0][-1][0]:
            right_idx = closest_contour_idx[1]
            left_idx = closest_contour_idx[0]

        right = contours[right_idx]
        left = contours[left_idx]
        
        gap_threshold = 30
        
        # start of segment is after a gap
        if right[1] > gap_threshold:
            # make a beeline to the segment
            self.turn(right[-1])

        else:
            # identify point in future - higher up in the image

            # if there is a right angle it needs special handling

            # what are the orientations of directions

            max_future_pixels = 50
            orientations = np.atan2(*(right[-max_future_pixels:] - right[-(max_future_pixels-1):])[::-1])
            diff_orientations = orientations[1:] - orientations[:-1]
            # within 20 degrees - right angle
            if np.abs(np.max(diff_orientations) - np.pi/2) < 0.4:
                self.right_angle_detected = True 
            else:
                # follow local line
                # can turn up to 45 degrees at -2 to 2
                self.turn_to_point(right[-15])


            
    def turn_to_point(self, pt):
        dir = np.atan2(*((pt - self.robot_center)[::-1]))
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


    

