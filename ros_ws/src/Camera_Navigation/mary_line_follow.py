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

        # focus on ROI (bottom half)
        print(f"img shape is {img.shape}")
        height = img.shape[0]
        width = img.shape[1]

	
        cv2_img = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)

        
        roi_strt = int(height * 0.70) # bottom 30%
        roi = img[roi_strt:height, :, :]

        # blue color thresholding
        b = roi[:, :, 0]
        g = roi[:, :, 1]
        r = roi[:, :, 2]


        blue_score = b - int(0.5 * (g + r))  # simple blue score
        blue_mask = (blue_score > self.color_threshold)

	
        # blur to help with noise picked up
        blurred = cv2.GaussianBlur(blue_mask, (3, 3), 0)
	
        # threshold for the lines
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
	
        # cv2.RETR_EXTERNAL: if contours are nested - retrieves only outermost
        # cv2.CHAIN_APPROX_SIMPLE: Removes redundant points - memory efficient
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # want the two contours closest to us and closest to each other
        # maybe want something other than a for loop, but i don't anticipate many curves
        print(f"Number of contours: {contours}")
	
        # fraction grabbed from line_follow tuning 
        offset = 0.4*int(width/2)


        # alternatively could get the top two longest segments
        robot_center = np.array([int(width/2) + offset, 0])
        closest_contour_idx = np.array([-1, -1])
        min_dist = np.array([np.max(height, width) + 50, 0])
        min_dist[1] = min_dist[0]
        for idx, contour in enumerate(contours):
            # try distances to the bottom 20 points on the screen in case it curves away
            dist = np.min(np.pow(robot_center - contour[-20:,1,:], 2))
		
            if any(closest_contour_idx==-1):
                i_dist = (closest_contour_idx==-1).argmax()
                min_dist[i_dist] = dist
                closest_contour_idx[i_dist] = idx
            elif any(dist < min_dist):
                i_dist = (dist < min_dist).argmax()
                min_dist[i_dist] = dist
                closest_contour_idx[i_dist] = idx
        
        # got the two closest blue contours

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
            dir = np.atan2(*(right[-1] - robot_center)[::-1])

        else:
            # identify point in future - higher up in the image

            # if there is a right angle it needs special handling

            # what are the orientations of directions

            current_direction = np.atan2(*(right[-10] - right[-1] )[::-1])
        

            future_direction = np.atan2(*(right[-20] - right[-10] )[::-1])


            # go towards future direction point
            # angle_threshold = np.pi/4
            # if np.abs(future_direction - current_direction) > angle_threshold:
            #     # start turning
            # else:
            #     # go straight towards line

	


            turn = float(np.clip(-self.kp * error, -self.max_turn, self.max_turn))

            # speed + publish

            twist = Twist()
            twist.linear.x = self.forward_speed
            twist.angular.z = turn
            self.vel_pub.publish(twist)
        
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


    

