import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from custom_messages.msg import Slow
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class StopSignDetect(Node):


    def __init__(self):
        super().__init__('stop_sign_detect')
        self.stop_sign_detected = False
        self.stop_sign_distance = float('inf')
        self.stop_sign_angle = 0.0
        self.bridge = CvBridge()
        # Load the cascade classifier (ensure the path is correct)
        self.stop_sign_cascade = cv2.CascadeClassifier('cascade_stop_sign.xml')

        # Subscribers
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # Publishers
        self.stop_sign_pub = self.create_publisher(Bool, 'stop_sign_detected', 10)
        self.slow_pub = self.create_publisher(Slow, 'slow_down', 10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        stop_signs = self.stop_sign_cascade.detectMultiScale(gray, 1.3, 5)

        detected = False
        
        for (x, y, w, h) in stop_signs:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            detected = True

        if detected:
            if not self.stop_sign_detected:
                self.get_logger().info('Stop sign detected!')
                self.stop_sign_detected = True
                self.publish_stop_sign_detected()
                self.stop_robot()
                # Pause for 5 seconds to simulate looking both ways
                rclpy.timer.sleep(5)
                self.get_logger().info('Looking both ways...')
                self.resume_robot()
                self.get_logger().info('Resuming movement after stop sign.')
                self.stop_sign_detected = False
                self.temporarily_ignore_stop_signs(5)  # Ignore stop signs for the next 5 seconds
        else:
            self.stop_sign_detected = False

        # Optional: Show image for debugging (remove for headless operation)
        # cv2.imshow('Stop Sign Detection', cv_image)
        # cv2.waitKey(1)


    ## helper functions ##
    def publish_stop_sign_detected(self):
        msg = Bool()
        msg.data = self.stop_sign_detected
        self.stop_sign_pub.publish(msg)

    def stop_robot(self):
        msg = Slow()
        msg.should_slow_down = True
        self.slow_pub.publish(msg)

    def resume_robot(self):
        msg = Slow()
        msg.should_slow_down = False
        self.slow_pub.publish(msg)

    def temporarily_ignore_stop_signs(self, duration):
        self.stop_sign_detected = False
        rclpy.timer.sleep(duration)
        self.stop_sign_detected = True
    
    

def main(args=None):
    rclpy.init(args=args)
    stop_sign_detect = StopSignDetect()
    rclpy.spin(stop_sign_detect)
    stop_sign_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




    


