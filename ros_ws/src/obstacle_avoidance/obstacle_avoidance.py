import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Header
import math
import matplotlib.pyplot as plt
import numpy as np

## TOPIC SUBSCRIBER 
class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('ObstacleAvoidance')
        self.subscription = self.create_subscription(
            Header,
            'scan',
            self.listener_callback,
            100)
        self.secs = []
        self.nanosecs = []
        self.shutdown = False
        #self.subscription  # prevent unused variable warning
    

    def listener_callback(self, feed):
        current_time = self.get_clock().now().to_msg()
        num_nanosecs = current_time.nanosec - feed.stamp.nanosec
        self.nanosecs.append(num_nanosecs)
        if len(self.nanosecs) >= 400:
            self.histogram()
            self.shutdown = True
            

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()
    while not subscriber.shutdown:
        rclpy.spin_once(subscriber)

    # let it destroy itself when it's ready
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()