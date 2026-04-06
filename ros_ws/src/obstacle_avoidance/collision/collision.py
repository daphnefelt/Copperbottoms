#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from custom_messages.msg import Slow
 
class CollisionDetector(Node):

    def scan_callback(self, msg):
        # 1. Setup constants and convert to numpy
        ranges = np.array(msg.ranges)
        half_cone = math.radians(15)
        min_angle_degrees = msg.angle_min*180/math.pi
        max_angle_degrees = msg.angle_max*180/math.pi
        angle_increment_degrees = msg.angle_increment*180/math.pi
#        print(f"min angle radian: {msg.angle_min}, angle: {min_angle_degrees}")
#        print(f"max angle radian: {msg.angle_max}, angle: {max_angle_degrees}")
#        print(f"increment angle radian: {msg.angle_increment}, angle: {angle_increment_degrees}")
#
        # 2. Calculate indices with explicit integer casting
        # Formula: index = (target_angle - min_angle) / increment
        start_idx = int(((-half_cone) - msg.angle_min) / msg.angle_increment)
        end_idx = int((half_cone - msg.angle_min) / msg.angle_increment)

        # 3. Slice the array
        front_ranges = ranges[start_idx : end_idx + 1]

        # 4. Filter out zeros and non-finite values (Inf/NaN)
        # This prevents the rover from "ghost stopping" due to sensor noise
        valid_ranges = front_ranges[(front_ranges > msg.range_min) & (np.isfinite(front_ranges))]

        # Check if we actually have data left after filtering
        if valid_ranges.size > 0:
            min_distance = np.min(valid_ranges)
        else:
            min_distance = float('inf') # No obstacles detected

        # 5. Logic for stopping and decelerating
        stop_msg = Bool()
        slow_msg = Slow()
        
        slow_dis = 2.3 # start slowing down at this distance in meters
        stop_dis = 1.3 # stop completely at this distance in meters

        if min_distance < slow_dis and min_distance > stop_dis: # decellerate
            self.get_logger().warn(f"Obstacle detected, decelerating! Distance: {min_distance:.2f}m")
            # Stop the motors
            twist_msg = Twist()
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.0

            self.vel_pub.publish(twist_msg)
            
            slow_msg.slowcmdvel = 0.1
            slow_msg.slowcmdang = 0.0
            slow_msg.slowcmdlogi = True


        elif min_distance <= stop_dis: # 1.3 because lidar is not at front or back of robot, the .3 should account
            self.get_logger().warn(f"Obstacle detected! Distance: {min_distance:.2f}m")
            
            # Stop the motors
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
            
            stop_msg.data = True
            slow_msg.slowcmdlogi = False
        else:
            stop_msg.data = False
            slow_msg.slowcmdlogi = False

        self.stop_move_pub.publish(stop_msg)
        self.slow_move_pub.publish(slow_msg)

    def __init__(self):

        super().__init__('collision_detector')
        # subscribe to lidar scanner
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        # publish to velocity and stop motion 
        # think about using a QoSProfile instead of 10
        self.vel_pub= self.create_publisher(Twist, 'cmd_vel', 10)
        self.stop_move_pub = self.create_publisher(Bool, 'stop_move', 10)
        self.slow_move_pub = self.create_publisher(Slow, 'slow_move', 10)
        





        
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CollisionDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
 
