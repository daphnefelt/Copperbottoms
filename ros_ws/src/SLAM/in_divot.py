import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool

def div_1(pose):
    x, y = pose
    if 9 < x < 15 and y < 10:
        return True
    return False, -180 # heading cmd

def div_2(pose):
    x, y = pose
    if 10 < y < 20 and x < 10:
        return True
    return False, 90

def div_3(pose):
    x, y = pose
    if 15 < y < 22 and x > 30:
        return True
    return False, -90

def in_divot(pose):
    div1 = div_1(pose)
    if div1[0] == True:
        return div1[1]
    div2 = div_2(pose)
    if div2[0] == True:
        return div2[1]
    div3 = div_3(pose)
    if div3[0] == True:
        return div3
    return None

class Hardcoded(Node):
    def __init__(self):
        super().__init__('hardcoded')
        self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self._pose_cb, 10)
        self.pub = self.create_publisher(Bool, '/in_divot', 10)

    def _publish(self, lin: float, ang: float):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.vel_pub.publish(t)

    def _pose_cb(self, msg):
        pose = msg.pose.pose.position.x, msg.pose.pose.position.y, math.degrees(2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        x, y, theta = pose
        goal_theta = in_divot((x, y))
        if goal_theta is not None:
            self.get_logger().info(f"In divot at pose {pose}!")
            delta_yaw = (goal_theta - theta + 180) % 360 - 180
            p = 1/20 /2
            ang = self.turn_speed * (delta_yaw) * p
            self._publish(fwd, ang)
            self.pub.publish(Bool(data=True))
        else:
            self.pub.publish(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Hardcoded()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
