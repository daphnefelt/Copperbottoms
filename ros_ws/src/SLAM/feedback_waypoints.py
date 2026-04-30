import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from laser_scan_util import *
from constants import *
import numpy as np
import json
from enum import Enum

class MODE(Enum):
    NORMAL = 1
    RECOVERY = 2

SMALLEST_TURN_RADIUS = 1.05/2

class WaypointFollower(Node):
    def __init__(self, waypoint_file, landmark_file='', time_start=0, every_n=10):
        super().__init__('waypoint_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self.pose_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_update, 10)
        self.waypoints = self.load_waypoints(waypoint_file, every_n)
        self.waypoints = self.waypoints[self.waypoints[:,:,:,3] >=time_start]
        # accept the last values for the landmarks as the true values
        self.landmarks = self.load_landmarks(landmark_file)
        self.current_idx = 0
        self.goal_tolerance = 0.5
        self.linear_speed = 0.3
        self.back_up_speed = 0.5
        self.angular_speed = 0.3
        self.max_angular_speed = 1.5
        self.detection_radius = 1.2
        self.collision_imminent_dist = 0.5
        self.front_dist = np.inf
        


        # once in recovery need enough distance to turn and clear
        # minimum turning radius + buffer
        self.clearing_dist = FEET_TO_METERS*1.0  + SMALLEST_TURN_RADIUS
        self.mode = MODE.NORMAL


    def load_landmarks(self, filename):
        if filename == None or len(filename) == 0:
            return {}
        with open(filename, 'r') as file:
            landmarks = json.load(file)
        return landmarks
        

    def lidar_update(self, msg: LaserScan):
        self.front_dist = np.min(get_front(msg, np.radians(5)))
        if self.front_dist < self.collision_imminent_dist:
            self.mode = MODE.RECOVERY

        # when to leave recovery mode
        if self.mode == MODE.RECOVERY:
            # if driving in wrong direction - almost no backup correction needed
            # if perp need at least minimum turning radius distance
            # if driving in correct direction - pretty sure this is more distance than needed
            if self.front_dist >= self.clearing_dist:
                self.mode = MODE.NORMAL



    def load_waypoints(self, filename, every_n):
        data = np.loadtxt(filename)
        return data[::every_n, :4]  
    def get_goal_waypoint(self, pose):
        last_in_radius_idx = None
        last_in_radius_goal = None
        for idx in range(self.current_idx, len(self.waypoints)):
            wp = self.waypoints[idx]
            if np.hypot(wp[0] - pose[0], wp[1] - pose[1]) <= self.detection_radius:
                last_in_radius_idx = idx
                last_in_radius_goal = wp
            else:
                break
        return last_in_radius_idx, last_in_radius_goal




    def pose_cb(self, msg):
        if self.current_idx >= len(self.waypoints):
            self.cmd_pub.publish(Twist())  # Stop
            self.get_logger().info("All waypoints reached!")
            return
        
        if self.mode == MODE.RECOVERY:
            # back up away from the collision point and reorient
            
            cmd = Twist()
            cmd.linear.x = -self.back_up_speed
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = 2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        idx, goal = self.get_goal_waypoint((x, y))
        print(f"Current pose: ({x:.2f}, {y:.2f}, {np.degrees(th):.1f} deg), Goal idx: {idx}, Goal: {goal}")
        if goal is None:
            self.cmd_pub.publish(Twist())  # Stop
            self.get_logger().info("No waypoints close enough :(")
            return
        self.current_idx = idx
        dx, dy = goal[0] - x, goal[1] - y
        dist = np.hypot(dx, dy)
        angle_to_goal = np.arctan2(dy, dx)
        #angle_diff = self.normalize_angle(angle_to_goal - th)

        cmd = Twist()
        if dist > self.goal_tolerance:
            cmd.linear.x = self.linear_speed * (dist > self.goal_tolerance)
            cmd.angular.z = self.max_angular_speed * (self.normalize_angle_diff(angle_to_goal-th) / 2.0)
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle_diff(a, b):
        # assuming angles were in -pi to pi format - if the difference is >=0 then it is from 0 to 2 pi
        # if the difference is negative its from 0 to -2 pi

        # make it relative to a from 0 to 2pi if difference is greater than pi then right turn otherwise left turn
        diff = (b - a) % 2*np.pi 
        # right turn
        if(diff > np.pi):
            return diff - 2*np.pi
        # left turn
        return diff

    @staticmethod
    # for angle difference if angle b = pi and angle a = 0 b-a = pi this gives -pi
    def normalize_angle(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower('pose_history_save.txt', every_n=4)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()