import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import numpy as np
from easydubins import dubin_path

class WaypointFollower(Node):
    def __init__(self, waypoint_file, every_n=10):
        super().__init__('waypoint_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self.pose_cb, 10)
        self.waypoints = self.load_waypoints(waypoint_file, every_n)
        self.current_idx = 0
        self.goal_tolerance = 0.5
        self.linear_speed = 0.3
        self.angular_speed = 0.3
        self.detection_radius = 1
        self.turning_radius = 0.5
        # No Dubins path state needed; Ackermann steering only

    def load_waypoints(self, filename, every_n):
        data = np.loadtxt(filename)
        return data[::every_n, :3]  # x, y, theta

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
        if last_in_radius_idx is None:
            # find the closest next waypoint bc all else failed
            print("No waypoints in radius, finding closest next waypoint")
            last_in_radius_idx = self.current_idx
            last_in_radius_goal = self.waypoints[last_in_radius_idx] if last_in_radius_idx < len(self.waypoints) else None
        return last_in_radius_idx, last_in_radius_goal

    def pose_cb(self, msg):
        if self.current_idx >= len(self.waypoints):
            self.cmd_pub.publish(Twist())  # Stop
            self.get_logger().info("All waypoints reached!")
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

        # Ackermann steering control to next waypoint
        dx = goal[0] - x
        dy = goal[1] - y
        dist = np.hypot(dx, dy)
        angle_to_goal = np.arctan2(dy, dx)
        heading_error = self.normalize_angle(angle_to_goal - th)

        # Parameters for Ackermann
        L = 0.165
        v = self.linear_speed if dist > self.goal_tolerance else 0.0
        # Pure Pursuit steering law
        steering_angle = np.arctan2(2 * L * np.sin(heading_error), dist) if dist > 0.05 else 0.0

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = steering_angle
        self.cmd_pub.publish(cmd)

    @staticmethod
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