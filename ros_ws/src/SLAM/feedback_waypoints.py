import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import numpy as np

class WaypointFollower(Node):
    def __init__(self, waypoint_file, every_n=10):
        super().__init__('waypoint_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self.pose_cb, 10)
        self.waypoints = self.load_waypoints(waypoint_file, every_n)
        self.current_idx = 0
        self.goal_tolerance = 0.2
        self.linear_speed = 0.3
        self.angular_speed = 0.3

    def load_waypoints(self, filename, every_n):
        data = np.loadtxt(filename)
        return data[::every_n, :2]  # Only x, y

    def pose_cb(self, msg):
        if self.current_idx >= len(self.waypoints):
            self.cmd_pub.publish(Twist())  # Stop
            self.get_logger().info("All waypoints reached!")
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = 2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        goal = self.waypoints[self.current_idx]
        dx, dy = goal[0] - x, goal[1] - y
        dist = np.hypot(dx, dy)
        angle_to_goal = np.arctan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - th)

        cmd = Twist()
        if dist > self.goal_tolerance:
            cmd.linear.x = self.linear_speed * (dist > self.goal_tolerance)
            cmd.angular.z = self.angular_speed * angle_diff
        else:
            self.current_idx += 1
            self.get_logger().info(f"New waypoint: {self.waypoints[self.current_idx] if self.current_idx < len(self.waypoints) else 'DONE'}")
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower('pose_history.txt', every_n=4)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()