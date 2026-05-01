import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

x_walls = [4, 35]
y_walls = [3, 22.5]

def get_hallway(pose):
    x, y = pose
    if x_walls[0] < x < x_walls[1]:
        y_middle = (y_walls[0] + y_walls[1]) / 2
        if y < y_middle:
            return 0 # bottom hallway
        else:
            return 1 # top hallway
    if y_walls[0] < y < y_walls[1]:
        x_middle = (x_walls[0] + x_walls[1]) / 2
        if x < x_middle:
            return 2 # left hallway
        else:
            return 3 # right hallway
    return None # not in a hallway, need to turn right

def pose_goal_from_hallway(hallway):
    pose_map = {
        0: 180, # bottom hallway, face left
        1: 0,   # top hallway, face right
        2: 90,  # left hallway, face up
        3: -90, # right hallway, face down
    }
    return pose_map.get(hallway, None)

def hallway_to_print(hallway):
    hallway_map = {
        0: "bottom",
        1: "top",
        2: "left",
        3: "right",
    }
    return hallway_map.get(hallway, "TURN RIGHT")

class Hardcoded(Node):
    def __init__(self):
        super().__init__('hardcoded')
        self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self._pose_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.forward_speed = 0.25
        self.turn_speed = 0.3
        self.sharp_turn_speed = 0.75
        self.backup_speed = 0.25
        self.backup_time = 1.0
        self.turn_p = 1/20 # turn full at 20 degrees off
        self.right_turn_duration = 0.5
        self.right_turn_cooldown = 2.0
        self._right_turn_start = -math.inf

    def _publish(self, lin: float, ang: float):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.vel_pub.publish(t)

    def right_turn(self):
        now = time.time()
        elapsed = now - self._right_turn_start
        if elapsed < self.right_turn_duration:
            # still inside the active 0.5s turn window
            self._publish(self.forward_speed, -1 * self.sharp_turn_speed * 2)
        elif elapsed < self.right_turn_duration + self.right_turn_cooldown:
            # cooldown — ignore the call, go straight
            self._publish(self.forward_speed, 0.0)
        else:
            # cooldown expired — start a fresh 0.5s turn
            self._right_turn_start = now
            self._publish(self.forward_speed, -1 * self.sharp_turn_speed * 2)

    def _pose_cb(self, msg):
        pose = msg.pose.pose.position.x, msg.pose.pose.position.y, math.degrees(2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        x, y, self.current_yaw = pose
        hallway = get_hallway((x, y))
        print(f"pose {pose} hallway {hallway_to_print(hallway)}")
        if hallway is not None:
            goal_yaw = pose_goal_from_hallway(hallway)
            delta_yaw = (goal_yaw - self.current_yaw + 180) % 360 - 180
            print(f"delta_yaw {delta_yaw}")
            self._publish(self.forward_speed, self.turn_speed * (delta_yaw) * self.turn_p)
        else:
            print("right turnnnn")
            self.right_turn()

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