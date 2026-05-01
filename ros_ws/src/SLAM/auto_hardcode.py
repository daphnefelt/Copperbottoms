import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

x_walls = [5, 32]
y_walls = [3, 21.5]

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
    if x < x_walls[0] and y < y_walls[0]:
        return 2 # bottom left corner, treat as left hallway
    if x < x_walls[0] and y > y_walls[1]:
        return 1 # top left corner, treat as top hallway
    if x > x_walls[1] and y > y_walls[1]:
        return 3 # top right corner, treat as right hallway
    if x > x_walls[1] and y < y_walls[0]:
        return 0 # bottom right corner, treat as bottom hallway
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
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.forward_speed = 0.25
        self.turn_speed = 0.3
        self.sharp_turn_speed = 0.75
        self.backup_speed = 0.25
        self.backup_time = 1.0
        self.turn_p = 1/20 / 2 # turn full at 20 degrees off
        self.right_turn_duration = 1.0
        self.right_turn_cooldown = 10.0
        self._right_turn_start = -math.inf

        # auto-backup recovery
        self.front_threshold = 0.3
        self.stop_time = 0.2
        self.recovery_turn_time = 0.7
        self.front_dist = float('inf')
        self.current_yaw = 0.0
        self.mode = 'NORMAL'  # 'NORMAL' | 'STOPPING' | 'BACKING_UP' | 'TURNING'
        self._mode_start = 0.0
        self._backup_turn_dir = -1.0

        # side-wall avoidance
        self.side_threshold = 0.5  # m — start skewing away when either side is closer than this
        self.kp_side = 0.5
        self.right_dist = float('inf')
        self.left_dist = float('inf')

    def _publish(self, lin: float, ang: float):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.vel_pub.publish(t)

    def _cone_min(self, msg: LaserScan, center_rad: float, half_rad: float) -> float:
        lo = int((center_rad - half_rad - msg.angle_min) / msg.angle_increment)
        hi = int((center_rad + half_rad - msg.angle_min) / msg.angle_increment)
        n = len(msg.ranges)
        lo = max(0, lo)
        hi = min(n - 1, hi)
        cone = np.array(msg.ranges[lo:hi + 1], dtype=float)
        cone[np.isinf(cone)] = msg.range_max
        valid = cone[(cone > msg.range_min) & np.isfinite(cone)]
        return float(np.min(valid)) if valid.size > 0 else 0.0  # no valid readings, treat as very close

    def _scan_cb(self, msg: LaserScan):
        self.front_dist = self._cone_min(msg, 0.0, math.radians(5.0))
        self.right_dist = self._cone_min(msg, math.radians(-90.0), math.radians(15.0))
        self.left_dist  = self._cone_min(msg, math.radians( 90.0), math.radians(15.0))

    def _pose_cb(self, msg):
        pose = msg.pose.pose.position.x, msg.pose.pose.position.y, math.degrees(2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        x, y, self.current_yaw = pose
        hallway = get_hallway((x, y))
        now = time.time()
        print(f"pose {pose} hallway {hallway_to_print(hallway)} mode={self.mode} front={self.front_dist:.2f}m")

        # ── recovery state machine ──────────────────────────────────────
        if self.mode == 'STOPPING':
            if now - self._mode_start < self.stop_time:
                self._publish(0.0, 0.0)
                return
            self.mode = 'BACKING_UP'
            self._mode_start = now

        if self.mode == 'BACKING_UP':
            if now - self._mode_start < self.backup_time:
                self._publish(-self.backup_speed, 0.0)
                return
            self.mode = 'TURNING'
            self._mode_start = now

        if self.mode == 'TURNING':
            if now - self._mode_start < self.recovery_turn_time:
                self._publish(0.0, self._backup_turn_dir * self.sharp_turn_speed * 2)
                return
            self.mode = 'NORMAL'

        # ── trigger recovery when a wall is right in front ─────────────
        if self.front_dist < self.front_threshold:
            # pick turn direction the way normal driving would have:
            # toward goal_yaw if in a hallway, else right (the right_turn fallback)
            if hallway is not None:
                goal_yaw = pose_goal_from_hallway(hallway)
                delta_yaw = (goal_yaw - self.current_yaw + 180) % 360 - 180
                self._backup_turn_dir = 1.0 if delta_yaw > 0 else -1.0
            else:
                self._backup_turn_dir = -1.0
            print(f"AUTO BACKUP: front={self.front_dist:.2f}m  turn_dir={self._backup_turn_dir:+.0f}")
            self.mode = 'STOPPING'
            self._mode_start = now
            self._publish(0.0, 0.0)
            return

        # ── normal driving ──────────────────────────────────────────────
        if hallway is not None:
            goal_yaw = pose_goal_from_hallway(hallway)
            delta_yaw = (goal_yaw - self.current_yaw + 180) % 360 - 180
            print(f"delta_yaw {delta_yaw}")
            fwd = self.forward_speed
            if abs(delta_yaw) > 30:
                print("big turnnnn")
                fwd = 0.2
            ang = self.turn_speed * (delta_yaw) * self.turn_p

            # also nudge away from side walls if too close
            right_skew = max(0.0, self.side_threshold - self.right_dist) * self.kp_side
            left_skew  = max(0.0, self.side_threshold - self.left_dist)  * self.kp_side
            ang_bias = right_skew - left_skew
            if ang_bias != 0.0:
                print(f"side skew right={self.right_dist:.2f}m left={self.left_dist:.2f}m bias={ang_bias:+.2f}")
            self._publish(fwd, ang + ang_bias)
        else:
            # stop
            print("Not in a hallway, stopping")
            self._publish(0.0, 0.0)

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