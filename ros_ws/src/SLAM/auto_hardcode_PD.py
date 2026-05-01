import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

x_walls = [7, 31]
y_walls = [3, 20.5]


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
        self.sharp_turn_speed = 0.55
        self.backup_speed = 0.25
        self.backup_time = 1.0
        self.turn_p = 1/20 /2  # turn full at 20 degrees off
        self.sharp_turn_p = 1/20 /4
        self.turn_d = 0.00  # tune this — try something like 0.05 to start
        self._prev_delta_yaw = 0.0
        self._prev_yaw_time = None
        self._prev_hallway = None
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

        # side-wall nudge
        self.side_threshold = 1.0      # m — trigger a nudge when either side gets closer than this
        self.shift_time = 0.5          # base length of a nudge phase (s)
        self.shift_speed = 0.3         # base angular magnitude during a nudge (rad/s)
        self.nudge_cooldown = 1.0      # min time between consecutive nudges (s)
        self.right_dist = float('inf')
        self.left_dist = float('inf')
        self._nudge_phase = 'NONE'     # 'NONE' | 'AWAY' | 'BACK' | 'SETTLE'
        self._nudge_start = 0.0
        self._nudge_dir = 0.0          # +1 = nudge left (CCW), -1 = nudge right (CW)
        self._nudge_cooldown_start = -math.inf

        self.hall = 0
        self.transitioning = False

    def _publish(self, lin: float, ang: float):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.vel_pub.publish(t)

    def get_hallway(self, pose):
        x, y = pose
        new_hall = None
        if x_walls[0] < x < x_walls[1]:
            y_middle = (y_walls[0] + y_walls[1]) / 2
            if y < y_middle:
                new_hall = 0 # bottom hallway
            else:
                new_hall = 1 # top hallway
        elif y_walls[0] < y < y_walls[1]:
            x_middle = (x_walls[0] + x_walls[1]) / 2
            if x < x_middle:
                new_hall = 2 # left hallway
            else:
                new_hall = 3 # right hallway
        elif x < x_walls[0] and y < y_walls[0]:
            new_hall = 2 # bottom left corner, treat as left hallway
        elif x < x_walls[0] and y > y_walls[1]:
            new_hall = 1 # top left corner, treat as top hallway
        elif x > x_walls[1] and y > y_walls[1]:
            new_hall = 3 # top right corner, treat as right hallway
        elif x > x_walls[1] and y < y_walls[0]:
            new_hall = 0 # bottom right corner, treat as bottom hallway
        if new_hall != self.hall:
            self.get_logger().info(f"Entering transitioning... previous hall {self.hall} new hall {new_hall}")
            self.transitioning = True
        return new_hall # not in a hallway, need to turn right



    def _cone_num_greater_max(self, msg: LaserScan, center_rad: float, half_rad: float, max_range) -> float:
        lo = int((center_rad - half_rad - msg.angle_min) / msg.angle_increment)
        hi = int((center_rad + half_rad - msg.angle_min) / msg.angle_increment)
        n = len(msg.ranges)
        lo = max(0, lo)
        hi = min(n - 1, hi)
        cone = np.array(msg.ranges[lo:hi + 1], dtype=float)
        cone[np.isinf(cone)] = msg.range_max
        return np.sum((cone > msg.range_min) & np.isfinite(cone) & (cone > max_range))

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
        # look ahead
        if self.transitioning:
            self.get_logger().info(f"In transition: Look ahead...")
            if self._cone_num_greater_max(msg, math.radians(-88.0), math.radians(10.0), 5.0) >= 5:
                self.get_logger().info(f"Gap on right ahead!")
                self.hall = (self.hall + 1) % 4
                self.transitioning = False

    def _nudge(self, direction: float, now: float) -> bool:
        if self._nudge_phase == 'NONE':
            self._nudge_phase = 'AWAY'
            self._nudge_start = now
            self._nudge_dir = direction

        if self._nudge_phase == 'AWAY':
            if now - self._nudge_start < self.shift_time * 1.5:
                self._publish(self.forward_speed, self._nudge_dir * self.shift_speed * 1.5)
                return True
            self._nudge_phase = 'BACK'
            self._nudge_start = now

        if self._nudge_phase == 'BACK':
            if now - self._nudge_start < self.shift_time:
                self._publish(self.forward_speed, -self._nudge_dir * self.shift_speed)
                return True
            self._nudge_phase = 'SETTLE'
            self._nudge_start = now

        if self._nudge_phase == 'SETTLE':
            if now - self._nudge_start < self.shift_time * 2:
                self._publish(self.forward_speed, -self._nudge_dir * self.shift_speed * 0.4)
                return True
            self._nudge_phase = 'NONE'
            self._nudge_cooldown_start = now

        return False
            

    def _pose_cb(self, msg):
        pose = msg.pose.pose.position.x, msg.pose.pose.position.y, math.degrees(2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        x, y, self.current_yaw = pose
        hallway = self.get_hallway((x, y))
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
            # if a nudge is mid-execution, let it finish — it overrides goal-tracking
            if self._nudge_phase != 'NONE':
                self._nudge(self._nudge_dir, now)
                return

            # else, maybe start a new nudge if a side wall is too close (and cooldown elapsed)
            in_cooldown = (now - self._nudge_cooldown_start) < self.nudge_cooldown
            side_close = self.right_dist < self.side_threshold or self.left_dist < self.side_threshold
            if side_close and not in_cooldown:
                # nudge away from whichever side is closer
                direction = 1.0 if self.right_dist < self.left_dist else -1.0
                print(f"NUDGE {'LEFT' if direction > 0 else 'RIGHT'}: right={self.right_dist:.2f}m left={self.left_dist:.2f}m")
                self._nudge(direction, now)
                return

            # otherwise: usual goal-tracking command
            goal_yaw = pose_goal_from_hallway(hallway)
            delta_yaw = (goal_yaw - self.current_yaw + 180) % 360 - 180
            print(f"delta_yaw {delta_yaw}")
            p = self.sharp_turn_p if abs(delta_yaw) > 30 else self.turn_p
            fwd = self.forward_speed
            p = self.sharp_turn_p if abs(delta_yaw) > 30 else self.turn_p
            if abs(delta_yaw) > 30:
                print("big turnnnn")
                fwd = 0.2
            ang = self.turn_speed * (delta_yaw) * p
            self._publish(fwd, ang)
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
