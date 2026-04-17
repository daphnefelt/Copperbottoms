### Follow the Gap w/ Lidar

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class FollowTheGap(Node):

    def __init__(self):
        super().__init__('follow_the_gap')

        # params
        self.forward_speed = 0.3
        self.max_turn = 0.8
        self.kp = 1.0 # p gain on heading error
        self.bubble_radius = 0.5 # m - zeroed out around closest point
        self.safety_dist = 0.25 # m - readings below this treated as 0
        self.fov_deg = 180.0  # degrees to consider in front of the robot
        self.min_gap_width  = 5 # min free beams in a row for a valid gap
        self.plot = True

        scan_topic = '/scan'
        cmd_vel_topic = '/cmd_vel'

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            f'Follow The Gap node started')

        if self.plot:
            plt.ion()
            self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(6, 6))

    # helpers

    def _debug_plot(self, fov_angles, fov_ranges_raw, fov_ranges_bubbled, gap, best_in_gap, goal_angle):
        self.ax.clear()
        self.ax.set_theta_zero_location('N') # 0 rad = top = forward
        self.ax.set_theta_direction(-1) # clockwise matches robot-right

        # all FOV beams (after inf to range_max, before bubble)
        self.ax.plot(fov_angles, fov_ranges_raw, color='steelblue', linewidth=0.8, label='scan')

        # gap beams highlighted
        if gap is not None:
            g_start, g_end = gap
            self.ax.plot(fov_angles[g_start:g_end+1], fov_ranges_bubbled[g_start:g_end+1], color='limegreen', linewidth=2.0, label='best gap')

        # goal direction
        self.ax.annotate('', xy=(goal_angle, max(fov_ranges_raw) * 0.85), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color='red', lw=2))

        self.ax.set_title(f'goal={np.degrees(goal_angle):.1f}°', va='bottom')
        self.ax.legend(loc='lower right', fontsize=8)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _apply_safety_bubble(self, ranges: np.ndarray, angles: np.ndarray) -> np.ndarray:
        # Take out the bubble area around closest obstacle
        working = ranges.copy()
        closest_idx = int(np.argmin(np.where(working > 0, working, np.inf)))
        closest_dist = working[closest_idx]

        half_angle = np.arctan2(self.bubble_radius, closest_dist)
        bubble_mask = np.abs(angles - angles[closest_idx]) <= half_angle
        working[bubble_mask] = 0.0
        return working

    def _find_best_gap(self, ranges: np.ndarray):
        # widest continuous gap of nonzero readings
        in_gap = False
        best_start = best_end = cur_start = 0
        best_width = 0

        for i, r in enumerate(ranges):
            if r > 0:
                if not in_gap:
                    cur_start = i
                    in_gap = True
            else:
                if in_gap:
                    width = i - cur_start
                    if width > best_width:
                        best_width = width
                        best_start, best_end = cur_start, i - 1
                    in_gap = False

        # gap that runs to the end of the array
        if in_gap:
            width = len(ranges) - cur_start
            if width > best_width:
                best_width = width
                best_start, best_end = cur_start, len(ranges) - 1

        return (best_start, best_end) if best_width >= self.min_gap_width else None

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        # cut to front FOV
        half_fov = np.radians(self.fov_deg / 2.0)
        fov_mask = np.abs(angles) <= half_fov
        fov_ranges = ranges[fov_mask].copy()
        fov_angles = angles[fov_mask]

        # replace inf (no return = open space) with range_max.
        # zero out only nan and too-close readings.
        fov_ranges = np.where(np.isposinf(fov_ranges), msg.range_max, fov_ranges)
        fov_ranges = np.where(np.isfinite(fov_ranges) & (fov_ranges > self.safety_dist), fov_ranges, 0.0)

        # safety bubble
        fov_ranges_pre_bubble = fov_ranges.copy()
        fov_ranges = self._apply_safety_bubble(fov_ranges, fov_angles)

        if fov_ranges.max() == 0.0:
            self._publish(0.0, 0.0)
            self.get_logger().warn('Too close to obstacles, stopping.')
            return

        # find widest gap
        gap = self._find_best_gap(fov_ranges)

        if gap is None:
            self._publish(0.0, 0.0)
            self.get_logger().warn('No valid gap found, stopping.')
            return

        gap_start, gap_end = gap

        # aim for deepest point in gap
        gap_ranges = fov_ranges[gap_start : gap_end + 1]
        best_in_gap = gap_start + int(np.argmax(gap_ranges))
        goal_angle  = float(fov_angles[best_in_gap])

        # proportional steering (positive angle is left, positive angular.z)
        turn = float(np.clip(-self.kp * goal_angle, -self.max_turn, self.max_turn))

        if self.plot:
            self._debug_plot(fov_angles, fov_ranges_pre_bubble, fov_ranges, gap, best_in_gap, goal_angle)

        self._publish(self.forward_speed, turn)
        self.get_logger().info(
            f'gap=[{gap_start},{gap_end}], '
            f'goal={np.degrees(goal_angle):.1f}°, turn={turn:.3f}')
            
    def _publish(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x  = linear
        twist.angular.z = angular
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FollowTheGap()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()