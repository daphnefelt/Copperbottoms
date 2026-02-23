#!/usr/bin/env python3
"""
cmd_vel_to_mavlink.py

Subscribes to /cmd_vel (from teleop_twist_keyboard) and translates
Twist messages to MAVLink RC overrides for a Pixhawk rover.

Run alongside:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Launch:
    python3 cmd_vel_to_mavlink.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time


# ── PWM constants ────────────────────────────────────────────────────────
STEER_CENTER     = 1425
STEER_LEFT       = 1750
STEER_RIGHT      = 1100
THROTTLE_NEUTRAL = 1501
THROTTLE_FWD_MAX = 1750
THROTTLE_REV_MAX = 1250

# teleop_twist_keyboard defaults (used to normalise input)
MAX_LINEAR  = 0.5   # m/s — matches teleop default
MAX_ANGULAR = 1.0   # rad/s — matches teleop default


def linear_map(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another."""
    value = max(in_min, min(in_max, value))  # clamp
    return int(out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min))


class CmdVelToMavlink(Node):

    def __init__(self, master):
        super().__init__('cmd_vel_to_mavlink')
        self.master = master
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('Subscribed to /cmd_vel — ready for teleop')

    def cmd_vel_callback(self, msg: Twist):
        linear  = msg.linear.x    # positive = forward, negative = reverse
        angular = msg.angular.z   # positive = left, negative = right

        # ── Throttle (linear.x → ch3) ────────────────────────────────────
        if linear > 0.0:
            ch3 = linear_map(linear, 0.0, MAX_LINEAR, THROTTLE_NEUTRAL, THROTTLE_FWD_MAX)
        elif linear < 0.0:
            ch3 = linear_map(-linear, 0.0, MAX_LINEAR, THROTTLE_NEUTRAL, THROTTLE_REV_MAX)
        else:
            ch3 = THROTTLE_NEUTRAL

        # ── Steering (angular.z → ch1) ────────────────────────────────────
        if angular > 0.0:
            ch1 = linear_map(angular, 0.0, MAX_ANGULAR, STEER_CENTER, STEER_LEFT)
        elif angular < 0.0:
            ch1 = linear_map(-angular, 0.0, MAX_ANGULAR, STEER_CENTER, STEER_RIGHT)
        else:
            ch1 = STEER_CENTER

        self.send_rc(ch1, ch3)
        self.get_logger().info(f'linear={linear:.2f} angular={angular:.2f} → ch1={ch1} ch3={ch3}')

    def send_rc(self, ch1, ch3):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            ch1, 0, ch3, 0, 0, 0, 0, 0)


def set_mode(master, mode_name):
    mode_map = {'MANUAL': 0, 'HOLD': 4, 'STEERING': 3, 'AUTO': 10}
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_map[mode_name], 0, 0, 0, 0, 0)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if msg and msg.result == 0:
        print(f'✓ {mode_name} mode')


def arm(master):
    print('Arming...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 21196, 0, 0, 0, 0, 0)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if msg and msg.result == 0:
        print('✓ Armed\n')
        return True
    print('✗ Arming failed\n')
    return False


def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)
    print('✓ Disarmed')


def main():
    # ── Connect to Pixhawk ───────────────────────────────────────────────
    print('Connecting to Pixhawk...')
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
    master.wait_heartbeat()
    print(f'Connected! System: {master.target_system}\n')

    set_mode(master, 'MANUAL')
    time.sleep(0.5)

    if not arm(master):
        return

    # ── Start ROS2 node ──────────────────────────────────────────────────
    rclpy.init()
    node = CmdVelToMavlink(master)

    print('Ready! In another terminal run:')
    print('  ros2 run teleop_twist_keyboard teleop_twist_keyboard\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # ── Shutdown ─────────────────────────────────────────────────────────
    print('\nShutting down...')
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        STEER_CENTER, 0, THROTTLE_NEUTRAL, 0, 0, 0, 0, 0)
    time.sleep(0.3)
    disarm(master)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()