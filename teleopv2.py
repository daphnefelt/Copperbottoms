#!/usr/bin/env python3
"""
cmd_vel_to_mavlink.py — LATCHED MODE

Commands hold their state until a new command arrives.
Forward stays forward. Left stays left. Send a zero Twist to stop.

In teleop_twist_keyboard:
  i = forward      , = reverse
  j = turn left    l = turn right
  k = STOP (sends zero)

Run alongside:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
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
THROTTLE_FWD     = 1600   # adjust for desired forward speed
THROTTLE_REV     = 1400   # adjust for desired reverse speed

# RC heartbeat — keeps Pixhawk happy with a steady signal
RC_HZ = 20


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


class CmdVelToMavlink(Node):

    def __init__(self, master):
        super().__init__('cmd_vel_to_mavlink')
        self.master = master

        # Latched state — holds last commanded values
        self.ch1 = STEER_CENTER
        self.ch3 = THROTTLE_NEUTRAL

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer sends RC at steady rate so Pixhawk doesn't time out
        self.create_timer(1.0 / RC_HZ, self.send_rc)

        self.get_logger().info('Ready — commands are latched until changed')
        self.get_logger().info('  i=fwd  ,=rev  j=left  l=right  k=STOP')

    def cmd_vel_callback(self, msg: Twist):
        linear  = msg.linear.x
        angular = msg.angular.z

        # ── Throttle — only update if a linear command was given ──────────
        if linear > 0.0:
            self.ch3 = THROTTLE_FWD
            throttle_str = "FWD"
        elif linear < 0.0:
            self.ch3 = THROTTLE_REV
            throttle_str = "REV"
        elif angular == 0.0:
            # k was pressed (full zero Twist) — stop everything
            self.ch3 = THROTTLE_NEUTRAL
            self.ch1 = STEER_CENTER
            self.get_logger().info('■ STOP')
            return
        else:
            throttle_str = "(held)"  # pure steer, don't touch throttle

        # ── Steering — only update if an angular command was given ────────
        if angular > 0.0:
            self.ch1 = STEER_LEFT
            steer_str = "LEFT"
        elif angular < 0.0:
            self.ch1 = STEER_RIGHT
            steer_str = "RIGHT"
        else:
            steer_str = "(held)"  # pure throttle, don't touch steering

        self.get_logger().info(
            f'throttle={throttle_str} steer={steer_str} → ch1={self.ch1} ch3={self.ch3}')

    def send_rc(self):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            self.ch1, 0, self.ch3, 0, 0, 0, 0, 0)


def main():
    print('Connecting to Pixhawk...')
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
    master.wait_heartbeat()
    print(f'Connected! System: {master.target_system}\n')

    set_mode(master, 'MANUAL')
    time.sleep(0.5)

    if not arm(master):
        return

    rclpy.init()
    node = CmdVelToMavlink(master)

    print('In the teleop terminal:')
    print('  i = forward    , = reverse')
    print('  j = left       l = right')
    print('  k = STOP\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    print('\nShutting down...')
    node.master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        STEER_CENTER, 0, THROTTLE_NEUTRAL, 0, 0, 0, 0, 0)
    time.sleep(0.3)
    disarm(master)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()