#!/usr/bin/env python3
"""
Test ONLY steering - throttle stays at neutral (1500us = stopped)
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

# Arm
print("Arming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(2)

print("Testing steering ONLY (throttle = 1500 = STOP)\n")

print("1. Steering LEFT (1100)")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1100,  # Ch1 = steering left
    0,     # Ch2 = unused
    1400,  # Ch3 = throttle NEUTRAL (stopped)
    0, 0, 0, 0, 0)

for i in range(3):
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        print(f"  Steering={msg.servo1_raw}  Throttle={msg.servo3_raw}")
    time.sleep(0.5)

print("\n2. Steering CENTER (1500)")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500,  # Ch1 = center
    0,
   1601,  # Ch3 = still neutral
    0, 0, 0, 0, 0)

for i in range(3):
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        print(f"  Steering={msg.servo1_raw}  Throttle={msg.servo3_raw}")
    time.sleep(0.5)

print("\n3. Steering RIGHT (1900)")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1900,  # Ch1 = steering right
    0,
   1601,  # Ch3 = still neutral
   0 , 0, 0, 0, 0)

for i in range(3):
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        print(f"  Steering={msg.servo1_raw}  Throttle={msg.servo3_raw}")
    time.sleep(0.5)

print("\n4. Back to CENTER")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500,  # Ch1 = center
    0,
   1601,  # Ch3 = neutral
    0, 0, 0, 0, 0)

time.sleep(1)

# Release and disarm
print("\nReleasing RC override and disarming...")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

print("Done\n")
print("Check: Throttle should have stayed at 1500 (neutral) the whole time")
