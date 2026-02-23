#!/usr/bin/env python3
"""
Find the correct throttle neutral point (where motor stops)
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

print("=== FINDING THROTTLE NEUTRAL POINT ===")
print("⚠️  WHEELS OFF THE GROUND!\n")

# Check SERVO3 parameters
print("Checking SERVO3 config...")
master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b'SERVO3_MIN',
    -1)
msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
if msg:
    servo3_min = int(msg.param_value)
    print(f"SERVO3_MIN = {servo3_min}")

master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b'SERVO3_MAX',
    -1)
msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
if msg:
    servo3_max = int(msg.param_value)
    print(f"SERVO3_MAX = {servo3_max}")

master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b'SERVO3_TRIM',
    -1)
msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
if msg:
    servo3_trim = int(msg.param_value)
    print(f"SERVO3_TRIM = {servo3_trim} <- This is likely the STOP value\n")

input("Press Enter when wheels are OFF the ground...")

# Arm
print("\nArming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(2)

# Start testing from SERVO3_TRIM
print(f"\nTesting around SERVO3_TRIM ({servo3_trim}):\n")

for offset in [-100, -50, 0, 50, 100]:
    throttle = servo3_trim + offset
    print(f"Testing {throttle}us (TRIM{offset:+d})")
    
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500,     # Ch1 = steering center
        0,
        throttle, # Ch3 = test value
        0, 0, 0, 0, 0)
    
    time.sleep(2)
    
    response = input("  Motor spinning? (y/n): ").lower()
    if response == 'n':
        print(f"\n✓ NEUTRAL POINT FOUND: {throttle}us")
        print(f"   Use this value for stopped throttle")
        break

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

