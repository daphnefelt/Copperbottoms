#!/usr/bin/env python3
"""
Arm vehicle and test steering specifically
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

# Check current arming state
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
print(f"Current state: {'ARMED' if armed else 'DISARMED'}\n")

if not armed:
    # Force arm
    print("Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,      # Arm
        21196,  # Force (bypass pre-arm checks)
        0, 0, 0, 0, 0)
    
    time.sleep(2)
    
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    print(f"After arming: {'ARMED' if armed else 'DISARMED'}\n")

if armed:
    print("Vehicle is ARMED. Testing steering...\n")
    
    # Send steering command
    print("Sending: Ch1=1200 (left), Ch3=1500 (neutral throttle)")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1200, 0, 1500, 0, 0, 0, 0, 0)
    
    print("\nMonitoring servo outputs for 5 seconds...")
    start = time.time()
    while time.time() - start < 5:
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
        if msg:
            print(f"SERVO: S1={msg.servo1_raw}  S3={msg.servo3_raw}")
        time.sleep(0.5)
    
    # Release and disarm
    print("\nReleasing RC override...")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0)
    
    time.sleep(1)
    
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    print("Done\n")
else:
    print("✗ Cannot arm vehicle - check for pre-arm errors")
    print("  Run: python3 ~/check_prearm.py")
