#!/usr/bin/env python3
"""
Check why arming is failing and test in HOLD mode
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

print("=== CHECKING PRE-ARM STATUS ===\n")

# Request pre-arm status
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
    0, 0, 0, 0, 0, 0, 0, 0)

time.sleep(1)

# Listen for STATUSTEXT messages (error messages)
print("Waiting for pre-arm check results...")
start = time.time()
messages = []
while time.time() - start < 3:
    msg = master.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        text = msg.text
        messages.append(text)
        print(f"  {text}")
    time.sleep(0.1)

if not messages:
    print("  (No error messages)")

print("\n=== TRYING HOLD MODE (doesn't require RC) ===\n")

# Switch to HOLD mode
print("Switching to HOLD mode...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4,  # HOLD = 4
    0, 0, 0, 0, 0)
time.sleep(1)

# Try arming
print("Attempting to arm in HOLD mode...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)

time.sleep(2)

# Check arm status
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
mode_mapping = {0: 'MANUAL', 3: 'STEERING', 4: 'HOLD', 10: 'AUTO', 15: 'GUIDED'}
mode = mode_mapping.get(msg.custom_mode, f'UNKNOWN({msg.custom_mode})')

print(f"\nResult:")
print(f"  Mode: {mode}")
print(f"  Armed: {'YES' if armed else 'NO'}")

if armed:
    print("\n✓ ARMED IN HOLD MODE!")
    print("Testing direct servo command...")
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        1, 1200, 0, 0, 0, 0, 0)
    
    time.sleep(1)
    
    for i in range(3):
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
        if msg:
            print(f"  SERVO: S1={msg.servo1_raw}  S3={msg.servo3_raw}")
        time.sleep(0.5)
    
    # Disarm
    print("\nDisarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
else:
    print("\n✗ Still can't arm")
    print("\nPossible issues:")
    print("  1. RC failsafe active (no RC receiver)")
    print("  2. Safety switch not pressed")
    print("  3. GPS/EKF not ready (if required)")
    print("\nTry:")
    print("  Set ARMING_CHECK = 0 (disable pre-arm checks)")
    print("  Set FS_ACTION = 0 (disable RC failsafe)")
