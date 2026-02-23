#!/usr/bin/env python3
"""
Test steering in MANUAL mode with detailed debugging
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

# Get current mode
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
mode_mapping = {0: 'MANUAL', 3: 'STEERING', 4: 'HOLD', 10: 'AUTO', 15: 'GUIDED'}
current_mode = mode_mapping.get(msg.custom_mode, f'UNKNOWN({msg.custom_mode})')
armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
print(f"Current Mode: {current_mode}")
print(f"Armed: {'YES' if armed else 'NO'}\n")

# Switch to MANUAL mode
print("Switching to MANUAL mode...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    0,  # MANUAL = 0
    0, 0, 0, 0, 0)
time.sleep(1)

# Arm
print("Arming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(2)

# Verify mode and arming
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
mode = mode_mapping.get(msg.custom_mode, f'UNKNOWN({msg.custom_mode})')
armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
print(f"\nAfter setup:")
print(f"  Mode: {mode}")
print(f"  Armed: {'YES' if armed else 'NO'}\n")

if armed:
    # Test with direct servo command instead of RC override
    print("=== TEST 1: Direct servo command ===")
    print("Sending SERVO1 = 1200us directly...")
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        1,      # Servo number (1)
        1200,   # PWM value
        0, 0, 0, 0, 0)
    
    time.sleep(0.5)
    
    for i in range(5):
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
        if msg:
            print(f"  SERVO: S1={msg.servo1_raw}  S3={msg.servo3_raw}")
        time.sleep(0.5)
    
    # Reset to trim
    print("\nResetting to trim (1450us)...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        1, 1450, 0, 0, 0, 0, 0)
    time.sleep(1)
    
    # Disarm
    print("\nDisarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    print("Done\n")
else:
    print("✗ Failed to arm")
