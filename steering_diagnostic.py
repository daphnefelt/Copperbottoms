#!/usr/bin/env python3
"""
Check if steering control is working
Find the actual steering response range
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

def set_mode(mode_name):
    """Set flight mode"""
    # Rover modes: MANUAL=0, HOLD=4, STEERING=3, AUTO=10
    mode_map = {'MANUAL': 0, 'HOLD': 4, 'STEERING': 3, 'AUTO': 10}
    
    if mode_name not in mode_map:
        print(f"Unknown mode: {mode_name}")
        return False
    
    mode_id = mode_map[mode_name]
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
        0, 0, 0, 0, 0)
    
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if msg and msg.result == 0:
        print(f"✓ Switched to {mode_name} mode\n")
        return True
    else:
        print(f"✗ Failed to switch to {mode_name}\n")
        return False

print("=== STEERING DIAGNOSTIC ===")
print("⚠️  WHEELS OFF THE GROUND!\n")

input("Press Enter when wheels are OFF ground...")

# Set MANUAL mode first
print("Setting MANUAL mode...")
set_mode('MANUAL')

# Arm
print("\nArming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(2)

# Test steering range (based on test_servo_manual.py: 1100-1750, center: 1425)
test_values = [1100, 1200, 1300, 1400, 1425, 1450, 1500, 1550, 1600, 1650, 1700, 1750]

print("\nTesting steering range (1100-1750, center: 1425):\n")
print("Value  | Servo Output | Steering Position")
print("-------|--------------|------------------")

for steering in test_values:
    # Fixed mapping: Ch1=steering, Ch3=throttle (wheels were moving when Ch3 changed)
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        steering, # Ch1 = steering (CORRECTED)
        0,
        1500,     # Ch3 = throttle neutral (was moving wheels)
        0, 0, 0, 0, 0)
    
    time.sleep(1)
    
    # Get servo output (steering commands to ch3 affect servo1)
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        servo1 = msg.servo1_raw  # Steering output (SERVO1_FUNCTION=26)
    else:
        servo1 = "N/A"
    
    # Ask user
    response = input(f"{steering} | {servo1:12}| Steering (l=left/c=center/r=right): ").lower()
    
    if 'l' in response or 'left' in response:
        print(f"← LEFT steering at value {steering}")
    elif 'r' in response or 'right' in response:
        print(f"→ RIGHT steering at value {steering}")
    elif 'c' in response or 'center' in response:
        print(f"| CENTER steering at value {steering}")
    else:
        print(f"  No clear response at value {steering}")
    print("")

# Release and disarm
print("\nReleasing and disarming...")
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
