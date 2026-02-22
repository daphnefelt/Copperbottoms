#!/usr/bin/env python3
"""
Check if throttle control is reversed or broken
Find where the motor actually stops
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

print("=== THROTTLE DIAGNOSTIC ===")
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

# Test full range
test_values = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000]

print("\nTesting throttle range (1000-2000):\n")
print("Value  | Servo Output | Motor Status")
print("-------|--------------|---------------")

for throttle in test_values:
    # NOTE: Pixhawk has channels swapped - ch1=throttle, ch3=steering
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        throttle, # Ch1 = throttle (swapped!)
        0,
        1500,     # Ch3 = steering center (swapped!)
        0, 0, 0, 0, 0)
    
    time.sleep(1)
    
    # Get servo output (throttle is on servo3, but controlled by ch1)
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        servo3 = msg.servo3_raw  # This should now change!
    else:
        servo3 = "N/A"
    
    # Ask user
    response = input(f"{throttle} | {servo3:12}| Motor (s=spinning/x=stopped): ").lower()
    
    if response == 's' or 'spin' in response:
        print(f"✓ MOTOR SPINNING at throttle {throttle}")
    else:
        print(f"  Motor stopped at throttle {throttle}")
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
