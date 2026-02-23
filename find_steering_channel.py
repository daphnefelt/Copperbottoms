#!/usr/bin/env python3
"""
Test ALL RC channels to find which one controls SERVO1 (steering)
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

def set_mode(mode_name):
    """Set flight mode"""
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

def get_servo_outputs():
    """Get all servo outputs"""
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
    if msg:
        return {
            'servo1': msg.servo1_raw,
            'servo2': msg.servo2_raw, 
            'servo3': msg.servo3_raw,
            'servo4': msg.servo4_raw
        }
    return None

print("=== FIND STEERING CHANNEL ===")
print("⚠️  WHEELS OFF THE GROUND!\n")

input("Press Enter when wheels are OFF ground...")

print("Setting MANUAL mode...")
set_mode('MANUAL')

print("\nArming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(2)

# Get baseline
print("Baseline servo outputs:")
baseline = get_servo_outputs()
if baseline:
    for servo, value in baseline.items():
        print(f"{servo}: {value}")
print()

# Test each RC channel individually
channels_to_test = [1, 2, 3, 4, 5, 6, 7, 8]

for ch in channels_to_test:
    print(f"=== TESTING RC CHANNEL {ch} ===")
    
    # Create channel list with neutral values
    channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
    channels[ch-1] = 1300  # Set test channel to LEFT value
    
    print(f"Sending 1300 to RC Channel {ch}...")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        channels[0], channels[1], channels[2], channels[3],
        channels[4], channels[5], channels[6], channels[7])
    
    time.sleep(1.5)
    
    # Get servo outputs after command
    outputs = get_servo_outputs()
    if outputs and baseline:
        print("Servo changes:")
        changes = {}
        for servo in ['servo1', 'servo2', 'servo3', 'servo4']:
            delta = outputs[servo] - baseline[servo]
            changes[servo] = delta
            if abs(delta) > 10:
                print(f"  {servo}: {baseline[servo]} → {outputs[servo]} (Δ{delta}) *** CHANGED ***")
            else:
                print(f"  {servo}: {baseline[servo]} → {outputs[servo]} (Δ{delta})")
        
        # Check if SERVO1 (steering) changed
        if abs(changes['servo1']) > 10:
            print(f"\n🎯 RC CHANNEL {ch} CONTROLS STEERING (SERVO1)!")
            
        # Check if wheels moved (throttle)
        if abs(changes['servo3']) > 10:
            print(f"\n🚗 RC CHANNEL {ch} CONTROLS THROTTLE (SERVO3)!")
            
    print()
    
    # Reset to neutral
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500)
    time.sleep(1)

# Release and disarm
print("Releasing and disarming...")
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

print("\n=== SUMMARY ===")
print("This script tested all 8 RC channels to find which one controls:")
print("• SERVO1 (steering)")
print("• SERVO3 (throttle)")
print("Use the channel that caused SERVO1 to change for steering commands.")
