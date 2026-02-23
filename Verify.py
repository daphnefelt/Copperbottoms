#!/usr/bin/env python3
"""
Verify exact channel/servo mapping
Test both steering and throttle responses individually
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
    """Get current servo output values"""
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        return {
            'servo1': msg.servo1_raw,  # Should be steering (SERVO1_FUNCTION=26) 
            'servo3': msg.servo3_raw   # Should be throttle (SERVO3_FUNCTION=70)
        }
    return None

print("=== CHANNEL/SERVO MAPPING VERIFICATION ===")
print("⚠️  WHEELS OFF THE GROUND!\n")

input("Press Enter when wheels are OFF ground...")

# Set MANUAL mode and arm
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

# Get baseline servo outputs
print("Baseline servo outputs:")
baseline = get_servo_outputs()
if baseline:
    print(f"SERVO1 (steering): {baseline['servo1']}")
    print(f"SERVO3 (throttle): {baseline['servo3']}")
print()

# Test 1: Send steering command to RC Ch3, look for change in SERVO1
print("TEST 1: Steering command (RC Ch3 → SERVO1)")
print("Sending 1300 to RC Channel 3 (steering)...")

master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500,  # Ch1 = throttle neutral
    0,     # Ch2
    1300,  # Ch3 = steering LEFT
    0, 0, 0, 0, 0)

time.sleep(1.5)
outputs = get_servo_outputs()
if outputs:
    print(f"SERVO1 (steering): {baseline['servo1']} → {outputs['servo1']} (Δ{outputs['servo1'] - baseline['servo1']})")
    print(f"SERVO3 (throttle): {baseline['servo3']} → {outputs['servo3']} (Δ{outputs['servo3'] - baseline['servo3']})")
    
    if abs(outputs['servo1'] - baseline['servo1']) > 10:
        print("✓ SERVO1 responds to RC Ch3 - STEERING mapping correct!")
    else:
        print("✗ SERVO1 not responding to RC Ch3")
print()

# Test 2: Send throttle command to RC Ch1, look for change in SERVO3  
print("TEST 2: Throttle command (RC Ch1 → SERVO3)")
print("Sending 1700 to RC Channel 1 (throttle)...")

master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1700,  # Ch1 = throttle FORWARD
    0,     # Ch2  
    1500,  # Ch3 = steering neutral
    0, 0, 0, 0, 0)

time.sleep(1.5)
outputs2 = get_servo_outputs()
if outputs2:
    print(f"SERVO1 (steering): {outputs['servo1']} → {outputs2['servo1']} (Δ{outputs2['servo1'] - outputs['servo1']})")
    print(f"SERVO3 (throttle): {outputs['servo3']} → {outputs2['servo3']} (Δ{outputs2['servo3'] - outputs['servo3']})")
    
    if abs(outputs2['servo3'] - outputs['servo3']) > 10:
        print("✓ SERVO3 responds to RC Ch1 - THROTTLE mapping correct!")
    else:
        print("✗ SERVO3 not responding to RC Ch1")
print()

# Test 3: Try opposite mapping to confirm
print("TEST 3: Wrong mapping test")
print("Sending steering command to RC Ch1 (should NOT affect steering)...")

master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1200,  # Ch1 = trying steering on wrong channel
    0,     # Ch2
    1500,  # Ch3 = steering neutral  
    0, 0, 0, 0, 0)

time.sleep(1.5)
outputs3 = get_servo_outputs()
if outputs3:
    print(f"SERVO1 (steering): {outputs2['servo1']} → {outputs3['servo1']} (Δ{outputs3['servo1'] - outputs2['servo1']})")
    
    if abs(outputs3['servo1'] - outputs2['servo1']) < 10:
        print("✓ SERVO1 correctly ignores RC Ch1 - Confirms RC Ch3 for steering")
    else:
        print("✗ SERVO1 unexpectedly responds to RC Ch1")
print()

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

print("\nDone! This test will confirm if:")
print("• RC Ch3 commands control SERVO1 (steering)")
print("• RC Ch1 commands control SERVO3 (throttle)")
print("• Wrong channel mappings are ignored")
