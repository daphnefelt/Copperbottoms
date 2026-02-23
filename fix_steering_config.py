list#!/usr/bin/env python3
"""
Check steering servo configuration - why SERVO1 isn't responding to any RC channel
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

def get_param(param_name):
    """Get parameter value"""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1)
    
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg:
        # Handle both string and bytes param_id
        param_id = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode('utf-8')
        param_id = param_id.rstrip('\x00')
        if param_id == param_name:
            return msg.param_value
    return None

def set_param(param_name, value):
    """Set parameter value"""
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    
    # Wait for confirmation
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    if msg:
        # Handle both string and bytes param_id
        param_id = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode('utf-8')
        param_id = param_id.rstrip('\x00')
        if param_id == param_name:
            print(f"✓ Set {param_name} = {msg.param_value}")
            return True
    print(f"✗ Failed to set {param_name}")
    return False

print("=== STEERING SERVO CONFIGURATION CHECK ===")

print("1. Servo Functions:")
servo_functions = {}
for servo_num in [1, 2, 3, 4]:
    param_name = f'SERVO{servo_num}_FUNCTION'
    value = get_param(param_name)
    servo_functions[servo_num] = value
    print(f"SERVO{servo_num}_FUNCTION: {value}")

print(f"\n2. RC Channel Mapping:")
rcmap_params = ['RCMAP_ROLL', 'RCMAP_PITCH', 'RCMAP_THROTTLE', 'RCMAP_YAW', 'RCMAP_STEERING']
for param in rcmap_params:
    value = get_param(param)
    print(f"{param}: {value}")

print(f"\n3. Analysis:")
if servo_functions[1] == 26:
    print("✓ SERVO1_FUNCTION = 26 (Ground Steering) - CORRECT")
else:
    print(f"✗ SERVO1_FUNCTION = {servo_functions[1]} (should be 26 for Ground Steering)")

if servo_functions[3] == 70:
    print("✓ SERVO3_FUNCTION = 70 (Throttle) - CORRECT")
else:
    print(f"✗ SERVO3_FUNCTION = {servo_functions[3]} (should be 70 for Throttle)")

steering_channel = get_param('RCMAP_STEERING')
if steering_channel:
    print(f"✓ RCMAP_STEERING = {steering_channel} (steering mapped to RC channel {int(steering_channel)})")
else:
    print("✗ RCMAP_STEERING not configured!")

print(f"\n4. DIAGNOSIS:")
print("From channel test results:")
print("• ALL RC channels 1-8 affect SERVO3 (throttle)")  
print("• NO RC channels affect SERVO1 (steering)")
print("")

if servo_functions[1] != 26:
    print("PROBLEM: SERVO1_FUNCTION is not set to 26 (Ground Steering)")
    print("SOLUTION: Setting SERVO1_FUNCTION = 26...")
    if set_param('SERVO1_FUNCTION', 26):
        print("✓ Fixed!")
    else:
        print("✗ Failed to fix SERVO1_FUNCTION")

if not steering_channel:
    print("PROBLEM: RCMAP_STEERING not configured")
    print("SOLUTION: Setting RCMAP_STEERING = 1 (use RC Ch1 for steering)...")
    if set_param('RCMAP_STEERING', 1):
        print("✓ Fixed!")
    else:
        print("✗ Failed to fix RCMAP_STEERING")

# Save parameters
print("\nSaving parameters...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,
    1, 0, 0, 0, 0, 0, 0)

time.sleep(2)

print("\n=== TESTING STEERING AFTER FIX ===")
print("Testing if RC Ch1 now controls SERVO1...")

# Get baseline
msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
if msg:
    baseline_servo1 = msg.servo1_raw
    print(f"Baseline SERVO1: {baseline_servo1}")

# Send steering command to RC Ch1
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1300,  # Ch1 = steering LEFT
    1500, 1500, 1500, 1500, 1500, 1500, 1500)

time.sleep(1.5)

# Check if SERVO1 changed
msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
if msg:
    new_servo1 = msg.servo1_raw
    delta = new_servo1 - baseline_servo1
    print(f"After RC Ch1=1300: SERVO1 = {new_servo1} (Δ{delta})")
    
    if abs(delta) > 10:
        print("🎉 SUCCESS! Steering now works!")
    else:
        print("❌ Still not working")

# Release overrides
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0)

print("\nDone! Check if steering control is now working.")
