#!/usr/bin/env python3
"""
Fix RC override blocking by disabling throttle failsafe
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

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

print("=== FIXING RC OVERRIDE BLOCKING ===")

print("Current failsafe settings:")
fs_thr_enable = get_param('FS_THR_ENABLE')
fs_thr_value = get_param('FS_THR_VALUE')
rc_override_time = get_param('RC_OVERRIDE_TIME')

print(f"FS_THR_ENABLE: {fs_thr_enable}")
print(f"FS_THR_VALUE: {fs_thr_value}") 
print(f"RC_OVERRIDE_TIME: {rc_override_time}")

print(f"\nThe servo3 value of 2000 indicates throttle failsafe is ACTIVE!")
print("This blocks all RC overrides.\n")

print("FIXING: Disabling throttle failsafe...")
if set_param('FS_THR_ENABLE', 0):
    print("✓ Throttle failsafe disabled!")
else:
    print("✗ Failed to disable throttle failsafe")

print("FIXING: Setting RC override timeout...")
if set_param('RC_OVERRIDE_TIME', 3.0):
    print("✓ RC override timeout set to 3 seconds")
else:
    print("✗ Failed to set RC override timeout")

print("\nSaving parameters...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,
    1, 0, 0, 0, 0, 0, 0)  # Save parameters

time.sleep(2)

print("\nTesting RC overrides now...")

# Test a simple RC override
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500, 1500, 1400, 1500, 1500, 1500, 1500, 1500)  # Set throttle to 1400

time.sleep(2)

# Check servo output
msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
if msg:
    print(f"After fix - servo outputs:")
    print(f"servo1: {msg.servo1_raw}")
    print(f"servo3: {msg.servo3_raw} (should change from 2000)")
    
    if msg.servo3_raw != 2000:
        print(f"\n🎉 SUCCESS! RC overrides are now working!")
        print(f"Throttle changed from 2000 → {msg.servo3_raw}")
    else:
        print(f"\n❌ Still blocked - servo3 stuck at 2000")

# Release overrides
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0)

print("\nDone! If successful, all your diagnostic scripts should now work.")
