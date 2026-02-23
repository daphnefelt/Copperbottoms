#!/usr/bin/env python3
"""
Fix servo configuration for rover:
- SERVO1_FUNCTION = 26 (Ground Steering)  
- SERVO3_FUNCTION = 70 (Throttle)
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

def set_param(param_name, value):
    """Set a parameter value"""
    print(f"Setting {param_name} = {value}")
    
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    
    # Wait for confirmation
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg and msg.param_id.strip('\x00') == param_name:
        print(f"✓ {param_name} = {msg.param_value}")
        return True
    else:
        print(f"✗ Failed to set {param_name}")
        return False

print("=== FIXING SERVO CONFIGURATION ===\n")

print("Current issues:")
print("- SERVO1_FUNCTION: 51 (should be 26 for steering)")
print("- SERVO3_FUNCTION: 26 (should be 70 for throttle)")
print()

confirm = input("Fix servo configuration? (y/n): ")
if confirm.lower() != 'y':
    print("Cancelled")
    exit()

print("\nFixing servo functions...")

# Fix steering (servo1)
success1 = set_param('SERVO1_FUNCTION', 26.0)  # Ground Steering

# Fix throttle (servo3) 
success2 = set_param('SERVO3_FUNCTION', 70.0)  # Throttle

if success1 and success2:
    print(f"\n✅ SERVO CONFIGURATION FIXED!")
    print(f"- SERVO1_FUNCTION = 26 (Ground Steering)")
    print(f"- SERVO3_FUNCTION = 70 (Throttle)")
    print(f"\nNow try the steering and throttle diagnostics again!")
else:
    print(f"\n❌ Some parameters failed to set")

print(f"\nNote: You may need to reboot the Pixhawk for changes to take full effect.")
