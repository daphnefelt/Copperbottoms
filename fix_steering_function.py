#!/usr/bin/env python3
"""
Fix steering by changing SERVO1_FUNCTION to RCPassThru mode
This allows RC override commands to work without a physical RC receiver
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

print("=== FIXING SERVO 1 STEERING ===\n")
print("Problem: SERVO1_FUNCTION=27 (Steering) requires physical RC receiver")
print("Solution: Change to SERVO1_FUNCTION=51 (RCPassThru1)")
print("          This allows software RC override commands to work\n")

# Check current value
print("Checking current SERVO1_FUNCTION...")
master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b'SERVO1_FUNCTION',
    -1)

msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
if msg:
    current_value = int(msg.param_value)
    print(f"Current: SERVO1_FUNCTION = {current_value}")
    
    if current_value == 27:
        print("         (27 = Steering - requires physical RC)\n")
    elif current_value == 51:
        print("         (51 = RCPassThru1 - already set!)\n")
        print("✓ No change needed, already in passthrough mode")
        exit(0)

# Change to RCPassThru
print("\nSetting SERVO1_FUNCTION = 51 (RCPassThru1)...")

master.mav.param_set_send(
    master.target_system,
    master.target_component,
    b'SERVO1_FUNCTION',
    51.0,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

time.sleep(2)

# Verify the change
master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b'SERVO1_FUNCTION',
    -1)

msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
if msg:
    new_value = int(msg.param_value)
    print(f"✓ SERVO1_FUNCTION = {new_value}")
    
    if new_value == 51:
        print("\n✓ SUCCESS! Steering is now in RCPassThru mode")
        print("\nNow test steering with:")
        print("  python3 ~/test_steering_armed.py")
        print("\nOr test with RC overrides:")
        print("  python3 ~/test_servo_manual.py")
    else:
        print("\n✗ Change failed, try using Mission Planner to change parameter")
else:
    print("\n✗ Could not verify change")

print("\nNote: Parameter changes are saved to Pixhawk automatically")
