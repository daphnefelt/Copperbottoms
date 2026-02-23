#!/usr/bin/env python3
"""
Expand servo ranges and test wider PWM values
"""

from pymavlink import mavutil
import time

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print(f"Connected!\n")

def set_param(param_name, param_value):
    """Set a parameter"""
    print(f"Setting {param_name} = {param_value}")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    
    time.sleep(0.5)
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg:
        print(f"  ✓ Set to {msg.param_value}\n")
        return True
    return False

print("=== EXPAND SERVO RANGES ===\n")

print("Expanding to standard 1000-2000 range...\n")
set_param('SERVO1_MIN', 1000.0)
set_param('SERVO1_MAX', 2000.0)
set_param('SERVO3_MIN', 1000.0)
set_param('SERVO3_MAX', 2000.0)

print("\n=== TESTING WIDER RANGES ===\n")

print("Arming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(1)

print("Setting MANUAL mode...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    0,  # MANUAL
    0, 0, 0, 0, 0)
time.sleep(1)

print("\nServo 1 (Steering) - Full range with wider limits")
print("  MIN (1000)...")
master.mav.rc_channels_override_send(master.target_system, master.target_component, 1000, 0, 1500, 0, 0, 0, 0, 0)
time.sleep(2)

print("  MAX (2000)...")
master.mav.rc_channels_override_send(master.target_system, master.target_component, 2000, 0, 1500, 0, 0, 0, 0, 0)
time.sleep(2)

print("  CENTER (1500)...")
master.mav.rc_channels_override_send(master.target_system, master.target_component, 1500, 0, 1500, 0, 0, 0, 0, 0)
time.sleep(1)

print("\nServo 3 (Throttle) - Full range with wider limits")
print("  MIN (1000)...")
master.mav.rc_channels_override_send(master.target_system, master.target_component, 1500, 0, 1000, 0, 0, 0, 0, 0)
time.sleep(2)

print("  MAX (2000)...")
master.mav.rc_channels_override_send(master.target_system, master.target_component, 1500, 0, 2000, 0, 0, 0, 0, 0)
time.sleep(2)

print("  NEUTRAL (1500)...")
master.mav.rc_channels_override_send(master.target_system, master.target_component, 1500, 0, 1500, 0, 0, 0, 0, 0)

print("\n✓ Done! Watch for magnitude of servo movement.")
print("  If they move more -> wider ranges work")
print("  If same movement -> hardware limits narrower than 1000-2000")

