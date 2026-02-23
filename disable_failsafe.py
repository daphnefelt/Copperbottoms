#!/usr/bin/env python3
"""
Disable RC failsafe to allow control without RC receiver
"""

from pymavlink import mavutil
import time

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}\n")

def set_param(param_name, param_value):
    """Set a parameter"""
    print(f"Setting {param_name} = {param_value}")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    
    # Wait for confirmation
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg:

	# Handle both bytes and str types
        param_id = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode('utf8')
        param_id = param_id.strip('\x00')
        if param_id == param_name:
            print(f"  ✓ {param_name} = {msg.param_value}\n")
            return True
    print(f"  ✗ Failed to set {param_name}\n")
    return False
   

def get_param(param_name):
    """Get a parameter value"""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf8'),
        -1)
    
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg:
        return msg.param_value
    return None

print("=== DISABLE RC FAILSAFE ===\n")

print("Current parameters:")
print(f"FS_ACTION: {get_param('FS_ACTION')}")
print(f"FS_TIMEOUT: {get_param('FS_TIMEOUT')}")
print(f"FS_THR_ENABLE: {get_param('FS_THR_ENABLE')}")
print()

print("Disabling RC failsafe for testing...")
print("(This allows control without RC receiver)\n")

# FS_ACTION: What to do on failsafe (0=disabled, 1=hold, 2=RTL, etc)
set_param('FS_ACTION', 0.0)

# FS_TIMEOUT: RC timeout in seconds (0=disabled)
set_param('FS_TIMEOUT', 0.0)

# FS_THR_ENABLE: Throttle failsafe (0=disabled)
set_param('FS_THR_ENABLE', 0.0)

print("\n✓ RC failsafe disabled!")
print("\nNow try: python3 ~/test_servo_manual.py")
print("Servos should respond to RC overrides in MANUAL mode\n")

