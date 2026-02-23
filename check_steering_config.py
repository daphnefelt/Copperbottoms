#!/usr/bin/env python3
"""
Check servo1 configuration for steering
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

print("=== SERVO1 (STEERING) CONFIGURATION ===\n")

# Request servo1 parameters
params_to_check = [
    'SERVO1_FUNCTION',
    'SERVO1_MIN', 
    'SERVO1_MAX',
    'SERVO1_TRIM',
    'SERVO1_REVERSED'
]

for param in params_to_check:
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param.encode('utf-8'),
        -1)
    
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg and msg.param_id.strip('\x00') == param:
        print(f"{param}: {msg.param_value}")
    else:
        print(f"{param}: TIMEOUT")

print(f"\nExpected for rover steering:")
print(f"SERVO1_FUNCTION: 26 (Ground Steering)")
print(f"SERVO1_MIN: ~1100")  
print(f"SERVO1_MAX: ~1900")
print(f"SERVO1_TRIM: ~1500")
print(f"SERVO1_REVERSED: 0 (normal)")

print(f"\nNote: If SERVO1_FUNCTION is not 26, that's the problem!")

