#!/usr/bin/env python3
"""
Diagnose servo output configuration
"""

from pymavlink import mavutil
import time

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}\n")

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

print("=== SERVO OUTPUT CONFIGURATION ===\n")

# Check servo functions
print("Servo Functions (SERVO_n_FUNCTION):")
for i in range(1, 9):
    func = get_param(f'SERVO{i}_FUNCTION')
    if func is not None:
        # Rover servo functions: 0=disabled, 1=flap, 26=throttle, 27=steering
        func_names = {
            0: 'Disabled',
            1: 'Flap', 
            26: 'Ground Steering',
            27: 'Steering',
            70: 'Throttle (Throttle)',
        }
        func_name = func_names.get(int(func), f'Unknown({int(func)})')
        print(f"  SERVO{i}_FUNCTION = {int(func)} ({func_name})")

print("\n\nServo Min/Max (PWM range):")
for i in [1, 3]:
    min_pwm = get_param(f'SERVO{i}_MIN')
    max_pwm = get_param(f'SERVO{i}_MAX')
    if min_pwm and max_pwm:
        print(f"  SERVO{i}: {int(min_pwm)} - {int(max_pwm)} us")

print("\n\nMotor Configuration:")
print(f"MOTOR_TYPE = {get_param('MOTOR_TYPE')} (1=single, 2=dual)")
print(f"SKID_STEER_OUT = {get_param('SKID_STEER_OUT')}")

print("\n\nCommon Issues:")
print("1. SERVO1_FUNCTION and SERVO3_FUNCTION not set to steering/throttle")
print("2. Servos have no power (separate BEC needed)")
print("3. Servo cables not connected to Pixhawk outputs")
print("4. Outputs disabled by SERVO_DISABLED parameter")

print("\n\n=== HARDWARE CHECK ===")
print("Do your servos have external power?")
print("  - Most servos/ESCs need a separate BEC (5V supply)")
print("  - Pixhawk signal pins alone won't power servos")
print("\nCheck connections:")
print("  - Steering servo: connected to Pixhawk output 1?")
print("  - ESC/Throttle: connected to Pixhawk output 3?")
print("  - Both have power supply?")
