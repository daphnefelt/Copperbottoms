#!/usr/bin/env python3
"""
Bypass RCMAP and directly control SERVO1 using servo output commands
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
        print(f"✓ Switched to {mode_name} mode")
        return True
    else:
        print(f"✗ Failed to switch to {mode_name}")
        return False

def direct_servo_control(servo_num, pwm_value):
    """Directly control a servo using DO_SET_SERVO command"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_num,    # Servo number (1-8)
        pwm_value,    # PWM value (1000-2000)
        0, 0, 0, 0, 0)
    
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
    return msg and msg.result == 0

print("=== DIRECT SERVO CONTROL TEST ===")
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

print("\n=== TESTING DIRECT SERVO1 CONTROL ===")
print("This bypasses RC overrides and directly controls SERVO1")

# Get baseline
msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
if msg:
    baseline = {
        'servo1': msg.servo1_raw,
        'servo2': msg.servo2_raw,
        'servo3': msg.servo3_raw,
        'servo4': msg.servo4_raw
    }
    print(f"Baseline servos: {baseline}")

# Test direct servo control
test_values = [1200, 1300, 1400, 1500, 1600, 1700, 1800]

print(f"\nTesting direct SERVO1 control:")
print("Value | Servo1 | User Response")
print("------|--------|---------------")

for pwm in test_values:
    print(f"Setting SERVO1 to {pwm}...")
    
    if direct_servo_control(1, pwm):
        time.sleep(1.5)
        
        # Check servo output
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
        if msg:
            actual_servo1 = msg.servo1_raw
            delta = actual_servo1 - baseline['servo1']
            
            # Ask user for steering position
            response = input(f"{pwm:4} | {actual_servo1:6} | Steering (l=left/c=center/r=right): ").lower()
            
            if 'l' in response or 'left' in response:
                print(f"← LEFT steering at PWM {pwm} (servo output: {actual_servo1})")
            elif 'r' in response or 'right' in response:
                print(f"→ RIGHT steering at PWM {pwm} (servo output: {actual_servo1})")
            elif 'c' in response or 'center' in response:
                print(f"| CENTER steering at PWM {pwm} (servo output: {actual_servo1})")
            else:
                print(f"  No clear response at PWM {pwm}")
        else:
            print(f"No servo output received for {pwm}")
    else:
        print(f"Failed to set SERVO1 to {pwm}")
    
    print()

print("\n=== ALTERNATIVE: RC OVERRIDE WITH DIRECT MAPPING ===")
print("Since RCMAP_STEERING doesn't work, let's manually map RC Ch1 → SERVO1")

# Try a custom mapping approach
print("Testing manual RC Ch1 → SERVO1 mapping...")

# Send command to RC Ch1 and see if we can get SERVO1 to change
# by using the fact that throttle mapping might be interfering

master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1300,  # Ch1 = try steering 
    1500,  # Ch2 = neutral
    1500,  # Ch3 = throttle neutral (this was working)
    1500,  # Ch4 = neutral
    1500, 1500, 1500, 1500)

time.sleep(1.5)

msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
if msg:
    print(f"After RC override - SERVO1: {baseline['servo1']} → {msg.servo1_raw}")
    if abs(msg.servo1_raw - baseline['servo1']) > 10:
        print("✓ RC override now affects SERVO1!")
    else:
        print("✗ RC override still doesn't affect SERVO1")

# Clean up
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0)

# Set servo back to center
direct_servo_control(1, 1450)

# Disarm
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

print("\n=== SUMMARY ===")
print("This test checked:")
print("1. Direct SERVO1 control (bypasses RC system)")
print("2. RC override after direct servo commands")
print("3. Actual steering response ranges")
print("\nIf direct control works but RC override doesn't,")
print("the issue is in the RC→Servo mapping configuration.")
