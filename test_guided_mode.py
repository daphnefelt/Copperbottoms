#!/usr/bin/env python3
"""
Control rover using GUIDED mode commands (no RC needed)
"""

from pymavlink import mavutil
import time

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}\n")

def set_mode(mode_name):
    """Set flight mode"""
    mode_map = {'MANUAL': 0, 'HOLD': 4, 'STEERING': 3, 'GUIDED': 15, 'AUTO': 10}
    
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
    
    time.sleep(0.5)
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg and msg.custom_mode == mode_id:
        print(f"✓ Mode: {mode_name}\n")
        return True
    else:
        print(f"✗ Failed to switch to {mode_name}\n")
        return False

def arm_vehicle():
    """Force arm"""
    print("Arming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 21196, 0, 0, 0, 0, 0)
    time.sleep(1)
    print("✓ Armed\n")

def disarm_vehicle():
    """Disarm"""
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    time.sleep(1)
    print("✓ Disarmed\n")

def send_movement(steering, throttle):
    """
    Send movement command in GUIDED mode
    steering: -1.0 (left) to 1.0 (right)
    throttle: -1.0 (reverse) to 1.0 (forward)
    """
    master.mav.manual_control_send(
        master.target_system,
        int(steering * 1000),  # Steering (-1000 to 1000)
        int(throttle * 1000),  # Throttle (-1000 to 1000)
        0,  # Unused
        0,  # Unused
        0)  # Buttons

def monitor_servos(duration=3):
    """Monitor servo outputs for a few seconds"""
    print("Servo outputs:")
    start = time.time()
    while time.time() - start < duration:
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
        if msg:
            print(f"  Ch1: {msg.servo1_raw}  Ch3: {msg.servo3_raw}")
            time.sleep(0.5)

print("=== GUIDED MODE SERVO TEST ===")
print("⚠️  SAFETY: Keep wheels OFF the ground!\n")

# Set GUIDED mode
if set_mode('GUIDED'):
    arm_vehicle()
    
    try:
        print("Test 1: Steering LEFT (steering=-0.5, throttle=0)")
        send_movement(-0.5, 0.0)
        monitor_servos(3)
        
        print("\nTest 2: Steering RIGHT (steering=0.5, throttle=0)")
        send_movement(0.5, 0.0)
        monitor_servos(3)
        
        print("\nTest 3: CENTER (steering=0, throttle=0)")
        send_movement(0.0, 0.0)
        monitor_servos(3)
        
        confirm = input("\nWHEELS OFF GROUND? Test throttle? (y/n): ")
        if confirm.lower() == 'y':
            print("\nTest 4: SLOW FORWARD (steering=0, throttle=0.2)")
            send_movement(0.0, 0.2)
            monitor_servos(4)
            
            print("\nTest 5: STOP")
            send_movement(0.0, 0.0)
            monitor_servos(2)
        
        print("\n--- Analysis ---")
        print("If servos changed: SUCCESS! GUIDED mode works")
        print("If still stuck: Check Pixhawk parameters:")
        print("  - FS_ACTION (failsafe action)")
        print("  - FS_TIMEOUT (failsafe timeout)")
        print("  - Or connect an actual RC receiver")
        
    except KeyboardInterrupt:
        print("\nStopping...")
    
    send_movement(0.0, 0.0)
    time.sleep(1)
    disarm_vehicle()
    set_mode('HOLD')
else:
    print("Failed to set GUIDED mode")

