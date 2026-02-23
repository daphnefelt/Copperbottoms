#!/usr/bin/env python3
"""
Test script to manually control Pixhawk servos
Use this to identify which servo controls what on your rover
"""

from pymavlink import mavutil
import time

# Connect to Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}\n")

def arm_vehicle():
    """Arm the Pixhawk"""
    print("Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    
    # Wait for arm confirmation
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if msg and msg.result == 0:
        print("✓ Armed successfully!\n")
        return True
    else:
        print("✗ Arming failed! Check safety switches and pre-arm checks.\n")
        return False

def disarm_vehicle():
    """Disarm the Pixhawk"""
    print("Disarming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    print("✓ Disarmed\n")

def set_servo(channel, pwm):
    """Set a servo channel to a specific PWM value (1000-2000)"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,  # Servo number
        pwm,      # PWM value
        0, 0, 0, 0, 0)
    print(f"Set servo {channel} to {pwm} us")

def center_all_servos():
    """Center all servos (1500us = neutral)"""
    for i in range(1, 9):
        set_servo(i, 1500)
    print("All servos centered at 1500us\n")

# Main test loop
print("=== SERVO CONTROL TEST ===")
print("Servo range: 1000-2000 (1500 = center)")
print("⚠️  SAFETY: Keep wheels OFF the ground!")
print("Press Ctrl+C to exit\n")

# First, center everything
center_all_servos()
time.sleep(1)

# Arm the vehicle
armed = arm_vehicle()

try:
    while True:
        print("\nOptions:")
        print("0. ARM vehicle" if not armed else "0. DISARM vehicle")
        print("1. Test Servo 1 (steering?)")
        print("2. Test Servo 3 (throttle?)")
        print("3. Set specific servo")
        print("4. Center all servos")
        print("5. Emergency stop & disarm")
        
        choice = input("\nEnter choice (0-5): ").strip()
        
        if choice == '0':
            if armed:
                disarm_vehicle()
                armed = False
            else:
                armed = arm_vehicle()
                
        elif choice == '1':
            if not armed:
                print("⚠️  Vehicle not armed! Arm first (option 0)")
                continue
            print("\nTesting Servo 1 - Left/Right sweep")
            set_servo(1, 1200)  # Left
            time.sleep(1)
            set_servo(1, 1500)  # Center
            time.sleep(1)
            set_servo(1, 1800)  # Right
            time.sleep(1)
            set_servo(1, 1500)  # Back to center
            
        elif choice == '2':
            if not armed:
                print("⚠️  Vehicle not armed! Arm first (option 0)")
                continue
            print("\nTesting Servo 3 - slow forward sweep")
            print("WARNING: Make sure wheels are off the ground!")
            confirm = input("Continue? (y/n): ")
            if confirm.lower() == 'y':
                set_servo(3, 1500)  # Neutral
                time.sleep(1)
                set_servo(3, 1550)  # Slow forward
                time.sleep(2)
                set_servo(3, 1500)  # Stop
                
        elif choice == '3':
            if not armed:
                print("⚠️  Vehicle not armed! Arm first (option 0)")
                continue
            channel = int(input("Enter servo channel (1-8): "))
            pwm = int(input("Enter PWM value (1000-2000): "))
            if 1 <= channel <= 8 and 1000 <= pwm <= 2000:
                set_servo(channel, pwm)
            else:
                print("Invalid values!")
                
        elif choice == '4':
            center_all_servos()
            
        elif choice == '5':
            print("EMERGENCY STOP!")
            center_all_servos()
            disarm_vehicle()
            armed = False
            
        else:
            print("Invalid choice!")

except KeyboardInterrupt:
    print("\n\n⚠️  EMERGENCY STOP - Centering all servos and disarming")
    center_all_servos()
    disarm_vehicle()
    print("Done!")
