#!/usr/bin/env python3
"""
Test servo control using RC overrides in MANUAL mode
"""

from pymavlink import mavutil
import time

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}\n")

def set_mode(mode_name):
    """Set flight mode"""
    # Rover modes: MANUAL=0, HOLD=4, STEERING=3, AUTO=10
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
        print(f"✓ Switched to {mode_name} mode\n")
        return True
    else:
        print(f"✗ Failed to switch to {mode_name}\n")
        return False

def arm_vehicle():
    """Force arm"""
    print("Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 21196, 0, 0, 0, 0, 0)
    
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if msg and msg.result == 0:
        print("✓ Armed\n")
        return True
    else:
        print("✗ Arming failed\n")
        return False

def disarm_vehicle():
    """Disarm"""
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    print("✓ Disarmed\n")

def set_rc_override(ch1=0, ch3=0):
    """
    Set RC overrides (1000-2000, or 0 to release)
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        ch1, 0, ch3, 0, 0, 0, 0, 0)
    
def stop_all():
    """Release all RC overrides"""
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0)
    print("Released all RC overrides\n")

print("=== RC OVERRIDE SERVO TEST ===")
print("⚠️  SAFETY: Keep wheels OFF the ground!\n")

# Switch to MANUAL mode
set_mode('MANUAL')
time.sleep(1)

# Arm
armed = arm_vehicle()

if armed:
    try:
        while True:
            print("\nOptions:")
            print("1. Test Steering (Channel 1)")
            print("2. Test Throttle (Channel 3) - WHEELS OFF GROUND!")
            print("3. Custom RC values")
            print("4. Stop/Center all (1500)")
            print("5. Release overrides")
            print("0. Disarm & exit")
            
            choice = input("\nChoice: ").strip()
            
            if choice == '1':
                print("Testing steering: Left -> Center -> Right")
                print("(PWM range: 1100-1750, center: 1425)")
                set_rc_override(ch1=1100, ch3=1425)  # Left (SERVO1 min)
                time.sleep(1.5)
                set_rc_override(ch1=1425, ch3=1425)  # Center
                time.sleep(1)
                set_rc_override(ch1=1750, ch3=1425)  # Right (SERVO1 max)
                time.sleep(1.5)
                set_rc_override(ch1=1425, ch3=1425)  # Center
                print("Done\n")
                
            elif choice == '2':
                confirm = input("WHEELS OFF GROUND? (y/n): ")
                if confirm.lower() == 'y':
                    print("Testing throttle: neutral -> forward -> stop")
                    print("(PWM range: 1100-1900, neutral: 1500)")
                    set_rc_override(ch1=1425, ch3=1500)  # Neutral
                    time.sleep(1)
                    set_rc_override(ch1=1425, ch3=1650)  # Slow forward
                    time.sleep(2)
                    set_rc_override(ch1=1425, ch3=1500)  # Stop
                    print("Done\n")
                    
            elif choice == '3':
                print("Steering range: 1100-1750 (center: 1425)")
                print("Throttle range: 1100-1900 (neutral: 1500)")
                ch1 = int(input("Ch1 steering (1100-1750): "))
                ch3 = int(input("Ch3 throttle (1100-1900): "))
                set_rc_override(ch1=ch1, ch3=ch3)
                print(f"Set: Ch1={ch1}, Ch3={ch3}\n")
                
            elif choice == '4':
                set_rc_override(ch1=1500, ch3=1425)  # Servo1 center: 1425, Servo3 neutral: 1500
                print("Centered\n")
                
            elif choice == '5':
                stop_all()
                
            elif choice == '0':
                stop_all()
                disarm_vehicle()
                break
                
    except KeyboardInterrupt:
        print("\n\nEmergency stop!")
        stop_all()
        disarm_vehicle()
else:
    print("Cannot test - arming failed")

