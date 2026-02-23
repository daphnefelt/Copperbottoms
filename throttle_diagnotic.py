#!/usr/bin/env python3
"""
Find which RC channel controls SERVO3 (throttle) in MANUAL mode
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

print("=== THROTTLE CHANNEL FINDER (MANUAL MODE) ===")
print("⚠️  WHEELS OFF THE GROUND!\n")

input("Press Enter when wheels are OFF ground...")

# Set to MANUAL mode first
print("\nSetting to MANUAL mode...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    0,  # MANUAL mode for ArduPilot rover
    0, 0, 0, 0, 0)
time.sleep(1)

# Verify mode change
print("Checking flight mode...")
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
if msg:
    if msg.custom_mode == 0:
        print("✅ Successfully set to MANUAL mode")
    else:
        print(f"⚠️  Warning: Mode is {msg.custom_mode}, expected 0 for MANUAL")
else:
    print("❌ Could not verify flight mode")

# Arm
print("\nArming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(2)

# Check arming status
print("Checking arming status...")
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
    print("✅ Successfully armed")
else:
    print("❌ Failed to arm - check safety switches and parameters")

# Test both RC overrides and manual_control commands
print("\n=== Testing RC Channel Overrides ===")
print("Channel | SERVO3 Output | Effect")
print("--------|---------------|--------")

base_channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
motor_channel_found = False

for test_channel in range(1, 5):  # Test channels 1-4
    print(f"\n--- Testing RC Channel {test_channel} (RC Override) ---")
    
    # Set all channels to neutral
    channels = base_channels.copy()
    
    for test_value in [1300, 1700]:  # Test low and high
        channels[test_channel-1] = test_value  # Set test channel
        
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            channels[0], channels[1], channels[2], channels[3], 
            channels[4], channels[5], channels[6], channels[7])
        
        time.sleep(1.5)
        
        # Get servo output
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
        if msg:
            servo3 = msg.servo3_raw
        else:
            servo3 = "N/A"
        
        # Ask user what moved
        response = input(f"Ch{test_channel}={test_value} | {servo3:13} | What moved? (s=steering/m=motor/n=nothing): ").lower()
        
        if 'm' in response or 'motor' in response:
            print(f"🎯 FOUND IT! RC Channel {test_channel} controls the motor!")
            print(f"    SERVO3 output: {servo3}")
            motor_channel_found = True
            break
        elif 's' in response or 'steering' in response:
            print(f"    RC Channel {test_channel} controls steering")
        else:
            print(f"    RC Channel {test_channel} - no response")
    
    if motor_channel_found:
        break

# If RC overrides didn't work, try manual_control commands
if not motor_channel_found:
    print("\n=== RC Override Failed - Testing Manual Control ===")
    print("Manual_control allows direct servo control in MANUAL mode")
    
    # Clear all RC overrides first
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(1)
    
    print("\nTesting manual_control throttle (X parameter)...")
    for throttle_value in [-500, 0, 500]:  # Range -1000 to 1000
        print(f"\nSending manual_control with X={throttle_value}")
        
        master.mav.manual_control_send(
            master.target_system,
            throttle_value,  # X (throttle)
            0,               # Y (steering) 
            0,               # Z
            0,               # R
            0)               # Buttons
        
        time.sleep(2)
        
        # Get servo output
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
        if msg:
            servo3 = msg.servo3_raw
            print(f"SERVO3 output: {servo3}")
        else:
            print("No servo output received")
        
        response = input("Did the motor move? (y/n): ").lower()
        if 'y' in response:
            print("🎯 Manual control throttle is working!")
            motor_channel_found = True
            break
        
    if not motor_channel_found:
        print("\nTesting manual_control steering (Y parameter)...")
        for steering_value in [-500, 0, 500]:  # Range -1000 to 1000
            print(f"\nSending manual_control with Y={steering_value}")
            
            master.mav.manual_control_send(
                master.target_system,
                0,               # X (throttle)
                steering_value,  # Y (steering) 
                0,               # Z
                0,               # R
                0)               # Buttons
            
            time.sleep(2)
            
            response = input("Did anything move? (s=steering/m=motor/n=nothing): ").lower()
            if 'm' in response:
                print("🎯 Manual control Y parameter controls motor!")
                motor_channel_found = True
                break
            elif 's' in response:
                print("Manual control Y parameter controls steering")

# Final status
if not motor_channel_found:
    print("\n❌ THROTTLE CONTROL NOT WORKING")
    print("Possible issues:")
    print("- Safety parameters blocking throttle")
    print("- ESC not properly calibrated") 
    print("- Wrong servo function assignments")
    print("- Hardware connection problem")

# Release and disarm
print("\nStopping all controls...")

# Clear RC overrides
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0)

# Send neutral manual_control commands
for _ in range(3):  # Send multiple times to ensure it's received
    master.mav.manual_control_send(
        master.target_system,
        0,   # X (throttle) = 0
        0,   # Y (steering) = 0
        0,   # Z
        0,   # R  
        0)   # Buttons
    time.sleep(0.1)

print("Disarming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

print("✅ Diagnostic complete\n")
