#!/usr/bin/env python3
"""
Monitor actual servo output values while sending RC overrides
"""

from pymavlink import mavutil
import time

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}\n")

# Switch to MANUAL mode
print("Setting MANUAL mode...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    0,  # MANUAL mode
    0, 0, 0, 0, 0)
time.sleep(1)

# Force arm
print("Arming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(1)

print("\n=== MONITORING SERVO OUTPUTS ===")
print("Sending RC override: Ch1=1200 (left), Ch3=1500 (neutral)")
print("Watch the servo values below - they should change!\n")

# Send RC override
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1200,  # Ch1 - steering left
    0,
    1500,  # Ch3 - throttle neutral
    0, 0, 0, 0, 0)

# Monitor for 5 seconds
print("Current servo outputs:")
start = time.time()
while time.time() - start < 5:
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        print(f"  Servo1: {msg.servo1_raw}us  Servo3: {msg.servo3_raw}us")
        time.sleep(0.5)

print("\nChanging to: Ch1=1800 (right), Ch3=1600 (forward)")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1800,  # Ch1 - steering right
    0,
    1600,  # Ch3 - throttle forward
    0, 0, 0, 0, 0)

start = time.time()
while time.time() - start < 5:
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        print(f"  Servo1: {msg.servo1_raw}us  Servo3: {msg.servo3_raw}us")
        time.sleep(0.5)

print("\n--- Analysis ---")
print("If servo values changed: Hardware/power issue - check:")
print("  - Servos/ESC have external power supply")
print("  - Servo signal wires connected to correct Pixhawk outputs")
print("  - ESC is calibrated for the PWM range")
print("\nIf servo values did NOT change: Configuration issue:")
print("  - Outputs may be disabled")
print("  - Wrong firmware mode")
print("  - Parameters need adjustment")

# Clean up
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0)

