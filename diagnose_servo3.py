#!/usr/bin/env python3
"""
Monitor actual servo 3 output while sending throttle commands
"""

from pymavlink import mavutil
import time

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print(f"Connected!\n")

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

print("Arming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(1)

print("\n=== MONITORING SERVO 3 OUTPUT ===\n")

print("Sending throttle command (1200)...")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500, 0, 1200, 0, 0, 0, 0, 0)

print("Servo 3 values:")
for i in range(5):
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        print(f"  Servo3: {msg.servo3_raw}us")
    time.sleep(0.5)

print("\nSending throttle command (1800)...")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500, 0, 1800, 0, 0, 0, 0, 0)

print("Servo 3 values:")
for i in range(5):
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        print(f"  Servo3: {msg.servo3_raw}us")
    time.sleep(0.5)

print("\n--- Diagnosis ---")
print("If Servo3 changed: Hardware issue (ESC not powered, not connected)")
print("If Servo3 DIDN'T change: Servo function/parameter issue")
print("\nCheck:")
print("  1. Is ESC signal cable connected to Pixhawk output 3?")
print("  2. Does ESC have power (battery connected)?")
print("  3. Run: python3 ~/check_servo_config.py")
print("     Look at SERVO3_FUNCTION value")

