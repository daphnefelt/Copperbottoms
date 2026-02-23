#!/usr/bin/env python3
"""
Find the actual SERVO output value that stops the motor
Ignore RC input values - only look at SERVO_OUTPUT_RAW
"""

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
master.wait_heartbeat()
print("Connected!\n")

print("=== SERVO OUTPUT DIAGNOSTIC ===")
print("⚠️  WHEELS OFF THE GROUND!\n")

input("Press Enter when wheels are OFF ground...")

# Arm
print("\nArming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 21196, 0, 0, 0, 0, 0)
time.sleep(2)

# Test full range
test_values = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000]

print("\nTesting RC input values and monitoring actual SERVO output:\n")
print("RC Input | Actual SERVO3 | Motor Status")
print("---------|---------------|---------------")

for throttle in test_values:
    # Send RC command
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500,     # Ch1 = steering center
        0,
        throttle, # Ch3 = throttle
        0, 0, 0, 0, 0)
    
    time.sleep(1)
    
    # Get ACTUAL servo output
    servo3_output = None
    for _ in range(5):
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
        if msg:
            servo3_output = msg.servo3_raw
    
    if servo3_output is None:
        servo3_output = "N/A"
    
    # Ask user
    response = input(f"{throttle:8} | {servo3_output:13}| Spin/Stop: ").lower()
    
    if response == 's' or 'stop' in response:
        print(f"\n✓ MOTOR STOPS at SERVO output {servo3_output}us")
        print(f"  (when RC throttle input = {throttle})")
        break
    else:
        print("")

# Release and disarm
print("\nReleasing and disarming...")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

print("Done\n")
print("Use the SERVO output value (not RC input) in your test scripts!")
