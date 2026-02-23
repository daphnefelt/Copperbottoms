#!/usr/bin/env python3
from pymavlink import mavutil
import time

# Connect to Pixhawk
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}, Component ID: {master.target_component}")

# Request data streams
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10, 1)

# Read messages for 10 seconds
print("\nReceiving messages...")
start = time.time()
while time.time() - start < 10:
    msg = master.recv_match(blocking=True, timeout=1)
    if msg:
        print(f"{msg.get_type()}: {msg}")
