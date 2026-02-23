#!/usr/bin/env python3
"""
Find parameters that control RC override behavior for throttle
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

print("=== RC OVERRIDE CONTROL PARAMETERS ===\n")

# Parameters that might control RC override behavior
override_params = [
    ('RC_OVERRIDE_TIME', 'RC override timeout (0=disabled, >0=seconds)'),
    ('RC_FEEL_RP', 'RC feel for roll/pitch'),
    ('RC_OPTIONS', 'RC options bitmask'),
    ('SERVO3_FUNCTION', 'SERVO3 function (70=throttle)'),
    ('SERVO3_MIN', 'SERVO3 minimum PWM'),
    ('SERVO3_MAX', 'SERVO3 maximum PWM'),
    ('SERVO3_TRIM', 'SERVO3 neutral PWM'),
    ('SERVO3_REVERSED', 'SERVO3 direction (0=normal, 1=reversed)'),
    ('THR_MIN', 'Minimum throttle'),
    ('THR_MAX', 'Maximum throttle'),
    ('MOT_PWM_MIN', 'Motor PWM minimum'),
    ('MOT_PWM_MAX', 'Motor PWM maximum'),
    ('FRAME_CLASS', 'Vehicle frame class (0=undefined, 1=rover)'),
    ('FRAME_TYPE', 'Vehicle frame type'),
]

current_values = {}
for param_name, description in override_params:
    value = get_param(param_name)
    current_values[param_name] = value
    if value is not None:
        print(f"{param_name:16} = {value:8.0f}  ({description})")
    else:
        print(f"{param_name:16} = {'Not found':>8}  ({description})")

print(f"\n=== ANALYSIS ===")

# Check for issues
issues = []

rc_override_time = current_values.get('RC_OVERRIDE_TIME', 0)
if rc_override_time == 0:
    issues.append("RC_OVERRIDE_TIME=0 - RC overrides are DISABLED!")

servo3_function = current_values.get('SERVO3_FUNCTION', 0)
if servo3_function != 70:
    issues.append(f"SERVO3_FUNCTION={servo3_function} - Not set to 70 (throttle)")

frame_class = current_values.get('FRAME_CLASS', 0)
if frame_class == 0:
    issues.append("FRAME_CLASS=0 - Vehicle type not set (should be 1 for rover)")

if issues:
    print("🎯 FOUND THE PROBLEM!")
    for issue in issues:
        print(f"   • {issue}")
    
    if rc_override_time == 0:
        print(f"\n💡 SOLUTION: Set RC_OVERRIDE_TIME to enable RC overrides")
        print(f"   RC_OVERRIDE_TIME = 3  (RC overrides active for 3 seconds)")
        print(f"   This will allow RC override commands to control servos")
else:
    print("⚠️  No obvious RC override blocking found")
    print("The issue might be firmware-specific or hardware-related")

print(f"\n=== RECOMMENDED FIX ===")
if rc_override_time == 0:
    print("1. Set RC_OVERRIDE_TIME = 3 (enables 3-second RC override timeout)")
    print("2. This should allow RC Channel 3 commands to reach SERVO3")
    print("3. Retest with: python3 ~/test_throttle_simple.py")
else:
    print("RC override parameters look okay. Check:")
    print("1. SERVO3 output pin hardware connection")
    print("2. ESC signal wire connected to SERVO3")
    print("3. Different flight controller firmware version")

print("\nDone.")

