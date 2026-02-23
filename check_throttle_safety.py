#!/usr/bin/env python3
"""
Check safety parameters that might block throttle RC overrides
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

print("=== SAFETY PARAMETERS THAT CAN BLOCK THROTTLE ===\n")

# Key safety parameters for rovers
safety_params = [
    ('ARMING_REQUIRE', 'Arming requirements (0=none, 1=basic checks)'),
    ('ARMING_CHECK', 'Pre-arm safety checks mask'),
    ('FS_THR_ENABLE', 'Throttle failsafe (0=disabled, 1=enabled)'),
    ('FS_THR_VALUE', 'Throttle failsafe PWM value'),
    ('RC3_MIN', 'RC3 (throttle) minimum PWM'),
    ('RC3_MAX', 'RC3 (throttle) maximum PWM'),
    ('RC3_TRIM', 'RC3 (throttle) trim/neutral PWM'),
    ('MOT_PWM_TYPE', 'Motor PWM output type'),
    ('PILOT_THR_FILT', 'Pilot throttle filter'),
]

current_values = {}
for param_name, description in safety_params:
    value = get_param(param_name)
    current_values[param_name] = value
    if value is not None:
        print(f"{param_name:15} = {value:8.0f}  ({description})")
    else:
        print(f"{param_name:15} = {'Not found':>8}  ({description})")

print(f"\n=== ANALYSIS ===")

# Check for common issues
issues = []

arming_require = current_values.get('ARMING_REQUIRE', 0)
if arming_require > 0:
    issues.append(f"ARMING_REQUIRE={arming_require} - May require additional safety checks")

fs_thr_enable = current_values.get('FS_THR_ENABLE', 0)
if fs_thr_enable > 0:
    issues.append(f"FS_THR_ENABLE={fs_thr_enable} - Throttle failsafe enabled, may block RC overrides")

rc3_min = current_values.get('RC3_MIN', 1000)
rc3_max = current_values.get('RC3_MAX', 2000)
if rc3_min is None or rc3_max is None:
    issues.append("RC3_MIN/RC3_MAX not set - RC Channel 3 not properly configured")

if not issues:
    print("✅ No obvious safety parameter issues found")
    print("\nPossible causes:")
    print("1. Vehicle mode requirement - try MANUAL mode")
    print("2. ESC not connected or powered")  
    print("3. SERVO3 output pin hardware issue")
    print("4. Need to enable RC override functionality")
else:
    print("⚠️  Potential issues found:")
    for issue in issues:
        print(f"   • {issue}")

print(f"\n=== NEXT STEPS ===")
print("1. Try setting FS_THR_ENABLE = 0 to disable throttle failsafe")
print("2. Ensure vehicle is in MANUAL mode during RC override tests")
print("3. Check that ESC has external power (separate from USB)")
print("4. Verify SERVO3 output pin connections")

# Check current flight mode
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
if msg:
    custom_mode = msg.custom_mode
    base_mode = msg.base_mode
    print(f"\nCurrent flight mode: {custom_mode} (base_mode: {base_mode})")
    if custom_mode != 0:  # MANUAL = 0 for rovers
        print("⚠️  Not in MANUAL mode - RC overrides may not work")

print("\nDone.")
