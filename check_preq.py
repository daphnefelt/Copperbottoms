#!/usr/bin/env python3
"""
Prerequisite Checker for Rover Diagnostics
Checks all required components for rover testing
"""

import os
import sys
import subprocess
import importlib.util

def check_python_module(module_name):
    """Check if a Python module is installed"""
    try:
        spec = importlib.util.find_spec(module_name)
        return spec is not None
    except ImportError:
        return False

def run_command(cmd, capture_output=True):
    """Run a shell command and return result"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=capture_output, 
                               text=True, timeout=10)
        return result.returncode == 0, result.stdout, result.stderr
    except Exception as e:
        return False, "", str(e)

def check_ros2():
    """Check if ROS2 Humble is installed and sourced"""
    # Check if ROS2 command exists
    success, stdout, stderr = run_command("which ros2")
    if not success:
        return False, "ros2 command not found"
    
    # Check ROS2 version
    success, stdout, stderr = run_command("ros2 --version")
    if success and "humble" in stdout.lower():
        return True, f"ROS2 Humble found: {stdout.strip()}"
    elif success:
        return False, f"Wrong ROS2 version: {stdout.strip()}"
    else:
        return False, "Could not determine ROS2 version"

def check_pixhawk_connection():
    """Check for Pixhawk USB connections"""
    devices = []
    
    # Check for ACM devices
    for device in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']:
        if os.path.exists(device):
            devices.append(device)
    
    if not devices:
        return False, "No USB serial devices found"
    
    # Try to get device info
    device_info = []
    for device in devices:
        success, stdout, stderr = run_command(f"udevadm info --name={device} --query=property | grep -E 'ID_MODEL|ID_VENDOR'")
        if success:
            device_info.append(f"{device}: {stdout.strip()}")
        else:
            device_info.append(f"{device}: Available")
    
    return True, f"Found devices: {', '.join(devices)}\n" + "\n".join(device_info)

def test_pymavlink_connection():
    """Test actual connection to Pixhawk using pymavlink"""
    try:
        from pymavlink import mavutil
        
        # Try common device paths
        devices = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
        
        for device in devices:
            if os.path.exists(device):
                try:
                    print(f"  Testing connection to {device}...")
                    m = mavutil.mavlink_connection(device, baud=921600)
                    
                    # Wait for heartbeat with timeout
                    print("  Waiting for heartbeat...")
                    m.wait_heartbeat(timeout=5)
                    
                    return True, f"Successfully connected to Pixhawk on {device} (System ID: {m.target_system})"
                    
                except Exception as e:
                    continue
        
        return False, f"Could not connect to Pixhawk on any device: {devices}"
        
    except Exception as e:
        return False, f"PyMAVLink connection test failed: {e}"

def main():
    """Main prerequisite checking function"""
    print("=== ROVER DIAGNOSTICS PREREQUISITE CHECKER ===\n")
    
    checks = []
    
    # 1. Check Python
    print("1. Checking Python...")
    python_version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    print(f"   ✓ Python {python_version}")
    checks.append(("Python", True, f"Version {python_version}"))
    
    # 2. Check pymavlink
    print("\n2. Checking pymavlink...")
    has_pymavlink = check_python_module("pymavlink")
    if has_pymavlink:
        try:
            import pymavlink
            version = getattr(pymavlink, '__version__', 'unknown')
            print(f"   ✓ pymavlink installed (version: {version})")
            checks.append(("pymavlink", True, f"Version {version}"))
        except:
            print("   ✓ pymavlink installed")
            checks.append(("pymavlink", True, "Installed"))
    else:
        print("   ✗ pymavlink NOT installed")
        print("     Install with: pip install pymavlink")
        checks.append(("pymavlink", False, "Not installed"))
    
    # 3. Check ROS2
    print("\n3. Checking ROS2 Humble...")
    ros2_ok, ros2_msg = check_ros2()
    if ros2_ok:
        print(f"   ✓ {ros2_msg}")
        checks.append(("ROS2 Humble", True, ros2_msg))
    else:
        print(f"   ✗ {ros2_msg}")
        print("     Source ROS2 with: source /opt/ros/humble/setup.bash")
        checks.append(("ROS2 Humble", False, ros2_msg))
    
    # 4. Check USB devices
    print("\n4. Checking USB connections...")
    usb_ok, usb_msg = check_pixhawk_connection()
    if usb_ok:
        print(f"   ✓ {usb_msg}")
        checks.append(("USB Devices", True, usb_msg.split('\n')[0]))
    else:
        print(f"   ✗ {usb_msg}")
        print("     Check USB connection to Pixhawk")
        checks.append(("USB Devices", False, usb_msg))
    
    # 5. Test Pixhawk connection (if pymavlink available)
    print("\n5. Testing Pixhawk connection...")
    if has_pymavlink and usb_ok:
        pixhawk_ok, pixhawk_msg = test_pymavlink_connection()
        if pixhawk_ok:
            print(f"   ✓ {pixhawk_msg}")
            checks.append(("Pixhawk Connection", True, pixhawk_msg))
        else:
            print(f"   ✗ {pixhawk_msg}")
            checks.append(("Pixhawk Connection", False, pixhawk_msg))
    else:
        print("   ⚠ Skipped (pymavlink or USB not available)")
        checks.append(("Pixhawk Connection", False, "Skipped - prerequisites not met"))
    
    # Summary
    print("\n" + "="*50)
    print("PREREQUISITE SUMMARY:")
    print("="*50)
    
    all_good = True
    for name, status, details in checks:
        status_icon = "✓" if status else "✗"
        print(f"{status_icon} {name:<20} - {details}")
        if not status:
            all_good = False
    
    print("\n" + "="*50)
    if all_good:
        print("🎉 ALL PREREQUISITES MET! Ready for rover diagnostics.")
    else:
        print("⚠️  Some prerequisites missing. Fix above issues before proceeding.")
    
    print("\nNext steps:")
    print("1. Fix any missing prerequisites")
    print("2. Run: python3 ~/check_steering_config.py")
    print("3. Run: python3 ~/test_servo_manual.py")

if __name__ == "__main__":
    main()

