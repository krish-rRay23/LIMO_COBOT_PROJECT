#!/usr/bin/env python3
"""
Comprehensive MyCobot Diagnostic Tool
This script performs extensive testing to identify communication issues
"""

import time
import sys
import serial
import serial.tools.list_ports
import os
import glob
from pymycobot.mycobot import MyCobot

def check_system_info():
    """Check system information and permissions"""
    print("=== SYSTEM DIAGNOSTICS ===")
    
    # Check available ports
    print("\n1. Available Serial Ports:")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"   {port.device}: {port.description}")
    
    # Check USB devices
    print("\n2. USB Devices:")
    try:
        usb_devices = os.popen('lsusb').read()
        for line in usb_devices.split('\n'):
            if line.strip():
                print(f"   {line}")
    except:
        print("   Could not list USB devices")
    
    # Check ttyUSB devices
    print("\n3. TTY USB Devices:")
    tty_devices = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    for device in tty_devices:
        try:
            stat = os.stat(device)
            print(f"   {device}: permissions {oct(stat.st_mode)[-3:]}")
        except Exception as e:
            print(f"   {device}: error - {e}")
    
    # Check user groups
    print("\n4. User Groups:")
    try:
        groups = os.popen('groups').read().strip()
        print(f"   Current user groups: {groups}")
        if 'dialout' in groups:
            print("   ✓ User is in dialout group")
        else:
            print("   ✗ User is NOT in dialout group (this could be the issue!)")
    except:
        print("   Could not check user groups")

def test_raw_serial(port, baud):
    """Test raw serial communication"""
    print(f"\n=== RAW SERIAL TEST ({port} at {baud}) ===")
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"✓ Serial port opened successfully")
        
        # Try to send a simple command
        test_commands = [
            b'\xfe\xfe\x02\x20\x00\x00\xdc',  # Get angles command
            b'\xfe\xfe\x02\x21\x00\x00\xdd',  # Get coords command
            b'\xfe\xfe\x02\x1a\x00\x00\xd6',  # Is power on command
        ]
        
        for i, cmd in enumerate(test_commands):
            print(f"Sending command {i+1}: {cmd.hex()}")
            ser.write(cmd)
            time.sleep(0.1)
            
            # Try to read response
            response = ser.read(20)  # Read up to 20 bytes
            if response:
                print(f"Response: {response.hex()}")
            else:
                print("No response received")
        
        ser.close()
        print("✓ Raw serial test completed")
        
    except Exception as e:
        print(f"✗ Raw serial test failed: {e}")

def test_mycobot_methods(port, baud):
    """Test various MyCobot methods"""
    print(f"\n=== MYCOBOT METHODS TEST ({port} at {baud}) ===")
    
    try:
        mc = MyCobot(port, str(baud))
        time.sleep(2)
        
        # Test various methods
        methods_to_test = [
            ('get_angles', lambda: mc.get_angles()),
            ('get_coords', lambda: mc.get_coords()),
            ('is_power_on', lambda: mc.is_power_on()),
            ('get_system_version', lambda: mc.get_system_version()),
            ('is_controller_connected', lambda: mc.is_controller_connected()),
            ('is_moving', lambda: mc.is_moving()),
            ('get_speed', lambda: mc.get_speed()),
            ('get_joint_min_angle', lambda: mc.get_joint_min_angle(1)),
            ('get_joint_max_angle', lambda: mc.get_joint_max_angle(1)),
        ]
        
        for method_name, method_func in methods_to_test:
            try:
                result = method_func()
                print(f"  {method_name}: {result}")
            except Exception as e:
                print(f"  {method_name}: ERROR - {e}")
        
    except Exception as e:
        print(f"✗ MyCobot methods test failed: {e}")

def test_alternative_approaches(port):
    """Test alternative communication approaches"""
    print(f"\n=== ALTERNATIVE APPROACHES TEST ===")
    
    # Test with different MyCobot classes if available
    try:
        from pymycobot import MyCobot280
        print("Trying MyCobot280 class...")
        mc280 = MyCobot280(port, 115200)
        time.sleep(2)
        angles = mc280.get_angles()
        print(f"MyCobot280 angles: {angles}")
    except ImportError:
        print("MyCobot280 class not available")
    except Exception as e:
        print(f"MyCobot280 test failed: {e}")
    
    # Try with different initialization parameters
    try:
        print("Trying with debug=True...")
        mc_debug = MyCobot(port, '115200', debug=True)
        time.sleep(2)
        angles = mc_debug.get_angles()
        print(f"Debug mode angles: {angles}")
    except Exception as e:
        print(f"Debug mode test failed: {e}")

def check_robot_physical_state():
    """Check for physical robot state indicators"""
    print("\n=== PHYSICAL STATE CHECKLIST ===")
    print("Please verify the following:")
    print("1. ✓ Robot power LED is ON")
    print("2. ✓ USB cable is firmly connected")
    print("3. ✓ Robot is not in emergency stop mode")
    print("4. ✓ Robot display (if any) shows normal status")
    print("5. ✓ No error LEDs are blinking")
    print("6. ✓ Robot is not overheated")
    print("7. ✓ All joints can move freely (when powered off)")
    
    response = input("\nAre all the above conditions met? (y/n): ")
    if response.lower() != 'y':
        print("⚠️  Please address physical issues before continuing")
        return False
    return True

def suggest_solutions():
    """Suggest possible solutions"""
    print("\n=== SUGGESTED SOLUTIONS ===")
    print("Based on the diagnostic results, try these solutions:")
    print()
    print("1. **Add user to dialout group** (if not already):")
    print("   sudo usermod -a -G dialout $USER")
    print("   # Then log out and log back in")
    print()
    print("2. **Try different port permissions:**")
    print("   sudo chmod 666 /dev/ttyUSB0")
    print()
    print("3. **Reset the robot:**")
    print("   - Power off the robot completely")
    print("   - Unplug USB cable")
    print("   - Wait 10 seconds")
    print("   - Plug USB cable back in")
    print("   - Power on the robot")
    print()
    print("4. **Check if robot is in bootloader mode:**")
    print("   - Look for any special LED patterns")
    print("   - Check if robot responds to physical button presses")
    print()
    print("5. **Try MyCobot Studio software:**")
    print("   - Download official MyCobot Studio")
    print("   - Test connection with official software first")
    print()
    print("6. **Update robot firmware:**")
    print("   - Use MyCobot Studio to update firmware")
    print("   - This might resolve communication issues")
    print()
    print("7. **Check for hardware issues:**")
    print("   - Try a different USB cable")
    print("   - Try a different computer")
    print("   - Contact ElephantRobotics support")

def main():
    """Main diagnostic function"""
    print("MyCobot Comprehensive Diagnostic Tool")
    print("=" * 60)
    
    # Get port from command line or use default
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    # Run diagnostics
    check_system_info()
    
    # Check physical state
    if not check_robot_physical_state():
        return
    
    # Test raw serial communication
    test_raw_serial(port, 115200)
    test_raw_serial(port, 1000000)
    
    # Test MyCobot methods
    test_mycobot_methods(port, 115200)
    test_mycobot_methods(port, 1000000)
    
    # Test alternative approaches
    test_alternative_approaches(port)
    
    # Suggest solutions
    suggest_solutions()
    
    print("\n" + "=" * 60)
    print("DIAGNOSTIC COMPLETE")
    print("Review the output above to identify potential issues.")

if __name__ == '__main__':
    main()