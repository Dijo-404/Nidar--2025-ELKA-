#!/usr/bin/env python3
"""
Motor Spin Test Script

Spins all motors simultaneously at a specified throttle percentage.
Uses STABILIZE mode with force-arm to bypass GPS requirements.

Usage:
    python test_motor_spin.py                    # Default: 10% for 5 seconds
    python test_motor_spin.py --throttle 15 --duration 10
    python test_motor_spin.py --device /dev/ttyACM0 --baud 115200
"""

import argparse
import time
import sys
from pymavlink import mavutil


def spin_motors(device: str, baud: int, throttle: int, duration: int):
    """
    Connect to drone and spin all motors simultaneously.
    
    Args:
        device: Serial device path
        baud: Baud rate
        throttle: Throttle percentage (1-100)
        duration: Duration in seconds
    """
    print("=" * 60)
    print(f"MOTOR SPIN TEST - {throttle}% THROTTLE FOR {duration} SECONDS")
    print("=" * 60)
    print()
    
    # Connect
    print(f"[1/5] Connecting to {device} at {baud} baud...")
    try:
        conn = mavutil.mavlink_connection(device, baud=baud, source_system=255)
        msg = conn.wait_heartbeat(timeout=10)
        if not msg:
            print("✗ No heartbeat received - connection failed")
            return False
        print(f"✓ Connected to System {conn.target_system}")
    except Exception as e:
        print(f"✗ Connection error: {e}")
        return False
    print()
    
    # Set STABILIZE mode (no GPS required)
    print("[2/5] Setting STABILIZE mode...")
    mode_mapping = conn.mode_mapping()
    if 'STABILIZE' not in mode_mapping:
        print("✗ STABILIZE mode not available")
        conn.close()
        return False
    
    mode_id = mode_mapping['STABILIZE']
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    
    # Verify mode change
    start = time.time()
    mode_set = False
    while time.time() - start < 5:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            mode_set = True
            break
    
    if mode_set:
        print("✓ STABILIZE mode set")
    else:
        print("⚠ Mode change not confirmed, continuing anyway...")
    print()
    
    # Arm with force-arm
    print("[3/5] Arming (force-arm enabled)...")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,      # Arm
        21196,  # Force arm magic number
        0, 0, 0, 0, 0
    )
    
    # Wait for arm confirmation
    start = time.time()
    armed = False
    while time.time() - start < 15:
        msg = conn.recv_match(type=['HEARTBEAT', 'COMMAND_ACK'], blocking=True, timeout=1)
        if msg:
            if msg.get_type() == 'HEARTBEAT':
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    armed = True
                    print("✓ ARMED")
                    break
            elif msg.get_type() == 'COMMAND_ACK':
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"✗ Arm command rejected (result={msg.result})")
                        break
    
    if not armed:
        print("✗ Arm failed - retrying with extended timeout...")
        # One more attempt
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(3)
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            print("✓ ARMED (on retry)")
        else:
            print("✗ Arm failed")
            conn.close()
            return False
    print()
    
    # Spin all motors simultaneously using RC throttle override
    print(f"[4/5] SPINNING ALL MOTORS AT {throttle}%...")
    print(f"      Duration: {duration} seconds")
    print("      Press Ctrl+C to stop early")
    print()
    
    # Convert throttle percentage to PWM (1000-2000 range)
    # 0% = 1000, 100% = 2000
    throttle_pwm = 1000 + int(throttle * 10)  # e.g., 10% = 1100 PWM
    
    print(f"      Throttle PWM: {throttle_pwm}")
    print()
    
    try:
        # Use RC_CHANNELS_OVERRIDE to set throttle (channel 3)
        # This controls all motors simultaneously through the flight controller's mixer
        # Send override commands continuously during the test
        
        start_time = time.time()
        while time.time() - start_time < duration:
            # RC channels: 1=roll, 2=pitch, 3=throttle, 4=yaw
            # 0 or 65535 = no override for that channel
            conn.mav.rc_channels_override_send(
                conn.target_system,
                conn.target_component,
                65535,        # Chan 1 (roll) - no override
                65535,        # Chan 2 (pitch) - no override  
                throttle_pwm, # Chan 3 (throttle) - our throttle value
                65535,        # Chan 4 (yaw) - no override
                65535,        # Chan 5
                65535,        # Chan 6
                65535,        # Chan 7
                65535         # Chan 8
            )
            
            remaining = int(duration - (time.time() - start_time))
            print(f"      ⏱  {remaining} seconds remaining... (PWM: {throttle_pwm})", end='\r')
            time.sleep(0.1)  # Send at 10Hz
        
        # Release RC override
        conn.mav.rc_channels_override_send(
            conn.target_system,
            conn.target_component,
            0, 0, 0, 0, 0, 0, 0, 0  # Release all overrides
        )
        
        print()
        print()
        print("✓ Motor test complete!")
        
    except KeyboardInterrupt:
        # Release RC override on interrupt
        conn.mav.rc_channels_override_send(
            conn.target_system,
            conn.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print("\n\n⚠ Interrupted by user!")
    
    print()
    
    # Disarm
    print("[5/5] Disarming...")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,      # Disarm
        21196,  # Force disarm
        0, 0, 0, 0, 0
    )
    time.sleep(1)
    print("✓ Disarmed")
    
    conn.close()
    print()
    print("=" * 60)
    print("TEST COMPLETE!")
    print("=" * 60)
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Spin all drone motors simultaneously',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python test_motor_spin.py                           # 10% for 5 seconds
    python test_motor_spin.py --throttle 15             # 15% for 5 seconds
    python test_motor_spin.py --throttle 20 --duration 10
    python test_motor_spin.py --device /dev/ttyUSB0     # Different device
"""
    )
    
    parser.add_argument('--device', '-d', default='/dev/ttyACM0',
                        help='Serial device (default: /dev/ttyACM0)')
    parser.add_argument('--baud', '-b', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--throttle', '-t', type=int, default=10,
                        help='Throttle percentage 1-100 (default: 10)')
    parser.add_argument('--duration', '-s', type=int, default=5,
                        help='Duration in seconds (default: 5)')
    
    args = parser.parse_args()
    
    # Validate throttle
    if args.throttle < 1 or args.throttle > 100:
        print("Error: Throttle must be between 1 and 100")
        sys.exit(1)
    
    # Safety warning for high throttle
    if args.throttle > 30:
        print("⚠️  WARNING: High throttle value!")
        print("    Make sure props are REMOVED or drone is SECURED!")
        response = input("    Continue? (y/N): ")
        if response.lower() != 'y':
            print("Aborted.")
            sys.exit(0)
    
    success = spin_motors(args.device, args.baud, args.throttle, args.duration)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
