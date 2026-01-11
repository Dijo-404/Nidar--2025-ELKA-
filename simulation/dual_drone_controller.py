#!/usr/bin/env python3
"""
Dual Drone Controller for ArduPilot
Controls two drones simultaneously using pymavlink and threading.

Usage:
    SITL Mode:
        sim_vehicle.py --count=2
        python dual_drone_controller.py --sitl

    Real Flight:
        python dual_drone_controller.py --drone1 /dev/ttyUSB0 --drone2 /dev/ttyUSB1
"""

import time
import threading
import argparse
from pymavlink import mavutil


class DroneController:
    """
    A helper class to control a single drone via MAVLink.
    
    Args:
        connection_string: MAVLink connection string (e.g., '/dev/ttyUSB0', 'udpin:127.0.0.1:14550')
        target_system_id: The System ID of the target drone
        sitl_mode: If True, use force-arm for SITL testing
    """
    
    # Battery voltage thresholds by cell count
    BATTERY_MIN_VOLTAGE = {
        3: 10.5,   # 3S: 3.5V per cell minimum
        4: 14.0,   # 4S: 3.5V per cell minimum
        6: 21.0,   # 6S: 3.5V per cell minimum
    }
    
    def __init__(self, connection_string: str, target_system_id: int, 
                 sitl_mode: bool = False, battery_cells: int = 4):
        self.connection_string = connection_string
        self.target_system_id = target_system_id
        self.sitl_mode = sitl_mode
        self.battery_cells = battery_cells
        self.min_voltage = self.BATTERY_MIN_VOLTAGE.get(battery_cells, 14.0)
        self.master = None
        
    def log(self, message: str):
        """Print a status message with drone ID prefix."""
        print(f"[ID:{self.target_system_id}] {message}")
    
    def connect(self):
        """Establish MAVLink connection and wait for heartbeat."""
        self.log(f"Connecting to {self.connection_string}...")
        self.master = mavutil.mavlink_connection(
            self.connection_string,
            source_system=255,  # GCS system ID
            source_component=0,
            baud=57600  # Standard telemetry baud rate
        )
        
        self.log("Waiting for heartbeat...")
        self.master.wait_heartbeat(timeout=30)
        self.log(f"Heartbeat received! (System: {self.master.target_system}, Component: {self.master.target_component})")
        
        # Set target system to our specific drone
        self.master.target_system = self.target_system_id
    
    # =========================================================================
    # SAFETY CHECK FUNCTIONS
    # =========================================================================
    
    def wait_for_gps_lock(self, min_satellites: int = 6, timeout: float = 60) -> bool:
        """
        Wait for GPS 3D fix before arming.
        
        Args:
            min_satellites: Minimum number of satellites required
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if GPS lock acquired, False if timeout
        """
        self.log(f"Waiting for GPS lock ({min_satellites}+ satellites)...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == self.target_system_id:
                fix_type = msg.fix_type
                satellites = msg.satellites_visible
                
                # Fix type 3 = 3D fix
                if fix_type >= 3 and satellites >= min_satellites:
                    self.log(f"GPS lock acquired: {satellites} satellites, fix type {fix_type}")
                    return True
                else:
                    self.log(f"GPS: {satellites} sats, fix type {fix_type} (waiting for 3D fix)")
        
        self.log("ERROR: GPS lock timeout - UNSAFE TO FLY")
        return False
    
    def check_battery(self, min_percent: int = 20) -> bool:
        """
        Check battery level before flight.
        Uses the min_voltage set based on battery_cells in __init__.
        
        Args:
            min_percent: Minimum battery percentage
            
        Returns:
            True if battery OK, False if low
        """
        self.log(f"Checking battery ({self.battery_cells}S, min {self.min_voltage}V)...")
        msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        
        if msg and msg.get_srcSystem() == self.target_system_id:
            voltage = msg.voltage_battery / 1000.0  # mV to V
            percent = msg.battery_remaining
            
            self.log(f"Battery: {voltage:.2f}V, {percent}%")
            
            if voltage < self.min_voltage:
                self.log(f"ERROR: Battery voltage too low ({voltage:.2f}V < {self.min_voltage}V)")
                return False
            if percent >= 0 and percent < min_percent:  # -1 means unknown
                self.log(f"ERROR: Battery percentage too low ({percent}% < {min_percent}%)")
                return False
            
            self.log("Battery OK")
            return True
        
        self.log("WARNING: Could not read battery status")
        return True  # Proceed with caution if can't read
    
    def wait_for_ekf_ready(self, timeout: float = 30) -> bool:
        """
        Wait for EKF to be healthy before arming.
        
        Returns:
            True if EKF is ready, False if timeout
        """
        self.log("Waiting for EKF ready...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == self.target_system_id:
                # Check velocity and position variance flags
                flags = msg.flags
                velocity_ok = flags & 0x01  # Velocity horizontal OK
                pos_horiz_ok = flags & 0x02  # Position horizontal OK
                pos_vert_ok = flags & 0x04  # Position vertical OK
                
                if velocity_ok and pos_horiz_ok and pos_vert_ok:
                    self.log("EKF is healthy")
                    return True
        
        self.log("WARNING: EKF readiness timeout")
        return False
    
    def pre_arm_checks(self) -> bool:
        """
        Run all pre-arm safety checks.
        
        Returns:
            True if all checks pass, False otherwise
        """
        self.log("=" * 40)
        self.log("RUNNING PRE-ARM SAFETY CHECKS")
        self.log("=" * 40)
        
        if self.sitl_mode:
            self.log("SITL MODE: Skipping GPS and EKF checks")
            return True
        
        # 1. Check battery
        if not self.check_battery():
            return False
        
        # 2. Wait for GPS lock
        if not self.wait_for_gps_lock():
            return False
        
        # 3. Wait for EKF ready
        if not self.wait_for_ekf_ready():
            return False
        
        self.log("=" * 40)
        self.log("ALL PRE-ARM CHECKS PASSED")
        self.log("=" * 40)
        return True
    
    # =========================================================================
    # FLIGHT CONTROL FUNCTIONS
    # =========================================================================
        
    def set_mode(self, mode_name: str) -> bool:
        """
        Set the flight mode.
        
        Args:
            mode_name: Mode name (e.g., 'GUIDED', 'STABILIZE', 'LAND', 'RTL')
        """
        self.log(f"Setting mode to {mode_name}...")
        
        # Get mode ID from mode mapping
        mode_mapping = self.master.mode_mapping()
        if mode_name not in mode_mapping:
            self.log(f"ERROR: Unknown mode {mode_name}")
            return False
            
        mode_id = mode_mapping[mode_name]
        
        # Send mode change command
        self.master.mav.set_mode_send(
            self.target_system_id,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Wait for mode change confirmation
        start_time = time.time()
        while time.time() - start_time < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == self.target_system_id:
                if msg.custom_mode == mode_id:
                    self.log(f"Mode changed to {mode_name}")
                    return True
        
        self.log(f"WARNING: Mode change to {mode_name} not confirmed")
        return False
    
    def arm(self) -> bool:
        """Arm the drone motors with safety checks."""
        self.log("Arming...")
        
        # Use force arm only in SITL mode
        force_arm = 21196 if self.sitl_mode else 0
        
        self.master.mav.command_long_send(
            self.target_system_id,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Arm (1) / Disarm (0)
            force_arm,  # Force arm (SITL only) or normal arm (real flight)
            0, 0, 0, 0, 0
        )
        
        # Wait for COMMAND_ACK first
        start_time = time.time()
        ack_received = False
        while time.time() - start_time < 5:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg:
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.log("Arm command accepted")
                        ack_received = True
                        break
                    else:
                        self.log(f"Arm command rejected: result={msg.result}")
                        return False
        
        if not ack_received:
            self.log("WARNING: No ACK received for arm command")
        
        # Verify armed status via heartbeat
        start_time = time.time()
        while time.time() - start_time < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == self.target_system_id:
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.log("Armed successfully!")
                    return True
        
        self.log("ERROR: Arm failed - check pre-arm messages in GCS")
        return False
    
    def disarm(self):
        """Disarm the drone motors."""
        self.log("Disarming...")
        
        force_disarm = 21196 if self.sitl_mode else 0
        
        self.master.mav.command_long_send(
            self.target_system_id,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            0,  # Disarm
            force_disarm,
            0, 0, 0, 0, 0
        )
        
        time.sleep(1)
        self.log("Disarmed")
    
    def takeoff(self, altitude: float) -> bool:
        """
        Command the drone to take off to specified altitude.
        
        Args:
            altitude: Target altitude in meters
        """
        self.log(f"Taking off to {altitude}m...")
        
        self.master.mav.command_long_send(
            self.target_system_id,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            0,  # Pitch (ignored)
            0,  # Empty
            0,  # Empty
            0,  # Yaw (NaN for unchanged)
            0,  # Latitude (ignored for GUIDED)
            0,  # Longitude (ignored for GUIDED)
            altitude  # Altitude
        )
        
        # Wait until altitude is reached
        if self._wait_for_altitude(altitude):
            self.log(f"Reached target altitude of {altitude}m")
            return True
        return False
    
    def _wait_for_altitude(self, target_altitude: float, tolerance: float = 1.0) -> bool:
        """Wait until the drone reaches the target altitude."""
        self.log(f"Waiting to reach {target_altitude}m...")
        
        start_time = time.time()
        timeout = 45  # Longer timeout for real flight
        
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == self.target_system_id:
                current_alt = msg.relative_alt / 1000.0  # Convert mm to meters
                if abs(current_alt - target_altitude) < tolerance:
                    return True
                    
        self.log("WARNING: Altitude wait timeout")
        return False
    
    def send_local_position(self, x: float, y: float, z: float):
        """
        Send a position target in the local NED frame.
        
        Args:
            x: Position North (meters)
            y: Position East (meters)
            z: Position Down (meters, negative is up)
        """
        self.log(f"Moving to local position: x={x}, y={y}, z={z}")
        
        # Type mask: use position only (ignore velocity and acceleration)
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.target_system_id,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x, y, z,  # Position
            0, 0, 0,  # Velocity (ignored)
            0, 0, 0,  # Acceleration (ignored)
            0, 0      # Yaw, yaw_rate (ignored)
        )
    
    def send_ned_velocity(self, vx: float, vy: float, vz: float, duration: float = 1.0):
        """
        Send velocity commands in NED frame.
        
        Args:
            vx: Velocity North (m/s)
            vy: Velocity East (m/s)
            vz: Velocity Down (m/s, positive is down)
            duration: How long to send the command (seconds)
        """
        self.log(f"Sending velocity: vx={vx}, vy={vy}, vz={vz} for {duration}s")
        
        # Type mask: use velocity only
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.master.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.target_system_id,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0, 0, 0,      # Position (ignored)
                vx, vy, vz,   # Velocity
                0, 0, 0,      # Acceleration (ignored)
                0, 0          # Yaw, yaw_rate (ignored)
            )
            time.sleep(0.1)  # Send at 10Hz
    
    def emergency_rtl(self):
        """Trigger emergency Return-to-Launch."""
        self.log("!!! EMERGENCY RTL TRIGGERED !!!")
        self.set_mode('RTL')
    
    def land(self):
        """Command the drone to land."""
        self.log("Landing...")
        self.set_mode('LAND')
        
        # Wait for landing
        start_time = time.time()
        while time.time() - start_time < 30:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == self.target_system_id:
                current_alt = msg.relative_alt / 1000.0
                if current_alt < 0.5:
                    self.log("Landed")
                    return
        self.log("Landing complete")
    
    def close(self):
        """Close the MAVLink connection."""
        if self.master:
            self.master.close()
            self.log("Connection closed")


def run_drone_mission(connection_string: str, system_id: int, move_x: float, 
                      sitl_mode: bool = False, battery_cells: int = 4):
    """
    Execute the full mission sequence for a single drone.
    
    Args:
        connection_string: MAVLink connection string
        system_id: Target system ID
        move_x: X position to move to (North/South)
        sitl_mode: Use SITL mode (force arm enabled)
        battery_cells: Number of battery cells (3S, 4S, 6S)
    """
    drone = DroneController(connection_string, system_id, sitl_mode, battery_cells)
    
    try:
        # 1. Connect and wait for heartbeat
        drone.connect()
        time.sleep(1)
        
        # 2. Run pre-arm safety checks
        if not drone.pre_arm_checks():
            drone.log("PRE-ARM CHECKS FAILED - ABORTING MISSION")
            return
        
        # 3. Switch to GUIDED mode
        if not drone.set_mode('GUIDED'):
            drone.log("Failed to set GUIDED mode - ABORTING")
            return
        time.sleep(1)
        
        # 4. Arm the drone
        if not drone.arm():
            drone.log("Failed to arm - ABORTING")
            return
        time.sleep(2)
        
        # 5. Takeoff to 10 meters
        if not drone.takeoff(10):
            drone.log("Takeoff failed - attempting RTL")
            drone.emergency_rtl()
            return
        time.sleep(3)
        
        # 6. Move to target position
        direction = "North" if move_x > 0 else "South"
        drone.log(f"Moving {direction} to x={move_x}...")
        drone.send_local_position(move_x, 0, -10)  # z is negative for up in NED
        time.sleep(10)  # Wait for movement
        
        # 7. Hold position briefly
        drone.log("Holding position...")
        time.sleep(5)
        
        # 8. Return to origin
        drone.log("Returning to origin...")
        drone.send_local_position(0, 0, -10)
        time.sleep(10)
        
        # 9. Land
        drone.land()
        
    except KeyboardInterrupt:
        drone.log("KEYBOARD INTERRUPT - Emergency RTL")
        drone.emergency_rtl()
    except Exception as e:
        drone.log(f"ERROR: {e}")
        drone.emergency_rtl()
    finally:
        drone.disarm()
        drone.close()


def main():
    """Main function to control both drones simultaneously."""
    parser = argparse.ArgumentParser(
        description='Dual Drone Controller for ArduPilot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  SITL Mode:
    sim_vehicle.py --count=2
    python dual_drone_controller.py --sitl
    
  Real Flight (USB Telemetry):
    python dual_drone_controller.py --drone1 /dev/ttyUSB0 --drone2 /dev/ttyUSB1
    
  Real Flight (UDP):
    python dual_drone_controller.py --drone1 udp:192.168.1.10:14550 --drone2 udp:192.168.1.11:14550
        """
    )
    
    parser.add_argument('--drone1', default='udpin:127.0.0.1:14550',
                        help='Drone 1 connection string (default: SITL)')
    parser.add_argument('--drone2', default='udpin:127.0.0.1:14560',
                        help='Drone 2 connection string (default: SITL)')
    parser.add_argument('--sitl', action='store_true',
                        help='SITL mode: enables force-arm, skips GPS/EKF checks')
    parser.add_argument('--altitude', type=float, default=10.0,
                        help='Takeoff altitude in meters (default: 10)')
    parser.add_argument('--distance', type=float, default=10.0,
                        help='Distance to fly North/South in meters (default: 10)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    if args.sitl:
        print("Dual Drone Controller - SITL MODE")
        print("WARNING: Force arm enabled, safety checks bypassed")
    else:
        print("Dual Drone Controller - REAL FLIGHT MODE")
        print("Full pre-arm safety checks enabled")
    print("=" * 60)
    print()
    print(f"Drone 1: {args.drone1}")
    print(f"Drone 2: {args.drone2}")
    print(f"Altitude: {args.altitude}m")
    print(f"Distance: {args.distance}m")
    print()
    
    # Configuration for both drones
    # Drone 1: 4S battery (min 14.0V)
    # Drone 2: 6S battery (min 21.0V)
    drones_config = [
        {
            'connection': args.drone1,
            'system_id': 1,
            'move_x': args.distance,   # Move North
            'battery_cells': 4         # 4S battery
        },
        {
            'connection': args.drone2,
            'system_id': 2,
            'move_x': -args.distance,  # Move South
            'battery_cells': 6         # 6S battery
        }
    ]
    
    # Create threads for concurrent execution
    threads = []
    for config in drones_config:
        thread = threading.Thread(
            target=run_drone_mission,
            args=(config['connection'], config['system_id'], config['move_x'], 
                  args.sitl, config['battery_cells']),
            name=f"Drone-{config['system_id']}"
        )
        threads.append(thread)
    
    print("Starting concurrent drone missions...")
    print("-" * 60)
    
    try:
        # Start all threads
        for thread in threads:
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received - waiting for drones to RTL...")
        for thread in threads:
            thread.join(timeout=30)
    
    print("-" * 60)
    print("All drone missions completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
