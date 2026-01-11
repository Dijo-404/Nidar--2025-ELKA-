#!/usr/bin/env python3
"""
Dual Drone Controller for ArduPilot SITL
Controls two drones simultaneously using pymavlink and threading.

Usage:
    1. Start SITL with: sim_vehicle.py --count=2
    2. Run this script: python dual_drone_controller.py
"""

import time
import threading
from pymavlink import mavutil


class DroneController:
    """
    A helper class to control a single drone via MAVLink.
    
    Args:
        connection_string: MAVLink connection string (e.g., 'udpin:127.0.0.1:14550')
        target_system_id: The System ID of the target drone
    """
    
    def __init__(self, connection_string: str, target_system_id: int):
        self.connection_string = connection_string
        self.target_system_id = target_system_id
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
            source_component=0
        )
        
        self.log("Waiting for heartbeat...")
        self.master.wait_heartbeat(timeout=30)
        self.log(f"Heartbeat received! (System: {self.master.target_system}, Component: {self.master.target_component})")
        
        # Set target system to our specific drone
        self.master.target_system = self.target_system_id
        
    def set_mode(self, mode_name: str):
        """
        Set the flight mode.
        
        Args:
            mode_name: Mode name (e.g., 'GUIDED', 'STABILIZE', 'LAND')
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
    
    def arm(self):
        """Arm the drone motors."""
        self.log("Arming...")
        
        self.master.mav.command_long_send(
            self.target_system_id,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Arm (1) / Disarm (0)
            21196,  # Force arm (magic number to bypass pre-arm checks in SITL)
            0, 0, 0, 0, 0
        )
        
        # Wait for arm confirmation
        start_time = time.time()
        while time.time() - start_time < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == self.target_system_id:
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.log("Armed successfully!")
                    return True
        
        self.log("WARNING: Arm not confirmed")
        return False
    
    def disarm(self):
        """Disarm the drone motors."""
        self.log("Disarming...")
        
        self.master.mav.command_long_send(
            self.target_system_id,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            0,  # Disarm
            21196,  # Force disarm
            0, 0, 0, 0, 0
        )
        
        time.sleep(1)
        self.log("Disarmed")
    
    def takeoff(self, altitude: float):
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
        self._wait_for_altitude(altitude)
        self.log(f"Reached target altitude of {altitude}m")
    
    def _wait_for_altitude(self, target_altitude: float, tolerance: float = 0.5):
        """Wait until the drone reaches the target altitude."""
        self.log(f"Waiting to reach {target_altitude}m...")
        
        start_time = time.time()
        timeout = 30  # seconds
        
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
    
    def land(self):
        """Command the drone to land."""
        self.log("Landing...")
        self.set_mode('LAND')
        
        # Wait for landing
        time.sleep(10)
        self.log("Landed")
    
    def close(self):
        """Close the MAVLink connection."""
        if self.master:
            self.master.close()
            self.log("Connection closed")


def run_drone_mission(connection_string: str, system_id: int, move_x: float):
    """
    Execute the full mission sequence for a single drone.
    
    Args:
        connection_string: MAVLink connection string
        system_id: Target system ID
        move_x: X position to move to (North/South)
    """
    drone = DroneController(connection_string, system_id)
    
    try:
        # 1. Connect and wait for heartbeat
        drone.connect()
        time.sleep(1)
        
        # 2. Switch to GUIDED mode
        drone.set_mode('GUIDED')
        time.sleep(1)
        
        # 3. Arm the drone
        drone.arm()
        time.sleep(2)
        
        # 4. Takeoff to 10 meters
        drone.takeoff(10)
        time.sleep(3)
        
        # 5. Move to target position
        direction = "North" if move_x > 0 else "South"
        drone.log(f"Moving {direction} to x={move_x}...")
        drone.send_local_position(move_x, 0, -10)  # z is negative for up in NED
        time.sleep(10)  # Wait for movement
        
        # 6. Hold position briefly
        drone.log("Holding position...")
        time.sleep(5)
        
        # 7. Return to origin
        drone.log("Returning to origin...")
        drone.send_local_position(0, 0, -10)
        time.sleep(10)
        
        # 8. Land
        drone.land()
        
    except Exception as e:
        drone.log(f"ERROR: {e}")
    finally:
        drone.disarm()
        drone.close()


def main():
    """Main function to control both drones simultaneously."""
    print("=" * 60)
    print("Dual Drone Controller - ArduPilot SITL")
    print("=" * 60)
    print()
    
    # Configuration for both drones
    drones_config = [
        {
            'connection': 'udpin:127.0.0.1:14550',
            'system_id': 1,
            'move_x': 10   # Move North
        },
        {
            'connection': 'udpin:127.0.0.1:14560',
            'system_id': 2,
            'move_x': -10  # Move South
        }
    ]
    
    # Create threads for concurrent execution
    threads = []
    for config in drones_config:
        thread = threading.Thread(
            target=run_drone_mission,
            args=(config['connection'], config['system_id'], config['move_x']),
            name=f"Drone-{config['system_id']}"
        )
        threads.append(thread)
    
    print("Starting concurrent drone missions...")
    print("-" * 60)
    
    # Start all threads
    for thread in threads:
        thread.start()
    
    # Wait for all threads to complete
    for thread in threads:
        thread.join()
    
    print("-" * 60)
    print("All drone missions completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
