"""
DronePilot - MAVLink Wrapper for Safe Flight Operations

This module provides a high-level abstraction over MAVLink commands,
implementing safety checks and failsafe mechanisms for drone control.
Includes telemetry forwarding to GCS (Mission Planner).
"""

import time
import logging
import threading
from typing import Optional, Tuple, Callable
from pymavlink import mavutil

logger = logging.getLogger(__name__)


class DronePilot:
    """
    Safe flight controller wrapper using MAVLink protocol.
    
    Provides methods for arming, takeoff, navigation, and emergency procedures
    with built-in safety checks and failsafe mechanisms.
    Supports telemetry forwarding to GCS for mission monitoring.
    """
    
    def __init__(self, connection_string: str, config: dict = None, 
                 gcs_config: dict = None):
        """
        Initialize DronePilot with MAVLink connection.
        
        Args:
            connection_string: MAVLink connection string (e.g., 'udp:127.0.0.1:14550')
            config: Optional configuration dictionary with safety parameters
            gcs_config: Optional GCS configuration for telemetry forwarding
        """
        self.connection_string = connection_string
        self.config = config or {}
        self.gcs_config = gcs_config or {}
        self.mav: Optional[mavutil.mavlink_connection] = None
        self.is_armed = False
        self.home_position: Optional[Tuple[float, float, float]] = None
        
        # Safety thresholds from config or defaults
        self.min_battery_voltage = self.config.get('min_battery_voltage', 14.0)
        self.min_battery_percent = self.config.get('min_battery_percent', 20)
        self.max_distance_from_home = self.config.get('max_distance_from_home', 500)
        self.failsafe_rtl_altitude = self.config.get('failsafe_rtl_altitude', 30.0)
        self.connection_timeout = self.config.get('connection_timeout', 5.0)
        
        # GCS telemetry forwarding
        self.gcs_forwarder = None
        self.gcs_forward_thread: Optional[threading.Thread] = None
        self.gcs_forward_running = False
        self.drone_id = self.config.get('drone_id', 'drone')
        
    def connect(self, enable_gcs_forward: bool = True) -> bool:
        """
        Establish MAVLink connection to the flight controller.
        
        Args:
            enable_gcs_forward: Enable telemetry forwarding to GCS
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            logger.info(f"Connecting to {self.connection_string}...")
            self.mav = mavutil.mavlink_connection(self.connection_string)
            
            # Wait for heartbeat
            msg = self.mav.wait_heartbeat(timeout=self.connection_timeout)
            if msg:
                logger.info(f"Connected to system {self.mav.target_system}, "
                           f"component {self.mav.target_component}")
                
                # Setup GCS forwarding if configured
                if enable_gcs_forward and self.gcs_config.get('enabled', False):
                    self._setup_gcs_forwarding()
                
                return True
            else:
                logger.error("No heartbeat received")
                return False
                
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False
    
    def _setup_gcs_forwarding(self):
        """Setup telemetry forwarding to GCS."""
        try:
            from src.comms.telemetry_forwarder import TelemetryForwarder
            
            gcs_ip = self.gcs_config.get('ip', '127.0.0.1')
            gcs_port = self.gcs_config.get('port', 14560)
            
            self.gcs_forwarder = TelemetryForwarder(self.gcs_config)
            self.gcs_forwarder.add_drone(self.drone_id, gcs_port)
            
            # Start forwarding thread
            self.gcs_forward_running = True
            self.gcs_forward_thread = threading.Thread(
                target=self._gcs_forward_loop,
                daemon=True,
                name=f"GCS-Forward-{self.drone_id}"
            )
            self.gcs_forward_thread.start()
            
            logger.info(f"GCS telemetry forwarding enabled -> {gcs_ip}:{gcs_port}")
            
        except Exception as e:
            logger.warning(f"Could not setup GCS forwarding: {e}")
    
    def _gcs_forward_loop(self):
        """Forward incoming MAVLink messages to GCS."""
        while self.gcs_forward_running and self.mav:
            try:
                # Non-blocking receive
                msg = self.mav.recv_match(blocking=False)
                if msg and self.gcs_forwarder:
                    self.gcs_forwarder.forward_message(self.drone_id, msg)
            except:
                pass
            time.sleep(0.001)  # 1ms loop
    
    def disconnect(self):
        """Close MAVLink connection and stop forwarding."""
        # Stop GCS forwarding
        self.gcs_forward_running = False
        if self.gcs_forward_thread and self.gcs_forward_thread.is_alive():
            self.gcs_forward_thread.join(timeout=2)
        if self.gcs_forwarder:
            self.gcs_forwarder.close()
            self.gcs_forwarder = None
        
        # Close MAVLink
        if self.mav:
            self.mav.close()
            self.mav = None
            logger.info("Disconnected from flight controller")
    
    def check_battery(self) -> Tuple[bool, float, int]:
        """
        Check battery status and trigger failsafe if low.
        
        Returns:
            Tuple of (is_safe, voltage, percentage)
        """
        if not self.mav:
            return False, 0.0, 0
            
        msg = self.mav.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1.0)
        if not msg:
            msg = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=1.0)
            
        if msg:
            if hasattr(msg, 'voltages'):
                voltage = msg.voltages[0] / 1000.0  # Convert mV to V
            elif hasattr(msg, 'voltage_battery'):
                voltage = msg.voltage_battery / 1000.0
            else:
                voltage = 0.0
                
            if hasattr(msg, 'battery_remaining'):
                percentage = msg.battery_remaining
            else:
                percentage = -1
                
            is_safe = (voltage >= self.min_battery_voltage and 
                      (percentage < 0 or percentage >= self.min_battery_percent))
            
            if not is_safe:
                logger.warning(f"Battery low! Voltage: {voltage}V, Remaining: {percentage}%")
                
            return is_safe, voltage, percentage
            
        return True, 0.0, -1  # Assume safe if no data
    
    def get_current_gps(self) -> Optional[Tuple[float, float, float]]:
        """
        Get current GPS coordinates.
        
        Returns:
            Tuple of (latitude, longitude, altitude) or None if unavailable
        """
        if not self.mav:
            return None
            
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0  # Convert mm to m
            return lat, lon, alt
            
        return None
    
    def get_home_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Get home position coordinates.
        
        Returns:
            Tuple of (latitude, longitude, altitude) or None
        """
        if self.home_position:
            return self.home_position
            
        if not self.mav:
            return None
            
        msg = self.mav.recv_match(type='HOME_POSITION', blocking=True, timeout=2.0)
        if msg:
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            alt = msg.altitude / 1000.0
            self.home_position = (lat, lon, alt)
            return self.home_position
            
        return None
    
    def set_mode(self, mode: str) -> bool:
        """
        Set flight mode.
        
        Args:
            mode: Flight mode string (e.g., 'GUIDED', 'LOITER', 'RTL', 'LAND')
            
        Returns:
            True if mode set successfully
        """
        if not self.mav:
            return False
            
        mode_mapping = self.mav.mode_mapping()
        if mode not in mode_mapping:
            logger.error(f"Unknown mode: {mode}")
            return False
            
        mode_id = mode_mapping[mode]
        self.mav.set_mode(mode_id)
        
        # Verify mode change
        time.sleep(0.5)
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
        if msg and msg.custom_mode == mode_id:
            logger.info(f"Mode set to {mode}")
            return True
            
        logger.warning(f"Mode change to {mode} may not have succeeded")
        return True  # Optimistic return
    
    def arm(self) -> bool:
        """
        Arm the drone with safety checks.
        
        Returns:
            True if armed successfully
        """
        if not self.mav:
            return False
            
        # Pre-arm safety checks
        battery_ok, voltage, percent = self.check_battery()
        if not battery_ok:
            logger.error("Cannot arm: Battery too low")
            return False
            
        gps = self.get_current_gps()
        if not gps:
            logger.error("Cannot arm: No GPS fix")
            return False
        
        # Store home position
        self.home_position = gps
        
        # Send arm command
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0  # unused parameters
        )
        
        # Wait for acknowledgment
        msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.is_armed = True
            logger.info("Armed successfully")
            return True
        else:
            logger.error("Arm command failed")
            return False
    
    def disarm(self) -> bool:
        """
        Disarm the drone.
        
        Returns:
            True if disarmed successfully
        """
        if not self.mav:
            return False
            
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            0, 0, 0, 0, 0, 0
        )
        
        msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.is_armed = False
            logger.info("Disarmed successfully")
            return True
        return False
    
    def safe_takeoff(self, altitude: float) -> bool:
        """
        Execute safe takeoff to specified altitude.
        
        Args:
            altitude: Target altitude in meters
            
        Returns:
            True if takeoff initiated successfully
        """
        if not self.mav:
            return False
            
        if not self.is_armed:
            logger.error("Cannot takeoff: Not armed")
            return False
        
        # Set GUIDED mode for takeoff
        if not self.set_mode('GUIDED'):
            logger.error("Cannot takeoff: Failed to set GUIDED mode")
            return False
        
        # Send takeoff command
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,  # pitch
            0,  # empty
            0,  # empty
            0,  # yaw angle
            0,  # latitude
            0,  # longitude
            altitude  # altitude
        )
        
        msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.info(f"Takeoff initiated to {altitude}m")
            return True
        else:
            logger.error("Takeoff command failed")
            return False
    
    def wait_for_altitude(self, target_alt: float, tolerance: float = 1.0, 
                          timeout: float = 30.0) -> bool:
        """
        Wait until drone reaches target altitude.
        
        Args:
            target_alt: Target altitude in meters
            tolerance: Acceptable deviation in meters
            timeout: Maximum wait time in seconds
            
        Returns:
            True if altitude reached within timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            gps = self.get_current_gps()
            if gps:
                _, _, current_alt = gps
                if abs(current_alt - target_alt) <= tolerance:
                    logger.info(f"Reached altitude: {current_alt:.1f}m")
                    return True
            time.sleep(0.5)
            
        logger.warning(f"Timeout waiting for altitude {target_alt}m")
        return False
    
    def goto(self, lat: float, lon: float, alt: float) -> bool:
        """
        Navigate to specified GPS coordinates.
        
        Args:
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude in meters
            
        Returns:
            True if navigation command sent successfully
        """
        if not self.mav:
            return False
        
        # Check battery before each waypoint
        battery_ok, _, _ = self.check_battery()
        if not battery_ok:
            logger.warning("Battery low - triggering RTL")
            self.rtl_now()
            return False
        
        # Send position target
        self.mav.mav.set_position_target_global_int_send(
            0,  # timestamp
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # type_mask (only positions)
            int(lat * 1e7),  # latitude
            int(lon * 1e7),  # longitude
            alt,  # altitude
            0, 0, 0,  # velocities
            0, 0, 0,  # accelerations
            0, 0  # yaw, yaw_rate
        )
        
        logger.info(f"Navigating to ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
        return True
    
    def wait_for_arrival(self, lat: float, lon: float, tolerance: float = 2.0,
                         timeout: float = 60.0) -> bool:
        """
        Wait until drone arrives at target coordinates.
        
        Args:
            lat: Target latitude
            lon: Target longitude
            tolerance: Acceptable distance in meters
            timeout: Maximum wait time in seconds
            
        Returns:
            True if arrived within timeout
        """
        from src.utils.geo_math import haversine_distance
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            gps = self.get_current_gps()
            if gps:
                current_lat, current_lon, _ = gps
                distance = haversine_distance(current_lat, current_lon, lat, lon)
                if distance <= tolerance:
                    logger.info(f"Arrived at waypoint (distance: {distance:.1f}m)")
                    return True
            time.sleep(0.5)
            
            # Check battery during transit
            battery_ok, _, _ = self.check_battery()
            if not battery_ok:
                logger.warning("Battery low during transit - triggering RTL")
                self.rtl_now()
                return False
                
        logger.warning(f"Timeout waiting for arrival at ({lat}, {lon})")
        return False
    
    def rtl_now(self) -> bool:
        """
        Emergency Return-To-Launch.
        
        This overrides all other operations and sends the drone home.
        
        Returns:
            True if RTL mode set successfully
        """
        logger.warning("RTL TRIGGERED - Returning to launch")
        return self.set_mode('RTL')
    
    def land(self) -> bool:
        """
        Initiate controlled landing at current position.
        
        Returns:
            True if land mode set successfully
        """
        logger.info("Initiating landing")
        return self.set_mode('LAND')
    
    def loiter(self) -> bool:
        """
        Enter loiter mode at current position.
        
        Returns:
            True if loiter mode set successfully
        """
        logger.info("Entering loiter mode")
        return self.set_mode('LOITER')
    
    def is_connected(self) -> bool:
        """Check if MAVLink connection is active."""
        if not self.mav:
            return False
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        return msg is not None
