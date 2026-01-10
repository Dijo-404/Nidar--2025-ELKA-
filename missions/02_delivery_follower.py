#!/usr/bin/env python3
"""
Delivery Follower Mission - Drone 2 Main Executable

This script executes the delivery mission for Drone 2 (The Deliverer):
1. Takeoff and enter loiter mode
2. Wait for coordinates from relay (Drone 1 detections)
3. Process target queue with FIFO ordering
4. Navigate to targets and drop payloads
5. RTL when payload exhausted or mission complete

Usage:
    python 02_delivery_follower.py [--config CONFIG_PATH] [--test]
"""

import os
import sys
import time
import yaml
import logging
import argparse
import threading
from queue import Queue, Empty
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.base.drone_pilot import DronePilot
from src.comms.bridge_client import BridgeClient, CoordinateMessage
from src.utils.state_machine import StateMachine, MissionState
from src.utils.geo_math import haversine_distance

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('DeliveryFollower')


@dataclass
class TargetLocation:
    """Target location for payload delivery."""
    lat: float
    lon: float
    alt: float
    timestamp: float
    source: str
    priority: int = 0


class DeliveryFollowerMission:
    """
    Delivery mission controller for Drone 2.
    
    Manages target queue, payload drops, and coordination with
    the survey drone through the ground relay.
    """
    
    def __init__(self, config_path: str):
        """
        Initialize delivery mission.
        
        Args:
            config_path: Path to mission_params.yaml
        """
        # Load configuration
        self.config = self._load_config(config_path)
        self.network_config = self._load_config(
            os.path.join(os.path.dirname(config_path), 'network_map.yaml')
        )
        
        # Initialize state machine
        self.state_machine = StateMachine(MissionState.IDLE)
        
        # Initialize components (will be created in setup)
        self.drone: DronePilot = None
        self.payload: PayloadServo = None
        self.comms: BridgeClient = None
        
        # Target queue (FIFO for race condition handling)
        self.target_queue: Queue[TargetLocation] = Queue(maxsize=50)
        self.targets_processed = 0
        self.current_target: Optional[TargetLocation] = None
        
        # Configuration
        delivery_config = self.config.get('flight', {}).get('delivery', {})
        self.hold_altitude = delivery_config.get('hold_altitude', 25.0)
        self.cruise_altitude = delivery_config.get('cruise_altitude', 15.0)
        self.approach_speed = delivery_config.get('approach_speed', 3.0)
        
        payload_config = self.config.get('payload', {})
        self.drop_altitude = payload_config.get('drop_altitude', 10.0)
        
        # Flags
        self.abort_requested = False
        self.mission_active = False
        
        # Home position for loiter
        self.home_position: Optional[tuple] = None
    
    def _load_config(self, path: str) -> dict:
        """Load YAML configuration file."""
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    
    def setup(self) -> bool:
        """
        Set up all mission components.
        
        Returns:
            True if setup successful
        """
        logger.info("Setting up Delivery Follower mission...")
        self.state_machine.transition_to(MissionState.PREFLIGHT, "Starting setup")
        
        try:
            # 1. Initialize drone pilot
            mavlink_conn = self.network_config.get('drone2', {}).get(
                'mavlink_connection', 'udp:127.0.0.1:14551'
            )
            safety_config = self.config.get('safety', {})
            
            self.drone = DronePilot(mavlink_conn, safety_config)
            
            if not self.drone.connect():
                logger.error("Failed to connect to drone")
                return False
            
            # 2. Initialize payload mechanism (ESP32 or MAVLink)
            payload_config = self.config.get('payload', {})
            
            if payload_config.get('use_esp32', False):
                from src.base.payload_esp32 import PayloadESP32
                self.payload = PayloadESP32(payload_config)
                logger.info(f"ESP32 Payload initialized: {self.payload.remaining()} units")
            else:
                from src.base.payload_servo import PayloadServo
                self.payload = PayloadServo(self.drone.mav, payload_config)
                logger.info(f"MAVLink Payload initialized: {self.payload.remaining()} units")
            
            # 3. Initialize communications
            relay_config = self.network_config.get('ground_relay', {})
            drone_config = self.network_config.get('drone2', {})
            
            self.comms = BridgeClient(
                identity=drone_config.get('zmq_identity', 'DELIVERER'),
                config=relay_config
            )
            
            # Set callback for receiving coordinates
            self.comms.on_coordinate_received = self._on_coordinates_received
            
            if not self.comms.connect():
                logger.error("Failed to connect to relay")
                return False
            
            logger.info("Setup complete")
            return True
            
        except Exception as e:
            logger.error(f"Setup failed: {e}")
            return False
    
    def _on_coordinates_received(self, coord_msg: CoordinateMessage):
        """
        Callback when coordinates received from relay.
        
        Args:
            coord_msg: Coordinate message from surveyor drone
        """
        # Check if we have payload remaining
        if self.payload.is_empty():
            logger.warning("Received coordinates but payload exhausted - ignoring")
            return
        
        # Create target and add to queue
        target = TargetLocation(
            lat=coord_msg.lat,
            lon=coord_msg.lon,
            alt=coord_msg.alt,
            timestamp=coord_msg.timestamp,
            source=coord_msg.source_id
        )
        
        try:
            self.target_queue.put_nowait(target)
            logger.info(f"Target queued from {coord_msg.source_id}: "
                       f"({target.lat:.6f}, {target.lon:.6f}) "
                       f"[Queue size: {self.target_queue.qsize()}]")
        except:
            logger.warning("Target queue full - discarding oldest")
            try:
                self.target_queue.get_nowait()
                self.target_queue.put_nowait(target)
            except:
                pass
    
    def execute(self) -> bool:
        """
        Execute the delivery mission.
        
        Returns:
            True if mission completed successfully
        """
        logger.info("Starting delivery mission execution")
        self.mission_active = True
        
        try:
            # Arm and takeoff to holding altitude
            if not self._arm_and_takeoff():
                return False
            
            # Store home position
            self.home_position = self.drone.get_current_gps()
            
            # Enter loiter mode
            self.state_machine.transition_to(MissionState.LOITER, "Waiting for targets")
            self.drone.loiter()
            
            logger.info("Entered loiter mode - waiting for targets from surveyor")
            
            # Main mission loop
            while self.mission_active and not self.abort_requested:
                # Check payload status
                if self.payload.is_empty():
                    logger.warning("PAYLOAD EXHAUSTED - Initiating RTL")
                    break
                
                # Check battery
                battery_ok, _, battery_pct = self.drone.check_battery()
                if not battery_ok:
                    logger.warning("BATTERY LOW - Initiating RTL")
                    break
                
                # Update comms status
                if self.comms:
                    self.comms.set_status(
                        self.state_machine.get_state_name().upper(),
                        battery_percent=battery_pct,
                        payload_remaining=self.payload.remaining()
                    )
                
                # Try to get next target
                try:
                    target = self.target_queue.get(timeout=1.0)
                    
                    # Process target
                    if not self._process_target(target):
                        logger.warning("Target processing failed")
                    
                except Empty:
                    # No targets, continue loitering
                    pass
            
            # RTL
            return self._return_to_launch()
            
        except Exception as e:
            logger.error(f"Mission execution error: {e}")
            self._emergency_rtl()
            return False
    
    def _arm_and_takeoff(self) -> bool:
        """Arm the drone and take off to holding altitude."""
        logger.info("Arming and taking off...")
        
        if not self.drone.arm():
            logger.error("Arming failed")
            return False
        
        self.state_machine.transition_to(MissionState.ARMED, "Armed")
        
        if not self.drone.safe_takeoff(self.hold_altitude):
            logger.error("Takeoff failed")
            return False
        
        self.state_machine.transition_to(MissionState.TAKEOFF, "Taking off")
        
        if not self.drone.wait_for_altitude(self.hold_altitude, tolerance=2.0, timeout=30.0):
            logger.error("Failed to reach holding altitude")
            return False
        
        logger.info(f"Reached holding altitude: {self.hold_altitude}m")
        return True
    
    def _process_target(self, target: TargetLocation) -> bool:
        """
        Process a single target - navigate and drop payload.
        
        Args:
            target: Target location for delivery
            
        Returns:
            True if successful
        """
        self.current_target = target
        self.targets_processed += 1
        
        logger.info(f"Processing target #{self.targets_processed}: "
                   f"({target.lat:.6f}, {target.lon:.6f})")
        
        # 1. Transit to target area
        self.state_machine.transition_to(MissionState.TRANSIT, "En route to target")
        
        if not self.drone.goto(target.lat, target.lon, self.cruise_altitude):
            logger.error("Navigation to target failed")
            return False
        
        if not self.drone.wait_for_arrival(target.lat, target.lon, tolerance=5.0, timeout=120.0):
            logger.warning("Transit timeout - aborting this target")
            self.state_machine.transition_to(MissionState.LOITER, "Transit failed")
            return False
        
        # 2. Approach to drop altitude
        self.state_machine.transition_to(MissionState.APPROACH, "Approaching drop point")
        
        if not self.drone.goto(target.lat, target.lon, self.drop_altitude):
            logger.error("Approach failed")
            return False
        
        if not self.drone.wait_for_altitude(self.drop_altitude, tolerance=1.0, timeout=20.0):
            logger.warning("Failed to reach drop altitude")
        
        # Wait to stabilize
        time.sleep(2.0)
        
        # 3. Execute drop
        self.state_machine.transition_to(MissionState.DROP, "Dropping payload")
        
        if not self.payload.drop():
            logger.error("Payload drop failed")
            # Continue anyway - might be mechanical issue
        else:
            logger.info(f"PAYLOAD DROPPED! Remaining: {self.payload.remaining()}")
        
        # 4. Return to holding altitude
        self.drone.goto(target.lat, target.lon, self.hold_altitude)
        self.drone.wait_for_altitude(self.hold_altitude, tolerance=2.0, timeout=15.0)
        
        # 5. Return to loiter
        self.state_machine.transition_to(MissionState.LOITER, "Drop complete")
        self.current_target = None
        
        logger.info(f"Target processed. Queue remaining: {self.target_queue.qsize()}, "
                   f"Payload remaining: {self.payload.remaining()}")
        
        return True
    
    def _return_to_launch(self) -> bool:
        """Execute RTL procedure."""
        logger.info("Returning to launch...")
        
        self.state_machine.transition_to(MissionState.RTL, "Mission complete")
        self.mission_active = False
        
        if not self.drone.rtl_now():
            logger.error("RTL command failed")
            return False
        
        # Wait for landing
        self.state_machine.transition_to(MissionState.LANDING, "Landing")
        time.sleep(30)  # In reality, monitor altitude
        
        self.state_machine.transition_to(MissionState.LANDED, "Landed")
        
        # Final statistics
        logger.info("="*50)
        logger.info("MISSION COMPLETE")
        logger.info(f"  Targets processed: {self.targets_processed}")
        logger.info(f"  Payloads dropped: {self.payload.drops_performed}")
        logger.info(f"  Payloads remaining: {self.payload.remaining()}")
        logger.info("="*50)
        
        return True
    
    def _emergency_rtl(self):
        """Emergency RTL procedure."""
        logger.error("EMERGENCY RTL TRIGGERED")
        self.state_machine.force_state(MissionState.EMERGENCY, "Emergency")
        self.mission_active = False
        
        if self.drone:
            self.drone.rtl_now()
    
    def shutdown(self):
        """Clean shutdown of all components."""
        logger.info("Shutting down...")
        self.mission_active = False
        
        if self.comms:
            self.comms.disconnect()
        
        if self.drone:
            self.drone.disconnect()
        
        logger.info("Shutdown complete")
    
    def abort(self):
        """Request mission abort."""
        logger.warning("Abort requested")
        self.abort_requested = True
        self.mission_active = False
    
    def get_status(self) -> dict:
        """Get current mission status."""
        return {
            'state': self.state_machine.get_state_name(),
            'targets_processed': self.targets_processed,
            'queue_size': self.target_queue.qsize(),
            'payload_remaining': self.payload.remaining() if self.payload else 0,
            'current_target': {
                'lat': self.current_target.lat,
                'lon': self.current_target.lon
            } if self.current_target else None
        }


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Delivery Follower Mission - Drone 2")
    parser.add_argument(
        '--config', 
        default='config/mission_params.yaml',
        help='Path to mission configuration file'
    )
    parser.add_argument(
        '--test',
        action='store_true',
        help='Run in test mode (no actual flight)'
    )
    
    args = parser.parse_args()
    
    # Resolve config path
    script_dir = Path(__file__).parent.parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = script_dir / config_path
    
    logger.info("="*60)
    logger.info("ANTIGRAVITY CORE - Delivery Follower Mission")
    logger.info("="*60)
    
    mission = DeliveryFollowerMission(str(config_path))
    
    try:
        if not mission.setup():
            logger.error("Mission setup failed")
            sys.exit(1)
        
        if args.test:
            logger.info("TEST MODE - Simulating mission")
            logger.info(f"Initial payload count: {mission.payload.remaining()}")
            logger.info("Would enter loiter and wait for coordinates...")
            
            # Simulate receiving a coordinate
            logger.info("Simulating coordinate reception...")
            time.sleep(2)
        else:
            if not mission.execute():
                logger.error("Mission execution failed")
                sys.exit(1)
        
        logger.info("Mission completed")
        
    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
        mission.abort()
    finally:
        mission.shutdown()


if __name__ == "__main__":
    main()
