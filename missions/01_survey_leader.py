#!/usr/bin/env python3
"""
Survey Leader Mission - Drone 1 Main Executable

This script executes the survey mission for Drone 1 (The Surveyor):
1. Load KML and generate sweep waypoints
2. Execute flight path while scanning for humans
3. Send detected human coordinates to ground relay
4. Complete survey and RTL

Usage:
    python 01_survey_leader.py [--config CONFIG_PATH] [--kml KML_PATH] [--test]
"""

import os
import sys
import time
import yaml
import logging
import argparse
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.base.drone_pilot import DronePilot
from src.intelligence.path_finder import PathFinder
from src.intelligence.human_detector import HumanDetector, DetectionResult
from src.comms.bridge_client import BridgeClient
from src.utils.state_machine import StateMachine, MissionState
from src.utils.geo_math import haversine_distance

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('SurveyLeader')


class SurveyLeaderMission:
    """
    Survey mission controller for Drone 1.
    
    Coordinates path planning, human detection, and communication
    to execute the survey and detection mission.
    """
    
    def __init__(self, config_path: str, kml_path: str = None):
        """
        Initialize survey mission.
        
        Args:
            config_path: Path to mission_params.yaml
            kml_path: Path to KML file (optional, uses config default)
        """
        # Load configuration
        self.config = self._load_config(config_path)
        self.network_config = self._load_config(
            os.path.join(os.path.dirname(config_path), 'network_map.yaml')
        )
        
        # Determine KML path
        if kml_path:
            self.kml_path = kml_path
        else:
            self.kml_path = os.path.join(
                os.path.dirname(config_path), 
                'geofence', 
                'sector_alpha.kml'
            )
        
        # Initialize state machine
        self.state_machine = StateMachine(MissionState.IDLE)
        
        # Initialize components (will be created in setup)
        self.drone: DronePilot = None
        self.path_finder: PathFinder = None
        self.detector: HumanDetector = None
        self.comms: BridgeClient = None
        
        # Mission state
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.detections_sent = 0
        self.detection_log_dir = os.path.join(
            os.path.dirname(config_path), '..', 'logs', 'detections'
        )
        
        # Flags
        self.abort_requested = False
    
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
        logger.info("Setting up Survey Leader mission...")
        self.state_machine.transition_to(MissionState.PREFLIGHT, "Starting setup")
        
        try:
            # 1. Initialize path finder and load KML
            logger.info(f"Loading KML from {self.kml_path}")
            self.path_finder = PathFinder(self.config.get('path_planning', {}))
            
            if not self.path_finder.load_kml(self.kml_path):
                logger.error("Failed to load KML file")
                return False
            
            # Generate waypoints
            flight_config = self.config.get('flight', {}).get('survey', {})
            self.waypoints = self.path_finder.generate_sweep_pattern(
                spacing=self.config.get('path_planning', {}).get('sweep_spacing', 7.0),
                interval=self.config.get('path_planning', {}).get('waypoint_interval', 10.0),
                altitude=flight_config.get('cruise_altitude', 20.0)
            )
            
            if not self.waypoints:
                logger.error("No waypoints generated")
                return False
            
            logger.info(f"Generated {len(self.waypoints)} waypoints")
            
            # 2. Initialize drone pilot
            mavlink_conn = self.network_config.get('drone1', {}).get(
                'mavlink_connection', 'udp:127.0.0.1:14550'
            )
            safety_config = self.config.get('safety', {})
            
            self.drone = DronePilot(mavlink_conn, safety_config)
            
            if not self.drone.connect():
                logger.error("Failed to connect to drone")
                return False
            
            # 3. Initialize human detector
            detection_config = self.config.get('detection', {})
            model_path = detection_config.get('model_path')
            
            if not model_path or not os.path.exists(model_path):
                logger.error(f"YOLO model not found: {model_path}")
                return False
            
            self.detector = HumanDetector(model_path, detection_config)
            self.detector.on_detection = self._on_human_detected
            
            # 4. Initialize communications
            relay_config = self.network_config.get('ground_relay', {})
            drone_config = self.network_config.get('drone1', {})
            
            self.comms = BridgeClient(
                identity=drone_config.get('zmq_identity', 'SURVEYOR'),
                config=relay_config
            )
            
            if not self.comms.connect():
                logger.warning("Failed to connect to relay - continuing in standalone mode")
            
            # Create detection log directory
            os.makedirs(self.detection_log_dir, exist_ok=True)
            
            logger.info("Setup complete")
            return True
            
        except Exception as e:
            logger.error(f"Setup failed: {e}")
            return False
    
    def _on_human_detected(self, result: DetectionResult):
        """
        Callback when human is detected.
        
        Args:
            result: Detection result with bounding boxes
        """
        if self.abort_requested:
            return
        
        # Get current drone GPS
        gps = self.drone.get_current_gps()
        if not gps:
            logger.warning("Detection occurred but no GPS available")
            return
        
        lat, lon, alt = gps
        
        logger.info(f"HUMAN DETECTED at drone position ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
        
        # Transition to detected state briefly
        prev_state = self.state_machine.get_state()
        self.state_machine.transition_to(MissionState.DETECTED, "Human detected")
        
        # Send coordinates to relay
        if self.comms and self.comms.is_connected:
            if self.comms.send_coordinates(lat, lon, alt, result.timestamp):
                self.detections_sent += 1
                logger.info(f"Coordinates sent to relay (total: {self.detections_sent})")
            else:
                logger.error("Failed to send coordinates")
        
        # Save detection image
        if result.frame is not None:
            self.detector.save_detection_image(
                result, 
                self.detection_log_dir,
                prefix=f"detection_{self.detections_sent}"
            )
        
        # Return to previous state
        self.state_machine.transition_to(prev_state, "Continuing survey")
    
    def execute(self) -> bool:
        """
        Execute the survey mission.
        
        Returns:
            True if mission completed successfully
        """
        logger.info("Starting survey mission execution")
        
        try:
            # Arm and takeoff
            if not self._arm_and_takeoff():
                return False
            
            # Start human detection
            camera_config = self.config.get('camera', {})
            rtsp_url = camera_config.get('rtsp_url')
            
            if rtsp_url:
                logger.info(f"Starting detection stream: {rtsp_url}")
                self.detector.start_stream(rtsp_url)
            else:
                logger.warning("No RTSP URL configured - detection disabled")
            
            # Execute waypoints
            self.state_machine.transition_to(MissionState.SURVEY, "Beginning survey")
            
            for i, waypoint in enumerate(self.waypoints):
                if self.abort_requested:
                    logger.warning("Abort requested - stopping survey")
                    break
                
                lat, lon, alt = waypoint
                self.current_waypoint_idx = i
                
                logger.info(f"Waypoint {i+1}/{len(self.waypoints)}: "
                           f"({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
                
                # Navigate to waypoint
                if not self.drone.goto(lat, lon, alt):
                    logger.error("Navigation command failed")
                    continue
                
                # Wait for arrival
                if not self.drone.wait_for_arrival(lat, lon, tolerance=2.0, timeout=60.0):
                    logger.warning("Waypoint arrival timeout - continuing to next")
                
                # Update comms status
                if self.comms:
                    battery_ok, _, battery_pct = self.drone.check_battery()
                    self.comms.set_status(
                        f"SURVEY_{i+1}_{len(self.waypoints)}",
                        battery_percent=battery_pct
                    )
            
            logger.info(f"Survey complete! Detections sent: {self.detections_sent}")
            
            # RTL
            return self._return_to_launch()
            
        except Exception as e:
            logger.error(f"Mission execution error: {e}")
            self._emergency_rtl()
            return False
    
    def _arm_and_takeoff(self) -> bool:
        """Arm the drone and take off to survey altitude."""
        logger.info("Arming and taking off...")
        
        if not self.drone.arm():
            logger.error("Arming failed")
            return False
        
        self.state_machine.transition_to(MissionState.ARMED, "Armed")
        
        altitude = self.config.get('flight', {}).get('survey', {}).get('cruise_altitude', 20.0)
        
        if not self.drone.safe_takeoff(altitude):
            logger.error("Takeoff failed")
            return False
        
        self.state_machine.transition_to(MissionState.TAKEOFF, "Taking off")
        
        if not self.drone.wait_for_altitude(altitude, tolerance=2.0, timeout=30.0):
            logger.error("Failed to reach takeoff altitude")
            return False
        
        logger.info(f"Reached survey altitude: {altitude}m")
        return True
    
    def _return_to_launch(self) -> bool:
        """Execute RTL procedure."""
        logger.info("Returning to launch...")
        
        # Stop detection
        if self.detector:
            self.detector.stop()
        
        self.state_machine.transition_to(MissionState.RTL, "Mission complete")
        
        if not self.drone.rtl_now():
            logger.error("RTL command failed")
            return False
        
        # Wait for landing (simplified)
        self.state_machine.transition_to(MissionState.LANDING, "Landing")
        time.sleep(30)  # In reality, monitor altitude
        
        self.state_machine.transition_to(MissionState.LANDED, "Landed")
        logger.info("Landed safely")
        
        return True
    
    def _emergency_rtl(self):
        """Emergency RTL procedure."""
        logger.error("EMERGENCY RTL TRIGGERED")
        self.state_machine.force_state(MissionState.EMERGENCY, "Emergency")
        
        if self.detector:
            self.detector.stop()
        
        if self.drone:
            self.drone.rtl_now()
    
    def shutdown(self):
        """Clean shutdown of all components."""
        logger.info("Shutting down...")
        
        if self.detector:
            self.detector.stop()
        
        if self.comms:
            self.comms.disconnect()
        
        if self.drone:
            self.drone.disconnect()
        
        logger.info("Shutdown complete")
    
    def abort(self):
        """Request mission abort."""
        logger.warning("Abort requested")
        self.abort_requested = True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Survey Leader Mission - Drone 1")
    parser.add_argument(
        '--config', 
        default='config/mission_params.yaml',
        help='Path to mission configuration file'
    )
    parser.add_argument(
        '--kml',
        help='Path to KML file (optional)'
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
    logger.info("ANTIGRAVITY CORE - Survey Leader Mission")
    logger.info("="*60)
    
    mission = SurveyLeaderMission(str(config_path), args.kml)
    
    try:
        if not mission.setup():
            logger.error("Mission setup failed")
            sys.exit(1)
        
        if args.test:
            logger.info("TEST MODE - Simulating mission")
            logger.info(f"Would execute {len(mission.waypoints)} waypoints")
            for i, wp in enumerate(mission.waypoints[:5]):
                logger.info(f"  WP{i+1}: {wp}")
            if len(mission.waypoints) > 5:
                logger.info(f"  ... and {len(mission.waypoints) - 5} more")
        else:
            if not mission.execute():
                logger.error("Mission execution failed")
                sys.exit(1)
        
        logger.info("Mission completed successfully")
        
    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
        mission.abort()
    finally:
        mission.shutdown()


if __name__ == "__main__":
    main()
