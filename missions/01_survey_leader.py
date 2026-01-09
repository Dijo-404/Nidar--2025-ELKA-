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
from src.intelligence.human_tracker import HumanTracker
from src.intelligence.geotagging import GeoTaggedDetection
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
        self.tracker: HumanTracker = None
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
            
            # 3. Initialize human tracker (with BoT-SORT + geotagging)
            detection_config = self.config.get('detection', {})
            tracking_config = self.config.get('tracking', {})
            model_path = detection_config.get('model_path')
            
            if not model_path or not os.path.exists(model_path):
                logger.error(f"YOLO model not found: {model_path}")
                return False
            
            # Merge detection and tracking configs
            tracker_config = {
                'confidence_threshold': detection_config.get('confidence_threshold', 0.7),
                'target_class_id': detection_config.get('target_class_id', 0),
                **tracking_config
            }
            self.tracker = HumanTracker(model_path, tracker_config)
            logger.info("HumanTracker (BoT-SORT + Geotagging) initialized")
            
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
    
    def _process_detections(self, geotagged_detections: list):
        """
        Process geotagged detections and send to relay.
        
        Args:
            geotagged_detections: List of GeoTaggedDetection objects
        """
        if self.abort_requested:
            return
        
        for detection in geotagged_detections:
            if not detection.target_lat or not detection.target_lon:
                continue
            
            logger.info(f"🎯 TARGET DETECTED (ID:{detection.track_id})")
            logger.info(f"   Drone GPS: ({detection.drone_lat:.6f}, {detection.drone_lon:.6f})")
            logger.info(f"   Target GPS: ({detection.target_lat:.6f}, {detection.target_lon:.6f})")
            logger.info(f"   Offset: {detection.offset_north_m:.1f}m N, {detection.offset_east_m:.1f}m E")
            
            # Transition to detected state briefly
            prev_state = self.state_machine.get_state()
            self.state_machine.transition_to(MissionState.DETECTED, "Human detected")
            
            # Send TARGET coordinates (not drone coordinates) to relay
            if self.comms and self.comms.is_connected:
                if self.comms.send_coordinates(
                    detection.target_lat,
                    detection.target_lon,
                    detection.drone_alt,  # Use drone altitude for delivery
                    time.time()
                ):
                    self.detections_sent += 1
                    logger.info(f"✅ Target coordinates sent to relay (total: {self.detections_sent})")
                else:
                    logger.error("Failed to send coordinates")
            
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
            
            # Execute waypoints with tracking
            self.state_machine.transition_to(MissionState.SURVEY, "Beginning survey")
            
            # Get camera config for frame dimensions
            camera_config = self.config.get('camera', {})
            frame_width = camera_config.get('frame_width', 1920)
            frame_height = camera_config.get('frame_height', 1080)
            
            # Start video capture
            rtsp_url = camera_config.get('rtsp_url')
            cap = None
            if rtsp_url:
                import cv2
                logger.info(f"Opening camera stream: {rtsp_url}")
                cap = cv2.VideoCapture(rtsp_url)
                if not cap.isOpened():
                    logger.warning("Failed to open camera - continuing without detection")
                    cap = None
            
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
                
                # Process frames while navigating
                while not self.drone.wait_for_arrival(lat, lon, tolerance=2.0, timeout=1.0):
                    if self.abort_requested:
                        break
                    
                    # Get current drone state
                    gps = self.drone.get_current_gps()
                    heading = self.drone.get_heading() or 0.0
                    
                    if cap and gps:
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            drone_lat, drone_lon, drone_alt = gps
                            
                            # Track and geotag
                            new_detections = self.tracker.track_and_geotag(
                                frame,
                                drone_lat, drone_lon, drone_alt, heading,
                                frame_width, frame_height
                            )
                            
                            # Process any new detections
                            if new_detections:
                                self._process_detections(new_detections)
                
                # Update comms status
                if self.comms:
                    battery_ok, _, battery_pct = self.drone.check_battery()
                    self.comms.set_status(
                        f"SURVEY_{i+1}_{len(self.waypoints)}",
                        battery_percent=battery_pct
                    )
            
            # Release camera
            if cap:
                cap.release()
            
            logger.info(f"Survey complete! Detections sent: {self.detections_sent}")
            logger.info(f"Unique tracks geotagged: {self.tracker.get_geotagged_count()}")
            
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
        
        # Tracker doesn't need explicit stop (no background thread)
        
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
        
        # Tracker doesn't need explicit stop
        
        if self.drone:
            self.drone.rtl_now()
    
    def shutdown(self):
        """Clean shutdown of all components."""
        logger.info("Shutting down...")
        
        # Tracker doesn't need explicit stop
        
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
