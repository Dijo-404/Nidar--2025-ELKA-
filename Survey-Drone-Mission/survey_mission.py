#!/usr/bin/env python3
"""
Survey Drone Mission - Main Entry Point

Real-time survey mission with:
- KML waypoint loading
- Human detection (YOLO)
- GPS geotagging
- Live video display with overlays

Usage:
    python survey_mission.py                          # Use config.yaml defaults
    python survey_mission.py --source 0               # Webcam
    python survey_mission.py --kml path/to/file.kml   # Custom KML
    python survey_mission.py --simulate-gps           # Simulated GPS
    python survey_mission.py --no-detection           # Disable YOLO
"""

import argparse
import logging
import os
import sys
import time
import yaml
import cv2

from kml_processor import KMLProcessor
from human_detector import HumanDetector, DetectionResult
from geotagger import Geotagger
from video_display import VideoDisplay

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('SurveyMission')


class SurveyMission:
    """
    Main survey mission controller.
    
    Integrates all components for real-time survey operation.
    """
    
    def __init__(self, config_path: str = "config.yaml"):
        """Initialize survey mission."""
        self.config = self._load_config(config_path)
        
        # Components
        self.kml_processor: KMLProcessor = None
        self.detector: HumanDetector = None
        self.geotagger: Geotagger = None
        self.display: VideoDisplay = None
        self.video_capture: cv2.VideoCapture = None
        
        # State
        self.running = False
        self.current_waypoint = 0
        self.total_detections = 0
        self.frames_processed = 0
    
    def _load_config(self, path: str) -> dict:
        """Load YAML configuration."""
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            logger.warning(f"Failed to load config: {e}, using defaults")
            return {}
    
    def setup(self, 
              kml_path: str = None,
              video_source = None,
              simulate_gps: bool = False,
              enable_detection: bool = True) -> bool:
        """
        Set up all mission components.
        
        Args:
            kml_path: Path to KML file (overrides config)
            video_source: Video source (overrides config)
            simulate_gps: Use simulated GPS
            enable_detection: Enable YOLO detection
            
        Returns:
            True if setup successful
        """
        logger.info("=" * 50)
        logger.info("SURVEY DRONE MISSION - SETUP")
        logger.info("=" * 50)
        
        # 1. KML Processing
        kml_file = kml_path or self.config.get('survey', {}).get('kml_file')
        if kml_file:
            logger.info(f"Loading KML: {kml_file}")
            survey_config = self.config.get('survey', {})
            self.kml_processor = KMLProcessor(survey_config)
            
            if self.kml_processor.load(kml_file):
                waypoints = self.kml_processor.generate_waypoints()
                logger.info(f"Generated {len(waypoints)} waypoints")
            else:
                logger.warning("KML loading failed - continuing without waypoints")
        else:
            logger.info("No KML file specified")
        
        # 2. Human Detector
        if enable_detection:
            logger.info("Initializing human detector...")
            detection_config = self.config.get('detection', {})
            self.detector = HumanDetector(detection_config)
            
            if self.detector.is_available():
                logger.info("YOLO detector ready")
            else:
                logger.warning("YOLO detector not available")
        else:
            logger.info("Detection disabled")
        
        # 3. GPS Geotagger
        gps_config = self.config.get('gps', {}).copy()
        if simulate_gps:
            gps_config['source'] = 'simulated'
        
        logger.info(f"Initializing GPS ({gps_config.get('source', 'simulated')})...")
        self.geotagger = Geotagger(gps_config)
        
        # Start CSV logging
        output_dir = self.config.get('logging', {}).get('output_dir', 'output')
        csv_file = self.config.get('logging', {}).get('csv_file', 'detections.csv')
        self.geotagger.start_logging(output_dir, csv_file)
        
        # 4. Video Display
        display_config = self.config.get('display', {})
        self.display = VideoDisplay(display_config)
        
        # 5. Video Capture
        source = video_source if video_source is not None else self.config.get('video', {}).get('source', 0)
        logger.info(f"Opening video source: {source}")
        
        self.video_capture = cv2.VideoCapture(source)
        
        if not self.video_capture.isOpened():
            logger.error(f"Failed to open video source: {source}")
            return False
        
        # Set resolution if specified
        video_config = self.config.get('video', {})
        if 'width' in video_config:
            self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, video_config['width'])
        if 'height' in video_config:
            self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, video_config['height'])
        
        logger.info("Setup complete!")
        logger.info("=" * 50)
        
        return True
    
    def run(self):
        """
        Run the main mission loop.
        
        Processes video frames, runs detection, and displays results.
        """
        logger.info("Starting mission loop...")
        logger.info("Press 'q' to quit, 's' for screenshot, 'r' to reset count")
        
        self.running = True
        last_detection_time = 0
        detection_cooldown = 2.0  # Seconds between geotagging same detection
        
        try:
            while self.running:
                # Read frame
                ret, frame = self.video_capture.read()
                if not ret:
                    logger.warning("Failed to read frame")
                    time.sleep(0.1)
                    continue
                
                self.frames_processed += 1
                current_detection_count = 0
                unique_count = 0
                
                # Run detection with tracking
                if self.detector and self.detector.is_available():
                    result = self.detector.detect(frame, draw=True)
                    frame = result.frame
                    current_detection_count = result.count
                    unique_count = self.detector.get_unique_count()
                    
                    # Geotag NEW tracks only
                    if result.new_track_ids:
                        current_time = time.time()
                        if current_time - last_detection_time > detection_cooldown:
                            avg_conf = sum(d.confidence for d in result.detections) / len(result.detections) if result.detections else 0
                            for track_id in result.new_track_ids:
                                self.geotagger.geotag_detection(1, avg_conf)
                                self.total_detections += 1
                            last_detection_time = current_time
                
                # Get GPS position for display
                gps_pos = self.geotagger.get_position()
                gps_data = None
                if gps_pos:
                    gps_data = {
                        'lat': gps_pos.lat,
                        'lon': gps_pos.lon,
                        'alt': gps_pos.alt
                    }
                
                # Get waypoint info
                total_waypoints = 0
                if self.kml_processor:
                    total_waypoints = self.kml_processor.get_waypoint_count()
                
                # Display
                key = self.display.show(
                    frame,
                    gps_data=gps_data,
                    detection_count=current_detection_count,
                    total_detections=self.total_detections,
                    unique_count=unique_count,
                    current_waypoint=self.current_waypoint,
                    total_waypoints=total_waypoints
                )
                
                # Handle keyboard input
                if key == ord('q'):
                    logger.info("Quit requested")
                    self.running = False
                elif key == ord('s'):
                    output_dir = self.config.get('logging', {}).get('output_dir', 'output')
                    self.display.save_screenshot(frame, output_dir)
                elif key == ord('r'):
                    self.total_detections = 0
                    logger.info("Detection count reset")
                elif key == ord('n') and self.kml_processor:
                    # Next waypoint (for testing)
                    self.current_waypoint = min(self.current_waypoint + 1, total_waypoints)
                    
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        logger.info("Cleaning up...")
        
        self.running = False
        
        if self.video_capture:
            self.video_capture.release()
        
        if self.display:
            self.display.close()
        
        if self.geotagger:
            self.geotagger.close()
        
        cv2.destroyAllWindows()
        
        # Print summary
        logger.info("=" * 50)
        logger.info("MISSION SUMMARY")
        logger.info("=" * 50)
        logger.info(f"Frames processed: {self.frames_processed}")
        logger.info(f"Total detections: {self.total_detections}")
        logger.info(f"Geotagged events: {len(self.geotagger.geotagged_detections) if self.geotagger else 0}")
        logger.info("=" * 50)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Survey Drone Mission - Real-time human detection with geotagging',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python survey_mission.py                          # Use config defaults
    python survey_mission.py --source 0               # Use webcam
    python survey_mission.py --source video.mp4       # Use video file
    python survey_mission.py --kml area.kml           # Custom survey area
    python survey_mission.py --simulate-gps           # Simulated GPS
    python survey_mission.py --no-detection           # Disable YOLO
"""
    )
    
    parser.add_argument('--config', '-c', default='config.yaml',
                       help='Path to config file (default: config.yaml)')
    parser.add_argument('--source', '-s', default=None,
                       help='Video source: camera index, RTSP URL, or file path')
    parser.add_argument('--kml', '-k', default=None,
                       help='Path to KML survey area file')
    parser.add_argument('--simulate-gps', action='store_true',
                       help='Use simulated GPS coordinates')
    parser.add_argument('--no-detection', action='store_true',
                       help='Disable YOLO human detection')
    parser.add_argument('--no-gps', action='store_true',
                       help='Disable GPS (implies --simulate-gps)')
    
    args = parser.parse_args()
    
    # Parse video source
    video_source = args.source
    if video_source is not None:
        try:
            video_source = int(video_source)
        except ValueError:
            pass  # Keep as string (file path or URL)
    
    # Create and run mission
    mission = SurveyMission(args.config)
    
    if mission.setup(
        kml_path=args.kml,
        video_source=video_source,
        simulate_gps=args.simulate_gps or args.no_gps,
        enable_detection=not args.no_detection
    ):
        mission.run()
    else:
        logger.error("Setup failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
