"""
Mock Components for Testing Without Hardware

Provides mock implementations of hardware components for unit testing
and development without physical drones, cameras, or servos.
"""

import time
import logging
import random
from typing import Optional, Tuple, List, Callable
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class MockGPS:
    """Simulated GPS position."""
    lat: float = 12.9716
    lon: float = 77.5946
    alt: float = 0.0
    heading: float = 0.0


class MockDronePilot:
    """
    Mock drone pilot for testing without MAVLink connection.
    
    Simulates basic flight behavior including takeoff, navigation, and RTL.
    """
    
    def __init__(self, connection_string: str = None, config: dict = None, 
                 gcs_config: dict = None):
        self.connection_string = connection_string or "mock://drone"
        self.config = config or {}
        
        # Simulated state
        self.gps = MockGPS()
        self.is_armed = False
        self.is_flying = False
        self.mode = "STABILIZE"
        self.battery_voltage = 16.8
        self.battery_percent = 100
        
        # Target position
        self.target_lat: Optional[float] = None
        self.target_lon: Optional[float] = None
        self.target_alt: Optional[float] = None
        
        logger.info(f"MockDronePilot initialized: {self.connection_string}")
    
    def connect(self, enable_gcs_forward: bool = False) -> bool:
        logger.info("Mock: Connected to simulated drone")
        return True
    
    def disconnect(self):
        logger.info("Mock: Disconnected")
    
    def check_battery(self) -> Tuple[bool, float, int]:
        # Slowly drain battery
        self.battery_percent = max(0, self.battery_percent - 0.01)
        self.battery_voltage = 12.0 + (self.battery_percent / 100.0) * 4.8
        is_safe = self.battery_percent > 20
        return is_safe, self.battery_voltage, int(self.battery_percent)
    
    def get_current_gps(self) -> Optional[Tuple[float, float, float]]:
        return (self.gps.lat, self.gps.lon, self.gps.alt)
    
    def get_heading(self) -> float:
        return self.gps.heading
    
    def get_home_position(self) -> Optional[Tuple[float, float, float]]:
        return (12.9716, 77.5946, 0.0)
    
    def set_mode(self, mode: str) -> bool:
        self.mode = mode
        logger.info(f"Mock: Mode set to {mode}")
        return True
    
    def arm(self) -> bool:
        self.is_armed = True
        logger.info("Mock: Armed")
        return True
    
    def disarm(self) -> bool:
        self.is_armed = False
        logger.info("Mock: Disarmed")
        return True
    
    def safe_takeoff(self, altitude: float) -> bool:
        if not self.is_armed:
            return False
        self.is_flying = True
        self.target_alt = altitude
        self.mode = "GUIDED"
        logger.info(f"Mock: Taking off to {altitude}m")
        return True
    
    def wait_for_altitude(self, target_alt: float, tolerance: float = 1.0,
                          timeout: float = 30.0) -> bool:
        # Simulate climbing
        while self.gps.alt < target_alt - tolerance:
            self.gps.alt = min(target_alt, self.gps.alt + 2.0)
            time.sleep(0.1)
        logger.info(f"Mock: Reached altitude {self.gps.alt:.1f}m")
        return True
    
    def goto(self, lat: float, lon: float, alt: float) -> bool:
        self.target_lat = lat
        self.target_lon = lon
        self.target_alt = alt
        logger.info(f"Mock: Navigating to ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
        return True
    
    def wait_for_arrival(self, lat: float, lon: float, tolerance: float = 2.0,
                         timeout: float = 60.0) -> bool:
        # Simulate movement
        steps = 10
        lat_step = (lat - self.gps.lat) / steps
        lon_step = (lon - self.gps.lon) / steps
        alt_step = ((self.target_alt or self.gps.alt) - self.gps.alt) / steps
        
        for _ in range(steps):
            self.gps.lat += lat_step
            self.gps.lon += lon_step
            self.gps.alt += alt_step
            time.sleep(0.05)
        
        logger.info(f"Mock: Arrived at ({self.gps.lat:.6f}, {self.gps.lon:.6f})")
        return True
    
    def rtl_now(self) -> bool:
        self.mode = "RTL"
        logger.info("Mock: RTL initiated")
        return True
    
    def land(self) -> bool:
        self.mode = "LAND"
        self.is_flying = False
        self.gps.alt = 0
        logger.info("Mock: Landed")
        return True
    
    def loiter(self) -> bool:
        self.mode = "LOITER"
        logger.info("Mock: Loitering")
        return True
    
    def is_connected(self) -> bool:
        return True


class MockPayload:
    """
    Mock payload mechanism for testing.
    
    Simulates payload count and drop operations.
    """
    
    def __init__(self, config: dict = None, mav_connection=None):
        self.config = config or {}
        self.initial_count = self.config.get('initial_count', 10)
        self.payload_count = self.initial_count
        self.drops_performed = 0
        self.last_drop_time: Optional[float] = None
        
        logger.info(f"MockPayload initialized: {self.payload_count} units")
    
    def hold(self) -> bool:
        logger.debug("Mock: Servo hold position")
        return True
    
    def drop(self) -> bool:
        if self.payload_count <= 0:
            logger.warning("Mock: No payload remaining")
            return False
        
        self.payload_count -= 1
        self.drops_performed += 1
        self.last_drop_time = time.time()
        logger.info(f"Mock: Payload dropped! Remaining: {self.payload_count}")
        return True
    
    def remaining(self) -> int:
        return self.payload_count
    
    def is_empty(self) -> bool:
        return self.payload_count <= 0
    
    def get_stats(self) -> dict:
        return {
            'initial_count': self.initial_count,
            'remaining': self.payload_count,
            'drops_performed': self.drops_performed,
            'last_drop_time': self.last_drop_time,
            'is_empty': self.is_empty()
        }
    
    def reset_count(self, count: int = None):
        self.payload_count = count if count is not None else self.initial_count
        self.drops_performed = 0
        logger.info(f"Mock: Payload count reset to {self.payload_count}")
    
    def test_servo(self) -> bool:
        logger.info("Mock: Servo test - moving to drop position...")
        time.sleep(0.5)
        logger.info("Mock: Servo test - returning to hold...")
        time.sleep(0.5)
        logger.info("Mock: Servo test complete")
        return True


class MockCamera:
    """
    Mock camera for testing without RTSP stream.
    
    Generates synthetic frames with optional simulated detections.
    """
    
    def __init__(self, config: dict = None):
        self.config = config or {}
        self.frame_width = self.config.get('frame_width', 1920)
        self.frame_height = self.config.get('frame_height', 1080)
        self.is_open = False
        self.frame_count = 0
        
        # Simulated detections (list of [x, y, w, h] boxes)
        self.simulated_detections: List[List[float]] = []
        
        logger.info(f"MockCamera initialized: {self.frame_width}x{self.frame_height}")
    
    def open(self, url: str = None) -> bool:
        self.is_open = True
        logger.info(f"Mock: Camera opened")
        return True
    
    def read(self):
        """
        Read a frame.
        
        Returns:
            Tuple of (success, frame)
        """
        import numpy as np
        
        if not self.is_open:
            return False, None
        
        self.frame_count += 1
        
        # Generate a simple gradient frame
        frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        frame[:, :, 0] = 50  # Blue channel
        frame[:, :, 1] = 100  # Green channel
        frame[:, :, 2] = 50  # Red channel
        
        return True, frame
    
    def release(self):
        self.is_open = False
        logger.info("Mock: Camera released")
    
    def isOpened(self) -> bool:
        return self.is_open
    
    def add_simulated_detection(self, x: float, y: float, w: float, h: float):
        """Add a simulated detection box."""
        self.simulated_detections.append([x, y, w, h])
    
    def clear_detections(self):
        """Clear all simulated detections."""
        self.simulated_detections.clear()


class MockHumanTracker:
    """
    Mock human tracker for testing.
    
    Returns simulated tracking results without running YOLO.
    """
    
    def __init__(self, model_path: str = None, config: dict = None):
        self.config = config or {}
        self.frame_count = 0
        self.track_counter = 0
        self._geotagged_ids = set()
        
        # Simulated tracks: {track_id: (x, y, w, h)}
        self.simulated_tracks = {}
        
        logger.info("MockHumanTracker initialized")
    
    def add_simulated_person(self, track_id: int, x: float, y: float,
                              w: float = 50, h: float = 100):
        """Add a simulated tracked person."""
        self.simulated_tracks[track_id] = (x, y, w, h)
    
    def remove_simulated_person(self, track_id: int):
        """Remove a simulated person."""
        self.simulated_tracks.pop(track_id, None)
    
    def track_frame(self, frame, box_x=None, box_y=None, 
                    box_width=None, box_height=None):
        """Simulate tracking."""
        from dataclasses import dataclass
        
        @dataclass
        class MockTrackingResult:
            frame_count: int
            track_ids: List[int]
            boxes: List[List[float]]
            confidences: List[float]
            total_count: int
            new_counts: List[int]
            timestamp: float = 0
        
        self.frame_count += 1
        
        track_ids = list(self.simulated_tracks.keys())
        boxes = [[x - w/2, y - h/2, x + w/2, y + h/2] 
                 for x, y, w, h in self.simulated_tracks.values()]
        confidences = [0.9] * len(track_ids)
        
        return MockTrackingResult(
            frame_count=self.frame_count,
            track_ids=track_ids,
            boxes=boxes,
            confidences=confidences,
            total_count=len(track_ids),
            new_counts=[],
            timestamp=time.time()
        )
    
    def track_and_geotag(self, frame, drone_lat, drone_lon, drone_alt,
                         drone_heading=0.0, frame_width=None, frame_height=None):
        """Simulate tracking with geotagging."""
        from src.intelligence.geotagging import GeoTagger, GeoTaggedDetection
        
        new_geotagged = []
        
        geotagger = GeoTagger({
            'frame_width': frame_width or 1920,
            'frame_height': frame_height or 1080
        })
        
        for track_id, (x, y, w, h) in self.simulated_tracks.items():
            if track_id in self._geotagged_ids:
                continue
            
            self._geotagged_ids.add(track_id)
            
            geotagged = geotagger.geotag_detection(
                box_xywh=[x, y, w, h],
                confidence=0.9,
                drone_lat=drone_lat,
                drone_lon=drone_lon,
                drone_alt=drone_alt,
                drone_heading=drone_heading,
                track_id=track_id
            )
            new_geotagged.append(geotagged)
        
        return new_geotagged
    
    def get_geotagged_count(self) -> int:
        return len(self._geotagged_ids)
    
    def reset(self):
        self.frame_count = 0
        self._geotagged_ids.clear()
        self.simulated_tracks.clear()
