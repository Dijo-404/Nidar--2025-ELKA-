"""
GeoTagging - GSD-based Detection Coordinate Calculation

Converts detected object pixel positions to GPS coordinates using
Ground Sample Distance (GSD) calculation with heading compensation.

Key improvements over basic implementation:
- Heading (yaw) compensation for accurate N/E offset mapping
- Configurable camera parameters (sensor size, focal length)
- Support for gimbal pitch angles (non-nadir cameras)
"""

import math
import logging
from typing import Tuple, Optional, List
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

# WGS84 Earth radius in meters (mean radius)
EARTH_RADIUS_M = 6371000.0


@dataclass
class GeoTaggedDetection:
    """Detection result with GPS coordinates."""
    # Original detection data
    box_xywh: List[float]       # [center_x, center_y, width, height] in pixels
    confidence: float
    track_id: Optional[int] = None
    
    # Computed GPS location
    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    
    # Metadata
    drone_lat: Optional[float] = None
    drone_lon: Optional[float] = None
    drone_alt: Optional[float] = None
    drone_heading: Optional[float] = None
    gsd_m: Optional[float] = None
    offset_north_m: Optional[float] = None
    offset_east_m: Optional[float] = None


class GeoTagger:
    """
    Converts pixel detections to GPS coordinates using GSD calculation.
    
    The Ground Sample Distance (GSD) represents the real-world size of one pixel
    at the ground level: GSD = (sensor_width * altitude) / (focal_length * image_width)
    
    Pixel offsets from image center are then multiplied by GSD to get meter offsets,
    which are rotated by drone heading and added to drone GPS position.
    """
    
    def __init__(self, camera_config: dict = None):
        """
        Initialize GeoTagger with camera calibration parameters.
        
        Args:
            camera_config: Dictionary with camera parameters:
                - sensor_width_mm: Horizontal sensor size in mm
                - focal_length_mm: Camera focal length in mm
                - frame_width: Image width in pixels
                - frame_height: Image height in pixels
                - gimbal_pitch_deg: Gimbal pitch (90 = nadir)
        """
        config = camera_config or {}
        
        # Camera intrinsics (these affect GSD calculation accuracy)
        self.sensor_width_mm = config.get('sensor_width_mm', 7.6)
        self.focal_length_mm = config.get('focal_length_mm', 4.4)
        
        # Image dimensions
        self.frame_width = config.get('frame_width', 1920)
        self.frame_height = config.get('frame_height', 1080)
        
        # Gimbal settings (90 = straight down / nadir)
        self.gimbal_pitch_deg = config.get('gimbal_pitch_deg', 90)
        
        logger.info(f"GeoTagger initialized: sensor={self.sensor_width_mm}mm, "
                   f"focal={self.focal_length_mm}mm, "
                   f"resolution={self.frame_width}x{self.frame_height}, "
                   f"pitch={self.gimbal_pitch_deg}°")
    
    def calculate_gsd(self, altitude_m: float) -> float:
        """
        Calculate Ground Sample Distance (meters per pixel).
        
        GSD = (sensor_width * altitude) / (focal_length * image_width)
        
        Args:
            altitude_m: Drone altitude above ground in meters
            
        Returns:
            GSD in meters per pixel
        """
        if altitude_m <= 0:
            logger.warning("Invalid altitude for GSD calculation")
            return 0.0
        
        # For non-nadir cameras, effective altitude is greater
        if self.gimbal_pitch_deg < 90:
            pitch_rad = math.radians(self.gimbal_pitch_deg)
            if pitch_rad > 0:
                # Effective distance to ground = altitude / sin(pitch)
                effective_distance = altitude_m / math.sin(pitch_rad)
            else:
                effective_distance = altitude_m * 10  # Limit for near-horizontal
        else:
            effective_distance = altitude_m
        
        gsd = (self.sensor_width_mm * effective_distance) / \
              (self.focal_length_mm * self.frame_width)
        
        return gsd
    
    def pixel_to_offset_meters(self, pixel_x: float, pixel_y: float, 
                                altitude_m: float) -> Tuple[float, float, float]:
        """
        Convert pixel position to offset in meters from image center.
        
        Args:
            pixel_x: X coordinate in image (0 = left)
            pixel_y: Y coordinate in image (0 = top)
            altitude_m: Drone altitude above ground
            
        Returns:
            Tuple of (offset_x_m, offset_y_m, gsd) where:
                offset_x_m: Meters right of center (positive = right)
                offset_y_m: Meters forward of center (positive = up/forward in image)
                gsd: Ground sample distance used
        """
        gsd = self.calculate_gsd(altitude_m)
        
        if gsd <= 0:
            return 0.0, 0.0, 0.0
        
        # Image center
        center_x = self.frame_width / 2
        center_y = self.frame_height / 2
        
        # Pixel offset from center
        dx_pixels = pixel_x - center_x
        dy_pixels = center_y - pixel_y  # Inverted: +Y in image is down, but +Y in world is up
        
        # Convert to meters
        offset_x_m = dx_pixels * gsd
        offset_y_m = dy_pixels * gsd
        
        return offset_x_m, offset_y_m, gsd
    
    def rotate_by_heading(self, offset_x_m: float, offset_y_m: float,
                          heading_deg: float) -> Tuple[float, float]:
        """
        Rotate camera-relative offsets to true North/East offsets.
        
        Camera frame: +X is right, +Y is forward (up in image)
        World frame: +X is East, +Y is North
        
        When drone heading is 0° (North), camera +Y aligns with North.
        When drone heading is 90° (East), camera +Y aligns with East.
        
        Args:
            offset_x_m: Offset to right of camera center (meters)
            offset_y_m: Offset forward of camera center (meters)
            heading_deg: Drone heading in degrees (0 = North, 90 = East)
            
        Returns:
            Tuple of (offset_east_m, offset_north_m)
        """
        heading_rad = math.radians(heading_deg)
        
        # Rotation matrix for heading
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        
        # Camera +Y (forward) maps to heading direction
        # Camera +X (right) maps to heading + 90°
        offset_east_m = offset_x_m * cos_h + offset_y_m * sin_h
        offset_north_m = -offset_x_m * sin_h + offset_y_m * cos_h
        
        return offset_east_m, offset_north_m
    
    def offset_to_gps(self, lat: float, lon: float,
                      offset_north_m: float, offset_east_m: float) -> Tuple[float, float]:
        """
        Apply meter offsets to GPS coordinates.
        
        Uses WGS84-aware conversion that accounts for latitude in longitude calculation.
        
        Args:
            lat: Base latitude in degrees
            lon: Base longitude in degrees
            offset_north_m: Offset in north direction (meters)
            offset_east_m: Offset in east direction (meters)
            
        Returns:
            Tuple of (new_lat, new_lon)
        """
        # Meters per degree of latitude (approximately constant)
        meters_per_deg_lat = 111320.0
        
        # Meters per degree of longitude (varies with latitude)
        lat_rad = math.radians(lat)
        meters_per_deg_lon = 111320.0 * math.cos(lat_rad)
        
        if meters_per_deg_lon < 1:  # Near poles
            meters_per_deg_lon = 1
        
        # Apply offsets
        new_lat = lat + (offset_north_m / meters_per_deg_lat)
        new_lon = lon + (offset_east_m / meters_per_deg_lon)
        
        return new_lat, new_lon
    
    def geotag_detection(self, box_xywh: List[float], confidence: float,
                         drone_lat: float, drone_lon: float, drone_alt: float,
                         drone_heading: float = 0.0,
                         track_id: Optional[int] = None) -> GeoTaggedDetection:
        """
        Compute GPS coordinates for a detected object.
        
        Args:
            box_xywh: Detection box [center_x, center_y, width, height] in pixels
            confidence: Detection confidence
            drone_lat: Drone latitude
            drone_lon: Drone longitude
            drone_alt: Drone altitude above ground (meters)
            drone_heading: Drone heading in degrees (0 = North)
            track_id: Optional tracking ID
            
        Returns:
            GeoTaggedDetection with computed target GPS
        """
        result = GeoTaggedDetection(
            box_xywh=box_xywh,
            confidence=confidence,
            track_id=track_id,
            drone_lat=drone_lat,
            drone_lon=drone_lon,
            drone_alt=drone_alt,
            drone_heading=drone_heading
        )
        
        if not all([drone_lat, drone_lon, drone_alt]):
            logger.warning("Missing drone position data for geotagging")
            return result
        
        # Step 1: Convert pixel position to camera-relative offset (meters)
        det_center_x = box_xywh[0]
        det_center_y = box_xywh[1]
        
        offset_x_m, offset_y_m, gsd = self.pixel_to_offset_meters(
            det_center_x, det_center_y, drone_alt
        )
        result.gsd_m = gsd
        
        # Step 2: Rotate by drone heading to get North/East offsets
        offset_east_m, offset_north_m = self.rotate_by_heading(
            offset_x_m, offset_y_m, drone_heading
        )
        result.offset_north_m = offset_north_m
        result.offset_east_m = offset_east_m
        
        # Step 3: Apply offsets to drone GPS
        target_lat, target_lon = self.offset_to_gps(
            drone_lat, drone_lon, offset_north_m, offset_east_m
        )
        result.target_lat = target_lat
        result.target_lon = target_lon
        
        logger.debug(f"Geotagged: pixel=({det_center_x:.0f},{det_center_y:.0f}) "
                    f"-> offset=({offset_east_m:.1f}m E, {offset_north_m:.1f}m N) "
                    f"-> GPS=({target_lat:.6f}, {target_lon:.6f})")
        
        return result
    
    def geotag_detections(self, boxes: List[List[float]], 
                          confidences: List[float],
                          drone_lat: float, drone_lon: float, drone_alt: float,
                          drone_heading: float = 0.0,
                          track_ids: Optional[List[int]] = None) -> List[GeoTaggedDetection]:
        """
        Geotag multiple detections at once.
        
        Args:
            boxes: List of detection boxes [center_x, center_y, width, height]
            confidences: List of confidence values
            drone_lat: Drone latitude
            drone_lon: Drone longitude
            drone_alt: Drone altitude
            drone_heading: Drone heading (degrees, 0=North)
            track_ids: Optional list of track IDs
            
        Returns:
            List of GeoTaggedDetection objects
        """
        results = []
        
        for i, (box, conf) in enumerate(zip(boxes, confidences)):
            track_id = track_ids[i] if track_ids and i < len(track_ids) else None
            
            result = self.geotag_detection(
                box_xywh=box,
                confidence=conf,
                drone_lat=drone_lat,
                drone_lon=drone_lon,
                drone_alt=drone_alt,
                drone_heading=drone_heading,
                track_id=track_id
            )
            results.append(result)
        
        return results
