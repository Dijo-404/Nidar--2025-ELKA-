"""
GeoMath - Geospatial Calculations

Provides utilities for calculating distances, bearings, and position
estimates for drone navigation.
"""

import math
from typing import Tuple

# Earth's radius in meters
EARTH_RADIUS_M = 6371000.0


def haversine_distance(lat1: float, lon1: float, 
                       lat2: float, lon2: float) -> float:
    """
    Calculate the great-circle distance between two points on Earth.
    
    Uses the Haversine formula for accurate short-distance calculations.
    
    Args:
        lat1: Latitude of point 1 (degrees)
        lon1: Longitude of point 1 (degrees)
        lat2: Latitude of point 2 (degrees)
        lon2: Longitude of point 2 (degrees)
        
    Returns:
        Distance in meters
    """
    # Convert to radians
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    
    # Haversine formula
    a = (math.sin(delta_lat / 2) ** 2 + 
         math.cos(lat1_rad) * math.cos(lat2_rad) * 
         math.sin(delta_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return EARTH_RADIUS_M * c


def bearing(lat1: float, lon1: float, 
            lat2: float, lon2: float) -> float:
    """
    Calculate the initial bearing from point 1 to point 2.
    
    Args:
        lat1: Latitude of point 1 (degrees)
        lon1: Longitude of point 1 (degrees)
        lat2: Latitude of point 2 (degrees)
        lon2: Longitude of point 2 (degrees)
        
    Returns:
        Bearing in degrees (0-360, where 0 is North)
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    
    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
    
    initial_bearing = math.atan2(x, y)
    
    # Convert to degrees and normalize to 0-360
    bearing_deg = math.degrees(initial_bearing)
    return (bearing_deg + 360) % 360


def destination_point(lat: float, lon: float, 
                      bearing_deg: float, distance: float) -> Tuple[float, float]:
    """
    Calculate destination point given start point, bearing and distance.
    
    Args:
        lat: Starting latitude (degrees)
        lon: Starting longitude (degrees)
        bearing_deg: Bearing in degrees (0-360)
        distance: Distance in meters
        
    Returns:
        Tuple of (latitude, longitude) for destination point
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing_deg)
    
    angular_distance = distance / EARTH_RADIUS_M
    
    lat2 = math.asin(
        math.sin(lat_rad) * math.cos(angular_distance) +
        math.cos(lat_rad) * math.sin(angular_distance) * math.cos(bearing_rad)
    )
    
    lon2 = lon_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat_rad),
        math.cos(angular_distance) - math.sin(lat_rad) * math.sin(lat2)
    )
    
    return (math.degrees(lat2), math.degrees(lon2))


def estimate_ground_position(drone_lat: float, drone_lon: float,
                              altitude: float, 
                              camera_pitch: float = 90.0,
                              camera_yaw: float = 0.0) -> Tuple[float, float]:
    """
    Estimate ground position of detected object based on drone position.
    
    For simplicity, this uses nadir projection (directly below the drone)
    when camera is pointing straight down. For angled cameras, it projects
    the position based on altitude and camera angle.
    
    Args:
        drone_lat: Drone latitude (degrees)
        drone_lon: Drone longitude (degrees)
        altitude: Drone altitude above ground (meters)
        camera_pitch: Camera pitch angle (90 = nadir, 0 = horizontal)
        camera_yaw: Camera yaw direction (degrees, 0 = North)
        
    Returns:
        Tuple of (latitude, longitude) for estimated ground position
    """
    # If camera is pointing straight down, return drone position
    if camera_pitch >= 85.0:
        return (drone_lat, drone_lon)
    
    # Calculate horizontal distance to target
    # Using basic trigonometry: distance = altitude / tan(pitch)
    pitch_rad = math.radians(camera_pitch)
    
    if pitch_rad <= 0:
        # Camera pointing at or above horizon, return drone position
        return (drone_lat, drone_lon)
    
    horizontal_distance = altitude / math.tan(pitch_rad)
    
    # Limit to reasonable distance (avoid extreme projections)
    max_distance = altitude * 10  # Max 10x altitude
    horizontal_distance = min(horizontal_distance, max_distance)
    
    # Calculate destination point in the direction of camera yaw
    return destination_point(drone_lat, drone_lon, camera_yaw, horizontal_distance)


def meters_per_degree_lat(latitude: float) -> float:
    """
    Calculate meters per degree of latitude at given latitude.
    
    Args:
        latitude: Latitude in degrees
        
    Returns:
        Meters per degree of latitude
    """
    # At any latitude, 1 degree of latitude is approximately 111km
    return 111320.0


def meters_per_degree_lon(latitude: float) -> float:
    """
    Calculate meters per degree of longitude at given latitude.
    
    Args:
        latitude: Latitude in degrees
        
    Returns:
        Meters per degree of longitude
    """
    lat_rad = math.radians(latitude)
    return 111320.0 * math.cos(lat_rad)


def offset_position(lat: float, lon: float,
                    north_meters: float, east_meters: float) -> Tuple[float, float]:
    """
    Offset a position by meters in north and east directions.
    
    Args:
        lat: Base latitude (degrees)
        lon: Base longitude (degrees)
        north_meters: Offset in north direction (meters)
        east_meters: Offset in east direction (meters)
        
    Returns:
        Tuple of (new_latitude, new_longitude)
    """
    lat_offset = north_meters / meters_per_degree_lat(lat)
    lon_offset = east_meters / meters_per_degree_lon(lat)
    
    return (lat + lat_offset, lon + lon_offset)


def is_within_radius(lat1: float, lon1: float,
                     lat2: float, lon2: float,
                     radius: float) -> bool:
    """
    Check if point 2 is within a radius of point 1.
    
    Args:
        lat1: Center latitude (degrees)
        lon1: Center longitude (degrees)
        lat2: Point latitude (degrees)
        lon2: Point longitude (degrees)
        radius: Radius in meters
        
    Returns:
        True if point 2 is within radius of point 1
    """
    distance = haversine_distance(lat1, lon1, lat2, lon2)
    return distance <= radius


def bounding_box(lat: float, lon: float, 
                 radius: float) -> Tuple[float, float, float, float]:
    """
    Calculate a bounding box around a point.
    
    Args:
        lat: Center latitude (degrees)
        lon: Center longitude (degrees)
        radius: Radius in meters
        
    Returns:
        Tuple of (min_lat, min_lon, max_lat, max_lon)
    """
    lat_delta = radius / meters_per_degree_lat(lat)
    lon_delta = radius / meters_per_degree_lon(lat)
    
    return (
        lat - lat_delta,
        lon - lon_delta,
        lat + lat_delta,
        lon + lon_delta
    )
