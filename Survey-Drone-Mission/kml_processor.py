"""
KML Processor - Parse KML files and generate survey waypoints

Handles:
- KML polygon parsing
- Sweep pattern generation
- Waypoint export
"""

import logging
from typing import List, Tuple, Optional
import xml.etree.ElementTree as ET
from dataclasses import dataclass

logger = logging.getLogger(__name__)

try:
    from shapely.geometry import Polygon, LineString
    from shapely.ops import transform as shapely_transform
    import pyproj
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False
    logger.warning("shapely/pyproj not installed - using simple waypoint generation")


@dataclass
class Waypoint:
    """Survey waypoint with coordinates."""
    lat: float
    lon: float
    alt: float
    index: int = 0


class KMLProcessor:
    """
    Parse KML polygon and generate survey waypoints.
    """
    
    def __init__(self, config: dict = None):
        """Initialize KML processor."""
        self.config = config or {}
        self.polygon_coords: List[Tuple[float, float]] = []
        self.waypoints: List[Waypoint] = []
        self.polygon_name = ""
        
        # Default settings
        self.default_spacing = self.config.get('sweep_spacing', 15.0)
        self.default_interval = self.config.get('waypoint_interval', 20.0)
        self.default_altitude = self.config.get('altitude', 25.0)
    
    def load(self, filepath: str) -> bool:
        """
        Load and parse KML file.
        
        Args:
            filepath: Path to KML file
            
        Returns:
            True if polygon loaded successfully
        """
        try:
            tree = ET.parse(filepath)
            root = tree.getroot()
            
            # Handle KML namespace
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            
            # Find coordinates element
            coords_elem = root.find('.//kml:coordinates', ns)
            if coords_elem is None:
                # Try without namespace
                coords_elem = root.find('.//coordinates')
            
            if coords_elem is None:
                logger.error("No coordinates found in KML")
                return False
            
            # Parse coordinates
            coords_text = coords_elem.text.strip()
            self.polygon_coords = []
            
            for coord_str in coords_text.split():
                parts = coord_str.split(',')
                if len(parts) >= 2:
                    lon = float(parts[0])
                    lat = float(parts[1])
                    self.polygon_coords.append((lon, lat))
            
            # Get polygon name
            name_elem = root.find('.//kml:name', ns) or root.find('.//name')
            self.polygon_name = name_elem.text if name_elem is not None else "Survey Area"
            
            logger.info(f"Loaded KML: {self.polygon_name} with {len(self.polygon_coords)} points")
            return len(self.polygon_coords) >= 3
            
        except Exception as e:
            logger.error(f"Failed to load KML: {e}")
            return False
    
    def generate_waypoints(self, 
                          spacing: float = None,
                          interval: float = None,
                          altitude: float = None) -> List[Waypoint]:
        """
        Generate sweep pattern waypoints within the polygon.
        
        Args:
            spacing: Distance between sweep lines (meters)
            interval: Distance between waypoints (meters)
            altitude: Flight altitude (meters)
            
        Returns:
            List of Waypoint objects
        """
        spacing = spacing or self.default_spacing
        interval = interval or self.default_interval
        altitude = altitude or self.default_altitude
        
        if len(self.polygon_coords) < 3:
            logger.error("No polygon loaded")
            return []
        
        self.waypoints = []
        
        if SHAPELY_AVAILABLE:
            self._generate_with_shapely(spacing, interval, altitude)
        else:
            self._generate_simple(altitude)
        
        logger.info(f"Generated {len(self.waypoints)} waypoints")
        return self.waypoints
    
    def _generate_with_shapely(self, spacing: float, interval: float, altitude: float):
        """Generate waypoints using shapely for proper sweep pattern."""
        import math
        
        # Create polygon (convert lon,lat to lat,lon for shapely)
        coords_latlon = [(lat, lon) for lon, lat in self.polygon_coords]
        polygon = Polygon(coords_latlon)
        
        # Get bounding box
        minx, miny, maxx, maxy = polygon.bounds
        
        # Convert spacing to degrees (approximate)
        lat_center = (miny + maxy) / 2
        meters_per_deg_lat = 111320
        meters_per_deg_lon = 111320 * abs(math.cos(math.radians(lat_center)))
        
        spacing_lat = spacing / meters_per_deg_lat
        interval_lon = interval / meters_per_deg_lon
        
        # Generate sweep lines
        current_lat = miny
        line_num = 0
        idx = 0
        
        while current_lat <= maxy:
            # Create horizontal line
            line = LineString([(current_lat, minx - 0.001), (current_lat, maxx + 0.001)])
            
            # Intersect with polygon
            intersection = polygon.intersection(line)
            
            if not intersection.is_empty:
                if intersection.geom_type == 'LineString':
                    segments = [intersection]
                elif intersection.geom_type == 'MultiLineString':
                    segments = list(intersection.geoms)
                else:
                    segments = []
                
                for segment in segments:
                    coords = list(segment.coords)
                    
                    # Alternate direction for efficient path
                    if line_num % 2 == 1:
                        coords = coords[::-1]
                    
                    # Add waypoints along segment
                    for lat, lon in coords:
                        self.waypoints.append(Waypoint(
                            lat=lat, lon=lon, alt=altitude, index=idx
                        ))
                        idx += 1
            
            current_lat += spacing_lat
            line_num += 1
    
    def _generate_simple(self, altitude: float):
        """Simple waypoint generation (polygon corners only)."""
        for idx, (lon, lat) in enumerate(self.polygon_coords):
            self.waypoints.append(Waypoint(
                lat=lat, lon=lon, alt=altitude, index=idx
            ))
    
    def get_waypoints(self) -> List[Tuple[float, float, float]]:
        """Get waypoints as list of (lat, lon, alt) tuples."""
        return [(wp.lat, wp.lon, wp.alt) for wp in self.waypoints]
    
    def get_polygon_center(self) -> Optional[Tuple[float, float]]:
        """Get center of polygon."""
        if not self.polygon_coords:
            return None
        
        lons = [c[0] for c in self.polygon_coords]
        lats = [c[1] for c in self.polygon_coords]
        
        center_lat = sum(lats) / len(lats)
        center_lon = sum(lons) / len(lons)
        
        return (center_lat, center_lon)
    
    def get_waypoint_count(self) -> int:
        """Get number of generated waypoints."""
        return len(self.waypoints)


# Test
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    processor = KMLProcessor()
    if processor.load("sample_data/survey_area.kml"):
        waypoints = processor.generate_waypoints()
        print(f"\nGenerated {len(waypoints)} waypoints:")
        for wp in waypoints[:5]:
            print(f"  {wp.index}: ({wp.lat:.6f}, {wp.lon:.6f}, {wp.alt}m)")
        if len(waypoints) > 5:
            print(f"  ... and {len(waypoints) - 5} more")
