"""
PathFinder - KML to Waypoint Conversion

Parses KML polygon files and generates optimal sweep (lawnmower) patterns
for area survey missions.
"""

import logging
from typing import List, Tuple, Optional
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon, LineString
from shapely.ops import transform as shapely_transform
import pyproj

logger = logging.getLogger(__name__)


class PathFinder:
    """
    Converts KML polygon definitions to flight waypoints.
    
    Generates horizontal sweep patterns optimized for camera coverage
    with configurable spacing and waypoint intervals.
    """
    
    def __init__(self, config: dict = None):
        """
        Initialize PathFinder with optional configuration.
        
        Args:
            config: Configuration dictionary with planning parameters
        """
        self.config = config or {}
        
        # Default planning parameters
        self.sweep_spacing = self.config.get('sweep_spacing', 7.0)
        self.waypoint_interval = self.config.get('waypoint_interval', 10.0)
        self.default_altitude = self.config.get('cruise_altitude', 20.0)
        
        # State
        self.polygon_coords: List[Tuple[float, float]] = []
        self.waypoints: List[Tuple[float, float, float]] = []
        self.kml_file: Optional[str] = None
        
    def load_kml(self, filepath: str) -> bool:
        """
        Parse KML file and extract polygon coordinates.
        
        Args:
            filepath: Path to KML file
            
        Returns:
            True if polygon parsed successfully
        """
        try:
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            tree = ET.parse(filepath)
            root = tree.getroot()
            
            # Find polygon element
            polygon_elem = root.find('.//kml:Polygon', ns)
            if polygon_elem is None:
                logger.error("No Polygon found in KML file")
                return False
            
            # Extract coordinates
            coords_elem = polygon_elem.find('.//kml:coordinates', ns)
            if coords_elem is None or not coords_elem.text.strip():
                logger.error("No coordinates found in Polygon")
                return False
            
            coords_text = coords_elem.text.strip()
            coord_list = []
            
            for coord_str in coords_text.split():
                parts = coord_str.split(',')
                if len(parts) < 2:
                    continue
                lon = float(parts[0])
                lat = float(parts[1])
                coord_list.append((lon, lat))
            
            if len(coord_list) < 3:
                logger.error("Polygon must have at least 3 coordinates")
                return False
            
            self.polygon_coords = coord_list
            self.kml_file = filepath
            logger.info(f"Loaded polygon with {len(coord_list)} vertices from {filepath}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to parse KML: {e}")
            return False
    
    def _get_utm_crs(self, lat: float, lon: float) -> str:
        """
        Determine appropriate UTM CRS for given coordinates.
        
        Args:
            lat: Latitude
            lon: Longitude
            
        Returns:
            EPSG code string for UTM zone
        """
        zone_number = int((lon + 180) / 6) + 1
        if lat >= 0:
            return f"EPSG:{32600 + zone_number}"
        else:
            return f"EPSG:{32700 + zone_number}"
    
    def generate_sweep_pattern(self, spacing: float = None, 
                                interval: float = None,
                                altitude: float = None) -> List[Tuple[float, float, float]]:
        """
        Generate horizontal sweep (lawnmower) pattern within the polygon.
        
        Args:
            spacing: Distance between sweep lines in meters (default from config)
            interval: Distance between waypoints along each line in meters
            altitude: Flight altitude in meters
            
        Returns:
            List of (latitude, longitude, altitude) waypoint tuples
        """
        if not self.polygon_coords:
            logger.error("No polygon loaded - call load_kml first")
            return []
        
        # Use provided values or defaults
        spacing = spacing or self.sweep_spacing
        interval = interval or self.waypoint_interval
        altitude = altitude or self.default_altitude
        
        try:
            # Create shapely polygon (lon, lat format)
            poly_lonlat = Polygon(self.polygon_coords)
            if not poly_lonlat.is_valid or poly_lonlat.is_empty:
                logger.error("Invalid polygon geometry")
                return []
            
            # Get centroid for UTM zone determination
            centroid = poly_lonlat.centroid
            utm_crs = self._get_utm_crs(centroid.y, centroid.x)
            
            # Create coordinate transformers
            project_to_utm = pyproj.Transformer.from_crs(
                "EPSG:4326", utm_crs, always_xy=True).transform
            project_to_latlon = pyproj.Transformer.from_crs(
                utm_crs, "EPSG:4326", always_xy=True).transform
            
            # Transform polygon to UTM for metric calculations
            poly_utm = shapely_transform(project_to_utm, poly_lonlat)
            minx, miny, maxx, maxy = poly_utm.bounds
            
            # Generate horizontal sweep lines
            y = miny
            lines = []
            while y <= maxy:
                line = LineString([(minx, y), (maxx, y)])
                segment = line.intersection(poly_utm)
                
                if not segment.is_empty:
                    if segment.geom_type == 'LineString':
                        lines.append(segment)
                    elif segment.geom_type == 'MultiLineString':
                        for seg in segment.geoms:
                            lines.append(seg)
                y += spacing
            
            # Sort lines by Y coordinate
            lines = sorted(lines, key=lambda l: l.centroid.y)
            
            # Generate waypoints with alternating direction (boustrophedon)
            waypoints = []
            reverse = False
            
            for line in lines:
                length = line.length
                num_points = max(int(length // interval) + 1, 2)
                segment_points = []
                
                for i in range(num_points):
                    t = i / (num_points - 1)
                    pt = line.interpolate(t * length)
                    lon, lat = project_to_latlon(pt.x, pt.y)
                    # Return as (lat, lon, alt) for navigation
                    segment_points.append((lat, lon, altitude))
                
                if reverse:
                    segment_points.reverse()
                
                waypoints.extend(segment_points)
                reverse = not reverse
            
            self.waypoints = waypoints
            logger.info(f"Generated {len(waypoints)} waypoints with {spacing}m spacing")
            
            return waypoints
            
        except Exception as e:
            logger.error(f"Failed to generate sweep pattern: {e}")
            return []
    
    def get_waypoints(self) -> List[Tuple[float, float, float]]:
        """
        Get the generated waypoints.
        
        Returns:
            List of (latitude, longitude, altitude) tuples
        """
        return self.waypoints
    
    def get_polygon_center(self) -> Optional[Tuple[float, float]]:
        """
        Get the centroid of the loaded polygon.
        
        Returns:
            Tuple of (latitude, longitude) or None
        """
        if not self.polygon_coords:
            return None
        
        poly = Polygon(self.polygon_coords)
        centroid = poly.centroid
        return (centroid.y, centroid.x)  # Return as (lat, lon)
    
    def get_polygon_area(self) -> float:
        """
        Calculate approximate area of the polygon in square meters.
        
        Returns:
            Area in square meters
        """
        if not self.polygon_coords:
            return 0.0
        
        poly_lonlat = Polygon(self.polygon_coords)
        centroid = poly_lonlat.centroid
        utm_crs = self._get_utm_crs(centroid.y, centroid.x)
        
        project_to_utm = pyproj.Transformer.from_crs(
            "EPSG:4326", utm_crs, always_xy=True).transform
        poly_utm = shapely_transform(project_to_utm, poly_lonlat)
        
        return poly_utm.area
    
    def estimate_flight_time(self, speed: float = 5.0) -> float:
        """
        Estimate total flight time for the path.
        
        Args:
            speed: Cruise speed in m/s
            
        Returns:
            Estimated time in seconds
        """
        if len(self.waypoints) < 2:
            return 0.0
        
        from src.utils.geo_math import haversine_distance
        
        total_distance = 0.0
        for i in range(1, len(self.waypoints)):
            lat1, lon1, _ = self.waypoints[i-1]
            lat2, lon2, _ = self.waypoints[i]
            total_distance += haversine_distance(lat1, lon1, lat2, lon2)
        
        return total_distance / speed
    
    def export_to_kml(self, output_file: str) -> bool:
        """
        Export generated waypoints to a KML file.
        
        Args:
            output_file: Path for output KML file
            
        Returns:
            True if export successful
        """
        if not self.waypoints:
            logger.error("No waypoints to export")
            return False
        
        try:
            kml_ns = "http://www.opengis.net/kml/2.2"
            ET.register_namespace('', kml_ns)
            
            kml_elem = ET.Element(f"{{{kml_ns}}}kml")
            doc_elem = ET.SubElement(kml_elem, "Document")
            
            placemark = ET.SubElement(doc_elem, "Placemark")
            name = ET.SubElement(placemark, "name")
            name.text = "Generated Flight Path"
            
            ls = ET.SubElement(placemark, "LineString")
            tessellate = ET.SubElement(ls, "tessellate")
            tessellate.text = "1"
            altitudeMode = ET.SubElement(ls, "altitudeMode")
            altitudeMode.text = "relativeToGround"
            
            coords_elem = ET.SubElement(ls, "coordinates")
            # KML uses lon,lat,alt format
            coords_text = " ".join(
                f"{lon},{lat},{alt}" for lat, lon, alt in self.waypoints
            )
            coords_elem.text = coords_text
            
            tree = ET.ElementTree(kml_elem)
            tree.write(output_file, encoding="utf-8", xml_declaration=True)
            
            logger.info(f"Exported flight path to {output_file}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to export KML: {e}")
            return False
