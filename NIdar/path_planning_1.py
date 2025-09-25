import xml.etree.ElementTree as ET
import argparse
from shapely.geometry import Polygon, LineString
from shapely.ops import transform as shapely_transform
import pyproj

# Configuration constants
SPACING = 7.0  # meters between sweep lines
WAYPOINT_INTERVAL = 10.0  # meters between waypoints
ALTITUDE = 20.0  # flight altitude in meters

def parse_kml_polygon(kml_file):
    """Parse the first Polygon in a KML file and return coordinates."""
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    tree = ET.parse(kml_file)
    root = tree.getroot()
    
    polygon_elem = root.find('.//kml:Polygon', ns)
    if polygon_elem is None:
        raise ValueError("No Polygon found in KML file.")
    
    coords_elem = polygon_elem.find('.//kml:coordinates', ns)
    if coords_elem is None or not coords_elem.text.strip():
        raise ValueError("No coordinates found in Polygon.")
    
    coords_text = coords_elem.text.strip()
    coord_list = []
    
    for coord_str in coords_text.split():
        parts = coord_str.split(',')
        if len(parts) >= 2:
            coord_list.append((float(parts[0]), float(parts[1])))
    
    if len(coord_list) < 3:
        raise ValueError("Polygon must have at least 3 coordinates.")
    
    return coord_list

def get_utm_crs(lat, lon):
    """Return appropriate UTM CRS string for given coordinates."""
    zone_number = int((lon + 180) / 6) + 1
    return f"EPSG:{32600 + zone_number if lat >= 0 else 32700 + zone_number}"

def generate_sweep_waypoints(polygon_coords, spacing=SPACING, waypoint_interval=WAYPOINT_INTERVAL, altitude=ALTITUDE):
    """Generate horizontal sweep pattern within polygon."""
    poly_lonlat = Polygon(polygon_coords)
    if not poly_lonlat.is_valid or poly_lonlat.is_empty:
        raise ValueError("Invalid polygon geometry.")
    
    # Project to UTM for accurate distance calculations
    centroid = poly_lonlat.centroid
    utm_crs = get_utm_crs(centroid.y, centroid.x)
    
    project_to_utm = pyproj.Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True).transform
    project_to_latlon = pyproj.Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True).transform
    
    poly_utm = shapely_transform(project_to_utm, poly_lonlat)
    minx, miny, maxx, maxy = poly_utm.bounds
    
    # Generate horizontal sweep lines
    lines = []
    y = miny
    while y <= maxy:
        line = LineString([(minx, y), (maxx, y)])
        segment = line.intersection(poly_utm)
        
        if not segment.is_empty:
            if segment.geom_type == 'LineString':
                lines.append(segment)
            elif segment.geom_type == 'MultiLineString':
                lines.extend(segment.geoms)
        
        y += spacing
    
    # Sort lines by y-coordinate and generate waypoints
    lines.sort(key=lambda l: l.centroid.y)
    waypoints = []
    reverse = False
    
    for line in lines:
        length = line.length
        num_points = max(int(length // waypoint_interval) + 1, 2)
        
        segment_points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            pt = line.interpolate(t * length)
            lon, lat = project_to_latlon(pt.x, pt.y)
            segment_points.append((lon, lat, altitude))
        
        if reverse:
            segment_points.reverse()
        
        waypoints.extend(segment_points)
        reverse = not reverse
    
    return waypoints

def write_text_waypoints(output_file, waypoints):
    """Write waypoints to a text file."""
    with open(output_file, 'w') as f:
        f.write("# Generated Flight Path Waypoints\n")
        f.write("# Format: Longitude, Latitude, Altitude\n")
        f.write(f"# Total waypoints: {len(waypoints)}\n")
        f.write("# Longitude\tLatitude\tAltitude\n")
        
        for i, (lon, lat, alt) in enumerate(waypoints, 1):
            f.write(f"{lon:.8f}\t{lat:.8f}\t{alt:.2f}\n")
    
    print(f"Flight path with {len(waypoints)} waypoints written to {output_file}")

def write_kml_waypoints(output_file, waypoints):
    """Write waypoints to a KML file as a LineString."""
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
    coords_text = " ".join(f"{lon},{lat},{alt}" for lon, lat, alt in waypoints)
    coords_elem.text = coords_text
    
    tree = ET.ElementTree(kml_elem)
    tree.write(output_file, encoding="utf-8", xml_declaration=True)
    print(f"KML flight path written to {output_file}")

def main():
    parser = argparse.ArgumentParser(description="Generate horizontal sweep flight path from KML polygon.")
    parser.add_argument("input_kml", help="Input KML file containing a Polygon definition")
    parser.add_argument("output_file", help="Output file for the generated flight path")
    parser.add_argument("--format", choices=['txt', 'kml'], default='txt', 
                       help="Output format (default: txt)")
    args = parser.parse_args()
    try:
        polygon_coords = parse_kml_polygon(args.input_kml)
        waypoints = generate_sweep_waypoints(polygon_coords)
        
        if args.format == 'txt':
            write_text_waypoints(args.output_file, waypoints)
        else:
            write_kml_waypoints(args.output_file, waypoints)
            
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    main()