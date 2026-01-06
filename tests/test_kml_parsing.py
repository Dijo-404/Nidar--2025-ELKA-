"""
Test KML Parsing - Validates PathFinder module

Tests:
1. KML polygon parsing
2. Sweep pattern generation
3. Waypoint output format
4. Edge cases
"""

import os
import sys
import pytest
import tempfile
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.intelligence.path_finder import PathFinder


# Sample KML content for testing
SAMPLE_KML = '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <Placemark>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
              77.5900,12.9700,0
              77.5920,12.9700,0
              77.5920,12.9720,0
              77.5900,12.9720,0
              77.5900,12.9700,0
            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
  </Document>
</kml>
'''

INVALID_KML_NO_POLYGON = '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <Placemark>
      <name>No polygon here</name>
    </Placemark>
  </Document>
</kml>
'''


@pytest.fixture
def sample_kml_file():
    """Create a temporary KML file for testing."""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.kml', delete=False) as f:
        f.write(SAMPLE_KML)
        yield f.name
    os.unlink(f.name)


@pytest.fixture
def invalid_kml_file():
    """Create an invalid KML file for testing."""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.kml', delete=False) as f:
        f.write(INVALID_KML_NO_POLYGON)
        yield f.name
    os.unlink(f.name)


class TestKMLParsing:
    """Tests for KML file parsing."""
    
    def test_load_valid_kml(self, sample_kml_file):
        """Test loading a valid KML file."""
        pf = PathFinder()
        result = pf.load_kml(sample_kml_file)
        
        assert result is True
        assert len(pf.polygon_coords) >= 3
        
    def test_load_invalid_kml(self, invalid_kml_file):
        """Test loading KML without polygon."""
        pf = PathFinder()
        result = pf.load_kml(invalid_kml_file)
        
        assert result is False
        
    def test_load_nonexistent_file(self):
        """Test loading a file that doesn't exist."""
        pf = PathFinder()
        result = pf.load_kml('/nonexistent/path/file.kml')
        
        assert result is False
        
    def test_polygon_coordinates_format(self, sample_kml_file):
        """Test that polygon coordinates are in correct format."""
        pf = PathFinder()
        pf.load_kml(sample_kml_file)
        
        for coord in pf.polygon_coords:
            assert len(coord) == 2  # (lon, lat)
            assert isinstance(coord[0], float)
            assert isinstance(coord[1], float)


class TestWaypointGeneration:
    """Tests for sweep pattern waypoint generation."""
    
    def test_generate_waypoints(self, sample_kml_file):
        """Test basic waypoint generation."""
        pf = PathFinder()
        pf.load_kml(sample_kml_file)
        
        waypoints = pf.generate_sweep_pattern(
            spacing=10.0,
            interval=20.0,
            altitude=25.0
        )
        
        assert len(waypoints) > 0
        
    def test_waypoint_format(self, sample_kml_file):
        """Test waypoint output format (lat, lon, alt)."""
        pf = PathFinder()
        pf.load_kml(sample_kml_file)
        
        waypoints = pf.generate_sweep_pattern(altitude=20.0)
        
        for wp in waypoints:
            assert len(wp) == 3  # (lat, lon, alt)
            lat, lon, alt = wp
            
            # Check coordinate ranges
            assert -90 <= lat <= 90, "Latitude out of range"
            assert -180 <= lon <= 180, "Longitude out of range"
            assert alt == 20.0, "Altitude mismatch"
            
    def test_waypoint_altitude(self, sample_kml_file):
        """Test that waypoints have correct altitude."""
        pf = PathFinder()
        pf.load_kml(sample_kml_file)
        
        test_altitude = 35.0
        waypoints = pf.generate_sweep_pattern(altitude=test_altitude)
        
        for wp in waypoints:
            assert wp[2] == test_altitude
            
    def test_spacing_affects_count(self, sample_kml_file):
        """Test that smaller spacing produces more waypoints."""
        pf = PathFinder()
        pf.load_kml(sample_kml_file)
        
        wp_wide = pf.generate_sweep_pattern(spacing=50.0, interval=50.0)
        wp_narrow = pf.generate_sweep_pattern(spacing=10.0, interval=10.0)
        
        assert len(wp_narrow) > len(wp_wide)
        
    def test_no_polygon_loaded(self):
        """Test generating waypoints without loading KML first."""
        pf = PathFinder()
        waypoints = pf.generate_sweep_pattern()
        
        assert len(waypoints) == 0


class TestPolygonMetrics:
    """Tests for polygon area and center calculations."""
    
    def test_polygon_center(self, sample_kml_file):
        """Test polygon centroid calculation."""
        pf = PathFinder()
        pf.load_kml(sample_kml_file)
        
        center = pf.get_polygon_center()
        
        assert center is not None
        lat, lon = center
        
        # Should be within the polygon bounds
        assert 12.97 < lat < 12.98
        assert 77.59 < lon < 77.60
        
    def test_polygon_area(self, sample_kml_file):
        """Test polygon area calculation."""
        pf = PathFinder()
        pf.load_kml(sample_kml_file)
        
        area = pf.get_polygon_area()
        
        # ~220m x 220m = ~48,000 sq m (rough estimate)
        assert area > 0
        assert 30000 < area < 100000  # Reasonable range


class TestKMLExport:
    """Tests for KML waypoint export."""
    
    def test_export_waypoints(self, sample_kml_file):
        """Test exporting waypoints to KML."""
        pf = PathFinder()
        pf.load_kml(sample_kml_file)
        pf.generate_sweep_pattern()
        
        with tempfile.NamedTemporaryFile(suffix='.kml', delete=False) as f:
            output_path = f.name
        
        try:
            result = pf.export_to_kml(output_path)
            
            assert result is True
            assert os.path.exists(output_path)
            
            # Check file has content
            with open(output_path, 'r') as f:
                content = f.read()
                assert '<LineString>' in content
                assert '<coordinates>' in content
        finally:
            os.unlink(output_path)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
