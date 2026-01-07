#!/usr/bin/env python3
"""
Unit tests for GeoTagger - GSD-based detection coordinate calculation.

Tests:
1. GSD calculation at various altitudes
2. Center detection returns drone GPS
3. Offset detection returns correct GPS offset
4. Heading rotation works correctly
"""

import sys
import math
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.intelligence.geotagging import GeoTagger, GeoTaggedDetection


def test_gsd_calculation():
    """Test GSD calculation at various altitudes."""
    print("\n=== Test: GSD Calculation ===")
    
    geotagger = GeoTagger({
        'sensor_width_mm': 7.6,
        'focal_length_mm': 4.4,
        'frame_width': 1920,
        'frame_height': 1080
    })
    
    # Test at 10m altitude
    gsd_10m = geotagger.calculate_gsd(10.0)
    expected_10m = (7.6 * 10) / (4.4 * 1920)  # ~0.00899
    print(f"  GSD at 10m: {gsd_10m:.6f} m/px (expected: {expected_10m:.6f})")
    assert abs(gsd_10m - expected_10m) < 0.0001, "GSD at 10m incorrect"
    
    # Test at 50m altitude
    gsd_50m = geotagger.calculate_gsd(50.0)
    expected_50m = (7.6 * 50) / (4.4 * 1920)  # ~0.0449
    print(f"  GSD at 50m: {gsd_50m:.6f} m/px (expected: {expected_50m:.6f})")
    assert abs(gsd_50m - expected_50m) < 0.001, "GSD at 50m incorrect"
    
    # Verify linear relationship
    ratio = gsd_50m / gsd_10m
    print(f"  GSD ratio (50m/10m): {ratio:.2f} (expected: 5.00)")
    assert abs(ratio - 5.0) < 0.01, "GSD ratio should be 5x"
    
    print("  [PASS] GSD calculation")
    return True


def test_center_detection():
    """Detection at image center should return drone GPS."""
    print("\n=== Test: Center Detection ===")
    
    geotagger = GeoTagger({
        'sensor_width_mm': 7.6,
        'focal_length_mm': 4.4,
        'frame_width': 1920,
        'frame_height': 1080
    })
    
    # Detection exactly at center
    center_x = 1920 / 2  # 960
    center_y = 1080 / 2  # 540
    
    drone_lat = 12.9716
    drone_lon = 77.5946
    drone_alt = 20.0
    
    result = geotagger.geotag_detection(
        box_xywh=[center_x, center_y, 100, 200],
        confidence=0.9,
        drone_lat=drone_lat,
        drone_lon=drone_lon,
        drone_alt=drone_alt,
        drone_heading=0.0
    )
    
    print(f"  Drone:  ({drone_lat:.6f}, {drone_lon:.6f})")
    print(f"  Target: ({result.target_lat:.6f}, {result.target_lon:.6f})")
    print(f"  Offset: N={result.offset_north_m:.3f}m, E={result.offset_east_m:.3f}m")
    
    # Should be very close to drone position (< 1mm offset)
    assert abs(result.target_lat - drone_lat) < 1e-8, "Center lat should match drone"
    assert abs(result.target_lon - drone_lon) < 1e-8, "Center lon should match drone"
    
    print("  [PASS] Center detection")
    return True


def test_offset_detection():
    """Detection offset from center should compute correct GPS offset."""
    print("\n=== Test: Offset Detection ===")
    
    geotagger = GeoTagger({
        'sensor_width_mm': 7.6,
        'focal_length_mm': 4.4,
        'frame_width': 1920,
        'frame_height': 1080
    })
    
    # Detection 100 pixels to the right of center (heading North)
    center_x = 1920 / 2 + 100  # 1060
    center_y = 1080 / 2        # 540
    
    drone_lat = 12.9716
    drone_lon = 77.5946
    drone_alt = 20.0
    
    result = geotagger.geotag_detection(
        box_xywh=[center_x, center_y, 100, 200],
        confidence=0.9,
        drone_lat=drone_lat,
        drone_lon=drone_lon,
        drone_alt=drone_alt,
        drone_heading=0.0  # Facing North
    )
    
    # GSD at 20m
    gsd = geotagger.calculate_gsd(20.0)
    expected_east_offset = 100 * gsd  # Should be ~1.8m East
    
    print(f"  GSD at 20m: {gsd:.4f} m/px")
    print(f"  Expected East offset: {expected_east_offset:.2f}m")
    print(f"  Actual offset: N={result.offset_north_m:.3f}m, E={result.offset_east_m:.3f}m")
    
    # Should have positive East offset, near-zero North offset
    assert abs(result.offset_east_m - expected_east_offset) < 0.01, "East offset incorrect"
    assert abs(result.offset_north_m) < 0.01, "North offset should be ~0"
    
    # Longitude should increase (East is positive)
    assert result.target_lon > drone_lon, "Target should be East of drone"
    
    print("  [PASS] Offset detection")
    return True


def test_heading_rotation():
    """Heading rotation should correctly map camera frame to world frame."""
    print("\n=== Test: Heading Rotation ===")
    
    geotagger = GeoTagger({
        'sensor_width_mm': 7.6,
        'focal_length_mm': 4.4,
        'frame_width': 1920,
        'frame_height': 1080
    })
    
    drone_lat = 12.9716
    drone_lon = 77.5946
    drone_alt = 20.0
    
    # Detection 100 pixels FORWARD (up in image, which is toward camera direction)
    # At heading 90° (East), forward should become East offset
    center_x = 1920 / 2        # center
    center_y = 1080 / 2 - 100  # 100 pixels UP (forward)
    
    result_east = geotagger.geotag_detection(
        box_xywh=[center_x, center_y, 100, 200],
        confidence=0.9,
        drone_lat=drone_lat,
        drone_lon=drone_lon,
        drone_alt=drone_alt,
        drone_heading=90.0  # Facing East
    )
    
    print(f"  Heading 90° (East): Forward in camera -> East in world")
    print(f"  Offset: N={result_east.offset_north_m:.3f}m, E={result_east.offset_east_m:.3f}m")
    
    # Forward (up in image) at heading 90° should become East offset
    assert result_east.offset_east_m > 0.5, "Should have positive East offset"
    assert abs(result_east.offset_north_m) < 0.5, "North offset should be ~0"
    
    # Test heading 180° (South) - forward should become South (negative North)
    result_south = geotagger.geotag_detection(
        box_xywh=[center_x, center_y, 100, 200],
        confidence=0.9,
        drone_lat=drone_lat,
        drone_lon=drone_lon,
        drone_alt=drone_alt,
        drone_heading=180.0  # Facing South
    )
    
    print(f"  Heading 180° (South): Forward in camera -> South in world")
    print(f"  Offset: N={result_south.offset_north_m:.3f}m, E={result_south.offset_east_m:.3f}m")
    
    # Forward at heading 180° should become negative North (South)
    assert result_south.offset_north_m < -0.5, "Should have negative North (South) offset"
    
    print("  [PASS] Heading rotation")
    return True


def test_multiple_detections():
    """Test geotagging multiple detections at once."""
    print("\n=== Test: Multiple Detections ===")
    
    geotagger = GeoTagger({
        'frame_width': 1920,
        'frame_height': 1080
    })
    
    boxes = [
        [960, 540, 100, 200],  # Center
        [1060, 540, 100, 200], # Right of center
        [860, 540, 100, 200],  # Left of center
    ]
    confidences = [0.9, 0.8, 0.7]
    
    results = geotagger.geotag_detections(
        boxes=boxes,
        confidences=confidences,
        drone_lat=12.9716,
        drone_lon=77.5946,
        drone_alt=20.0,
        drone_heading=0.0
    )
    
    print(f"  Geotagged {len(results)} detections")
    for i, r in enumerate(results):
        print(f"    [{i}] ({r.target_lat:.6f}, {r.target_lon:.6f}) conf={r.confidence}")
    
    assert len(results) == 3, "Should have 3 results"
    assert results[0].target_lon < results[1].target_lon, "Right detection should have higher lon"
    assert results[2].target_lon < results[0].target_lon, "Left detection should have lower lon"
    
    print("  [PASS] Multiple detections")
    return True


def run_all_tests():
    """Run all geotagging tests."""
    print("=" * 60)
    print("GEOTAGGING UNIT TESTS")
    print("=" * 60)
    
    tests = [
        test_gsd_calculation,
        test_center_detection,
        test_offset_detection,
        test_heading_rotation,
        test_multiple_detections,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
        except AssertionError as e:
            print(f"  [FAIL] {e}")
            failed += 1
        except Exception as e:
            print(f"  [ERROR] {e}")
            failed += 1
    
    print("\n" + "=" * 60)
    print(f"RESULTS: {passed} passed, {failed} failed")
    print("=" * 60)
    
    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
