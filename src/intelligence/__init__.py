# Intelligence Layer - Decision Making
"""
Decision making modules for path planning and human detection.

Modules:
    - path_finder: KML parsing and waypoint generation
    - human_detector: YOLO-based human detection
    - geotagging: GSD-based detection coordinate calculation
"""

# Lazy imports to avoid dependency issues
def __getattr__(name):
    """Lazy import to avoid loading all dependencies at once."""
    if name == "PathFinder":
        from .path_finder import PathFinder
        return PathFinder
    elif name == "HumanDetector":
        from .human_detector import HumanDetector
        return HumanDetector
    elif name == "HumanTracker":
        from .human_tracker import HumanTracker
        return HumanTracker
    elif name == "DroneTrackerConfig":
        from .human_tracker import DroneTrackerConfig
        return DroneTrackerConfig
    elif name == "GeoTagger":
        from .geotagging import GeoTagger
        return GeoTagger
    elif name == "GeoTaggedDetection":
        from .geotagging import GeoTaggedDetection
        return GeoTaggedDetection
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = ["PathFinder", "HumanDetector", "HumanTracker", "DroneTrackerConfig", "GeoTagger", "GeoTaggedDetection"]

