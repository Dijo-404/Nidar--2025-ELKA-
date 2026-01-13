# Survey Drone Mission

Real-time survey mission system with human detection and GPS geotagging.

## Features

- **KML Processing**: Load survey area from KML file, generate sweep waypoints
- **Human Detection**: YOLO-based real-time human detection
- **GPS Geotagging**: Tag detections with GPS coordinates (MAVLink/serial/simulated)
- **Live Display**: Video feed with overlays (GPS, detections, waypoints, FPS)
- **CSV Logging**: Automatic logging of all geotagged detections

## Quick Start

```bash
# Using webcam with simulated GPS
python survey_mission.py --source 0 --simulate-gps

# Using RTSP stream with real GPS
python survey_mission.py --source "rtsp://192.168.144.25:8554/main.264"

# With custom KML survey area
python survey_mission.py --kml sample_data/survey_area.kml --source 0
```

## Files

| File | Description |
|------|-------------|
| `survey_mission.py` | Main entry point |
| `kml_processor.py` | KML parsing & waypoint generation |
| `human_detector.py` | YOLO-based detection |
| `geotagger.py` | GPS coordinate tagging |
| `video_display.py` | Live display with overlays |
| `config.yaml` | Configuration file |

## Command Line Options

```
--config, -c     Path to config file (default: config.yaml)
--source, -s     Video source: camera index, RTSP URL, or file
--kml, -k        Path to KML survey area file
--simulate-gps   Use simulated GPS coordinates
--no-detection   Disable YOLO human detection
--no-gps         Disable GPS (uses simulation)
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| `Q` | Quit |
| `S` | Save screenshot |
| `R` | Reset detection count |
| `N` | Next waypoint (testing) |

## Configuration

Edit `config.yaml` to customize:

```yaml
video:
  source: 0                    # Camera index or RTSP URL

detection:
  model: "yolov8n.pt"          # YOLO model
  confidence: 0.5              # Detection threshold

gps:
  source: "mavlink"            # mavlink, serial, or simulated
  mavlink_connection: "udpin:0.0.0.0:14550"

survey:
  kml_file: "sample_data/survey_area.kml"
  altitude: 25.0
  sweep_spacing: 15.0
```

## Output

- **CSV Log**: `output/detections.csv` - All geotagged detections
- **Screenshots**: `output/screenshot_*.jpg`

## Dependencies

```bash
pip install ultralytics opencv-python pyyaml shapely pyproj pymavlink
```
