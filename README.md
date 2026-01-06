# Antigravity Core

**Dual-Drone Mission Control System for Survey and Payload Delivery**

A robust, locally-networked system for coordinating two drones: one surveys an area using YOLO-based human detection, while the other delivers payloads to detected locations.

## 🚁 System Overview

```
┌─────────────────────┐         ┌─────────────────────┐
│   DRONE 1: SURVEYOR │         │  DRONE 2: DELIVERER │
│                     │         │                     │
│  • KML Path Planning│         │  • FIFO Target Queue│
│  • YOLO Detection   │   ZMQ   │  • Payload Servo    │
│  • GPS Transmission │◄───────►│  • Drop Execution   │
└─────────────────────┘         └─────────────────────┘
          │                               │
          │           ┌───────┐           │
          └──────────►│ RELAY │◄──────────┘
                      │(Laptop)│
                      └───────┘
```

## 📂 Project Structure

```
antigravity_core/
├── config/                     # Configuration files
│   ├── mission_params.yaml     # Flight, detection, payload settings
│   ├── network_map.yaml        # Network IPs and ports
│   └── geofence/
│       └── sector_alpha.kml    # Survey area definition
│
├── src/                        # Source code
│   ├── base/                   # Hardware abstraction
│   │   ├── drone_pilot.py      # MAVLink flight control
│   │   └── payload_servo.py    # Drop mechanism driver
│   │
│   ├── intelligence/           # Decision making
│   │   ├── path_finder.py      # KML → Waypoints
│   │   └── human_detector.py   # YOLO detection
│   │
│   ├── comms/                  # Networking
│   │   ├── bridge_client.py    # Drone ZMQ client
│   │   └── relay_server.py     # Ground station server
│   │
│   └── utils/                  # Helpers
│       ├── geo_math.py         # Geospatial calculations
│       └── state_machine.py    # Mission state tracking
│
├── missions/                   # Executables
│   ├── 00_ground_relay.py      # Laptop relay server
│   ├── 01_survey_leader.py     # Drone 1 mission
│   └── 02_delivery_follower.py # Drone 2 mission
│
├── tests/                      # Test suite
├── logs/                       # Auto-generated logs
└── requirements.txt            # Dependencies
```

## 🛠️ Installation

### Prerequisites

- Python 3.9+
- NVIDIA GPU with CUDA (recommended for YOLO)
- MAVLink-compatible flight controllers (ArduPilot/PX4)
- Local WiFi network

### Setup

```bash
# Clone repository
git clone <your-repo-url>
cd antigravity_core

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/macOS
# or: venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt
```

## ⚙️ Configuration

### 1. Network Setup (`config/network_map.yaml`)

Configure static IPs for your local network:

```yaml
ground_relay:
  ip: "192.168.1.100"    # Your laptop's IP
  zmq_port: 5555

drone1:
  mavlink_connection: "udp:127.0.0.1:14550"
  zmq_identity: "SURVEYOR"

drone2:
  mavlink_connection: "udp:127.0.0.1:14551"
  zmq_identity: "DELIVERER"
```

### 2. Mission Parameters (`config/mission_params.yaml`)

Key settings to adjust:

```yaml
flight:
  survey:
    cruise_altitude: 20.0    # Survey height (meters)
  delivery:
    hold_altitude: 25.0      # Loiter height
    
detection:
  model_path: "/path/to/your/weights/best.pt"
  confidence_threshold: 0.70
  
payload:
  initial_count: 10          # Payloads loaded
  drop_altitude: 10.0        # Drop height
```

### 3. Survey Area (`config/geofence/sector_alpha.kml`)

Define your survey polygon in KML format (can be created with Google Earth).

## 🚀 Usage

### Test Mode (No Flight)

```bash
# Test path planning
python missions/01_survey_leader.py --test

# Test delivery logic
python missions/02_delivery_follower.py --test
```

### Full Mission Execution

#### Step 1: Start Ground Relay (Laptop)

```bash
python missions/00_ground_relay.py
```

You'll see an interactive status display monitoring both drones.

#### Step 2: Start Delivery Drone (Drone 2)

```bash
python missions/02_delivery_follower.py
```

Drone 2 will takeoff to holding altitude and wait for coordinates.

#### Step 3: Start Survey Drone (Drone 1)

```bash
python missions/01_survey_leader.py --kml config/geofence/sector_alpha.kml
```

Drone 1 will:
1. Generate sweep pattern from KML
2. Execute survey while detecting humans
3. Send detected coordinates to Drone 2

## 🔧 Running Tests

```bash
# Run all tests
python -m pytest tests/ -v

# Run specific test file
python -m pytest tests/test_kml_parsing.py -v

# Run with coverage
python -m pytest tests/ --cov=src --cov-report=html
```

## 🛡️ Safety Features

### Battery Failsafe
- Continuous battery monitoring
- Automatic RTL below 20% or 14V

### Payload Exhaustion
- Strict count tracking
- Immediate RTL when payload = 0

### Race Condition Handling
- FIFO queue for targets
- Completes current drop before processing next

### Connection Loss
- Heartbeat monitoring
- Auto-reconnect attempts
- Buffered messages

## 📡 Network Architecture

The system uses ZeroMQ (ZMQ) for local network communication:

```
[Drone 1] ──DEALER──► [Relay ROUTER] ──► [Drone 2 DEALER]
                            │
                      Message Logging
                            │
                      Status Display
```

- **ZMQ ROUTER/DEALER**: Async bidirectional communication
- **Heartbeat**: 1-second interval, 5-second timeout
- **Acknowledgments**: Coordinate messages require ACK

## 🗺️ YOLO Model

The system uses your custom-trained YOLOv8 model located at:
```
/home/dj/Projects/Nidar--2025-ELKA-/best_model/dj_yolo_best/weights/best.pt
```

Detection parameters:
- Confidence threshold: 70%
- Target class: Person (ID 0)
- Frame skip: Process every 2nd frame

## 📊 Logs and Outputs

```
logs/
├── flight_logs/          # MAVLink telemetry
├── detections/           # Detection snapshots with bounding boxes
└── relay_*.log           # Message relay logs
```

## 🔍 Troubleshooting

### No GPS Fix
- Ensure GPS modules have clear sky view
- Wait for 3D fix (usually 30-60 seconds)

### MAVLink Connection Failed
- Check connection string in network_map.yaml
- Verify telemetry port forwarding
- Try: `mavlink_connection: "tcp:127.0.0.1:5760"`

### YOLO Detection Not Working
- Verify model path exists
- Check RTSP stream URL
- Ensure GPU/CUDA is available

### ZMQ Connection Issues
- Verify all devices on same network
- Check firewall rules (port 5555)
- Test with: `python -c "import zmq; print(zmq.zmq_version())"`

## 📝 License

MIT License - See LICENSE file for details.

## 👥 Team

Antigravity Core - ELKA 2025

---

**⚠️ IMPORTANT**: Always test in simulation first. Ensure compliance with local drone regulations.
