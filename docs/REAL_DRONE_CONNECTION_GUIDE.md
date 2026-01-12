# Real Drone Connection Guide

Complete step-by-step guide for connecting and controlling two physical drones with the Nidar system.

---

## Prerequisites

### Hardware Required

| Component | Drone 1 (Surveyor) | Drone 2 (Deliverer) |
|-----------|-------------------|---------------------|
| Flight Controller | Pixhawk / Cube | Pixhawk / Cube |
| Telemetry Radio | SiK Radio / WiFi | SiK Radio / WiFi |
| Video Transmitter | SIYI MK15 / A8 Mini | Optional |
| Companion Computer | Raspberry Pi 4 (Optional) | Raspberry Pi 4 (Optional) |

### Software Required

```bash
# Install dependencies
conda activate Nidar
pip install pymavlink pyserial opencv-python
```

---

## Network Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         SIYI TRANSMITTER HOTSPOT                            │
│                         Network: 192.168.144.0/24                           │
│                         SSID: SIYI_LINK (or similar)                        │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
          ┌───────────────────────────┼───────────────────────────┐
          │                           │                           │
          ▼                           ▼                           ▼
┌─────────────────────┐   ┌─────────────────────┐   ┌─────────────────────┐
│      DRONE 1        │   │   GROUND STATION    │   │      DRONE 2        │
│     (Surveyor)      │   │      (Laptop)       │   │    (Deliverer)      │
│                     │   │                     │   │                     │
│ IP: 192.168.144.25  │   │ IP: 192.168.144.100 │   │ IP: 192.168.144.26  │
│ MAVLink: UDP 14550  │   │ ZMQ Relay: 5555     │   │ MAVLink: UDP 14551  │
│ RTSP: 8554          │   │ GCS Ports: 14560/61 │   │                     │
│ SysID: 1            │   │                     │   │ SysID: 2            │
└─────────────────────┘   └─────────────────────┘   └─────────────────────┘
```

---

## Step 1: Flight Controller Configuration

### Configure MAVLink System IDs

Each drone must have a unique System ID. Connect via Mission Planner or QGroundControl:

**Drone 1 (Surveyor):**
```
SYSID_THISMAV = 1
```

**Drone 2 (Deliverer):**
```
SYSID_THISMAV = 2
```

### Enable MAVLink Telemetry

Set these parameters on each flight controller:

```
SERIAL1_PROTOCOL = 2    # MAVLink2
SERIAL1_BAUD = 57       # 57600 baud
SR1_POSITION = 10       # Position messages at 10Hz
SR1_EXT_STAT = 2        # Extended status at 2Hz
SR1_EXTRA1 = 10         # Attitude at 10Hz
SR1_EXTRA2 = 10         # VFR HUD at 10Hz
SR1_EXTRA3 = 2          # AHRS, battery at 2Hz
```

---

## Step 2: Telemetry Connection Options

### Option A: USB Serial (SiK Telemetry Radios)

Connect USB telemetry receivers to your laptop:

```bash
# Check connected devices
ls /dev/ttyUSB*

# Expected output:
# /dev/ttyUSB0  <- Drone 1
# /dev/ttyUSB1  <- Drone 2
```

**Fix Permission Issues:**
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

**Connection Strings:**
```
Drone 1: /dev/ttyUSB0
Drone 2: /dev/ttyUSB1
```

### Option B: WiFi/UDP (SIYI or ESP WiFi)

If using WiFi-based telemetry:

| Drone | IP Address | MAVLink Port |
|-------|------------|--------------|
| Drone 1 | 192.168.144.25 | 14550 |
| Drone 2 | 192.168.144.26 | 14551 |

**Connection Strings:**
```
Drone 1: udp:192.168.144.25:14550
Drone 2: udp:192.168.144.26:14551
```

### Option C: Mixed (Serial + UDP)

You can mix connection types:
```
Drone 1: /dev/ttyUSB0
Drone 2: udp:192.168.144.26:14551
```

---

## Step 3: Ground Station Network Setup

### Connect to SIYI Hotspot

1. Power on SIYI transmitter
2. Connect laptop to SIYI WiFi network
3. Verify connection:
   ```bash
   ip addr show | grep "192.168.144"
   ```

### Set Static IP (Recommended)

**Using nmcli:**
```bash
# Get connection name
nmcli connection show

# Set static IP
nmcli connection modify "SIYI_LINK" ipv4.addresses 192.168.144.100/24
nmcli connection modify "SIYI_LINK" ipv4.gateway 192.168.144.1
nmcli connection modify "SIYI_LINK" ipv4.method manual
nmcli connection up "SIYI_LINK"
```

**Using GUI:**
1. Open WiFi Settings
2. Click gear icon next to SIYI network
3. Go to IPv4 tab
4. Set Method: Manual
5. Add address: `192.168.144.100`, Netmask: `255.255.255.0`
6. Apply and reconnect

---

## Step 4: Configuration Files

### Update network_map.yaml

Edit `config/network_map.yaml`:

```yaml
# Ground Relay Server (Your laptop)
ground_relay:
  ip: "192.168.144.100"
  zmq_port: 5555
  status_port: 5556
  heartbeat_interval: 1.0
  heartbeat_timeout: 5.0

# Ground Control Station
gcs:
  enabled: true
  ip: "192.168.144.100"
  drone1_port: 14560
  drone2_port: 14561
  forward_all_messages: true

# Drone 1: The Surveyor
drone1:
  name: "surveyor"
  id: "DRONE_001"
  mavlink_connection: "udp:192.168.144.25:14550"  # WiFi
  # mavlink_connection: "/dev/ttyUSB0"            # Serial
  zmq_identity: "SURVEYOR"
  sysid: 1

# Drone 2: The Deliverer
drone2:
  name: "deliverer"
  id: "DRONE_002"
  mavlink_connection: "udp:192.168.144.26:14551"  # WiFi
  # mavlink_connection: "/dev/ttyUSB1"            # Serial
  zmq_identity: "DELIVERER"
  sysid: 2
```

### Update mission_params.yaml

Edit `config/mission_params.yaml`:

```yaml
# Camera settings for Drone 1
camera:
  rtsp_url: "rtsp://192.168.144.25:8554/main.264"
  sensor_width_mm: 7.6
  focal_length_mm: 4.4
  gimbal_pitch_deg: 90

# Detection settings
detection:
  model_path: "best_model/dj_yolo_best/weights/best.pt"
  confidence_threshold: 0.70

# Safety settings
safety:
  min_battery_voltage: 14.0     # 4S battery minimum
  min_battery_percent: 20
  max_distance_from_home: 500   # meters
  failsafe_rtl_altitude: 30.0
```

---

## Step 5: Verify Connections

### Test Network Connectivity

```bash
# Ping each drone
ping -c 3 192.168.144.25
ping -c 3 192.168.144.26

# Expected: < 50ms latency, 0% packet loss
```

### Test Video Feed

```bash
# Install ffplay if not available
sudo apt install ffmpeg

# Test RTSP stream
ffplay rtsp://192.168.144.25:8554/main.264
```

### Test MAVLink Connection

```bash
cd /home/dj/Projects/Nidar--2025-ELKA-

# Test with Serial
python tests/test_dual_drone_connection.py \
  --drone1 /dev/ttyUSB0 \
  --drone2 /dev/ttyUSB1

# Test with UDP
python tests/test_dual_drone_connection.py \
  --drone1 udp:192.168.144.25:14550 \
  --drone2 udp:192.168.144.26:14551
```

**Expected Output:**
```
============================================================
DUAL DRONE CONNECTION & ARMING TEST
============================================================
Mode: REAL FLIGHT

[STEP 1] Connecting to drones...
----------------------------------------
[Drone 1] Connecting to udp:192.168.144.25:14550...
[Drone 2] Connecting to udp:192.168.144.26:14551...
[Drone 1] ✓ Connected! (System: 1, Component: 1)
[Drone 2] ✓ Connected! (System: 2, Component: 1)

[STEP 2] Checking system status...
----------------------------------------
Drone 1 Status:
  Mode: STABILIZE
  Armed: False
  Battery: 16.45V (87%)
  GPS: Fix=3, Satellites=12

Drone 2 Status:
  Mode: STABILIZE
  Armed: False
  Battery: 16.32V (85%)
  GPS: Fix=3, Satellites=11
```

---

## Step 6: Connect and Arm (Real Flight)

> **WARNING:** Ensure propellers are removed for initial testing!

### Full Test with Video

```bash
python tests/test_dual_drone_connection.py \
  --drone1 udp:192.168.144.25:14550 \
  --drone2 udp:192.168.144.26:14551 \
  --video-url rtsp://192.168.144.25:8554/main.264 \
  --display-video \
  --hold-time 10
```

### Monitor in Mission Planner

1. Open Mission Planner
2. Select COM port or UDP connection
3. Click "Connect"
4. Verify telemetry on both drones

---

## Pre-Flight Checklist

### Before Powering Drones

- [ ] Batteries fully charged (>90%)
- [ ] Propellers removed for bench testing
- [ ] Flight area clear of obstacles
- [ ] Weather conditions safe (wind < 15 km/h)
- [ ] Permissions obtained if required

### After Connection

- [ ] Both drones show heartbeat
- [ ] GPS fix on both drones (>8 satellites)
- [ ] Battery voltage correct (>14V for 4S)
- [ ] Video feed working (Drone 1)
- [ ] No pre-arm errors in logs

### Before Arming

- [ ] Switch to GUIDED mode succeeds
- [ ] Propellers installed correctly (CW/CCW)
- [ ] Standing at safe distance
- [ ] Kill switch ready

---

## Troubleshooting

### Connection Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| No heartbeat | Wrong IP/port | Verify drone IP with `arp -a` |
| Timeout | Firewall blocking | `sudo ufw allow 14550:14560/udp` |
| Permission denied | User not in dialout | `sudo usermod -a -G dialout $USER` |
| Connection refused | Port in use | `sudo lsof -i :14550` |

### Arming Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Arm denied | Pre-arm checks failing | Check GCS for specific error |
| GPS required | No GPS lock | Wait for 3D fix (8+ sats) |
| Battery failsafe | Low voltage | Charge or replace battery |
| RC failsafe | No RC input | Configure RC correctly |

### Video Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| No video | Wrong RTSP URL | Verify URL with `ffplay` |
| High latency | Network congestion | Use wired connection if possible |
| Choppy video | Bandwidth limit | Lower resolution in camera settings |

---

## Quick Reference

### Connection String Formats

```bash
# Serial (USB)
/dev/ttyUSB0
/dev/ttyACM0

# UDP
udp:IP:PORT
udpin:IP:PORT   # Listen mode

# TCP
tcp:IP:PORT
tcpin:IP:PORT   # Listen mode

# Examples
/dev/ttyUSB0                    # USB serial
udp:192.168.144.25:14550        # UDP to drone
udpin:127.0.0.1:14550           # UDP listen mode
tcp:192.168.144.25:5760         # TCP connection
```

### Common Commands

```bash
# Test single drone
python -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udp:192.168.144.25:14550')
print('Waiting for heartbeat...')
m.wait_heartbeat()
print(f'Connected to system {m.target_system}')
"

# List serial ports
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Check network
ip route | grep 192.168.144
arp -a | grep 192.168.144

# Test RTSP
ffplay -rtsp_transport tcp rtsp://192.168.144.25:8554/main.264
```

---

## Safety Reminders

> **CRITICAL:** Always follow these safety guidelines for real flights:

1. **Remove propellers** for all bench testing
2. **Never arm** with people near the drones
3. **Always have** a manual RC override ready
4. **Test in SITL** before flying real hardware
5. **Start with low altitude** (<10m) for initial tests
6. **Maintain visual line of sight** at all times
7. **Check local regulations** before flying

---

## Related Documentation

- [Test Script Usage](../tests/test_dual_drone_connection.py)
- [Network Configuration](../config/network_map.yaml)
- [Mission Parameters](../config/mission_params.yaml)
- [ArduPilot MAVLink](https://ardupilot.org/dev/docs/mavlink-commands.html)
