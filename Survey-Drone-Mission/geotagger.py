"""
Geotagger - GPS coordinate tagging for detections

Tags human detections with GPS coordinates from:
- MAVLink connection (drone telemetry)
- Serial GPS device
- Simulated coordinates (for testing)
"""

import logging
import time
import csv
import os
from typing import Optional, Tuple, List
from dataclasses import dataclass, field
from datetime import datetime

logger = logging.getLogger(__name__)


@dataclass
class GPSPosition:
    """Current GPS position."""
    lat: float
    lon: float
    alt: float
    timestamp: float = field(default_factory=time.time)
    source: str = "unknown"
    
    def __str__(self):
        return f"({self.lat:.6f}, {self.lon:.6f}, {self.alt:.1f}m)"


@dataclass
class GeotaggedDetection:
    """Detection with GPS coordinates."""
    detection_id: int
    timestamp: float
    lat: float
    lon: float
    alt: float
    human_count: int
    avg_confidence: float
    
    def to_dict(self) -> dict:
        return {
            'id': self.detection_id,
            'timestamp': datetime.fromtimestamp(self.timestamp).isoformat(),
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt,
            'human_count': self.human_count,
            'confidence': self.avg_confidence
        }


class Geotagger:
    """
    GPS coordinate management and detection geotagging.
    """
    
    def __init__(self, config: dict = None):
        """
        Initialize geotagger.
        
        Args:
            config: GPS configuration dictionary
        """
        self.config = config or {}
        self.enabled = self.config.get('enabled', True)
        self.source = self.config.get('source', 'simulated')
        
        self.current_position: Optional[GPSPosition] = None
        self.geotagged_detections: List[GeotaggedDetection] = []
        self.detection_counter = 0
        
        self.mavlink_conn = None
        self.serial_conn = None
        
        # For simulated GPS
        self.sim_lat = 12.9710
        self.sim_lon = 77.5910
        self.sim_alt = 25.0
        
        # CSV logging
        self.csv_file = None
        self.csv_writer = None
        
        if self.enabled:
            self._connect()
    
    def _connect(self):
        """Connect to GPS source."""
        if self.source == 'mavlink':
            self._connect_mavlink()
        elif self.source == 'serial':
            self._connect_serial()
        elif self.source == 'simulated':
            logger.info("Using simulated GPS")
            self._update_simulated()
        else:
            logger.warning(f"Unknown GPS source: {self.source}")
    
    def _connect_mavlink(self):
        """Connect via MAVLink."""
        try:
            from pymavlink import mavutil
            
            conn_string = self.config.get('mavlink_connection', 'udpin:0.0.0.0:14550')
            logger.info(f"Connecting to MAVLink: {conn_string}")
            
            self.mavlink_conn = mavutil.mavlink_connection(conn_string)
            
            # Wait for heartbeat
            msg = self.mavlink_conn.wait_heartbeat(timeout=5)
            if msg:
                logger.info(f"MAVLink connected (System {self.mavlink_conn.target_system})")
            else:
                logger.warning("MAVLink connection timeout")
                self.mavlink_conn = None
                
        except Exception as e:
            logger.error(f"MAVLink connection failed: {e}")
            self.mavlink_conn = None
    
    def _connect_serial(self):
        """Connect via serial GPS."""
        try:
            import serial
            
            port = self.config.get('serial_port', '/dev/ttyACM0')
            baud = self.config.get('serial_baud', 9600)
            
            logger.info(f"Connecting to serial GPS: {port}")
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            logger.info("Serial GPS connected")
            
        except Exception as e:
            logger.error(f"Serial GPS connection failed: {e}")
            self.serial_conn = None
    
    def _update_simulated(self):
        """Update simulated GPS position (random drift)."""
        import random
        
        # Add small random drift
        self.sim_lat += random.uniform(-0.00001, 0.00001)
        self.sim_lon += random.uniform(-0.00001, 0.00001)
        
        self.current_position = GPSPosition(
            lat=self.sim_lat,
            lon=self.sim_lon,
            alt=self.sim_alt,
            source='simulated'
        )
    
    def update(self) -> Optional[GPSPosition]:
        """
        Update current GPS position.
        
        Returns:
            Current GPSPosition or None
        """
        if self.source == 'mavlink' and self.mavlink_conn:
            try:
                msg = self.mavlink_conn.recv_match(
                    type='GLOBAL_POSITION_INT',
                    blocking=True,
                    timeout=0.1
                )
                if msg:
                    self.current_position = GPSPosition(
                        lat=msg.lat / 1e7,
                        lon=msg.lon / 1e7,
                        alt=msg.relative_alt / 1000.0,
                        source='mavlink'
                    )
            except Exception as e:
                logger.debug(f"MAVLink GPS update error: {e}")
                
        elif self.source == 'serial' and self.serial_conn:
            try:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore')
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    self._parse_nmea(line)
            except Exception as e:
                logger.debug(f"Serial GPS update error: {e}")
                
        elif self.source == 'simulated':
            self._update_simulated()
        
        return self.current_position
    
    def _parse_nmea(self, line: str):
        """Parse NMEA GPGGA sentence."""
        try:
            parts = line.split(',')
            if len(parts) >= 10:
                lat_raw = float(parts[2])
                lat = int(lat_raw / 100) + (lat_raw % 100) / 60
                if parts[3] == 'S':
                    lat = -lat
                    
                lon_raw = float(parts[4])
                lon = int(lon_raw / 100) + (lon_raw % 100) / 60
                if parts[5] == 'W':
                    lon = -lon
                    
                alt = float(parts[9]) if parts[9] else 0.0
                
                self.current_position = GPSPosition(
                    lat=lat, lon=lon, alt=alt, source='serial'
                )
        except (ValueError, IndexError):
            pass
    
    def get_position(self) -> Optional[GPSPosition]:
        """Get current position (calls update)."""
        return self.update()
    
    def geotag_detection(self, human_count: int, avg_confidence: float) -> Optional[GeotaggedDetection]:
        """
        Create geotagged detection with current GPS position.
        
        Args:
            human_count: Number of humans detected
            avg_confidence: Average detection confidence
            
        Returns:
            GeotaggedDetection or None
        """
        pos = self.get_position()
        if pos is None:
            return None
        
        self.detection_counter += 1
        
        detection = GeotaggedDetection(
            detection_id=self.detection_counter,
            timestamp=time.time(),
            lat=pos.lat,
            lon=pos.lon,
            alt=pos.alt,
            human_count=human_count,
            avg_confidence=avg_confidence
        )
        
        self.geotagged_detections.append(detection)
        
        # Log to CSV
        if self.csv_writer:
            self._write_csv(detection)
        
        logger.info(f"Geotagged detection #{detection.detection_id}: "
                   f"{human_count} humans at {pos}")
        
        return detection
    
    def start_logging(self, output_dir: str, filename: str = "detections.csv"):
        """Start CSV logging of detections."""
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, filename)
        
        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'id', 'timestamp', 'lat', 'lon', 'alt', 'human_count', 'confidence'
        ])
        
        logger.info(f"Logging detections to: {filepath}")
    
    def _write_csv(self, detection: GeotaggedDetection):
        """Write detection to CSV."""
        self.csv_writer.writerow([
            detection.detection_id,
            datetime.fromtimestamp(detection.timestamp).isoformat(),
            f"{detection.lat:.6f}",
            f"{detection.lon:.6f}",
            f"{detection.alt:.1f}",
            detection.human_count,
            f"{detection.avg_confidence:.2f}"
        ])
        self.csv_file.flush()
    
    def stop_logging(self):
        """Stop CSV logging."""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
    
    def get_all_detections(self) -> List[GeotaggedDetection]:
        """Get all geotagged detections."""
        return self.geotagged_detections
    
    def close(self):
        """Clean up connections."""
        self.stop_logging()
        
        if self.mavlink_conn:
            self.mavlink_conn.close()
        if self.serial_conn:
            self.serial_conn.close()


# Test
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    geotagger = Geotagger({'source': 'simulated'})
    
    print(f"Position: {geotagger.get_position()}")
    
    # Simulate detection
    det = geotagger.geotag_detection(human_count=2, avg_confidence=0.85)
    print(f"Geotagged: {det.to_dict()}")
