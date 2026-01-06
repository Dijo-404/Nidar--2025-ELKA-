"""
TelemetryForwarder - MAVLink Telemetry Forwarding to GCS

Forwards MAVLink messages from drones to Mission Planner or other
ground control stations for real-time monitoring.
"""

import time
import socket
import logging
import threading
from typing import Optional, Dict, Any
from pymavlink import mavutil

logger = logging.getLogger(__name__)


class TelemetryForwarder:
    """
    Forwards MAVLink telemetry to Ground Control Station.
    
    Enables monitoring of drone missions in Mission Planner, QGroundControl,
    or other MAVLink-compatible GCS software.
    """
    
    def __init__(self, gcs_config: dict = None):
        """
        Initialize TelemetryForwarder.
        
        Args:
            gcs_config: GCS configuration dictionary with IP and ports
        """
        self.config = gcs_config or {}
        
        # GCS configuration
        self.enabled = self.config.get('enabled', True)
        self.gcs_ip = self.config.get('ip', '127.0.0.1')
        self.forward_all = self.config.get('forward_all_messages', True)
        self.rate_limit_hz = self.config.get('forward_rate_hz', 10)
        
        # UDP sockets for forwarding
        self.sockets: Dict[str, socket.socket] = {}
        self.ports: Dict[str, int] = {}
        
        # Rate limiting
        self.last_forward_time: Dict[str, float] = {}
        self.min_interval = 1.0 / self.rate_limit_hz if self.rate_limit_hz > 0 else 0
        
        # Statistics
        self.messages_forwarded = 0
        self.bytes_forwarded = 0
        
        # Thread safety
        self._lock = threading.Lock()
        
    def add_drone(self, drone_id: str, gcs_port: int) -> bool:
        """
        Add a drone for telemetry forwarding.
        
        Args:
            drone_id: Unique identifier for the drone
            gcs_port: UDP port to forward telemetry to
            
        Returns:
            True if setup successful
        """
        if not self.enabled:
            logger.info("Telemetry forwarding disabled")
            return False
        
        try:
            # Create UDP socket for this drone
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
            
            with self._lock:
                self.sockets[drone_id] = sock
                self.ports[drone_id] = gcs_port
                self.last_forward_time[drone_id] = 0
            
            logger.info(f"Telemetry forwarding enabled for {drone_id} -> "
                       f"{self.gcs_ip}:{gcs_port}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to setup forwarding for {drone_id}: {e}")
            return False
    
    def forward_message(self, drone_id: str, msg) -> bool:
        """
        Forward a MAVLink message to the GCS.
        
        Args:
            drone_id: Drone identifier
            msg: MAVLink message object
            
        Returns:
            True if forwarded successfully
        """
        if not self.enabled:
            return False
        
        if drone_id not in self.sockets:
            return False
        
        # Rate limiting for position messages
        now = time.time()
        msg_type = msg.get_type()
        
        # Apply rate limiting only to high-frequency messages
        high_freq_messages = [
            'GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED', 
            'ATTITUDE', 'VFR_HUD', 'RAW_IMU'
        ]
        
        if msg_type in high_freq_messages:
            with self._lock:
                if now - self.last_forward_time.get(drone_id, 0) < self.min_interval:
                    return False
                self.last_forward_time[drone_id] = now
        
        try:
            # Get message bytes
            msg_bytes = msg.get_msgbuf()
            
            # Send to GCS
            sock = self.sockets[drone_id]
            port = self.ports[drone_id]
            sock.sendto(msg_bytes, (self.gcs_ip, port))
            
            with self._lock:
                self.messages_forwarded += 1
                self.bytes_forwarded += len(msg_bytes)
            
            return True
            
        except BlockingIOError:
            # Non-blocking socket, buffer full
            return False
        except Exception as e:
            logger.debug(f"Forward error: {e}")
            return False
    
    def forward_raw(self, drone_id: str, data: bytes) -> bool:
        """
        Forward raw bytes to the GCS.
        
        Args:
            drone_id: Drone identifier
            data: Raw MAVLink bytes
            
        Returns:
            True if forwarded successfully
        """
        if not self.enabled or drone_id not in self.sockets:
            return False
        
        try:
            sock = self.sockets[drone_id]
            port = self.ports[drone_id]
            sock.sendto(data, (self.gcs_ip, port))
            
            with self._lock:
                self.messages_forwarded += 1
                self.bytes_forwarded += len(data)
            
            return True
        except:
            return False
    
    def get_stats(self) -> dict:
        """Get forwarding statistics."""
        with self._lock:
            return {
                'enabled': self.enabled,
                'gcs_ip': self.gcs_ip,
                'drones': list(self.sockets.keys()),
                'messages_forwarded': self.messages_forwarded,
                'bytes_forwarded': self.bytes_forwarded
            }
    
    def close(self):
        """Close all forwarding sockets."""
        with self._lock:
            for drone_id, sock in self.sockets.items():
                try:
                    sock.close()
                    logger.info(f"Closed telemetry forwarding for {drone_id}")
                except:
                    pass
            self.sockets.clear()
            self.ports.clear()


class MAVLinkBridge:
    """
    Bidirectional MAVLink bridge between drone and GCS.
    
    Allows Mission Planner to both monitor and send commands
    to drones through the mission scripts.
    """
    
    def __init__(self, drone_connection: str, gcs_ip: str, gcs_port: int):
        """
        Initialize MAVLink bridge.
        
        Args:
            drone_connection: MAVLink connection string to drone
            gcs_ip: GCS IP address
            gcs_port: GCS UDP port
        """
        self.drone_connection = drone_connection
        self.gcs_ip = gcs_ip
        self.gcs_port = gcs_port
        
        # Connections
        self.drone_mav: Optional[mavutil.mavlink_connection] = None
        self.gcs_socket: Optional[socket.socket] = None
        
        # Threading
        self.is_running = False
        self.forward_thread: Optional[threading.Thread] = None
        
        # Statistics
        self.to_gcs_count = 0
        self.from_gcs_count = 0
        
    def start(self) -> bool:
        """
        Start the MAVLink bridge.
        
        Returns:
            True if started successfully
        """
        try:
            # Connect to drone
            logger.info(f"Connecting to drone: {self.drone_connection}")
            self.drone_mav = mavutil.mavlink_connection(self.drone_connection)
            self.drone_mav.wait_heartbeat(timeout=5)
            logger.info("Connected to drone")
            
            # Create GCS socket
            self.gcs_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.gcs_socket.setblocking(False)
            
            # Bind to receive from GCS
            self.gcs_socket.bind(('0.0.0.0', self.gcs_port + 100))
            
            # Start forwarding thread
            self.is_running = True
            self.forward_thread = threading.Thread(
                target=self._forward_loop,
                daemon=True,
                name="MAVLinkBridge"
            )
            self.forward_thread.start()
            
            logger.info(f"MAVLink bridge started: drone <-> {self.gcs_ip}:{self.gcs_port}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start bridge: {e}")
            return False
    
    def _forward_loop(self):
        """Main forwarding loop."""
        while self.is_running:
            # Forward drone -> GCS
            try:
                msg = self.drone_mav.recv_match(blocking=False)
                if msg:
                    msg_bytes = msg.get_msgbuf()
                    self.gcs_socket.sendto(msg_bytes, (self.gcs_ip, self.gcs_port))
                    self.to_gcs_count += 1
            except:
                pass
            
            # Forward GCS -> drone (for commands from Mission Planner)
            try:
                data, addr = self.gcs_socket.recvfrom(1024)
                if data:
                    self.drone_mav.write(data)
                    self.from_gcs_count += 1
            except BlockingIOError:
                pass
            except:
                pass
            
            time.sleep(0.001)  # 1ms loop
    
    def stop(self):
        """Stop the bridge."""
        self.is_running = False
        
        if self.forward_thread and self.forward_thread.is_alive():
            self.forward_thread.join(timeout=2)
        
        if self.gcs_socket:
            self.gcs_socket.close()
        
        if self.drone_mav:
            self.drone_mav.close()
        
        logger.info(f"Bridge stopped. To GCS: {self.to_gcs_count}, From GCS: {self.from_gcs_count}")
