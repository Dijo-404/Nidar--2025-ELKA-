"""
BridgeClient - Drone Communication Client

ZMQ-based client running on drones for communication with ground relay.
Supports sending coordinates, receiving commands, and heartbeat monitoring.
"""

import time
import json
import logging
import threading
from typing import Optional, Tuple, Callable, List
from dataclasses import dataclass, asdict
from queue import Queue, Empty
import zmq

logger = logging.getLogger(__name__)


@dataclass
class CoordinateMessage:
    """GPS coordinate message format."""
    lat: float
    lon: float
    alt: float
    timestamp: float
    source_id: str
    message_id: int


@dataclass
class HeartbeatMessage:
    """Heartbeat message format."""
    source_id: str
    timestamp: float
    status: str
    battery_percent: int = -1
    payload_remaining: int = -1


class BridgeClient:
    """
    ZMQ-based communication client for drone-to-ground messaging.
    
    Uses DEALER socket for async bidirectional communication with
    the ground relay server.
    """
    
    def __init__(self, identity: str, config: dict = None):
        """
        Initialize BridgeClient.
        
        Args:
            identity: Unique client identifier (e.g., 'SURVEYOR', 'DELIVERER')
            config: Configuration dictionary with network parameters
        """
        self.identity = identity
        self.config = config or {}
        
        # Network configuration
        self.server_ip = self.config.get('ip', '127.0.0.1')
        self.server_port = self.config.get('zmq_port', 5555)
        self.heartbeat_interval = self.config.get('heartbeat_interval', 1.0)
        self.heartbeat_timeout = self.config.get('heartbeat_timeout', 5.0)
        
        # Protocol settings
        self.ack_required = self.config.get('ack_required', True)
        self.max_retries = self.config.get('max_retries', 3)
        self.retry_delay = self.config.get('retry_delay', 0.5)
        
        # ZMQ context and socket
        self.context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None
        self.is_connected = False
        
        # Threading
        self.receiver_thread: Optional[threading.Thread] = None
        self.heartbeat_thread: Optional[threading.Thread] = None
        self.is_running = False
        
        # Message tracking
        self.message_counter = 0
        self.pending_acks: dict = {}
        self.received_coords: Queue[CoordinateMessage] = Queue(maxsize=100)
        
        # Callbacks
        self.on_coordinate_received: Optional[Callable[[CoordinateMessage], None]] = None
        
        # State
        self.last_heartbeat_sent = 0
        self.last_heartbeat_received = 0
        self.current_status = "IDLE"
        self.battery_percent = -1
        self.payload_remaining = -1
    
    def connect(self, server_ip: str = None, port: int = None) -> bool:
        """
        Connect to the relay server.
        
        Args:
            server_ip: Server IP address (optional, uses config)
            port: Server port (optional, uses config)
            
        Returns:
            True if connection successful
        """
        if server_ip:
            self.server_ip = server_ip
        if port:
            self.server_port = port
        
        try:
            # Create ZMQ context
            self.context = zmq.Context()
            
            # Create DEALER socket for async communication
            self.socket = self.context.socket(zmq.DEALER)
            self.socket.setsockopt_string(zmq.IDENTITY, self.identity)
            self.socket.setsockopt(zmq.LINGER, 1000)
            self.socket.setsockopt(zmq.RCVTIMEO, 1000)
            self.socket.setsockopt(zmq.SNDTIMEO, 1000)
            
            # Connect to server
            connection_string = f"tcp://{self.server_ip}:{self.server_port}"
            logger.info(f"Connecting to {connection_string}...")
            self.socket.connect(connection_string)
            
            # Start receiver thread
            self.is_running = True
            self.receiver_thread = threading.Thread(
                target=self._receiver_loop,
                daemon=True,
                name=f"BridgeClient-{self.identity}-Receiver"
            )
            self.receiver_thread.start()
            
            # Start heartbeat thread
            self.heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop,
                daemon=True,
                name=f"BridgeClient-{self.identity}-Heartbeat"
            )
            self.heartbeat_thread.start()
            
            # Send initial heartbeat
            self._send_heartbeat()
            
            self.is_connected = True
            logger.info(f"Connected as {self.identity}")
            return True
            
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the relay server."""
        logger.info("Disconnecting...")
        self.is_running = False
        
        # Wait for threads
        if self.receiver_thread and self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=2.0)
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2.0)
        
        # Close socket and context
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        
        self.socket = None
        self.context = None
        self.is_connected = False
        logger.info("Disconnected")
    
    def _send_message(self, topic: str, data: dict) -> bool:
        """
        Send a message to the server.
        
        Args:
            topic: Message topic
            data: Message data dictionary
            
        Returns:
            True if sent successfully
        """
        if not self.socket:
            return False
        
        try:
            message = {
                'topic': topic,
                'data': data,
                'source': self.identity,
                'timestamp': time.time()
            }
            
            self.socket.send_json(message)
            return True
            
        except zmq.ZMQError as e:
            logger.error(f"Send error: {e}")
            return False
    
    def send_coordinates(self, lat: float, lon: float, alt: float = 0.0,
                         timestamp: float = None) -> bool:
        """
        Send GPS coordinates to the relay (for forwarding to other drones).
        
        Args:
            lat: Latitude
            lon: Longitude
            alt: Altitude in meters
            timestamp: Detection timestamp (default: now)
            
        Returns:
            True if sent successfully
        """
        self.message_counter += 1
        message_id = self.message_counter
        
        coord_msg = CoordinateMessage(
            lat=lat,
            lon=lon,
            alt=alt,
            timestamp=timestamp or time.time(),
            source_id=self.identity,
            message_id=message_id
        )
        
        success = self._send_message('COORD', asdict(coord_msg))
        
        if success:
            logger.info(f"Sent coordinates: ({lat:.6f}, {lon:.6f}, {alt:.1f})")
            
            if self.ack_required:
                self.pending_acks[message_id] = {
                    'message': coord_msg,
                    'sent_time': time.time(),
                    'retries': 0
                }
        
        return success
    
    def _send_heartbeat(self):
        """Send heartbeat message to server."""
        hb_msg = HeartbeatMessage(
            source_id=self.identity,
            timestamp=time.time(),
            status=self.current_status,
            battery_percent=self.battery_percent,
            payload_remaining=self.payload_remaining
        )
        
        if self._send_message('HB', asdict(hb_msg)):
            self.last_heartbeat_sent = time.time()
    
    def _heartbeat_loop(self):
        """Heartbeat sending loop."""
        while self.is_running:
            if time.time() - self.last_heartbeat_sent >= self.heartbeat_interval:
                self._send_heartbeat()
            
            # Check for pending acks that need retry
            self._check_pending_acks()
            
            time.sleep(0.1)
    
    def _check_pending_acks(self):
        """Check and retry unacknowledged messages."""
        now = time.time()
        expired = []
        
        for msg_id, pending in self.pending_acks.items():
            if now - pending['sent_time'] > self.retry_delay:
                if pending['retries'] < self.max_retries:
                    # Retry
                    logger.warning(f"Retrying message {msg_id}")
                    self._send_message('COORD', asdict(pending['message']))
                    pending['sent_time'] = now
                    pending['retries'] += 1
                else:
                    # Give up
                    logger.error(f"Message {msg_id} failed after {self.max_retries} retries")
                    expired.append(msg_id)
        
        for msg_id in expired:
            del self.pending_acks[msg_id]
    
    def _receiver_loop(self):
        """Message receiving loop."""
        while self.is_running:
            try:
                message = self.socket.recv_json()
                self._handle_message(message)
            except zmq.Again:
                # Timeout, continue
                continue
            except zmq.ZMQError as e:
                if self.is_running:
                    logger.error(f"Receive error: {e}")
                time.sleep(0.1)
    
    def _handle_message(self, message: dict):
        """
        Handle received message.
        
        Args:
            message: Received message dictionary
        """
        topic = message.get('topic', '')
        data = message.get('data', {})
        
        if topic == 'COORD':
            # Received coordinates (for deliverer drone)
            coord_msg = CoordinateMessage(**data)
            logger.info(f"Received coordinates from {coord_msg.source_id}: "
                       f"({coord_msg.lat:.6f}, {coord_msg.lon:.6f})")
            
            # Add to queue
            try:
                self.received_coords.put_nowait(coord_msg)
            except:
                # Queue full, log warning
                logger.warning("Coordinate queue full")
            
            # Trigger callback
            if self.on_coordinate_received:
                try:
                    self.on_coordinate_received(coord_msg)
                except Exception as e:
                    logger.error(f"Coordinate callback error: {e}")
        
        elif topic == 'ACK':
            # Acknowledgment received
            msg_id = data.get('message_id')
            if msg_id in self.pending_acks:
                del self.pending_acks[msg_id]
                logger.debug(f"ACK received for message {msg_id}")
        
        elif topic == 'HB':
            # Heartbeat response
            self.last_heartbeat_received = time.time()
    
    def receive_coordinates(self, timeout: float = 0.1) -> Optional[CoordinateMessage]:
        """
        Non-blocking receive of coordinates.
        
        Args:
            timeout: Maximum wait time in seconds
            
        Returns:
            CoordinateMessage or None if none available
        """
        try:
            return self.received_coords.get(timeout=timeout)
        except Empty:
            return None
    
    def get_all_pending_coordinates(self) -> List[CoordinateMessage]:
        """
        Get all pending coordinates from the queue.
        
        Returns:
            List of CoordinateMessage objects
        """
        coords = []
        while not self.received_coords.empty():
            try:
                coords.append(self.received_coords.get_nowait())
            except Empty:
                break
        return coords
    
    def set_status(self, status: str, battery_percent: int = -1, 
                   payload_remaining: int = -1):
        """
        Update client status for heartbeat messages.
        
        Args:
            status: Current status string
            battery_percent: Battery percentage (optional)
            payload_remaining: Remaining payload count (optional)
        """
        self.current_status = status
        if battery_percent >= 0:
            self.battery_percent = battery_percent
        if payload_remaining >= 0:
            self.payload_remaining = payload_remaining
    
    def is_server_alive(self) -> bool:
        """
        Check if server is responding to heartbeats.
        
        Returns:
            True if heartbeat received within timeout
        """
        if self.last_heartbeat_received == 0:
            return True  # Not yet established
        
        return (time.time() - self.last_heartbeat_received) < self.heartbeat_timeout
