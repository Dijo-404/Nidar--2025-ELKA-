"""
RelayServer - Ground Station Relay Server

ZMQ-based server running on the ground station laptop to relay
messages between drones in the local network.
"""

import time
import json
import logging
import threading
from typing import Dict, Optional, Set
from dataclasses import dataclass
from datetime import datetime
import zmq

logger = logging.getLogger(__name__)


@dataclass
class ClientInfo:
    """Information about a connected client."""
    identity: str
    last_heartbeat: float
    status: str
    battery_percent: int
    payload_remaining: int
    message_count: int


class RelayServer:
    """
    ZMQ-based relay server for drone-to-drone communication.
    
    Uses ROUTER socket to manage multiple drone clients,
    forwarding coordinate messages and monitoring connection health.
    """
    
    def __init__(self, config: dict = None):
        """
        Initialize RelayServer.
        
        Args:
            config: Configuration dictionary with server parameters
        """
        self.config = config or {}
        
        # Network configuration
        self.bind_ip = self.config.get('ip', '*')
        self.bind_port = self.config.get('zmq_port', 5555)
        self.heartbeat_timeout = self.config.get('heartbeat_timeout', 5.0)
        
        # ZMQ context and socket
        self.context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None
        
        # State
        self.is_running = False
        self.clients: Dict[str, ClientInfo] = {}
        self.server_thread: Optional[threading.Thread] = None
        
        # Statistics
        self.total_messages = 0
        self.total_coords_relayed = 0
        self.start_time: Optional[float] = None
        
        # Logging
        self.log_file: Optional[str] = None
    
    def start(self, bind_ip: str = None, port: int = None,
              log_file: str = None) -> bool:
        """
        Start the relay server.
        
        Args:
            bind_ip: IP to bind to (default: all interfaces)
            port: Port to bind to
            log_file: Optional path to message log file
            
        Returns:
            True if server started successfully
        """
        if bind_ip:
            self.bind_ip = bind_ip
        if port:
            self.bind_port = port
        if log_file:
            self.log_file = log_file
        
        try:
            # Create ZMQ context
            self.context = zmq.Context()
            
            # Create ROUTER socket for managing multiple clients
            self.socket = self.context.socket(zmq.ROUTER)
            self.socket.setsockopt(zmq.LINGER, 1000)
            self.socket.setsockopt(zmq.RCVTIMEO, 1000)
            
            # Bind to address
            bind_string = f"tcp://{self.bind_ip}:{self.bind_port}"
            logger.info(f"Binding to {bind_string}...")
            self.socket.bind(bind_string)
            
            # Start server thread
            self.is_running = True
            self.start_time = time.time()
            
            self.server_thread = threading.Thread(
                target=self._server_loop,
                daemon=True,
                name="RelayServer"
            )
            self.server_thread.start()
            
            logger.info(f"Relay server started on port {self.bind_port}")
            return True
            
        except Exception as e:
            logger.error(f"Server start failed: {e}")
            return False
    
    def stop(self):
        """Stop the relay server."""
        logger.info("Stopping relay server...")
        self.is_running = False
        
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=3.0)
        
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        
        self.socket = None
        self.context = None
        logger.info("Relay server stopped")
    
    def _server_loop(self):
        """Main server loop."""
        while self.is_running:
            try:
                # Receive message (ROUTER gives us [identity, message])
                frames = self.socket.recv_multipart()
                if len(frames) >= 2:
                    identity = frames[0].decode('utf-8')
                    message = json.loads(frames[1].decode('utf-8'))
                    self._handle_message(identity, message)
                    
            except zmq.Again:
                # Timeout, check for stale clients
                self._check_stale_clients()
                continue
            except zmq.ZMQError as e:
                if self.is_running:
                    logger.error(f"Server error: {e}")
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Message handling error: {e}")
    
    def _handle_message(self, identity: str, message: dict):
        """
        Handle received message from a client.
        
        Args:
            identity: Client identity
            message: Message dictionary
        """
        topic = message.get('topic', '')
        data = message.get('data', {})
        source = message.get('source', identity)
        timestamp = message.get('timestamp', time.time())
        
        self.total_messages += 1
        
        # Log message
        self._log_message(source, topic, data)
        
        if topic == 'HB':
            # Heartbeat message
            self._handle_heartbeat(identity, data)
        
        elif topic == 'COORD':
            # Coordinate message - relay to other drones
            self._relay_coordinates(identity, message)
        
        elif topic == 'STATUS':
            # Status update
            logger.info(f"Status from {identity}: {data}")
    
    def _handle_heartbeat(self, identity: str, data: dict):
        """
        Handle heartbeat message from client.
        
        Args:
            identity: Client identity
            data: Heartbeat data
        """
        now = time.time()
        
        if identity not in self.clients:
            # New client
            logger.info(f"New client connected: {identity}")
            self.clients[identity] = ClientInfo(
                identity=identity,
                last_heartbeat=now,
                status=data.get('status', 'UNKNOWN'),
                battery_percent=data.get('battery_percent', -1),
                payload_remaining=data.get('payload_remaining', -1),
                message_count=0
            )
        else:
            # Update existing client
            client = self.clients[identity]
            client.last_heartbeat = now
            client.status = data.get('status', client.status)
            client.battery_percent = data.get('battery_percent', client.battery_percent)
            client.payload_remaining = data.get('payload_remaining', client.payload_remaining)
        
        # Send heartbeat response
        self._send_to_client(identity, {'topic': 'HB', 'data': {'timestamp': now}})
    
    def _relay_coordinates(self, source_identity: str, message: dict):
        """
        Relay coordinate message to all other connected clients.
        
        Args:
            source_identity: Identity of sending client
            message: Original message
        """
        data = message.get('data', {})
        
        logger.info(f"Relaying coordinates from {source_identity}: "
                   f"({data.get('lat', 0):.6f}, {data.get('lon', 0):.6f})")
        
        self.total_coords_relayed += 1
        
        # Send acknowledgment to sender
        msg_id = data.get('message_id', 0)
        self._send_to_client(source_identity, {
            'topic': 'ACK',
            'data': {'message_id': msg_id, 'status': 'relayed'}
        })
        
        # Update sender message count
        if source_identity in self.clients:
            self.clients[source_identity].message_count += 1
        
        # Relay to all other clients
        for client_id in self.clients:
            if client_id != source_identity:
                self._send_to_client(client_id, message)
                logger.debug(f"Forwarded to {client_id}")
    
    def _send_to_client(self, identity: str, message: dict) -> bool:
        """
        Send message to a specific client.
        
        Args:
            identity: Target client identity
            message: Message dictionary
            
        Returns:
            True if sent successfully
        """
        if not self.socket:
            return False
        
        try:
            self.socket.send_multipart([
                identity.encode('utf-8'),
                json.dumps(message).encode('utf-8')
            ])
            return True
        except zmq.ZMQError as e:
            logger.error(f"Send to {identity} failed: {e}")
            return False
    
    def _check_stale_clients(self):
        """Remove clients that haven't sent heartbeat recently."""
        now = time.time()
        stale = []
        
        for client_id, client in self.clients.items():
            if now - client.last_heartbeat > self.heartbeat_timeout:
                stale.append(client_id)
        
        for client_id in stale:
            logger.warning(f"Client {client_id} timed out, removing")
            del self.clients[client_id]
    
    def _log_message(self, source: str, topic: str, data: dict):
        """
        Log message to file if configured.
        
        Args:
            source: Message source
            topic: Message topic
            data: Message data
        """
        if not self.log_file:
            return
        
        try:
            timestamp = datetime.now().isoformat()
            log_entry = {
                'timestamp': timestamp,
                'source': source,
                'topic': topic,
                'data': data
            }
            
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
                
        except Exception as e:
            logger.error(f"Logging failed: {e}")
    
    def get_connected_clients(self) -> Dict[str, ClientInfo]:
        """
        Get information about connected clients.
        
        Returns:
            Dictionary of client ID to ClientInfo
        """
        return self.clients.copy()
    
    def get_statistics(self) -> dict:
        """
        Get server statistics.
        
        Returns:
            Dictionary with server stats
        """
        uptime = time.time() - self.start_time if self.start_time else 0
        
        return {
            'uptime_seconds': uptime,
            'connected_clients': len(self.clients),
            'total_messages': self.total_messages,
            'total_coords_relayed': self.total_coords_relayed,
            'clients': {
                cid: {
                    'status': c.status,
                    'battery': c.battery_percent,
                    'payload': c.payload_remaining,
                    'messages': c.message_count
                }
                for cid, c in self.clients.items()
            }
        }
    
    def broadcast(self, topic: str, data: dict):
        """
        Broadcast message to all connected clients.
        
        Args:
            topic: Message topic
            data: Message data
        """
        message = {
            'topic': topic,
            'data': data,
            'source': 'RELAY',
            'timestamp': time.time()
        }
        
        for client_id in self.clients:
            self._send_to_client(client_id, message)
    
    def run_blocking(self):
        """
        Run server in blocking mode with status display.
        
        This is useful for running the server standalone.
        """
        logger.info("Running in blocking mode. Press Ctrl+C to stop.")
        
        try:
            while self.is_running:
                # Print status every 5 seconds
                stats = self.get_statistics()
                
                print(f"\n{'='*50}")
                print(f"Relay Server Status")
                print(f"{'='*50}")
                print(f"Uptime: {stats['uptime_seconds']:.1f}s")
                print(f"Connected clients: {stats['connected_clients']}")
                print(f"Total messages: {stats['total_messages']}")
                print(f"Coordinates relayed: {stats['total_coords_relayed']}")
                
                if stats['clients']:
                    print(f"\nClients:")
                    for cid, info in stats['clients'].items():
                        print(f"  {cid}: {info['status']} "
                              f"(battery: {info['battery']}%, "
                              f"payload: {info['payload']})")
                
                time.sleep(5.0)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
            self.stop()
