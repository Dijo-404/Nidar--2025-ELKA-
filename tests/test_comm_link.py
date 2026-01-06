"""
Test Communication Link - Validates BridgeClient and RelayServer

Tests:
1. ZMQ connection establishment
2. Message sending/receiving
3. Coordinate forwarding
4. Heartbeat mechanism
"""

import os
import sys
import time
import pytest
import threading
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.comms.bridge_client import BridgeClient, CoordinateMessage
from src.comms.relay_server import RelayServer


class TestRelayServer:
    """Tests for the relay server."""
    
    def test_server_start_stop(self):
        """Test server can start and stop."""
        server = RelayServer({'zmq_port': 15555})
        
        assert server.start(port=15555) is True
        assert server.is_running is True
        
        time.sleep(0.5)
        server.stop()
        
        assert server.is_running is False
        
    def test_server_statistics(self):
        """Test server statistics tracking."""
        server = RelayServer({'zmq_port': 15556})
        server.start(port=15556)
        
        time.sleep(0.5)
        stats = server.get_statistics()
        
        assert 'uptime_seconds' in stats
        assert 'connected_clients' in stats
        assert 'total_messages' in stats
        assert stats['connected_clients'] == 0
        
        server.stop()


class TestBridgeClient:
    """Tests for the bridge client."""
    
    @pytest.fixture
    def server_and_client(self):
        """Set up a server and client for testing."""
        # Start server
        server = RelayServer({'zmq_port': 15557, 'heartbeat_timeout': 10.0})
        server.start(port=15557)
        time.sleep(0.5)
        
        # Create client
        client = BridgeClient(
            identity='TEST_CLIENT',
            config={'ip': '127.0.0.1', 'zmq_port': 15557}
        )
        
        yield server, client
        
        # Cleanup
        if client.is_connected:
            client.disconnect()
        server.stop()
    
    def test_client_connect(self, server_and_client):
        """Test client connection to server."""
        server, client = server_and_client
        
        result = client.connect('127.0.0.1', 15557)
        
        assert result is True
        assert client.is_connected is True
        
    def test_client_disconnect(self, server_and_client):
        """Test client disconnection."""
        server, client = server_and_client
        
        client.connect('127.0.0.1', 15557)
        client.disconnect()
        
        assert client.is_connected is False
        
    def test_send_coordinates(self, server_and_client):
        """Test sending coordinates."""
        server, client = server_and_client
        
        client.connect('127.0.0.1', 15557)
        time.sleep(0.5)
        
        result = client.send_coordinates(12.9700, 77.5900, 20.0)
        
        assert result is True
        
    def test_heartbeat_sent(self, server_and_client):
        """Test that heartbeat is sent."""
        server, client = server_and_client
        
        client.connect('127.0.0.1', 15557)
        time.sleep(2.0)  # Wait for heartbeat
        
        assert client.last_heartbeat_sent > 0


class TestCoordinateRelay:
    """Tests for coordinate message relaying between clients."""
    
    @pytest.fixture
    def two_clients_with_server(self):
        """Set up a server with two clients."""
        # Start server
        server = RelayServer({
            'zmq_port': 15558, 
            'heartbeat_timeout': 10.0
        })
        server.start(port=15558)
        time.sleep(0.5)
        
        # Create clients
        client1 = BridgeClient(
            identity='SURVEYOR',
            config={'ip': '127.0.0.1', 'zmq_port': 15558}
        )
        client2 = BridgeClient(
            identity='DELIVERER',
            config={'ip': '127.0.0.1', 'zmq_port': 15558}
        )
        
        yield server, client1, client2
        
        # Cleanup
        if client1.is_connected:
            client1.disconnect()
        if client2.is_connected:
            client2.disconnect()
        server.stop()
    
    def test_coordinate_forwarding(self, two_clients_with_server):
        """Test that coordinates are forwarded between clients."""
        server, client1, client2 = two_clients_with_server
        
        # Track received coordinates
        received_coords = []
        client2.on_coordinate_received = lambda c: received_coords.append(c)
        
        # Connect both clients
        client1.connect('127.0.0.1', 15558)
        client2.connect('127.0.0.1', 15558)
        time.sleep(1.0)  # Wait for connections to establish
        
        # Send coordinates from client1
        test_lat = 12.9700
        test_lon = 77.5900
        test_alt = 20.0
        
        client1.send_coordinates(test_lat, test_lon, test_alt)
        
        # Wait for relay
        time.sleep(1.0)
        
        # Check if client2 received
        assert len(received_coords) > 0
        
        coord = received_coords[0]
        assert abs(coord.lat - test_lat) < 0.0001
        assert abs(coord.lon - test_lon) < 0.0001


class TestMessageQueuing:
    """Tests for message queue handling."""
    
    def test_coordinate_queue(self):
        """Test that coordinates are queued correctly."""
        client = BridgeClient(identity='TEST', config={})
        
        # Manually add to queue
        coord = CoordinateMessage(
            lat=12.97,
            lon=77.59,
            alt=20.0,
            timestamp=time.time(),
            source_id='TEST',
            message_id=1
        )
        
        client.received_coords.put(coord)
        
        # Retrieve
        retrieved = client.receive_coordinates(timeout=0.5)
        
        assert retrieved is not None
        assert retrieved.lat == coord.lat
        
    def test_empty_queue_returns_none(self):
        """Test that empty queue returns None."""
        client = BridgeClient(identity='TEST', config={})
        
        result = client.receive_coordinates(timeout=0.1)
        
        assert result is None
        
    def test_get_all_pending(self):
        """Test getting all pending coordinates."""
        client = BridgeClient(identity='TEST', config={})
        
        # Add multiple coordinates
        for i in range(5):
            coord = CoordinateMessage(
                lat=12.97 + i * 0.001,
                lon=77.59,
                alt=20.0,
                timestamp=time.time(),
                source_id='TEST',
                message_id=i
            )
            client.received_coords.put(coord)
        
        # Get all
        all_coords = client.get_all_pending_coordinates()
        
        assert len(all_coords) == 5
        assert client.received_coords.empty()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
