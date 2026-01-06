"""
Test Payload Trigger - Validates PayloadServo module

Tests:
1. Payload count tracking
2. Drop mechanism logic
3. Count exhaustion behavior
4. Statistics reporting
"""

import os
import sys
import pytest
from unittest.mock import MagicMock, patch
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.base.payload_servo import PayloadServo


class MockMAVConnection:
    """Mock MAVLink connection for testing."""
    
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = MagicMock()
        self.commands_sent = []
        
    def recv_match(self, type=None, blocking=True, timeout=None):
        """Mock receive that returns successful ACK."""
        msg = MagicMock()
        msg.result = 0  # MAV_RESULT_ACCEPTED
        return msg


@pytest.fixture
def mock_mav():
    """Create a mock MAVLink connection."""
    return MockMAVConnection()


@pytest.fixture
def payload_servo(mock_mav):
    """Create a PayloadServo with default config."""
    config = {
        'initial_count': 10,
        'servo_channel': 9,
        'servo_drop_pwm': 2000,
        'servo_hold_pwm': 1000,
        'drop_stabilize_time': 0.1  # Short for testing
    }
    return PayloadServo(mock_mav, config)


class TestPayloadInitialization:
    """Tests for payload servo initialization."""
    
    def test_initial_count(self, payload_servo):
        """Test initial payload count."""
        assert payload_servo.remaining() == 10
        
    def test_not_empty_initially(self, payload_servo):
        """Test that payload is not empty initially."""
        assert payload_servo.is_empty() is False
        
    def test_custom_initial_count(self, mock_mav):
        """Test custom initial count."""
        servo = PayloadServo(mock_mav, {'initial_count': 5})
        assert servo.remaining() == 5


class TestPayloadDrop:
    """Tests for payload drop functionality."""
    
    def test_single_drop(self, payload_servo):
        """Test single payload drop."""
        initial = payload_servo.remaining()
        
        result = payload_servo.drop()
        
        assert result is True
        assert payload_servo.remaining() == initial - 1
        
    def test_multiple_drops(self, payload_servo):
        """Test multiple sequential drops."""
        for i in range(5):
            result = payload_servo.drop()
            assert result is True
            
        assert payload_servo.remaining() == 5
        
    def test_drop_all_payloads(self, payload_servo):
        """Test dropping all payloads."""
        for i in range(10):
            payload_servo.drop()
            
        assert payload_servo.is_empty() is True
        assert payload_servo.remaining() == 0
        
    def test_drop_when_empty(self, payload_servo):
        """Test drop attempt when payload is exhausted."""
        # Empty the payload
        for i in range(10):
            payload_servo.drop()
        
        # Try to drop again
        result = payload_servo.drop()
        
        assert result is False
        assert payload_servo.remaining() == 0


class TestPayloadStatistics:
    """Tests for payload statistics tracking."""
    
    def test_drops_performed_count(self, payload_servo):
        """Test drops performed counter."""
        assert payload_servo.drops_performed == 0
        
        payload_servo.drop()
        payload_servo.drop()
        
        assert payload_servo.drops_performed == 2
        
    def test_last_drop_time(self, payload_servo):
        """Test last drop time tracking."""
        assert payload_servo.last_drop_time is None
        
        payload_servo.drop()
        
        assert payload_servo.last_drop_time is not None
        
    def test_get_stats(self, payload_servo):
        """Test statistics dictionary."""
        payload_servo.drop()
        payload_servo.drop()
        
        stats = payload_servo.get_stats()
        
        assert stats['initial_count'] == 10
        assert stats['remaining'] == 8
        assert stats['drops_performed'] == 2
        assert stats['is_empty'] is False


class TestPayloadReset:
    """Tests for payload count reset."""
    
    def test_reset_to_initial(self, payload_servo):
        """Test reset to initial count."""
        payload_servo.drop()
        payload_servo.drop()
        
        payload_servo.reset_count()
        
        assert payload_servo.remaining() == 10
        assert payload_servo.drops_performed == 0
        
    def test_reset_to_custom_count(self, payload_servo):
        """Test reset to custom count."""
        payload_servo.drop()
        
        payload_servo.reset_count(count=5)
        
        assert payload_servo.remaining() == 5


class TestPayloadEdgeCases:
    """Tests for edge cases."""
    
    def test_zero_initial_count(self, mock_mav):
        """Test with zero initial count."""
        servo = PayloadServo(mock_mav, {'initial_count': 0})
        
        assert servo.is_empty() is True
        assert servo.drop() is False
        
    def test_one_payload(self, mock_mav):
        """Test with single payload."""
        servo = PayloadServo(mock_mav, {'initial_count': 1})
        
        assert servo.remaining() == 1
        assert servo.drop() is True
        assert servo.is_empty() is True
        assert servo.drop() is False
        
    def test_rapid_drops(self, payload_servo):
        """Test rapid sequential drops."""
        results = []
        for i in range(10):
            results.append(payload_servo.drop())
        
        # All should succeed
        assert all(results)
        
        # 11th should fail
        assert payload_servo.drop() is False


class TestPayloadHold:
    """Tests for payload hold function."""
    
    def test_hold_command(self, payload_servo):
        """Test hold command is sent."""
        result = payload_servo.hold()
        
        assert result is True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
