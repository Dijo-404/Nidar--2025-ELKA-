"""
PayloadESP32 - ESP32-based Payload Drop Mechanism Driver

Controls the payload servo via ESP32 + PCA9685 PWM driver.
Communicates via serial (USB) or optionally HTTP.
"""

import time
import logging
import serial
from typing import Optional

logger = logging.getLogger(__name__)


class PayloadESP32:
    """
    ESP32-based payload drop mechanism controller.
    
    Communicates with ESP32 via serial to control PCA9685 servo driver.
    Drop-in replacement for PayloadServo with same interface.
    """
    
    def __init__(self, config: dict = None, mav_connection=None):
        """
        Initialize PayloadESP32 controller.
        
        Args:
            config: Configuration dictionary with ESP32 parameters
            mav_connection: Ignored (for backward compatibility)
        """
        self.config = config or {}
        
        # Serial configuration
        self.serial_port = self.config.get('esp32_serial_port', '/dev/ttyUSB0')
        self.serial_baud = self.config.get('esp32_baud_rate', 115200)
        self.serial_timeout = self.config.get('serial_timeout', 2.0)
        
        # Payload configuration
        self.initial_count = self.config.get('initial_count', 10)
        self.payload_count = self.initial_count
        self.drop_stabilize_time = self.config.get('drop_stabilize_time', 2.0)
        
        # State tracking
        self.drops_performed = 0
        self.last_drop_time: Optional[float] = None
        
        # Serial connection
        self.serial: Optional[serial.Serial] = None
        self._connected = False
        
        # Auto-connect
        self._connect()
    
    def _connect(self) -> bool:
        """Establish serial connection to ESP32."""
        try:
            logger.info(f"Connecting to ESP32 on {self.serial_port}...")
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baud,
                timeout=self.serial_timeout
            )
            
            # Wait for ESP32 to initialize
            time.sleep(2.0)
            
            # Clear any startup messages
            self.serial.reset_input_buffer()
            
            # Test connection with STATUS command
            self.serial.write(b"STATUS\n")
            time.sleep(0.5)
            response = self.serial.read_all().decode('utf-8', errors='ignore')
            
            if "STATUS" in response or "Initialized" in response:
                self._connected = True
                logger.info(f"ESP32 connected on {self.serial_port}")
                return True
            else:
                logger.warning(f"ESP32 responded but may not be ready: {response[:100]}")
                self._connected = True  # Optimistic
                return True
                
        except serial.SerialException as e:
            logger.error(f"Failed to connect to ESP32: {e}")
            self._connected = False
            return False
        except Exception as e:
            logger.error(f"ESP32 connection error: {e}")
            self._connected = False
            return False
    
    def _send_command(self, command: str, wait_response: bool = True) -> tuple:
        """
        Send command to ESP32 and get response.
        
        Args:
            command: Command string (DROP, HOLD, TEST, STATUS)
            wait_response: Whether to wait for response
            
        Returns:
            Tuple of (success: bool, response: str)
        """
        if not self._connected or not self.serial:
            logger.error("ESP32 not connected")
            return False, "Not connected"
        
        try:
            # Clear input buffer
            self.serial.reset_input_buffer()
            
            # Send command
            cmd_bytes = f"{command.upper()}\n".encode('utf-8')
            self.serial.write(cmd_bytes)
            logger.debug(f"Sent: {command}")
            
            if not wait_response:
                return True, ""
            
            # Wait for response
            time.sleep(0.5)
            
            # Read response (wait for OK or ERROR)
            response = ""
            start_time = time.time()
            while time.time() - start_time < self.serial_timeout:
                if self.serial.in_waiting:
                    chunk = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                    response += chunk
                    if "OK:" in response or "ERROR:" in response:
                        break
                time.sleep(0.1)
            
            logger.debug(f"Response: {response.strip()}")
            
            success = "OK:" in response or "SERVO:" in response
            return success, response.strip()
            
        except Exception as e:
            logger.error(f"Command error: {e}")
            return False, str(e)
    
    def hold(self) -> bool:
        """
        Set servo to hold position (retract/lock payload).
        
        Returns:
            True if command sent successfully
        """
        logger.debug("Setting ESP32 servo to HOLD position")
        success, response = self._send_command("HOLD")
        return success
    
    def drop(self) -> bool:
        """
        Execute payload drop sequence.
        
        Returns:
            True if drop executed successfully, False if no payloads remain
        """
        if self.payload_count <= 0:
            logger.warning("DROP ABORTED: No payloads remaining!")
            return False
        
        logger.info(f"Executing ESP32 drop sequence... (remaining: {self.payload_count})")
        
        # Send drop command (ESP32 handles the timing)
        success, response = self._send_command("DROP")
        
        if not success:
            logger.error(f"ESP32 drop command failed: {response}")
            return False
        
        # Wait for drop to complete (ESP32 also waits internally)
        time.sleep(self.drop_stabilize_time + 0.5)
        
        # Update counts
        self.payload_count -= 1
        self.drops_performed += 1
        self.last_drop_time = time.time()
        
        logger.info(f"Drop successful! Remaining payloads: {self.payload_count}")
        return True
    
    def remaining(self) -> int:
        """Get remaining payload count."""
        return self.payload_count
    
    def is_empty(self) -> bool:
        """Check if all payloads have been dropped."""
        return self.payload_count <= 0
    
    def get_stats(self) -> dict:
        """Get payload mechanism statistics."""
        return {
            'initial_count': self.initial_count,
            'remaining': self.payload_count,
            'drops_performed': self.drops_performed,
            'last_drop_time': self.last_drop_time,
            'is_empty': self.is_empty(),
            'connected': self._connected,
            'port': self.serial_port
        }
    
    def reset_count(self, count: int = None):
        """Reset payload count (after reloading)."""
        self.payload_count = count if count is not None else self.initial_count
        self.drops_performed = 0
        self.last_drop_time = None
        logger.info(f"Payload count reset to {self.payload_count}")
    
    def test_servo(self) -> bool:
        """Test servo operation without affecting payload count."""
        logger.info("Testing ESP32 servo mechanism...")
        success, response = self._send_command("TEST")
        
        if success:
            logger.info("Servo test completed successfully")
        else:
            logger.error(f"Servo test failed: {response}")
        
        return success
    
    def get_status(self) -> dict:
        """Get ESP32 status."""
        success, response = self._send_command("STATUS")
        return {
            'connected': self._connected,
            'response': response if success else None
        }
    
    def disconnect(self):
        """Close serial connection."""
        if self.serial:
            try:
                self.serial.close()
            except:
                pass
        self._connected = False
        logger.info("ESP32 disconnected")
    
    def __del__(self):
        """Cleanup on destruction."""
        self.disconnect()


# Backward compatibility alias
PayloadServo = PayloadESP32
