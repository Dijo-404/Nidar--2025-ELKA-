"""
PayloadServo - Payload Drop Mechanism Driver

Controls the servo mechanism for payload delivery with count tracking
and safety features.
"""

import time
import logging
from typing import Optional
from pymavlink import mavutil

logger = logging.getLogger(__name__)


class PayloadServo:
    """
    Servo-based payload drop mechanism controller.
    
    Manages payload count, servo PWM control, and drop timing
    with strict count enforcement for mission safety.
    """
    
    def __init__(self, mav_connection, config: dict = None):
        """
        Initialize PayloadServo with MAVLink connection.
        
        Args:
            mav_connection: Active pymavlink connection object
            config: Configuration dictionary with servo parameters
        """
        self.mav = mav_connection
        self.config = config or {}
        
        # Payload configuration
        self.initial_count = self.config.get('initial_count', 10)
        self.payload_count = self.initial_count
        
        # Servo configuration
        self.servo_channel = self.config.get('servo_channel', 9)
        self.servo_drop_pwm = self.config.get('servo_drop_pwm', 2000)
        self.servo_hold_pwm = self.config.get('servo_hold_pwm', 1000)
        self.drop_stabilize_time = self.config.get('drop_stabilize_time', 2.0)
        
        # State tracking
        self.drops_performed = 0
        self.last_drop_time: Optional[float] = None
        
        logger.info(f"PayloadServo initialized: {self.payload_count} payloads, "
                   f"channel {self.servo_channel}")
    
    def _set_servo_pwm(self, pwm: int) -> bool:
        """
        Set servo PWM value via MAVLink.
        
        Args:
            pwm: PWM value (typically 1000-2000)
            
        Returns:
            True if command sent successfully
        """
        if not self.mav:
            logger.error("No MAVLink connection")
            return False
        
        try:
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # confirmation
                self.servo_channel,  # servo number
                pwm,  # PWM value
                0, 0, 0, 0, 0  # unused parameters
            )
            
            # Wait for acknowledgment
            msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
            if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return True
            else:
                logger.warning(f"Servo command may not have been accepted")
                return True  # Optimistic
                
        except Exception as e:
            logger.error(f"Failed to set servo PWM: {e}")
            return False
    
    def hold(self) -> bool:
        """
        Set servo to hold position (retract/lock payload).
        
        Returns:
            True if command sent successfully
        """
        logger.debug(f"Setting servo to hold position (PWM: {self.servo_hold_pwm})")
        return self._set_servo_pwm(self.servo_hold_pwm)
    
    def drop(self) -> bool:
        """
        Execute payload drop sequence.
        
        This method:
        1. Checks if payloads remain
        2. Triggers servo to drop position
        3. Waits for stabilization
        4. Returns servo to hold position
        5. Decrements payload count
        
        Returns:
            True if drop executed successfully, False if no payloads remain
        """
        if self.payload_count <= 0:
            logger.warning("DROP ABORTED: No payloads remaining!")
            return False
        
        logger.info(f"Executing drop sequence... (remaining before: {self.payload_count})")
        
        # Step 1: Set servo to drop position
        if not self._set_servo_pwm(self.servo_drop_pwm):
            logger.error("Failed to set drop PWM")
            return False
        
        # Step 2: Wait for drop stabilization
        time.sleep(self.drop_stabilize_time)
        
        # Step 3: Return servo to hold position
        if not self._set_servo_pwm(self.servo_hold_pwm):
            logger.warning("Failed to return servo to hold - continuing anyway")
        
        # Step 4: Update counts
        self.payload_count -= 1
        self.drops_performed += 1
        self.last_drop_time = time.time()
        
        logger.info(f"Drop successful! Remaining payloads: {self.payload_count}")
        
        return True
    
    def remaining(self) -> int:
        """
        Get remaining payload count.
        
        Returns:
            Number of payloads remaining
        """
        return self.payload_count
    
    def is_empty(self) -> bool:
        """
        Check if all payloads have been dropped.
        
        Returns:
            True if no payloads remain
        """
        return self.payload_count <= 0
    
    def get_stats(self) -> dict:
        """
        Get payload mechanism statistics.
        
        Returns:
            Dictionary with drop statistics
        """
        return {
            'initial_count': self.initial_count,
            'remaining': self.payload_count,
            'drops_performed': self.drops_performed,
            'last_drop_time': self.last_drop_time,
            'is_empty': self.is_empty()
        }
    
    def reset_count(self, count: int = None):
        """
        Reset payload count (after reloading).
        
        Args:
            count: New payload count (defaults to initial_count)
        """
        self.payload_count = count if count is not None else self.initial_count
        self.drops_performed = 0
        self.last_drop_time = None
        logger.info(f"Payload count reset to {self.payload_count}")
    
    def test_servo(self) -> bool:
        """
        Test servo operation without affecting payload count.
        
        Cycles the servo through drop and hold positions.
        
        Returns:
            True if test completed successfully
        """
        logger.info("Testing servo mechanism...")
        
        # Move to drop position
        if not self._set_servo_pwm(self.servo_drop_pwm):
            logger.error("Servo test failed: could not set drop position")
            return False
        
        time.sleep(1.0)
        
        # Return to hold position
        if not self._set_servo_pwm(self.servo_hold_pwm):
            logger.error("Servo test failed: could not set hold position")
            return False
        
        logger.info("Servo test completed successfully")
        return True
