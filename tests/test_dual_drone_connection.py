#!/usr/bin/env python3
"""
Dual Drone Connection and Arming Test Script

This script tests connecting to both drones, arming them, and retrieving
the video feed from drone1.

Usage:
    SITL Mode:
        python test_dual_drone_connection.py --sitl
    
    Real Flight:
        python test_dual_drone_connection.py --drone1 /dev/ttyUSB0 --drone2 /dev/ttyUSB1
    
    With Video Feed:
        python test_dual_drone_connection.py --sitl --video-url rtsp://192.168.144.25:8554/main.264
"""

import sys
import os
import time
import threading
import argparse
import logging
from typing import Optional

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class DroneConnection:
    """
    A class to manage connection to a single drone for testing.
    
    Args:
        connection_string: MAVLink connection string
        drone_id: Identifier for this drone (1 or 2)
        sitl_mode: If True, use force-arm for SITL testing
    """
    
    def __init__(self, connection_string: str, drone_id: int, sitl_mode: bool = False):
        self.connection_string = connection_string
        self.drone_id = drone_id
        self.sitl_mode = sitl_mode
        self.master: Optional[mavutil.mavlink_connection] = None
        self.is_connected = False
        self.is_armed = False
        
    def log(self, message: str):
        """Print a status message with drone ID prefix."""
        logger.info(f"[Drone {self.drone_id}] {message}")
        
    def connect(self, timeout: float = 30.0) -> bool:
        """
        Establish MAVLink connection and wait for heartbeat.
        
        Args:
            timeout: Maximum time to wait for heartbeat
            
        Returns:
            True if connection successful
        """
        try:
            self.log(f"Connecting to {self.connection_string}...")
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                source_system=255,  # GCS system ID
                source_component=0,
                baud=57600
            )
            
            self.log("Waiting for heartbeat...")
            msg = self.master.wait_heartbeat(timeout=timeout)
            
            if msg:
                self.is_connected = True
                self.log(f"✓ Connected! (System: {self.master.target_system}, "
                        f"Component: {self.master.target_component})")
                return True
            else:
                self.log("✗ Heartbeat timeout - connection failed")
                return False
                
        except Exception as e:
            self.log(f"✗ Connection error: {e}")
            return False
    
    def get_system_status(self) -> dict:
        """
        Retrieve system status including battery, GPS, and mode.
        
        Returns:
            Dictionary with status information
        """
        status = {
            'battery_voltage': None,
            'battery_remaining': None,
            'gps_fix': None,
            'satellites': None,
            'mode': None,
            'armed': False
        }
        
        if not self.master:
            return status
            
        # Get heartbeat for mode and armed status
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
        if msg:
            mode_mapping = self.master.mode_mapping()
            # Find mode name from mode id
            for mode_name, mode_id in mode_mapping.items():
                if mode_id == msg.custom_mode:
                    status['mode'] = mode_name
                    break
            status['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            
        # Get battery status
        msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=2.0)
        if msg:
            status['battery_voltage'] = msg.voltage_battery / 1000.0
            status['battery_remaining'] = msg.battery_remaining
            
        # Get GPS status
        msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2.0)
        if msg:
            status['gps_fix'] = msg.fix_type
            status['satellites'] = msg.satellites_visible
            
        return status
    
    def set_mode(self, mode_name: str) -> bool:
        """
        Set the flight mode.
        
        Args:
            mode_name: Mode name (e.g., 'GUIDED', 'STABILIZE')
            
        Returns:
            True if mode set successfully
        """
        if not self.master:
            return False
            
        self.log(f"Setting mode to {mode_name}...")
        
        mode_mapping = self.master.mode_mapping()
        if mode_name not in mode_mapping:
            self.log(f"✗ Unknown mode: {mode_name}")
            return False
            
        mode_id = mode_mapping[mode_name]
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Verify mode change
        start_time = time.time()
        while time.time() - start_time < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == mode_id:
                self.log(f"✓ Mode set to {mode_name}")
                return True
                
        self.log(f"✗ Mode change to {mode_name} not confirmed")
        return False
    
    def arm(self) -> bool:
        """
        Arm the drone motors.
        
        Returns:
            True if armed successfully
        """
        if not self.master:
            return False
            
        self.log("Arming...")
        
        # Use force arm only in SITL mode
        force_arm = 21196 if self.sitl_mode else 0
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Arm
            force_arm,  # Force arm for SITL
            0, 0, 0, 0, 0
        )
        
        # Wait for ACK
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    break
                else:
                    self.log(f"✗ Arm command rejected: result={msg.result}")
                    return False
        
        # Verify armed status
        start_time = time.time()
        while time.time() - start_time < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                self.is_armed = True
                self.log("✓ Armed successfully!")
                return True
                
        self.log("✗ Arm failed - check pre-arm messages")
        return False
    
    def disarm(self) -> bool:
        """
        Disarm the drone motors.
        
        Returns:
            True if disarmed successfully
        """
        if not self.master:
            return False
            
        self.log("Disarming...")
        
        force_disarm = 21196 if self.sitl_mode else 0
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # Disarm
            force_disarm,
            0, 0, 0, 0, 0
        )
        
        time.sleep(1)
        self.is_armed = False
        self.log("✓ Disarmed")
        return True
    
    def close(self):
        """Close the MAVLink connection."""
        if self.master:
            self.master.close()
            self.is_connected = False
            self.log("Connection closed")


class VideoFeedCapture:
    """
    Captures and displays video feed from a drone's camera.
    
    Args:
        rtsp_url: RTSP stream URL
        drone_id: Identifier for the drone
    """
    
    def __init__(self, rtsp_url: str, drone_id: int = 1):
        self.rtsp_url = rtsp_url
        self.drone_id = drone_id
        self.cap = None
        self.running = False
        self.thread = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_count = 0
        
    def log(self, message: str):
        """Print a status message with prefix."""
        logger.info(f"[Drone {self.drone_id} Video] {message}")
        
    def start(self) -> bool:
        """
        Start capturing video feed in a background thread.
        
        Returns:
            True if capture started successfully
        """
        try:
            import cv2
            
            self.log(f"Opening video stream: {self.rtsp_url}")
            
            # Set RTSP transport to TCP for reliability
            os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
            
            self.cap = cv2.VideoCapture(self.rtsp_url)
            
            if not self.cap.isOpened():
                self.log("✗ Failed to open video stream")
                return False
                
            # Get video properties
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            self.log(f"✓ Video stream opened: {width}x{height} @ {fps:.1f} FPS")
            
            # Start capture thread
            self.running = True
            self.thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.thread.start()
            
            return True
            
        except ImportError:
            self.log("✗ OpenCV (cv2) not installed")
            return False
        except Exception as e:
            self.log(f"✗ Error starting video capture: {e}")
            return False
    
    def _capture_loop(self):
        """Background thread for capturing frames."""
        import cv2
        
        while self.running:
            try:
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.latest_frame = frame
                        self.frame_count += 1
                else:
                    self.log("Frame read failed - attempting reconnect...")
                    time.sleep(1)
                    self.cap.open(self.rtsp_url)
            except Exception as e:
                self.log(f"Capture error: {e}")
                time.sleep(0.1)
    
    def get_frame(self):
        """
        Get the latest captured frame.
        
        Returns:
            Latest frame or None if no frame available
        """
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None
    
    def display_live(self, duration: float = 10.0):
        """
        Display live video feed in a window.
        
        Args:
            duration: How long to display in seconds
        """
        try:
            import cv2
            
            self.log(f"Displaying live feed for {duration} seconds...")
            self.log("Press 'q' to quit early")
            
            window_name = f"Drone {self.drone_id} Video Feed"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            
            start_time = time.time()
            displayed_frames = 0
            
            while time.time() - start_time < duration:
                frame = self.get_frame()
                
                if frame is not None:
                    # Add info overlay
                    info_text = f"Drone {self.drone_id} | Frame: {self.frame_count}"
                    cv2.putText(frame, info_text, (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    cv2.imshow(window_name, frame)
                    displayed_frames += 1
                    
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.log("Display stopped by user")
                    break
                    
            cv2.destroyWindow(window_name)
            self.log(f"✓ Displayed {displayed_frames} frames")
            
        except ImportError:
            self.log("✗ OpenCV (cv2) not available for display")
    
    def save_snapshot(self, output_path: str = None) -> Optional[str]:
        """
        Save a snapshot from the video feed.
        
        Args:
            output_path: Path to save the image (auto-generated if None)
            
        Returns:
            Path to saved image or None if failed
        """
        try:
            import cv2
            
            frame = self.get_frame()
            if frame is None:
                self.log("✗ No frame available for snapshot")
                return None
                
            if output_path is None:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                output_path = f"drone{self.drone_id}_snapshot_{timestamp}.jpg"
                
            cv2.imwrite(output_path, frame)
            self.log(f"✓ Snapshot saved: {output_path}")
            return output_path
            
        except Exception as e:
            self.log(f"✗ Failed to save snapshot: {e}")
            return None
    
    def stop(self):
        """Stop video capture and release resources."""
        self.running = False
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
            
        if self.cap:
            self.cap.release()
            self.cap = None
            
        self.log("Video capture stopped")


def run_connection_test(args):
    """
    Run the dual drone connection and arming test.
    
    Args:
        args: Command line arguments
    """
    print("=" * 60)
    print("DUAL DRONE CONNECTION & ARMING TEST")
    print("=" * 60)
    
    if args.sitl:
        print("Mode: SITL (Force arm enabled)")
    else:
        print("Mode: REAL FLIGHT")
    print()
    
    # Initialize drone connections
    drone1 = DroneConnection(args.drone1, drone_id=1, sitl_mode=args.sitl)
    drone2 = DroneConnection(args.drone2, drone_id=2, sitl_mode=args.sitl)
    
    # Initialize video capture if URL provided
    video_capture = None
    if args.video_url:
        video_capture = VideoFeedCapture(args.video_url, drone_id=1)
    
    results = {
        'drone1_connected': False,
        'drone2_connected': False,
        'drone1_armed': False,
        'drone2_armed': False,
        'video_captured': False
    }
    
    try:
        # Step 1: Connect to both drones
        print("\n[STEP 1] Connecting to drones...")
        print("-" * 40)
        
        # Connect in parallel using threads
        def connect_drone(drone, results_key):
            if drone.connect(timeout=30):
                results[results_key] = True
                
        threads = [
            threading.Thread(target=connect_drone, args=(drone1, 'drone1_connected')),
            threading.Thread(target=connect_drone, args=(drone2, 'drone2_connected'))
        ]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
            
        # Check connection results
        if not results['drone1_connected']:
            print("\n[ERROR] Failed to connect to Drone 1")
        if not results['drone2_connected']:
            print("\n[ERROR] Failed to connect to Drone 2")
            
        if not (results['drone1_connected'] and results['drone2_connected']):
            print("\n[ABORT] Cannot proceed without both drones connected")
            return results
        
        # Step 2: Get system status
        print("\n[STEP 2] Checking system status...")
        print("-" * 40)
        
        for drone, name in [(drone1, "Drone 1"), (drone2, "Drone 2")]:
            status = drone.get_system_status()
            print(f"\n{name} Status:")
            print(f"  Mode: {status['mode']}")
            print(f"  Armed: {status['armed']}")
            print(f"  Battery: {status['battery_voltage']:.2f}V ({status['battery_remaining']}%)" 
                  if status['battery_voltage'] else "  Battery: Unknown")
            print(f"  GPS: Fix={status['gps_fix']}, Satellites={status['satellites']}"
                  if status['gps_fix'] else "  GPS: Unknown")
        
        # Step 3: Set GUIDED mode on both drones
        print("\n[STEP 3] Setting GUIDED mode...")
        print("-" * 40)
        
        mode_success = True
        if not drone1.set_mode('GUIDED'):
            print("[ERROR] Failed to set GUIDED mode on Drone 1")
            mode_success = False
        if not drone2.set_mode('GUIDED'):
            print("[ERROR] Failed to set GUIDED mode on Drone 2")
            mode_success = False
            
        if not mode_success:
            print("\n[ABORT] Cannot arm without GUIDED mode")
            return results
            
        time.sleep(1)
        
        # Step 4: Arm both drones
        print("\n[STEP 4] Arming both drones...")
        print("-" * 40)
        
        def arm_drone(drone, results_key):
            if drone.arm():
                results[results_key] = True
                
        threads = [
            threading.Thread(target=arm_drone, args=(drone1, 'drone1_armed')),
            threading.Thread(target=arm_drone, args=(drone2, 'drone2_armed'))
        ]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
            
        if results['drone1_armed']:
            print("\n✓ Drone 1: ARMED")
        else:
            print("\n✗ Drone 1: ARM FAILED")
            
        if results['drone2_armed']:
            print("✓ Drone 2: ARMED")
        else:
            print("✗ Drone 2: ARM FAILED")
        
        # Step 5: Start video capture from Drone 1
        if video_capture:
            print("\n[STEP 5] Starting video feed from Drone 1...")
            print("-" * 40)
            
            if video_capture.start():
                results['video_captured'] = True
                
                # Wait for frames to start coming in
                time.sleep(2)
                
                # Save a snapshot
                snapshot_path = video_capture.save_snapshot()
                
                # Display live feed if requested
                if args.display_video:
                    video_capture.display_live(duration=args.video_duration)
                else:
                    print(f"Video feed active - captured {video_capture.frame_count} frames")
                    print("(Use --display-video to show live feed)")
            else:
                print("[WARNING] Could not start video feed")
        
        # Keep drones armed for specified duration
        if results['drone1_armed'] or results['drone2_armed']:
            print(f"\n[STEP 6] Holding armed state for {args.hold_time} seconds...")
            print("-" * 40)
            time.sleep(args.hold_time)
        
    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Test interrupted by user")
        
    finally:
        # Cleanup
        print("\n[CLEANUP] Disarming and disconnecting...")
        print("-" * 40)
        
        if video_capture:
            video_capture.stop()
        
        if drone1.is_armed:
            drone1.disarm()
        if drone2.is_armed:
            drone2.disarm()
            
        drone1.close()
        drone2.close()
    
    # Print final results
    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)
    print(f"Drone 1 Connected: {'✓ Pass' if results['drone1_connected'] else '✗ Fail'}")
    print(f"Drone 2 Connected: {'✓ Pass' if results['drone2_connected'] else '✗ Fail'}")
    print(f"Drone 1 Armed:     {'✓ Pass' if results['drone1_armed'] else '✗ Fail'}")
    print(f"Drone 2 Armed:     {'✓ Pass' if results['drone2_armed'] else '✗ Fail'}")
    if args.video_url:
        print(f"Video Feed:        {'✓ Pass' if results['video_captured'] else '✗ Fail'}")
    print("=" * 60)
    
    return results


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Test dual drone connection, arming, and video feed',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    SITL Testing:
        python test_dual_drone_connection.py --sitl
        
    Real Flight (USB Telemetry):
        python test_dual_drone_connection.py --drone1 /dev/ttyUSB0 --drone2 /dev/ttyUSB1
        
    With Video Feed:
        python test_dual_drone_connection.py --sitl --video-url rtsp://192.168.144.25:8554/main.264
        
    Display Live Video:
        python test_dual_drone_connection.py --sitl --video-url <url> --display-video
        """
    )
    
    parser.add_argument('--drone1', default='udpin:127.0.0.1:14550',
                       help='Drone 1 MAVLink connection string (default: SITL)')
    parser.add_argument('--drone2', default='udpin:127.0.0.1:14560',
                       help='Drone 2 MAVLink connection string (default: SITL)')
    parser.add_argument('--sitl', action='store_true',
                       help='SITL mode: enables force-arm, skips GPS/EKF checks')
    parser.add_argument('--video-url', type=str, default=None,
                       help='RTSP URL for Drone 1 video feed')
    parser.add_argument('--display-video', action='store_true',
                       help='Display live video feed in a window')
    parser.add_argument('--video-duration', type=float, default=10.0,
                       help='Duration to display video feed (default: 10 seconds)')
    parser.add_argument('--hold-time', type=float, default=5.0,
                       help='Time to hold armed state before disarming (default: 5 seconds)')
    
    args = parser.parse_args()
    
    results = run_connection_test(args)
    
    # Exit with error code if any test failed
    all_passed = all([
        results['drone1_connected'],
        results['drone2_connected'],
        results['drone1_armed'],
        results['drone2_armed']
    ])
    
    if args.video_url:
        all_passed = all_passed and results['video_captured']
    
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
