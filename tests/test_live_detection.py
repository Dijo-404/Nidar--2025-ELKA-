#!/usr/bin/env python3
"""
Test Live Detection with ByteTrack - Stable bounding boxes with object tracking

This script tests the complete detection pipeline with tracking:
1. Connect to drone via MAVLink (optional)
2. Connect to SIYI camera via RTSP
3. Run YOLO model with ByteTrack for stable tracking
4. Display smooth, stable bounding boxes with track IDs
5. Show GPS coordinates when human detected

Usage:
    python test_live_detection.py [--no-mavlink] [--rtsp URL] [--save]

Press 'q' to quit, 's' to save current frame
"""

import os
import sys
import time
import argparse
import signal
import cv2
import yaml
import numpy as np
from pathlib import Path
from collections import defaultdict

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Global flag for graceful shutdown
SHUTDOWN_REQUESTED = False

def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    global SHUTDOWN_REQUESTED
    print("\n[INFO] Shutdown signal received, cleaning up...")
    SHUTDOWN_REQUESTED = True

# Register signal handlers
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Check for required dependencies
TRACKING_AVAILABLE = True
USE_SIMPLE_TRACKER = True  # Use lightweight tracker instead of ByteTrack

def check_dependencies():
    """Check if all required dependencies are installed."""
    global TRACKING_AVAILABLE, USE_SIMPLE_TRACKER
    missing = []
    
    try:
        import scipy
    except ImportError:
        missing.append("scipy")
        TRACKING_AVAILABLE = False
    
    try:
        from ultralytics import YOLO
    except ImportError:
        missing.append("ultralytics")
    
    # Check for ByteTrack (optional)
    try:
        import lap
        USE_SIMPLE_TRACKER = False  # ByteTrack available, but we still prefer simple
    except ImportError:
        USE_SIMPLE_TRACKER = True
    
    if missing:
        print("="*60)
        print("MISSING DEPENDENCIES")
        print("="*60)
        print(f"  Missing packages: {', '.join(missing)}")
        print(f"\n  Install with:")
        print(f"    pip install {' '.join(missing)}")
        print("="*60)
        
        if "ultralytics" in missing:
            sys.exit(1)
    
    print(f"  Tracker: {'Simple (lightweight)' if USE_SIMPLE_TRACKER else 'ByteTrack'}")

check_dependencies()


def load_config():
    """Load configuration files."""
    config_dir = Path(__file__).parent.parent / 'config'
    
    try:
        with open(config_dir / 'mission_params.yaml', 'r') as f:
            mission_config = yaml.safe_load(f)
        
        with open(config_dir / 'network_map.yaml', 'r') as f:
            network_config = yaml.safe_load(f)
        
        return mission_config, network_config
    except Exception as e:
        print(f"[ERROR] Failed to load config: {e}")
        return {}, {}


class BoundingBoxSmoother:
    """Smooths bounding boxes over time for stable display."""
    
    def __init__(self, smoothing_factor: float = 0.7):
        self.smoothing_factor = smoothing_factor
        self.tracked_boxes = {}
        self.last_seen = {}
        self.max_age = 30
    
    def update(self, track_id: int, box: list, frame_num: int) -> list:
        """Update and return smoothed bounding box."""
        try:
            box = np.array(box, dtype=np.float32)
            
            if track_id in self.tracked_boxes:
                prev_box = self.tracked_boxes[track_id]
                smoothed = self.smoothing_factor * prev_box + (1 - self.smoothing_factor) * box
            else:
                smoothed = box
            
            self.tracked_boxes[track_id] = smoothed
            self.last_seen[track_id] = frame_num
            self._cleanup(frame_num)
            
            return smoothed.astype(int).tolist()
        except Exception:
            return [int(x) for x in box]
    
    def _cleanup(self, current_frame: int):
        """Remove tracks not seen recently."""
        try:
            expired = [tid for tid, last in self.last_seen.items() 
                       if current_frame - last > self.max_age]
            for tid in expired:
                del self.tracked_boxes[tid]
                del self.last_seen[tid]
        except Exception:
            pass


def create_video_capture(rtsp_url: str, use_tcp: bool = True):
    """
    Create optimized video capture for RTSP stream.
    
    Args:
        rtsp_url: RTSP URL
        use_tcp: Use TCP transport (more reliable than UDP)
    
    Returns:
        cv2.VideoCapture object or None
    """
    # Set environment to avoid Qt warnings
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp|stimeout;5000000'
    
    # Try different backends
    backends = [
        (cv2.CAP_FFMPEG, "FFMPEG"),
        (cv2.CAP_GSTREAMER, "GStreamer"),
        (cv2.CAP_ANY, "Any"),
    ]
    
    for backend, name in backends:
        try:
            if use_tcp and backend == cv2.CAP_FFMPEG:
                # Use TCP transport for RTSP (more reliable)
                cap = cv2.VideoCapture(rtsp_url, backend)
            else:
                cap = cv2.VideoCapture(rtsp_url, backend)
            
            if cap.isOpened():
                # Set buffer size to 1 for low latency
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                return cap
        except Exception:
            continue
    
    # Fallback to default
    return cv2.VideoCapture(rtsp_url)


def test_rtsp_connection(rtsp_url: str) -> bool:
    """Test RTSP stream connection."""
    print(f"\nTesting RTSP connection: {rtsp_url}")
    print("  (This may take a few seconds...)")
    
    try:
        cap = create_video_capture(rtsp_url)
        
        if not cap or not cap.isOpened():
            print("  [FAIL] Could not open RTSP stream")
            return False
        
        # Try to read a frame with timeout
        start_time = time.time()
        ret = False
        frame = None
        
        while time.time() - start_time < 10:  # 10 second timeout
            ret, frame = cap.read()
            if ret and frame is not None:
                break
            time.sleep(0.1)
        
        cap.release()
        
        if ret and frame is not None:
            print(f"  [OK] RTSP stream working ({frame.shape[1]}x{frame.shape[0]})")
            return True
        else:
            print("  [FAIL] Could not read frame from stream")
            return False
    except Exception as e:
        print(f"  [FAIL] RTSP error: {e}")
        return False


def test_mavlink_connection(connection_string: str):
    """Test MAVLink connection and get GPS."""
    print(f"\nTesting MAVLink connection: {connection_string}")
    
    try:
        from pymavlink import mavutil
        
        mav = mavutil.mavlink_connection(connection_string)
        msg = mav.wait_heartbeat(timeout=5)
        
        if msg:
            print(f"  [OK] Connected to system {mav.target_system}")
            
            gps_msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if gps_msg:
                lat = gps_msg.lat / 1e7
                lon = gps_msg.lon / 1e7
                alt = gps_msg.relative_alt / 1000.0
                print(f"  [OK] GPS: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
            else:
                print("  [WARN] No GPS data received")
            
            mav.close()
            return True
        else:
            print("  [FAIL] No heartbeat received")
            return False
            
    except ImportError:
        print("  [SKIP] pymavlink not installed")
        return False
    except Exception as e:
        print(f"  [FAIL] {e}")
        return False


def test_yolo_model(model_path: str):
    """Test YOLO model loading."""
    print(f"\nTesting YOLO model: {model_path}")
    
    if not os.path.exists(model_path):
        print(f"  [FAIL] Model file not found")
        return None
    
    try:
        from ultralytics import YOLO
        model = YOLO(model_path)
        print(f"  [OK] Model loaded successfully")
        return model
    except Exception as e:
        print(f"  [FAIL] {e}")
        return None


def get_track_color(track_id: int) -> tuple:
    """Get consistent color for a track ID."""
    colors = [
        (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255),
        (0, 165, 255), (255, 255, 0), (128, 0, 128), (0, 128, 128),
    ]
    return colors[int(track_id) % len(colors)]


def run_live_detection(rtsp_url: str, model, mavlink_conn=None, 
                       save_detections: bool = False, confidence: float = 0.7):
    """Run live detection with ByteTrack tracking on RTSP stream."""
    global SHUTDOWN_REQUESTED
    
    print("\n" + "="*60)
    print("LIVE DETECTION" + (" WITH BYTETRACK" if TRACKING_AVAILABLE else " (NO TRACKING)"))
    print("Press 'q' to quit, 's' to save frame, 't' to toggle tracking")
    print("="*60 + "\n")
    
    cap = None
    mav = None
    
    try:
        # Open video stream with optimized settings
        print("[INFO] Opening RTSP stream...")
        cap = create_video_capture(rtsp_url)
        
        if not cap or not cap.isOpened():
            print("[ERROR] Failed to open RTSP stream")
            return
        
        # Create output directory
        output_dir = Path(__file__).parent.parent / 'logs' / 'detections'
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize smoother for stable bounding boxes
        smoother = BoundingBoxSmoother(smoothing_factor=0.6)
        
        # Initialize lightweight tracker
        from src.intelligence.simple_tracker import SimpleTracker
        tracker = SimpleTracker(max_age=10, min_iou=0.3)
        
        # Track history for drawing trails
        track_history = defaultdict(lambda: [])
        
        frame_count = 0
        detection_count = 0
        unique_tracks = set()
        fps_start = time.time()
        fps = 0
        use_tracking = TRACKING_AVAILABLE
        reconnect_attempts = 0
        max_reconnect_attempts = 5
        
        # MAVLink connection for GPS
        if mavlink_conn:
            try:
                from pymavlink import mavutil
                mav = mavutil.mavlink_connection(mavlink_conn)
                mav.wait_heartbeat(timeout=3)
                print("[INFO] MAVLink connected for GPS")
            except Exception as e:
                print(f"[WARN] MAVLink connection failed: {e}")
                mav = None
        
        print("[INFO] Starting detection loop (SimpleTracker enabled)...")
        
        while not SHUTDOWN_REQUESTED:
            # Read frame
            try:
                ret, frame = cap.read()
            except Exception as e:
                print(f"[ERROR] Frame read error: {e}")
                ret = False
            
            if not ret or frame is None:
                reconnect_attempts += 1
                print(f"[WARN] Stream lost, reconnecting... ({reconnect_attempts}/{max_reconnect_attempts})")
                
                if reconnect_attempts >= max_reconnect_attempts:
                    print("[ERROR] Max reconnect attempts reached, exiting")
                    break
                
                try:
                    cap.release()
                except:
                    pass
                
                time.sleep(2)
                
                try:
                    cap = create_video_capture(rtsp_url)
                except Exception as e:
                    print(f"[ERROR] Reconnect failed: {e}")
                
                continue
            
            # Reset reconnect counter on successful frame
            reconnect_attempts = 0
            frame_count += 1
            
            # Calculate FPS
            if frame_count % 30 == 0:
                elapsed = time.time() - fps_start
                fps = 30 / elapsed if elapsed > 0 else 0
                fps_start = time.time()
            
            # Run detection (always use simple detection, tracking is done separately)
            human_detected = False
            current_gps = None
            
            try:
                # Run YOLO detection only (no built-in tracking)
                results = model(frame, conf=confidence, classes=[0], verbose=False)
            except Exception as e:
                print(f"[WARN] Detection error: {e}")
                continue
            
            # Collect detections for tracking
            detections = []
            det_confs = []
            
            try:
                for result in results:
                    if result.boxes is not None and len(result.boxes) > 0:
                        boxes = result.boxes.xyxy.cpu().numpy()
                        confs = result.boxes.conf.cpu().numpy()
                        
                        for i, (box, conf) in enumerate(zip(boxes, confs)):
                            if conf >= confidence:
                                detections.append(box.tolist())
                                det_confs.append(conf)
                
                # Update tracker with detections
                tracked_objects = tracker.update(detections) if use_tracking else []
                
                # If no tracking, create fake track IDs
                if not tracked_objects and detections:
                    tracked_objects = [(i, det) for i, det in enumerate(detections)]
                
                # Process tracked objects
                for track_id, box in tracked_objects:
                    human_detected = True
                    detection_count += 1
                    unique_tracks.add(int(track_id))
                    
                    x1, y1, x2, y2 = [int(v) for v in box[:4]]
                    
                    # Smooth the bounding box
                    if use_tracking:
                        smoothed = smoother.update(
                            int(track_id), 
                            [x1, y1, x2, y2], 
                            frame_count
                        )
                        x1, y1, x2, y2 = smoothed
                    
                    # Get confidence for this detection
                    conf = det_confs[0] if det_confs else 0.7
                    
                    color = get_track_color(track_id)
                    
                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    
                    # Draw label
                    label = f"ID:{int(track_id)} {conf:.0%}"
                    (label_w, label_h), _ = cv2.getTextSize(
                        label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                    )
                    cv2.rectangle(frame, (x1, y1-label_h-10), 
                                (x1+label_w+10, y1), color, -1)
                    cv2.putText(frame, label, (x1+5, y1-5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # Track trail
                    if use_tracking:
                        center_x = int((x1 + x2) / 2)
                        center_y = int(y2)
                        track_history[int(track_id)].append((center_x, center_y))
                        
                        if len(track_history[int(track_id)]) > 30:
                            track_history[int(track_id)].pop(0)
                        
                        if len(track_history[int(track_id)]) > 1:
                            points = np.array(track_history[int(track_id)], dtype=np.int32)
                            cv2.polylines(frame, [points], False, color, 2)
            except Exception as e:
                print(f"[WARN] Processing error: {e}")
            
            # Get GPS when human detected
            if human_detected and mav:
                try:
                    gps_msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                    if gps_msg:
                        lat = gps_msg.lat / 1e7
                        lon = gps_msg.lon / 1e7
                        alt = gps_msg.relative_alt / 1000.0
                        current_gps = (lat, lon, alt)
                except:
                    pass
            
            # Draw status panel
            try:
                panel_height = 100
                cv2.rectangle(frame, (0, 0), (300, panel_height), (0, 0, 0), -1)
                cv2.rectangle(frame, (0, 0), (300, panel_height), (100, 100, 100), 2)
                
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                tracking_text = "ON" if can_track else "OFF"
                tracking_color = (0, 255, 0) if can_track else (0, 0, 255)
                cv2.putText(frame, f"Track: {tracking_text}", (120, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, tracking_color, 2)
                
                cv2.putText(frame, f"Detections: {detection_count}", (10, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f"Unique: {len(unique_tracks)}", (10, 75),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                if current_gps:
                    gps_text = f"GPS: {current_gps[0]:.6f}, {current_gps[1]:.6f}"
                    cv2.putText(frame, gps_text, (10, frame.shape[0]-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            except Exception:
                pass
            
            # Show frame
            try:
                cv2.imshow("Nidar - Live Detection", frame)
            except Exception as e:
                print(f"[ERROR] Display error: {e}")
                break
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[INFO] Quit requested")
                break
            elif key == ord('s'):
                try:
                    filename = output_dir / f"frame_{int(time.time())}.jpg"
                    cv2.imwrite(str(filename), frame)
                    print(f"[INFO] Saved: {filename}")
                except Exception as e:
                    print(f"[ERROR] Save failed: {e}")
            elif key == ord('t') and TRACKING_AVAILABLE:
                use_tracking = not use_tracking
                print(f"[INFO] Tracking: {'ON' if use_tracking else 'OFF'}")
                track_history.clear()
                smoother = BoundingBoxSmoother(smoothing_factor=0.6)
    
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\n[INFO] Cleaning up...")
        
        try:
            cv2.destroyAllWindows()
        except:
            pass
        
        try:
            if cap is not None:
                cap.release()
        except:
            pass
        
        try:
            if mav is not None:
                mav.close()
        except:
            pass
        
        print(f"\nSession complete:")
        print(f"  Total frames: {frame_count}")
        print(f"  Total detections: {detection_count}")
        print(f"  Unique tracks: {len(unique_tracks)}")


def main():
    parser = argparse.ArgumentParser(description="Test live detection with ByteTrack")
    parser.add_argument('--rtsp', help='RTSP URL (default: from config)')
    parser.add_argument('--mavlink', help='MAVLink connection string')
    parser.add_argument('--no-mavlink', action='store_true', help='Skip MAVLink')
    parser.add_argument('--model', help='Path to YOLO model')
    parser.add_argument('--confidence', type=float, default=0.7, help='Confidence threshold')
    parser.add_argument('--save', action='store_true', help='Auto-save detection frames')
    parser.add_argument('--test-only', action='store_true', help='Only test connections')
    
    args = parser.parse_args()
    
    # Load config
    mission_config, network_config = load_config()
    
    # Get parameters
    rtsp_url = args.rtsp or mission_config.get('camera', {}).get('rtsp_url', '')
    mavlink_conn = args.mavlink or network_config.get('drone1', {}).get('mavlink_connection', '')
    model_path = args.model or mission_config.get('detection', {}).get('model_path', '')
    
    print("="*60)
    print("NIDAR - Live Detection Test")
    print("="*60)
    print(f"\nRTSP URL: {rtsp_url}")
    print(f"MAVLink: {mavlink_conn}")
    print(f"Model: {model_path}")
    print(f"Confidence: {args.confidence}")
    print(f"Tracking: {'Available' if TRACKING_AVAILABLE else 'Not available (install lap)'}")
    
    # Test connections
    rtsp_ok = test_rtsp_connection(rtsp_url)
    
    mavlink_ok = False
    if not args.no_mavlink:
        mavlink_ok = test_mavlink_connection(mavlink_conn)
    
    model = test_yolo_model(model_path)
    
    # Summary
    print("\n" + "="*60)
    print("CONNECTION SUMMARY")
    print("="*60)
    print(f"  RTSP Stream: {'OK' if rtsp_ok else 'FAIL'}")
    print(f"  MAVLink: {'OK' if mavlink_ok else 'SKIP' if args.no_mavlink else 'FAIL'}")
    print(f"  YOLO Model: {'OK' if model else 'FAIL'}")
    
    if args.test_only:
        print("\nTest-only mode, exiting.")
        return
    
    if rtsp_ok and model:
        run_live_detection(
            rtsp_url, 
            model, 
            mavlink_conn=mavlink_conn if not args.no_mavlink else None,
            save_detections=args.save,
            confidence=args.confidence
        )
    else:
        print("\n[ERROR] Cannot run live detection. Fix the issues above.")
        sys.exit(1)


if __name__ == "__main__":
    main()
