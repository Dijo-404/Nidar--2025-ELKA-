#!/usr/bin/env python3
"""
Test Live Detection - Connect to drone camera and run YOLO detection

This script tests the complete detection pipeline:
1. Connect to drone via MAVLink (optional)
2. Connect to SIYI camera via RTSP
3. Run YOLO model on live video feed
4. Display detections with bounding boxes
5. Show GPS coordinates when human detected

Usage:
    python test_live_detection.py [--no-mavlink] [--rtsp URL] [--save]

Press 'q' to quit, 's' to save current frame
"""

import os
import sys
import time
import argparse
import cv2
import yaml
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))


def load_config():
    """Load configuration files."""
    config_dir = Path(__file__).parent.parent / 'config'
    
    with open(config_dir / 'mission_params.yaml', 'r') as f:
        mission_config = yaml.safe_load(f)
    
    with open(config_dir / 'network_map.yaml', 'r') as f:
        network_config = yaml.safe_load(f)
    
    return mission_config, network_config


def test_rtsp_connection(rtsp_url: str) -> bool:
    """Test RTSP stream connection."""
    print(f"\nTesting RTSP connection: {rtsp_url}")
    
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print("  [FAIL] Could not open RTSP stream")
        return False
    
    ret, frame = cap.read()
    cap.release()
    
    if ret and frame is not None:
        print(f"  [OK] RTSP stream working ({frame.shape[1]}x{frame.shape[0]})")
        return True
    else:
        print("  [FAIL] Could not read frame from stream")
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
            
            # Get GPS
            gps_msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if gps_msg:
                lat = gps_msg.lat / 1e7
                lon = gps_msg.lon / 1e7
                alt = gps_msg.relative_alt / 1000.0
                print(f"  [OK] GPS: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
            else:
                print("  [WARN] No GPS data received")
            
            mav.close()
            return mav
        else:
            print("  [FAIL] No heartbeat received")
            return None
            
    except ImportError:
        print("  [SKIP] pymavlink not installed")
        return None
    except Exception as e:
        print(f"  [FAIL] {e}")
        return None


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


def run_live_detection(rtsp_url: str, model, mavlink_conn=None, 
                       save_detections: bool = False, confidence: float = 0.7):
    """Run live detection on RTSP stream."""
    print("\n" + "="*50)
    print("LIVE DETECTION - Press 'q' to quit, 's' to save frame")
    print("="*50 + "\n")
    
    # Open video stream
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print("Failed to open RTSP stream")
        return
    
    # Create output directory
    output_dir = Path(__file__).parent.parent / 'logs' / 'detections'
    output_dir.mkdir(parents=True, exist_ok=True)
    
    frame_count = 0
    detection_count = 0
    fps_start = time.time()
    fps = 0
    
    # MAVLink connection for GPS
    mav = None
    if mavlink_conn:
        try:
            from pymavlink import mavutil
            mav = mavutil.mavlink_connection(mavlink_conn)
            mav.wait_heartbeat(timeout=3)
        except:
            mav = None
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Stream lost, reconnecting...")
                cap.release()
                time.sleep(2)
                cap = cv2.VideoCapture(rtsp_url)
                continue
            
            frame_count += 1
            
            # Calculate FPS
            if frame_count % 30 == 0:
                fps = 30 / (time.time() - fps_start)
                fps_start = time.time()
            
            # Run detection every 2nd frame
            if frame_count % 2 == 0:
                results = model(frame, verbose=False)
                
                # Process detections
                human_detected = False
                for result in results:
                    if result.boxes is not None:
                        for i, cls_id in enumerate(result.boxes.cls.tolist()):
                            if int(cls_id) == 0:  # Person class
                                conf = result.boxes.conf[i].item()
                                if conf >= confidence:
                                    human_detected = True
                                    detection_count += 1
                                    
                                    # Draw bounding box
                                    box = result.boxes.xyxy[i].tolist()
                                    x1, y1, x2, y2 = map(int, box)
                                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                    cv2.putText(frame, f"Human {conf:.2f}", (x1, y1-10),
                                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Get and display GPS if human detected
                if human_detected and mav:
                    try:
                        gps_msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                        if gps_msg:
                            lat = gps_msg.lat / 1e7
                            lon = gps_msg.lon / 1e7
                            alt = gps_msg.relative_alt / 1000.0
                            gps_text = f"GPS: {lat:.6f}, {lon:.6f}, {alt:.1f}m"
                            cv2.putText(frame, gps_text, (10, frame.shape[0]-40),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            print(f"[DETECTION] Human at ({lat:.6f}, {lon:.6f})")
                    except:
                        pass
            
            # Draw status info
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Detections: {detection_count}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, "Press 'q' to quit", (10, frame.shape[0]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # Show frame
            cv2.imshow("Nidar - Live Detection", frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save current frame
                filename = output_dir / f"frame_{int(time.time())}.jpg"
                cv2.imwrite(str(filename), frame)
                print(f"Saved: {filename}")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if mav:
            mav.close()
        
        print(f"\nSession complete:")
        print(f"  Total frames: {frame_count}")
        print(f"  Total detections: {detection_count}")


def main():
    parser = argparse.ArgumentParser(description="Test live detection with drone camera")
    parser.add_argument('--rtsp', help='RTSP URL (default: from config)')
    parser.add_argument('--mavlink', help='MAVLink connection string (default: from config)')
    parser.add_argument('--no-mavlink', action='store_true', help='Skip MAVLink connection')
    parser.add_argument('--model', help='Path to YOLO model (default: from config)')
    parser.add_argument('--confidence', type=float, default=0.7, help='Detection confidence threshold')
    parser.add_argument('--save', action='store_true', help='Auto-save detection frames')
    parser.add_argument('--test-only', action='store_true', help='Only test connections, no live view')
    
    args = parser.parse_args()
    
    # Load config
    mission_config, network_config = load_config()
    
    # Get parameters
    rtsp_url = args.rtsp or mission_config.get('camera', {}).get('rtsp_url')
    mavlink_conn = args.mavlink or network_config.get('drone1', {}).get('mavlink_connection')
    model_path = args.model or mission_config.get('detection', {}).get('model_path')
    
    print("="*50)
    print("NIDAR - Live Detection Test")
    print("="*50)
    print(f"\nRTSP URL: {rtsp_url}")
    print(f"MAVLink: {mavlink_conn}")
    print(f"Model: {model_path}")
    print(f"Confidence: {args.confidence}")
    
    # Test connections
    rtsp_ok = test_rtsp_connection(rtsp_url)
    
    mavlink_ok = None
    if not args.no_mavlink:
        mavlink_ok = test_mavlink_connection(mavlink_conn)
    
    model = test_yolo_model(model_path)
    
    # Summary
    print("\n" + "="*50)
    print("CONNECTION SUMMARY")
    print("="*50)
    print(f"  RTSP Stream: {'OK' if rtsp_ok else 'FAIL'}")
    print(f"  MAVLink: {'OK' if mavlink_ok else 'SKIP' if args.no_mavlink else 'FAIL'}")
    print(f"  YOLO Model: {'OK' if model else 'FAIL'}")
    
    if args.test_only:
        print("\nTest-only mode, exiting.")
        return
    
    # Run live detection if prerequisites met
    if rtsp_ok and model:
        run_live_detection(
            rtsp_url, 
            model, 
            mavlink_conn=mavlink_conn if not args.no_mavlink else None,
            save_detections=args.save,
            confidence=args.confidence
        )
    else:
        print("\nCannot run live detection. Fix the issues above.")
        sys.exit(1)


if __name__ == "__main__":
    main()
