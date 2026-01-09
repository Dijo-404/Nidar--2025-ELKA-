#!/usr/bin/env python3
"""
Test Human Tracker - BoT-SORT Human Tracking with Counting

This script tests the HumanTracker module:
1. Load YOLO model with BoT-SORT tracker
2. Process video file with persistent tracking
3. Count unique persons in counting zone
4. Output annotated video with tracking visualization

Usage:
    python test_human_tracker.py --video /path/to/video.mp4 [--output /path/to/output.mp4]
    python test_human_tracker.py --video sample.mp4 --no-display --confidence 0.5

Press 'q' to quit during playback
"""

import os
import sys
import argparse
import logging
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def check_dependencies():
    """Check if all required dependencies are installed."""
    missing = []
    
    try:
        from ultralytics import YOLO
    except ImportError:
        missing.append("ultralytics")
    
    try:
        import cv2
    except ImportError:
        missing.append("opencv-python")
    
    if missing:
        print("=" * 60)
        print("MISSING DEPENDENCIES")
        print("=" * 60)
        print(f"  Missing packages: {', '.join(missing)}")
        print(f"\n  Install with:")
        print(f"    pip install {' '.join(missing)}")
        print("=" * 60)
        sys.exit(1)


def load_config():
    """Load mission configuration."""
    import yaml
    
    config_path = Path(__file__).parent.parent / 'config' / 'mission_params.yaml'
    
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        logger.warning(f"Failed to load config: {e}")
        return {}


def main():
    parser = argparse.ArgumentParser(
        description="Test Human Tracker with BoT-SORT",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--video', '-v', type=str, required=True,
                       help='Path to input video file')
    parser.add_argument('--model', '-m', type=str, default=None,
                       help='Path to YOLO model weights')
    parser.add_argument('--confidence', type=float, default=0.4,
                       help='Detection confidence threshold')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output video file path')
    parser.add_argument('--no-display', action='store_true',
                       help='Disable display window')
    parser.add_argument('--target-height', type=int, default=720,
                       help='Target frame height')
    parser.add_argument('--box-ratio', type=float, default=0.9,
                       help='Counting box size ratio (0.0-1.0)')
    parser.add_argument('--imgsz', type=int, default=640,
                       help='YOLO inference image size')
    
    args = parser.parse_args()
    
    # Check dependencies
    check_dependencies()
    
    # Load config
    config = load_config()
    detection_config = config.get('detection', {})
    tracking_config = config.get('tracking', {})
    
    # Get model path
    model_path = args.model
    if not model_path:
        model_path = detection_config.get('model_path', '')
        if model_path and not os.path.isabs(model_path):
            project_root = Path(__file__).parent.parent
            model_path = str(project_root / model_path)
    
    print("=" * 60)
    print("NIDAR - Human Tracker Test (BoT-SORT)")
    print("=" * 60)
    print(f"\nVideo: {args.video}")
    print(f"Model: {model_path}")
    print(f"Confidence: {args.confidence}")
    print(f"Box Ratio: {args.box_ratio}")
    print(f"Display: {'No' if args.no_display else 'Yes'}")
    if args.output:
        print(f"Output: {args.output}")
    
    # Validate inputs
    if not os.path.exists(args.video):
        logger.error(f"Video file not found: {args.video}")
        sys.exit(1)
    
    if not model_path or not os.path.exists(model_path):
        logger.error(f"Model file not found: {model_path}")
        sys.exit(1)
    
    print("\n" + "=" * 60)
    print("STARTING TRACKER")
    print("Press 'q' to quit during playback")
    print("=" * 60 + "\n")
    
    # Import tracker
    from src.intelligence.human_tracker import DroneTrackerConfig, run_tracker
    
    # Create config
    tracker_config = DroneTrackerConfig(
        model_path=model_path,
        target_height=args.target_height,
        box_width_ratio=args.box_ratio,
        box_height_ratio=args.box_ratio,
        confidence_threshold=args.confidence,
        imgsz=args.imgsz,
        # Apply tracking config from YAML
        track_high_thresh=tracking_config.get('track_high_thresh', 0.5),
        track_low_thresh=tracking_config.get('track_low_thresh', 0.1),
        new_track_thresh=tracking_config.get('new_track_thresh', 0.7),
        track_buffer=tracking_config.get('track_buffer', 90),
        match_thresh=tracking_config.get('match_thresh', 0.7),
        with_reid=tracking_config.get('with_reid', True),
    )
    
    # Run tracker
    result = run_tracker(
        video_path=args.video,
        config=tracker_config,
        output_path=args.output,
        display=not args.no_display
    )
    
    if result:
        count, counted_ids = result
        print("\n" + "=" * 60)
        print("TRACKING COMPLETE")
        print("=" * 60)
        print(f"  Total Count: {count}")
        print(f"  Unique IDs: {sorted(counted_ids)}")
        print("=" * 60)


if __name__ == "__main__":
    main()
