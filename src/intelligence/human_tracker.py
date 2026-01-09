"""
Drone Human Tracking & Counting System using BoT-SORT
Optimized for aerial footage at 10m altitude with accurate tracking.

This module provides BoT-SORT based human tracking and counting for
drone missions. It maintains persistent track IDs across frames and
counts unique persons entering a defined zone.
"""

from ultralytics import YOLO
import cv2 as cv
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, Set, List, Callable
import argparse
import logging
from pathlib import Path
import time

logger = logging.getLogger(__name__)


@dataclass
class DroneTrackerConfig:
    """Configuration optimized for drone human tracking at 10m altitude."""
    
    # --- Video Processing ---
    target_height: int = 720  # Target frame height
    
    # --- Counting Box (as percentage of frame, 0.0 to 1.0) ---
    box_width_ratio: float = 0.9   # 90% of frame width
    box_height_ratio: float = 0.9  # 90% of frame height
    box_x_offset: Optional[int] = None  # None = center
    box_y_offset: Optional[int] = None  # None = center
    box_color: Tuple[int, int, int] = (0, 255, 0)
    box_thickness: int = 2
    
    # --- YOLO Detection (optimized for small objects from drone) ---
    model_path: str = '/home/dj/Projects/Nidar--2025-ELKA-/best_model/dj_yolo_best/weights/best.pt'
    confidence_threshold: float = 0.4
    iou_threshold: float = 0.5
    target_class_id: int = 0  # Person class
    imgsz: int = 640  # Detection size
    
    # --- BoT-SORT Tracker (tuned for stable ID assignment) ---
    tracker_type: str = "botsort"
    track_high_thresh: float = 0.5     # High confidence detection threshold
    track_low_thresh: float = 0.1      # Low confidence for second association
    new_track_thresh: float = 0.7      # Higher = fewer spurious new tracks
    track_buffer: int = 90             # 1.5 seconds at 60 FPS to keep lost tracks
    match_thresh: float = 0.7          # Lowered for better matching
    # Camera Motion Compensation
    gmc_method: str = "sparseOptFlow"
    # Re-ID (appearance matching)
    with_reid: bool = True
    proximity_thresh: float = 0.5
    appearance_thresh: float = 0.25
    
    # --- Display ---
    window_name: str = "Drone Tracker - BoT-SORT"
    show_fps: bool = True
    count_color: Tuple[int, int, int] = (0, 0, 255)


@dataclass
class TrackingResult:
    """Result of a tracking update."""
    frame_count: int
    track_ids: List[int]
    boxes: List[List[float]]  # [x1, y1, x2, y2] for each track
    confidences: List[float]
    total_count: int
    new_counts: List[int]  # IDs that were just counted this frame
    timestamp: float = field(default_factory=time.time)


def create_botsort_config(config: DroneTrackerConfig) -> str:
    """Create BoT-SORT tracker configuration YAML."""
    yaml_content = f"""# BoT-SORT Configuration (Optimized for Drone at 10m)
tracker_type: botsort

# Tracking thresholds
track_high_thresh: {config.track_high_thresh}
track_low_thresh: {config.track_low_thresh}
new_track_thresh: {config.new_track_thresh}
track_buffer: {config.track_buffer}
match_thresh: {config.match_thresh}

# Camera Motion Compensation (important for drone)
gmc_method: {config.gmc_method}

# Re-ID / Appearance
with_reid: {str(config.with_reid).lower()}
proximity_thresh: {config.proximity_thresh}
appearance_thresh: {config.appearance_thresh}
model: auto

# Other
fuse_score: true
"""
    # Write to temp file
    config_path = Path("/tmp/botsort_drone.yaml")
    config_path.write_text(yaml_content)
    return str(config_path)


class DroneHumanCounter:
    """
    Simple robust human counter - counts each unique track ID once when inside the box.
    """
    
    def __init__(self):
        self.counted_ids: Set[int] = set()
        self.count = 0
    
    def update(self, track_id: int, center: Tuple[int, int], 
               box_x: int, box_y: int, box_width: int, box_height: int) -> bool:
        """
        Check if track should be counted.
        Counts when a track is FIRST SEEN inside the counting box.
        Returns True if this update triggered a count.
        """
        center_x, center_y = center
        
        # Check if inside counting box
        inside_box = (box_x < center_x < box_x + box_width and
                      box_y < center_y < box_y + box_height)
        
        # Count if inside box and not already counted
        if inside_box and track_id not in self.counted_ids:
            self.counted_ids.add(track_id)
            self.count += 1
            logger.info(f"✅ Counted! ID: {track_id} | Total: {self.count}")
            return True
        
        return False
    
    def reset(self):
        """Reset the counter."""
        self.counted_ids.clear()
        self.count = 0


class HumanTracker:
    """
    BoT-SORT based human tracking system for drone missions.
    
    Provides persistent tracking IDs and counting functionality
    for aerial human detection.
    """
    
    def __init__(self, model_path: str = None, config: dict = None):
        """
        Initialize HumanTracker with YOLO model.
        
        Args:
            model_path: Path to YOLO model weights (.pt file)
            config: Optional configuration dictionary
        """
        self.config = DroneTrackerConfig()
        
        # Override config with provided values
        if config:
            for key, value in config.items():
                if hasattr(self.config, key):
                    setattr(self.config, key, value)
        
        if model_path:
            self.config.model_path = model_path
        
        # Create BoT-SORT config file
        self.tracker_config_path = create_botsort_config(self.config)
        logger.info(f"Created BoT-SORT config at: {self.tracker_config_path}")
        
        # Initialize counter
        self.counter = DroneHumanCounter()
        
        # Load model
        self.model: Optional[YOLO] = None
        self._load_model()
        
        # State
        self.frame_count = 0
        self.is_tracking = False
        
        # Callbacks
        self.on_new_count: Optional[Callable[[int, int], None]] = None
    
    def _load_model(self):
        """Load YOLO model from specified path."""
        try:
            logger.info(f"Loading YOLO model from {self.config.model_path}...")
            self.model = YOLO(self.config.model_path)
            
            # Warm up
            dummy = np.zeros((self.config.imgsz, self.config.imgsz, 3), dtype=np.uint8)
            self.model.predict(dummy, verbose=False)
            
            logger.info("Model loaded and warmed up")
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            raise
    
    def track_frame(self, frame: np.ndarray, 
                    box_x: int = None, box_y: int = None,
                    box_width: int = None, box_height: int = None) -> TrackingResult:
        """
        Track humans in a single frame.
        
        Args:
            frame: Input frame (BGR format)
            box_x, box_y, box_width, box_height: Counting zone (optional)
            
        Returns:
            TrackingResult with track information
        """
        self.frame_count += 1
        self.is_tracking = True
        
        h, w = frame.shape[:2]
        
        # Default counting zone
        if box_width is None:
            box_width = int(w * self.config.box_width_ratio)
        if box_height is None:
            box_height = int(h * self.config.box_height_ratio)
        if box_x is None:
            box_x = (w - box_width) // 2
        if box_y is None:
            box_y = (h - box_height) // 2
        
        # Run YOLO with BoT-SORT tracking
        results = self.model.track(
            frame,
            persist=True,
            tracker=self.tracker_config_path,
            conf=self.config.confidence_threshold,
            iou=self.config.iou_threshold,
            classes=[self.config.target_class_id],
            imgsz=self.config.imgsz,
            verbose=False
        )
        
        track_ids = []
        boxes = []
        confidences = []
        new_counts = []
        
        if results[0].boxes is not None and results[0].boxes.id is not None:
            boxes_arr = results[0].boxes.xyxy.cpu().numpy()
            ids_arr = results[0].boxes.id.cpu().numpy().astype(int)
            confs_arr = results[0].boxes.conf.cpu().numpy()
            
            for box, track_id, conf in zip(boxes_arr, ids_arr, confs_arr):
                x1, y1, x2, y2 = map(int, box)
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                
                track_ids.append(int(track_id))
                boxes.append([x1, y1, x2, y2])
                confidences.append(float(conf))
                
                # Update counter
                just_counted = self.counter.update(
                    track_id, (center_x, center_y),
                    box_x, box_y, box_width, box_height
                )
                
                if just_counted:
                    new_counts.append(int(track_id))
                    if self.on_new_count:
                        self.on_new_count(track_id, self.counter.count)
        
        return TrackingResult(
            frame_count=self.frame_count,
            track_ids=track_ids,
            boxes=boxes,
            confidences=confidences,
            total_count=self.counter.count,
            new_counts=new_counts
        )
    
    def draw_tracking(self, frame: np.ndarray, result: TrackingResult,
                      box_x: int = None, box_y: int = None,
                      box_width: int = None, box_height: int = None) -> np.ndarray:
        """
        Draw tracking visualization on frame.
        
        Args:
            frame: Input frame
            result: TrackingResult from track_frame()
            box_x, box_y, box_width, box_height: Counting zone
            
        Returns:
            Annotated frame
        """
        h, w = frame.shape[:2]
        
        # Default counting zone
        if box_width is None:
            box_width = int(w * self.config.box_width_ratio)
        if box_height is None:
            box_height = int(h * self.config.box_height_ratio)
        if box_x is None:
            box_x = (w - box_width) // 2
        if box_y is None:
            box_y = (h - box_height) // 2
        
        # Draw counting zone
        cv.rectangle(frame, (box_x, box_y), (box_x + box_width, box_y + box_height),
                     self.config.box_color, self.config.box_thickness)
        
        # Draw detections
        for i, (track_id, box, conf) in enumerate(zip(result.track_ids, result.boxes, result.confidences)):
            x1, y1, x2, y2 = box
            color = (0, 255, 0)  # Green
            
            # Flash effect for new counts
            if track_id in result.new_counts:
                cv.rectangle(frame, (box_x, box_y),
                           (box_x + box_width, box_y + box_height),
                           (0, 0, 255), 4)
            
            # Draw bounding box
            cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw ID and confidence
            label = f"ID:{track_id} {conf:.2f}"
            cv.putText(frame, label, (x1, y1 - 10),
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Draw center point
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            cv.circle(frame, (center_x, center_y), 4, color, -1)
        
        # Draw count
        cv.putText(frame, f"Count: {result.total_count}", (50, 80),
                  cv.FONT_HERSHEY_SIMPLEX, 2, self.config.count_color, 4)
        
        return frame
    
    def get_count(self) -> int:
        """Get current count."""
        return self.counter.count
    
    def get_counted_ids(self) -> Set[int]:
        """Get set of counted track IDs."""
        return self.counter.counted_ids.copy()
    
    def reset(self):
        """Reset tracker state."""
        self.counter.reset()
        self.frame_count = 0
        self.is_tracking = False
        self._geotagged_ids: Set[int] = set()
    
    def track_and_geotag(self, frame: np.ndarray,
                         drone_lat: float, drone_lon: float, 
                         drone_alt: float, drone_heading: float = 0.0,
                         frame_width: int = None, frame_height: int = None):
        """
        Track humans and compute geotagged GPS for NEW detections only.
        
        This is the primary method for mission integration. It:
        1. Runs BoT-SORT tracking on the frame
        2. For each NEW track ID (not previously geotagged), computes target GPS
        3. Returns only NEW geotagged detections to avoid duplicate transmissions
        
        Args:
            frame: Input frame (BGR format)
            drone_lat: Current drone latitude
            drone_lon: Current drone longitude
            drone_alt: Current drone altitude above ground (meters)
            drone_heading: Drone heading in degrees (0 = North)
            frame_width: Optional override for frame width (for GSD calculation)
            frame_height: Optional override for frame height
            
        Returns:
            List of GeoTaggedDetection objects for NEW tracks only
        """
        from src.intelligence.geotagging import GeoTagger, GeoTaggedDetection
        
        # Initialize geotagged IDs set if not exists
        if not hasattr(self, '_geotagged_ids'):
            self._geotagged_ids: Set[int] = set()
        
        # Get frame dimensions
        h, w = frame.shape[:2]
        actual_width = frame_width or w
        actual_height = frame_height or h
        
        # Create geotagger with current frame dimensions
        geotagger = GeoTagger({
            'frame_width': actual_width,
            'frame_height': actual_height,
            'sensor_width_mm': 7.6,  # Default values, can be overridden
            'focal_length_mm': 4.4,
            'gimbal_pitch_deg': 90,  # Nadir
        })
        
        # Run tracking
        self.frame_count += 1
        self.is_tracking = True
        
        results = self.model.track(
            frame,
            persist=True,
            tracker=self.tracker_config_path,
            conf=self.config.confidence_threshold,
            iou=self.config.iou_threshold,
            classes=[self.config.target_class_id],
            imgsz=self.config.imgsz,
            verbose=False
        )
        
        new_geotagged = []
        
        if results[0].boxes is not None and results[0].boxes.id is not None:
            boxes_arr = results[0].boxes.xyxy.cpu().numpy()
            ids_arr = results[0].boxes.id.cpu().numpy().astype(int)
            confs_arr = results[0].boxes.conf.cpu().numpy()
            
            for box, track_id, conf in zip(boxes_arr, ids_arr, confs_arr):
                track_id = int(track_id)
                
                # Only geotag NEW track IDs
                if track_id in self._geotagged_ids:
                    continue
                
                # Mark as geotagged
                self._geotagged_ids.add(track_id)
                
                # Calculate detection center in xywh format
                x1, y1, x2, y2 = box
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                width = x2 - x1
                height = y2 - y1
                
                # Geotag the detection
                geotagged = geotagger.geotag_detection(
                    box_xywh=[center_x, center_y, width, height],
                    confidence=float(conf),
                    drone_lat=drone_lat,
                    drone_lon=drone_lon,
                    drone_alt=drone_alt,
                    drone_heading=drone_heading,
                    track_id=track_id
                )
                
                if geotagged.target_lat and geotagged.target_lon:
                    new_geotagged.append(geotagged)
                    logger.info(f"🎯 New track ID:{track_id} geotagged -> "
                               f"Target: ({geotagged.target_lat:.6f}, {geotagged.target_lon:.6f})")
        
        return new_geotagged
    
    def get_geotagged_count(self) -> int:
        """Get count of geotagged track IDs."""
        if hasattr(self, '_geotagged_ids'):
            return len(self._geotagged_ids)
        return 0


def run_tracker(video_path: str, config: DroneTrackerConfig,
                output_path: Optional[str] = None, display: bool = True):
    """
    Main tracking and counting function using BoT-SORT.
    """
    # Validate inputs
    if not Path(video_path).exists():
        logger.error(f"Video file not found: {video_path}")
        return
    
    if not Path(config.model_path).exists():
        logger.error(f"Model file not found: {config.model_path}")
        return

    # Initialize tracker
    tracker = HumanTracker(config.model_path)
    tracker.config = config

    # Open video
    cap = cv.VideoCapture(video_path)
    if not cap.isOpened():
        logger.error(f"Failed to open video: {video_path}")
        return
    
    # Video properties
    total_frames = int(cap.get(cv.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv.CAP_PROP_FPS) or 30.0
    orig_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    orig_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    
    logger.info(f"Video: {total_frames} frames, {fps:.1f} FPS, {orig_w}x{orig_h}")

    # Calculate scaling
    scale = config.target_height / orig_h
    scaled_w = int(orig_w * scale)
    scaled_h = config.target_height
    
    # Box dimensions based on percentage of scaled frame
    scaled_box_w = int(scaled_w * config.box_width_ratio)
    scaled_box_h = int(scaled_h * config.box_height_ratio)
    
    # Box position (centered by default)
    box_x = config.box_x_offset if config.box_x_offset else (scaled_w - scaled_box_w) // 2
    box_y = config.box_y_offset if config.box_y_offset else (scaled_h - scaled_box_h) // 2
    box_x = max(0, min(box_x, scaled_w - scaled_box_w))
    box_y = max(0, min(box_y, scaled_h - scaled_box_h))
    
    logger.info(f"Counting zone: {scaled_box_w}x{scaled_box_h} at ({box_x}, {box_y})")
    
    # Video writer
    writer = None
    if output_path:
        fourcc = cv.VideoWriter_fourcc(*'mp4v')
        writer = cv.VideoWriter(output_path, fourcc, fps, (scaled_w, scaled_h))
        logger.info(f"Output: {output_path}")

    frame_count = 0
    process_times = deque(maxlen=30)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            frame_count += 1
            start_time = time.perf_counter()
            
            # Resize
            frame = cv.resize(frame, (scaled_w, scaled_h), interpolation=cv.INTER_LINEAR)
            
            # Track
            result = tracker.track_frame(frame, box_x, box_y, scaled_box_w, scaled_box_h)
            
            # Draw visualization
            frame = tracker.draw_tracking(frame, result, box_x, box_y, scaled_box_w, scaled_box_h)
            
            # Calculate and show FPS
            process_time = time.perf_counter() - start_time
            process_times.append(process_time)
            
            if config.show_fps and process_times:
                avg_fps = 1.0 / (sum(process_times) / len(process_times))
                cv.putText(frame, f"FPS: {avg_fps:.1f}", (scaled_w - 150, 40),
                          cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Write output
            if writer:
                writer.write(frame)
            
            # Display
            if display:
                cv.imshow(config.window_name, frame)
                if cv.waitKey(1) & 0xFF == ord("q"):
                    logger.info("User quit")
                    break
            
            # Progress
            if frame_count % 100 == 0:
                progress = (frame_count / total_frames) * 100
                avg_fps = 1.0 / (sum(process_times) / len(process_times)) if process_times else 0
                logger.info(f"Progress: {progress:.1f}% | FPS: {avg_fps:.1f} | Count: {result.total_count}")

    except KeyboardInterrupt:
        logger.info("Interrupted")
    except Exception as e:
        logger.error(f"Error: {e}")
        raise
    finally:
        cap.release()
        if writer:
            writer.release()
        cv.destroyAllWindows()
        
        final_count = tracker.get_count()
        counted_ids = tracker.get_counted_ids()
        
        logger.info(f"✅ Complete! Total count: {final_count}, Frames: {frame_count}")
        logger.info(f"   Unique IDs counted: {len(counted_ids)}")
        
        return final_count, counted_ids


def create_argument_parser() -> argparse.ArgumentParser:
    """Create command line argument parser."""
    parser = argparse.ArgumentParser(
        description='Drone Human Tracking with BoT-SORT',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument('--video', '-v', type=str, required=True,
                       help='Path to input video file')
    parser.add_argument('--model', '-m', type=str, default=None,
                       help='Path to YOLO model weights')
    parser.add_argument('--target-height', type=int, default=720,
                       help='Target frame height for processing')
    parser.add_argument('--box-ratio', type=float, default=0.9,
                       help='Counting box size as ratio of frame (0.0-1.0)')
    parser.add_argument('--confidence', type=float, default=0.4,
                       help='Detection confidence threshold')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output video file path')
    parser.add_argument('--no-display', action='store_true',
                       help='Disable display window')
    parser.add_argument('--imgsz', type=int, default=640,
                       help='YOLO inference image size')
    
    return parser


def main():
    """Entry point."""
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    parser = create_argument_parser()
    args = parser.parse_args()
    
    config = DroneTrackerConfig(
        target_height=args.target_height,
        box_width_ratio=args.box_ratio,
        box_height_ratio=args.box_ratio,
        confidence_threshold=args.confidence,
        imgsz=args.imgsz,
    )
    
    if args.model:
        config.model_path = args.model
    
    run_tracker(
        video_path=args.video,
        config=config,
        output_path=args.output,
        display=not args.no_display
    )


if __name__ == "__main__":
    main()
