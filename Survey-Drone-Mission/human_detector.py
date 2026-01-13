"""
Human Detector - YOLO-based human detection with BoT-SORT tracking

Uses custom YOLO model and BoT-SORT tracker for accurate human
tracking and counting from drone footage.
"""

import logging
import time
import tempfile
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass, field
from pathlib import Path
import cv2
import numpy as np

logger = logging.getLogger(__name__)

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    logger.warning("ultralytics not installed - detection disabled")


@dataclass
class Detection:
    """Single detection with tracking info."""
    box: Tuple[int, int, int, int]  # x1, y1, x2, y2
    confidence: float
    track_id: Optional[int] = None
    class_id: int = 0
    class_name: str = "person"


@dataclass
class DetectionResult:
    """Result from detection/tracking on a single frame."""
    detections: List[Detection] = field(default_factory=list)
    timestamp: float = 0.0
    frame: Optional[np.ndarray] = None
    inference_time: float = 0.0
    new_track_ids: List[int] = field(default_factory=list)  # Newly seen track IDs
    
    @property
    def count(self) -> int:
        return len(self.detections)
    
    @property
    def detected(self) -> bool:
        return self.count > 0
    
    @property
    def track_ids(self) -> List[int]:
        return [d.track_id for d in self.detections if d.track_id is not None]


def create_botsort_config(config: dict) -> str:
    """Create BoT-SORT tracker configuration YAML file."""
    yaml_content = f"""# BoT-SORT Configuration (Optimized for Drone)
tracker_type: botsort

# Tracking thresholds
track_high_thresh: {config.get('track_high_thresh', 0.5)}
track_low_thresh: {config.get('track_low_thresh', 0.1)}
new_track_thresh: {config.get('new_track_thresh', 0.7)}
track_buffer: {config.get('track_buffer', 90)}
match_thresh: {config.get('match_thresh', 0.7)}

# Camera Motion Compensation (important for drone)
gmc_method: {config.get('gmc_method', 'sparseOptFlow')}

# Re-ID / Appearance
with_reid: {str(config.get('with_reid', True)).lower()}
proximity_thresh: {config.get('proximity_thresh', 0.5)}
appearance_thresh: {config.get('appearance_thresh', 0.25)}
model: auto

# Other
fuse_score: true
"""
    # Write to temp file
    config_path = Path(tempfile.gettempdir()) / "botsort_survey.yaml"
    config_path.write_text(yaml_content)
    return str(config_path)


class HumanDetector:
    """
    YOLO-based human detector with BoT-SORT tracking.
    
    Uses custom trained model and maintains persistent track IDs
    across frames for accurate human counting.
    """
    
    def __init__(self, config: dict = None):
        """
        Initialize detector with optional tracking.
        
        Args:
            config: Detection and tracking configuration
        """
        self.config = config or {}
        self.model = None
        
        # Model settings
        self.model_path = self.config.get('model', 'yolov8n.pt')
        self.confidence = self.config.get('confidence', 0.4)
        self.iou_threshold = self.config.get('iou_threshold', 0.5)
        self.classes = self.config.get('classes', [0])
        self.imgsz = self.config.get('imgsz', 640)
        self.device = self.config.get('device', '')
        
        # Tracking settings
        self.use_tracker = self.config.get('use_tracker', True)
        self.tracker_config_path: Optional[str] = None
        
        # State
        self.total_detections = 0
        self.frames_processed = 0
        self.seen_track_ids: Set[int] = set()
        self.unique_count = 0
        
        if YOLO_AVAILABLE:
            self._setup()
    
    def _setup(self):
        """Set up model and tracker."""
        # Create BoT-SORT config if tracking enabled
        if self.use_tracker:
            self.tracker_config_path = create_botsort_config(self.config)
            logger.info(f"Created BoT-SORT config: {self.tracker_config_path}")
        
        # Load model
        self._load_model()
    
    def _load_model(self):
        """Load YOLO model."""
        try:
            logger.info(f"Loading YOLO model: {self.model_path}")
            self.model = YOLO(self.model_path)
            
            if self.device:
                self.model.to(self.device)
            
            # Warm up
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self.model.predict(dummy, verbose=False)
            
            logger.info("YOLO model loaded and warmed up")
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            self.model = None
    
    def detect(self, frame: np.ndarray, draw: bool = False) -> DetectionResult:
        """
        Run detection (and tracking if enabled) on a single frame.
        
        Args:
            frame: BGR image from OpenCV
            draw: Whether to draw boxes on frame
            
        Returns:
            DetectionResult with detections and tracking info
        """
        result = DetectionResult(timestamp=time.time())
        
        if self.model is None or frame is None:
            return result
        
        try:
            start_time = time.time()
            
            # Run inference with or without tracking
            if self.use_tracker and self.tracker_config_path:
                results = self.model.track(
                    frame,
                    persist=True,
                    tracker=self.tracker_config_path,
                    conf=self.confidence,
                    iou=self.iou_threshold,
                    classes=self.classes,
                    imgsz=self.imgsz,
                    verbose=False
                )
            else:
                results = self.model(
                    frame,
                    conf=self.confidence,
                    iou=self.iou_threshold,
                    classes=self.classes,
                    verbose=False
                )
            
            result.inference_time = time.time() - start_time
            
            # Process results
            if len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes
                
                # Check if tracking IDs available
                has_ids = self.use_tracker and boxes.id is not None
                
                for i in range(len(boxes)):
                    box = boxes.xyxy[i].cpu().numpy().astype(int)
                    conf = float(boxes.conf[i].cpu().numpy())
                    cls = int(boxes.cls[i].cpu().numpy())
                    
                    track_id = None
                    if has_ids:
                        track_id = int(boxes.id[i].cpu().numpy())
                        
                        # Check if this is a new track
                        if track_id not in self.seen_track_ids:
                            self.seen_track_ids.add(track_id)
                            self.unique_count += 1
                            result.new_track_ids.append(track_id)
                    
                    detection = Detection(
                        box=(box[0], box[1], box[2], box[3]),
                        confidence=conf,
                        track_id=track_id,
                        class_id=cls,
                        class_name="person"
                    )
                    result.detections.append(detection)
            
            self.frames_processed += 1
            self.total_detections += result.count
            
            # Draw boxes if requested
            if draw:
                result.frame = self.draw_detections(frame.copy(), result)
            else:
                result.frame = frame
                
        except Exception as e:
            logger.error(f"Detection error: {e}")
            result.frame = frame
        
        return result
    
    def draw_detections(self, frame: np.ndarray, result: DetectionResult) -> np.ndarray:
        """
        Draw detection boxes with track IDs on frame.
        
        Args:
            frame: Image to draw on
            result: Detection result
            
        Returns:
            Frame with boxes drawn
        """
        for det in result.detections:
            x1, y1, x2, y2 = det.box
            
            # Color based on whether newly tracked
            if det.track_id in result.new_track_ids:
                color = (0, 0, 255)  # Red for new
                thickness = 3
            else:
                color = (0, 255, 0)  # Green for existing
                thickness = 2
            
            # Draw box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            
            # Draw label with track ID
            if det.track_id is not None:
                label = f"ID:{det.track_id} {det.confidence:.2f}"
            else:
                label = f"{det.class_name}: {det.confidence:.2f}"
            
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(frame, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), color, -1)
            cv2.putText(frame, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Draw center point
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            cv2.circle(frame, (center_x, center_y), 4, color, -1)
        
        return frame
    
    def get_stats(self) -> dict:
        """Get detection and tracking statistics."""
        return {
            'frames_processed': self.frames_processed,
            'total_detections': self.total_detections,
            'unique_tracks': self.unique_count,
            'seen_track_ids': len(self.seen_track_ids),
            'model_loaded': self.model is not None,
            'tracker_enabled': self.use_tracker
        }
    
    def get_unique_count(self) -> int:
        """Get count of unique tracked individuals."""
        return self.unique_count
    
    def reset_tracking(self):
        """Reset tracking state."""
        self.seen_track_ids.clear()
        self.unique_count = 0
        logger.info("Tracking state reset")
    
    def is_available(self) -> bool:
        """Check if detector is ready."""
        return self.model is not None


# Test
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    # Test with custom model
    config = {
        'model': '/home/dj/Projects/Nidar--2025-ELKA-/best_model/dj_yolo_best/weights/best.pt',
        'use_tracker': True,
        'confidence': 0.4
    }
    
    detector = HumanDetector(config)
    print(f"Detector available: {detector.is_available()}")
    print(f"Stats: {detector.get_stats()}")
    
    # Test with webcam
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        print("Testing with webcam... Press 'q' to quit")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            result = detector.detect(frame, draw=True)
            
            if result.frame is not None:
                # Add stats overlay
                stats = f"Unique: {detector.get_unique_count()} | Current: {result.count}"
                cv2.putText(result.frame, stats, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow("Detection", result.frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        print(f"Final stats: {detector.get_stats()}")
