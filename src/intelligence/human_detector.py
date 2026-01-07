"""
HumanDetector - YOLO-based Human Detection

Runs YOLO model inference in a separate thread for real-time
human detection from video streams.
"""

import time
import logging
import threading
from queue import Queue, Empty
from typing import Optional, Tuple, List, Callable
from dataclasses import dataclass
import cv2
import numpy as np
from ultralytics import YOLO

logger = logging.getLogger(__name__)


@dataclass
class DetectionResult:
    """Result of a human detection attempt."""
    detected: bool
    boxes: List[List[float]]  # [x, y, w, h] for each detection
    confidences: List[float]
    timestamp: float
    frame: Optional[np.ndarray] = None


class HumanDetector:
    """
    YOLO-based human detection system with threaded processing.
    
    Runs detection in a background thread to avoid blocking flight control,
    with configurable confidence thresholds and callback support.
    """
    
    def __init__(self, model_path: str, config: dict = None):
        """
        Initialize HumanDetector with YOLO model.
        
        Args:
            model_path: Path to YOLO model weights (.pt file)
            config: Configuration dictionary with detection parameters
        """
        self.model_path = model_path
        self.config = config or {}
        
        # Detection parameters
        self.confidence_threshold = self.config.get('confidence_threshold', 0.70)
        self.nms_threshold = self.config.get('nms_threshold', 0.45)
        self.target_class_id = self.config.get('target_class_id', 0)  # 'person'
        self.frame_skip = self.config.get('frame_skip', 2)
        
        # Video stream parameters
        self.rtsp_url: Optional[str] = None
        self.frame_width = self.config.get('frame_width', 1920)
        self.frame_height = self.config.get('frame_height', 1080)
        self.reconnect_delay = self.config.get('reconnect_delay', 2.0)
        
        # State
        self.model: Optional[YOLO] = None
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_running = False
        self.detection_thread: Optional[threading.Thread] = None
        
        # Thread-safe communication
        self.result_queue: Queue[DetectionResult] = Queue(maxsize=10)
        self.frame_count = 0
        
        # Callbacks
        self.on_detection: Optional[Callable[[DetectionResult], None]] = None
        
        # Load model
        self._load_model()
    
    def _load_model(self):
        """Load YOLO model from specified path."""
        try:
            logger.info(f"Loading YOLO model from {self.model_path}...")
            self.model = YOLO(self.model_path)
            logger.info("YOLO model loaded successfully")
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            raise
    
    def detect(self, frame: np.ndarray, save_frame: bool = False) -> DetectionResult:
        """
        Run detection on a single frame.
        
        Args:
            frame: Input frame (BGR format from OpenCV)
            save_frame: Whether to include frame in result
            
        Returns:
            DetectionResult with detection information
        """
        if self.model is None:
            return DetectionResult(
                detected=False,
                boxes=[],
                confidences=[],
                timestamp=time.time()
            )
        
        try:
            # Run YOLO detection
            results = self.model(
                frame, 
                conf=self.confidence_threshold,
                classes=[self.target_class_id],
                verbose=False
            )
            
            # Parse detections
            boxes = []       # [x, y, w, h] format for result
            confidences = []
            
            if results[0].boxes is not None:
                for i, cls_id in enumerate(results[0].boxes.cls.tolist()):
                    if int(cls_id) == self.target_class_id:
                        conf = results[0].boxes.conf[i].item()
                        if conf >= self.confidence_threshold:
                            xywh = results[0].boxes.xywh[i].tolist()
                            boxes.append(xywh)
                            confidences.append(conf)
            
            detected = len(boxes) > 0
            
            if detected:
                logger.info(f"Human detected! {len(boxes)} detections, "
                           f"max confidence: {max(confidences):.2f}")
            
            return DetectionResult(
                detected=detected,
                boxes=boxes,
                confidences=confidences,
                timestamp=time.time(),
                frame=frame.copy() if save_frame and detected else None
            )
            
        except Exception as e:
            logger.error(f"Detection error: {e}")
            return DetectionResult(
                detected=False,
                boxes=[],
                confidences=[],
                timestamp=time.time()
            )
    
    def _create_video_capture(self, rtsp_url: str) -> Optional[cv2.VideoCapture]:
        """
        Create optimized video capture for RTSP stream.
        
        Uses TCP transport for reliability and minimal buffer for low latency.
        
        Args:
            rtsp_url: RTSP stream URL
            
        Returns:
            cv2.VideoCapture object or None
        """
        import os
        
        # Set environment to avoid Qt warnings and optimize RTSP
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
                cap = cv2.VideoCapture(rtsp_url, backend)
                
                if cap.isOpened():
                    # Set buffer size to 1 for low latency
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    logger.debug(f"Opened stream with {name} backend")
                    return cap
            except Exception as e:
                logger.debug(f"Backend {name} failed: {e}")
                continue
        
        # Fallback to default
        cap = cv2.VideoCapture(rtsp_url)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap
    
    def _open_stream(self) -> bool:
        """Open video stream connection with optimized settings."""
        if not self.rtsp_url:
            logger.error("No RTSP URL configured")
            return False
        
        try:
            logger.info(f"Opening video stream: {self.rtsp_url}")
            self.cap = self._create_video_capture(self.rtsp_url)
            
            if not self.cap or not self.cap.isOpened():
                logger.error("Failed to open video stream")
                return False
            
            logger.info("Video stream opened successfully (TCP transport, optimized)")
            return True
            
        except Exception as e:
            logger.error(f"Stream open error: {e}")
            return False
    
    def _detection_loop(self):
        """Main detection loop running in separate thread."""
        while self.is_running:
            # Check/reconnect stream
            if self.cap is None or not self.cap.isOpened():
                if not self._open_stream():
                    logger.warning("Stream reconnect failed, retrying...")
                    time.sleep(self.reconnect_delay)
                    continue
            
            # Read frame
            ret, frame = self.cap.read()
            if not ret or frame is None:
                logger.warning("Stream lost, reconnecting...")
                if self.cap:
                    self.cap.release()
                    self.cap = None
                time.sleep(self.reconnect_delay)
                continue
            
            # Frame skip for performance
            self.frame_count += 1
            if self.frame_count % self.frame_skip != 0:
                continue
            
            # Run detection
            result = self.detect(frame, save_frame=True)
            
            # Put result in queue (non-blocking)
            try:
                self.result_queue.put_nowait(result)
            except:
                # Queue full, discard old result
                try:
                    self.result_queue.get_nowait()
                    self.result_queue.put_nowait(result)
                except:
                    pass
            
            # Trigger callback if detection occurred
            if result.detected and self.on_detection:
                try:
                    self.on_detection(result)
                except Exception as e:
                    logger.error(f"Detection callback error: {e}")
    
    def start_stream(self, rtsp_url: str, 
                     on_detection: Callable[[DetectionResult], None] = None):
        """
        Start video stream processing in background thread.
        
        Args:
            rtsp_url: RTSP stream URL
            on_detection: Optional callback called when human detected
        """
        if self.is_running:
            logger.warning("Detection already running")
            return
        
        self.rtsp_url = rtsp_url
        self.on_detection = on_detection
        self.is_running = True
        self.frame_count = 0
        
        self.detection_thread = threading.Thread(
            target=self._detection_loop,
            daemon=True,
            name="HumanDetector"
        )
        self.detection_thread.start()
        logger.info("Human detection started")
    
    def stop(self):
        """Stop detection and release resources."""
        logger.info("Stopping human detection...")
        self.is_running = False
        
        if self.detection_thread and self.detection_thread.is_alive():
            self.detection_thread.join(timeout=3.0)
        
        if self.cap:
            self.cap.release()
            self.cap = None
        
        # Clear queue
        while not self.result_queue.empty():
            try:
                self.result_queue.get_nowait()
            except:
                break
        
        logger.info("Human detection stopped")
    
    def get_latest_result(self, timeout: float = 0.1) -> Optional[DetectionResult]:
        """
        Get the latest detection result from the queue.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            Latest DetectionResult or None if none available
        """
        try:
            return self.result_queue.get(timeout=timeout)
        except Empty:
            return None
    
    def get_latest_detection(self, timeout: float = 0.1) -> Optional[DetectionResult]:
        """
        Get the latest positive detection (human found).
        
        Drains the queue and returns only if a detection occurred.
        
        Args:
            timeout: Maximum time to wait
            
        Returns:
            DetectionResult where detected=True, or None
        """
        result = self.get_latest_result(timeout)
        if result and result.detected:
            return result
        return None
    
    def is_active(self) -> bool:
        """Check if detection is currently running."""
        return self.is_running and self.detection_thread is not None
    
    def save_detection_image(self, result: DetectionResult, 
                             output_dir: str, 
                             prefix: str = "detection") -> Optional[str]:
        """
        Save detection image with bounding boxes drawn.
        
        Args:
            result: DetectionResult with frame
            output_dir: Directory to save image
            prefix: Filename prefix
            
        Returns:
            Path to saved image or None
        """
        if result.frame is None:
            return None
        
        import os
        
        # Draw bounding boxes
        frame = result.frame.copy()
        for i, box in enumerate(result.boxes):
            x, y, w, h = box
            x1, y1 = int(x - w/2), int(y - h/2)
            x2, y2 = int(x + w/2), int(y + h/2)
            
            conf = result.confidences[i] if i < len(result.confidences) else 0
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"Human {conf:.2f}", (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Save image
        timestamp = int(result.timestamp)
        filename = f"{prefix}_{timestamp}.jpg"
        filepath = os.path.join(output_dir, filename)
        
        os.makedirs(output_dir, exist_ok=True)
        cv2.imwrite(filepath, frame)
        
        logger.info(f"Saved detection image: {filepath}")
        return filepath
