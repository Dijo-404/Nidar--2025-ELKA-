"""
Video Display - Real-time video feed with overlays

Displays:
- Live video feed
- Detection bounding boxes
- GPS coordinates
- Waypoint information
- Detection statistics
"""

import logging
import time
from typing import Optional, List, Tuple
import cv2
import numpy as np

logger = logging.getLogger(__name__)


class VideoDisplay:
    """
    Real-time video display with information overlays.
    """
    
    def __init__(self, config: dict = None):
        """
        Initialize video display.
        
        Args:
            config: Display configuration
        """
        self.config = config or {}
        self.window_name = "Survey Mission - Live Feed"
        self.window_width = self.config.get('window_width', 1280)
        self.window_height = self.config.get('window_height', 720)
        self.font_scale = self.config.get('font_scale', 0.6)
        
        self.show_gps = self.config.get('show_gps', True)
        self.show_detections = self.config.get('show_detections', True)
        self.show_waypoints = self.config.get('show_waypoints', True)
        
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.colors = {
            'text': (255, 255, 255),
            'background': (40, 40, 40),
            'detection': (0, 255, 0),
            'warning': (0, 165, 255),
            'gps': (255, 200, 0),
            'waypoint': (200, 200, 200)
        }
        
        self.window_created = False
        self.last_frame_time = time.time()
        self.fps = 0.0
    
    def create_window(self):
        """Create display window."""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        self.window_created = True
    
    def _draw_text_with_background(self, frame: np.ndarray, text: str, 
                                    pos: Tuple[int, int], color: Tuple[int, int, int],
                                    bg_color: Tuple[int, int, int] = None):
        """Draw text with background rectangle."""
        bg_color = bg_color or self.colors['background']
        
        (w, h), _ = cv2.getTextSize(text, self.font, self.font_scale, 1)
        x, y = pos
        
        cv2.rectangle(frame, (x - 2, y - h - 4), (x + w + 2, y + 4), bg_color, -1)
        cv2.putText(frame, text, pos, self.font, self.font_scale, color, 1, cv2.LINE_AA)
    
    def draw_gps_overlay(self, frame: np.ndarray, gps_data: dict):
        """
        Draw GPS information overlay.
        
        Args:
            frame: Image to draw on
            gps_data: {'lat': float, 'lon': float, 'alt': float}
        """
        if not self.show_gps or not gps_data:
            return
        
        lat = gps_data.get('lat', 0)
        lon = gps_data.get('lon', 0)
        alt = gps_data.get('alt', 0)
        
        text = f"GPS: {lat:.6f}, {lon:.6f} | Alt: {alt:.1f}m"
        self._draw_text_with_background(frame, text, (10, 30), self.colors['gps'])
    
    def draw_detection_overlay(self, frame: np.ndarray, detection_count: int, 
                               total_detections: int, unique_count: int = 0):
        """
        Draw detection statistics overlay.
        
        Args:
            frame: Image to draw on
            detection_count: Current frame detection count
            total_detections: Total detections this session
            unique_count: Unique tracked individuals
        """
        if not self.show_detections:
            return
        
        h = frame.shape[0]
        
        # Current detections
        if detection_count > 0:
            text = f"HUMANS DETECTED: {detection_count}"
            color = self.colors['detection']
        else:
            text = "No detections"
            color = self.colors['text']
        
        self._draw_text_with_background(frame, text, (10, h - 60), color)
        
        # Unique count (tracked individuals)
        if unique_count > 0:
            text = f"Unique Tracked: {unique_count}"
            self._draw_text_with_background(frame, text, (10, h - 35), self.colors['gps'])
        
        # Total count
        text = f"Total Detections: {total_detections}"
        self._draw_text_with_background(frame, text, (10, h - 10), self.colors['text'])
    
    def draw_waypoint_overlay(self, frame: np.ndarray, current_wp: int, 
                              total_wp: int, waypoints: List[Tuple[float, float, float]] = None):
        """
        Draw waypoint progress overlay.
        
        Args:
            frame: Image to draw on
            current_wp: Current waypoint index
            total_wp: Total waypoint count
            waypoints: Optional list of (lat, lon, alt) tuples
        """
        if not self.show_waypoints or total_wp == 0:
            return
        
        w = frame.shape[1]
        
        text = f"Waypoint: {current_wp}/{total_wp}"
        self._draw_text_with_background(frame, text, (w - 180, 30), self.colors['waypoint'])
        
        # Progress bar
        bar_width = 150
        bar_height = 10
        progress = current_wp / total_wp
        
        x = w - 170
        y = 45
        
        cv2.rectangle(frame, (x, y), (x + bar_width, y + bar_height), 
                     self.colors['background'], -1)
        cv2.rectangle(frame, (x, y), (x + int(bar_width * progress), y + bar_height), 
                     self.colors['detection'], -1)
        cv2.rectangle(frame, (x, y), (x + bar_width, y + bar_height), 
                     self.colors['waypoint'], 1)
    
    def draw_fps(self, frame: np.ndarray):
        """Draw FPS counter."""
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
        
        w = frame.shape[1]
        text = f"FPS: {self.fps:.1f}"
        self._draw_text_with_background(frame, text, (w - 100, frame.shape[0] - 15), 
                                        self.colors['text'])
    
    def draw_timestamp(self, frame: np.ndarray):
        """Draw current timestamp."""
        from datetime import datetime
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        w = frame.shape[1]
        
        self._draw_text_with_background(frame, timestamp, (w - 200, 30), self.colors['text'])
    
    def show(self, frame: np.ndarray, 
             gps_data: dict = None,
             detection_count: int = 0,
             total_detections: int = 0,
             unique_count: int = 0,
             current_waypoint: int = 0,
             total_waypoints: int = 0) -> int:
        """
        Display frame with all overlays.
        
        Args:
            frame: Video frame to display
            gps_data: GPS position data
            detection_count: Current detection count
            total_detections: Total detections
            unique_count: Unique tracked individuals
            current_waypoint: Current waypoint index
            total_waypoints: Total waypoint count
            
        Returns:
            Key code pressed (-1 if none)
        """
        if not self.window_created:
            self.create_window()
        
        if frame is None:
            return -1
        
        # Draw overlays
        self.draw_gps_overlay(frame, gps_data)
        self.draw_detection_overlay(frame, detection_count, total_detections, unique_count)
        self.draw_waypoint_overlay(frame, current_waypoint, total_waypoints)
        self.draw_fps(frame)
        
        # Instructions
        h = frame.shape[0]
        instructions = "Q: Quit | S: Screenshot | R: Reset count"
        self._draw_text_with_background(frame, instructions, (10, h - 70), 
                                        self.colors['waypoint'])
        
        cv2.imshow(self.window_name, frame)
        
        return cv2.waitKey(1) & 0xFF
    
    def save_screenshot(self, frame: np.ndarray, output_dir: str = "output") -> str:
        """Save current frame as screenshot."""
        import os
        from datetime import datetime
        
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"screenshot_{timestamp}.jpg"
        filepath = os.path.join(output_dir, filename)
        
        cv2.imwrite(filepath, frame)
        logger.info(f"Screenshot saved: {filepath}")
        
        return filepath
    
    def close(self):
        """Close display window."""
        if self.window_created:
            cv2.destroyWindow(self.window_name)
            self.window_created = False


# Test
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    display = VideoDisplay()
    
    # Test with webcam
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        print("Press 'q' to quit, 's' for screenshot")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            key = display.show(
                frame,
                gps_data={'lat': 12.9710, 'lon': 77.5910, 'alt': 25.0},
                detection_count=2,
                total_detections=15,
                current_waypoint=5,
                total_waypoints=20
            )
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                display.save_screenshot(frame)
        
        cap.release()
    
    display.close()
