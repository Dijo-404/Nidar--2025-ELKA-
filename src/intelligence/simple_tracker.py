"""
Simple Centroid Tracker - Lightweight object tracking

A simple, fast tracker that works by associating detections
based on centroid distance. No external dependencies required.

Ideal for real-time drone applications where speed matters.
"""

import numpy as np
from collections import OrderedDict
from scipy.spatial import distance as dist
from typing import List, Tuple, Optional


class CentroidTracker:
    """
    Simple centroid-based object tracker.
    
    Tracks objects by computing centroids and matching them
    across frames using Euclidean distance.
    
    Much lighter than ByteTrack - perfect for real-time drone detection.
    """
    
    def __init__(self, max_disappeared: int = 30, max_distance: float = 50.0):
        """
        Initialize CentroidTracker.
        
        Args:
            max_disappeared: Max frames before losing track
            max_distance: Max distance for centroid matching (pixels)
        """
        self.next_object_id = 0
        self.objects = OrderedDict()  # ID -> centroid
        self.disappeared = OrderedDict()  # ID -> frames disappeared
        self.bboxes = OrderedDict()  # ID -> bounding box
        
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance
    
    def register(self, centroid: Tuple[float, float], bbox: List[float]):
        """Register a new object."""
        self.objects[self.next_object_id] = centroid
        self.bboxes[self.next_object_id] = bbox
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1
    
    def deregister(self, object_id: int):
        """Remove a tracked object."""
        del self.objects[object_id]
        del self.bboxes[object_id]
        del self.disappeared[object_id]
    
    def update(self, detections: List[List[float]]) -> List[Tuple[int, List[float]]]:
        """
        Update tracker with new detections.
        
        Args:
            detections: List of [x1, y1, x2, y2] bounding boxes
            
        Returns:
            List of (track_id, bbox) tuples
        """
        # No detections - mark all as disappeared
        if len(detections) == 0:
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id)
            
            return [(oid, self.bboxes[oid]) for oid in self.objects.keys()]
        
        # Compute centroids for input detections
        input_centroids = np.zeros((len(detections), 2), dtype=np.float32)
        for i, det in enumerate(detections):
            x1, y1, x2, y2 = det[:4]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            input_centroids[i] = (cx, cy)
        
        # First frame - register all detections
        if len(self.objects) == 0:
            for i, det in enumerate(detections):
                self.register(tuple(input_centroids[i]), det)
        else:
            # Get existing object IDs and centroids
            object_ids = list(self.objects.keys())
            object_centroids = list(self.objects.values())
            
            # Compute distance between existing and new centroids
            D = dist.cdist(np.array(object_centroids), input_centroids)
            
            # Find closest matches
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            
            used_rows = set()
            used_cols = set()
            
            for (row, col) in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue
                
                # Check if distance is within threshold
                if D[row, col] > self.max_distance:
                    continue
                
                object_id = object_ids[row]
                self.objects[object_id] = tuple(input_centroids[col])
                self.bboxes[object_id] = detections[col]
                self.disappeared[object_id] = 0
                
                used_rows.add(row)
                used_cols.add(col)
            
            # Handle unmatched existing objects
            unused_rows = set(range(len(object_centroids))) - used_rows
            for row in unused_rows:
                object_id = object_ids[row]
                self.disappeared[object_id] += 1
                
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id)
            
            # Register new objects
            unused_cols = set(range(len(input_centroids))) - used_cols
            for col in unused_cols:
                self.register(tuple(input_centroids[col]), detections[col])
        
        return [(oid, self.bboxes[oid]) for oid in self.objects.keys()]
    
    def reset(self):
        """Reset tracker state."""
        self.next_object_id = 0
        self.objects.clear()
        self.disappeared.clear()
        self.bboxes.clear()


class SimpleTracker:
    """
    Even simpler tracker using IOU matching.
    
    Zero external dependencies - uses only numpy.
    Perfect for embedded/resource-constrained systems.
    """
    
    def __init__(self, max_age: int = 5, min_iou: float = 0.3):
        """
        Initialize SimpleTracker.
        
        Args:
            max_age: Max frames to keep track without detection
            min_iou: Minimum IOU for matching
        """
        self.max_age = max_age
        self.min_iou = min_iou
        self.tracks = {}  # track_id -> {'bbox': [], 'age': 0}
        self.next_id = 1
    
    @staticmethod
    def iou(box1: List[float], box2: List[float]) -> float:
        """Compute IOU between two boxes [x1, y1, x2, y2]."""
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        
        if x2 <= x1 or y2 <= y1:
            return 0.0
        
        intersection = (x2 - x1) * (y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        
        return intersection / (area1 + area2 - intersection + 1e-6)
    
    def update(self, detections: List[List[float]]) -> List[Tuple[int, List[float]]]:
        """
        Update tracks with new detections.
        
        Args:
            detections: List of [x1, y1, x2, y2] bounding boxes
            
        Returns:
            List of (track_id, bbox) tuples
        """
        # Age all tracks
        for tid in list(self.tracks.keys()):
            self.tracks[tid]['age'] += 1
            if self.tracks[tid]['age'] > self.max_age:
                del self.tracks[tid]
        
        if len(detections) == 0:
            return [(tid, t['bbox']) for tid, t in self.tracks.items()]
        
        # Match detections to tracks
        matched = set()
        for det in detections:
            best_iou = 0
            best_tid = None
            
            for tid, track in self.tracks.items():
                if tid in matched:
                    continue
                iou_score = self.iou(det, track['bbox'])
                if iou_score > best_iou and iou_score >= self.min_iou:
                    best_iou = iou_score
                    best_tid = tid
            
            if best_tid is not None:
                self.tracks[best_tid]['bbox'] = det
                self.tracks[best_tid]['age'] = 0
                matched.add(best_tid)
            else:
                # New track
                self.tracks[self.next_id] = {'bbox': det, 'age': 0}
                self.next_id += 1
        
        return [(tid, t['bbox']) for tid, t in self.tracks.items()]
    
    def reset(self):
        """Reset tracker state."""
        self.tracks.clear()
        self.next_id = 1
