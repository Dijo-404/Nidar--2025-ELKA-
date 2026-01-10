#!/usr/bin/env python3
"""
Dual-Drone Simulation Test

Tests the complete mission flow using mock components.
No real hardware or SITL required.

Usage:
    python simulation/test_dual_mission.py
    python simulation/test_dual_mission.py --detections 5
"""

import os
import sys
import time
import logging
import argparse
import threading
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from simulation.mock_components import (
    MockDronePilot,
    MockPayload,
    MockHumanTracker
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('SimTest')


class SimulatedSurveyDrone:
    """Simulates Drone 1 (Scout) behavior."""
    
    def __init__(self):
        self.pilot = MockDronePilot("mock://drone1")
        self.tracker = MockHumanTracker()
        self.detected_targets = []
        
    def run_survey(self, waypoints: list, num_detections: int = 3):
        """
        Simulate survey mission.
        
        Args:
            waypoints: List of (lat, lon, alt) waypoints
            num_detections: Number of simulated human detections
        """
        logger.info("=== DRONE 1: SURVEY MISSION START ===")
        
        # Connect and arm
        self.pilot.connect()
        self.pilot.arm()
        self.pilot.safe_takeoff(20.0)
        self.pilot.wait_for_altitude(20.0)
        
        # Add simulated detections
        for i in range(num_detections):
            self.tracker.add_simulated_person(
                track_id=i + 1,
                x=960 + (i * 100),  # Spread across frame
                y=540
            )
        
        # Fly waypoints
        for i, (lat, lon, alt) in enumerate(waypoints):
            logger.info(f"Waypoint {i+1}/{len(waypoints)}: ({lat:.6f}, {lon:.6f})")
            
            self.pilot.goto(lat, lon, alt)
            self.pilot.wait_for_arrival(lat, lon)
            
            # Simulate detection at each waypoint
            gps = self.pilot.get_current_gps()
            if gps:
                drone_lat, drone_lon, drone_alt = gps
                heading = self.pilot.get_heading()
                
                # Get geotagged detections
                new_detections = self.tracker.track_and_geotag(
                    frame=None,  # Mock doesn't need real frame
                    drone_lat=drone_lat,
                    drone_lon=drone_lon,
                    drone_alt=drone_alt,
                    drone_heading=heading
                )
                
                for det in new_detections:
                    logger.info(f"TARGET DETECTED (ID:{det.track_id})")
                    logger.info(f"  Target GPS: ({det.target_lat:.6f}, {det.target_lon:.6f})")
                    self.detected_targets.append({
                        'lat': det.target_lat,
                        'lon': det.target_lon,
                        'alt': drone_alt,
                        'track_id': det.track_id
                    })
        
        # RTL
        logger.info("Survey complete - RTL")
        self.pilot.rtl_now()
        
        logger.info(f"=== SURVEY COMPLETE: {len(self.detected_targets)} targets ===")
        return self.detected_targets


class SimulatedDeliveryDrone:
    """Simulates Drone 2 (Delivery) behavior."""
    
    def __init__(self):
        self.pilot = MockDronePilot("mock://drone2")
        self.payload = MockPayload({'initial_count': 10})
        self.targets_queue = []
        self.drops_completed = 0
        
    def add_target(self, lat: float, lon: float, alt: float):
        """Add target to delivery queue."""
        self.targets_queue.append((lat, lon, alt))
        logger.info(f"Target added to queue. Queue size: {len(self.targets_queue)}")
        
    def run_delivery(self):
        """Run delivery mission."""
        logger.info("=== DRONE 2: DELIVERY MISSION START ===")
        
        # Connect and arm
        self.pilot.connect()
        self.pilot.arm()
        self.pilot.safe_takeoff(25.0)
        self.pilot.wait_for_altitude(25.0)
        
        # Loiter waiting for targets
        self.pilot.loiter()
        logger.info("Loitering... waiting for targets")
        
        # Process targets
        while self.targets_queue or self.drops_completed == 0:
            if not self.targets_queue:
                time.sleep(0.1)
                continue
            
            if self.payload.is_empty():
                logger.warning("Payload exhausted!")
                break
            
            # Get next target
            lat, lon, alt = self.targets_queue.pop(0)
            
            logger.info(f"Processing target: ({lat:.6f}, {lon:.6f})")
            
            # Navigate
            self.pilot.goto(lat, lon, 15.0)  # Cruise altitude
            self.pilot.wait_for_arrival(lat, lon)
            
            # Descend
            self.pilot.goto(lat, lon, 10.0)  # Drop altitude
            time.sleep(0.5)
            
            # Drop!
            if self.payload.drop():
                self.drops_completed += 1
                logger.info(f"DROP #{self.drops_completed} complete!")
            
            # Climb back
            self.pilot.goto(lat, lon, 25.0)
        
        # RTL
        logger.info("Delivery complete - RTL")
        self.pilot.rtl_now()
        
        stats = self.payload.get_stats()
        logger.info(f"=== DELIVERY COMPLETE ===")
        logger.info(f"  Drops: {stats['drops_performed']}")
        logger.info(f"  Remaining: {stats['remaining']}")
        
        return stats


def run_dual_mission_test(num_detections: int = 3):
    """
    Run complete dual-drone mission simulation.
    
    Args:
        num_detections: Number of simulated human detections
    """
    print("\n" + "=" * 60)
    print("NIDAR DUAL-DRONE MISSION SIMULATION")
    print("=" * 60 + "\n")
    
    # Create drones
    survey_drone = SimulatedSurveyDrone()
    delivery_drone = SimulatedDeliveryDrone()
    
    # Define survey waypoints
    waypoints = [
        (12.9716, 77.5946, 20.0),
        (12.9720, 77.5946, 20.0),
        (12.9720, 77.5950, 20.0),
        (12.9716, 77.5950, 20.0),
    ]
    
    # Run survey and collect targets
    print("\n--- PHASE 1: SURVEY ---\n")
    targets = survey_drone.run_survey(waypoints, num_detections)
    
    # Feed targets to delivery drone
    print("\n--- PHASE 2: DELIVERY ---\n")
    for target in targets:
        delivery_drone.add_target(target['lat'], target['lon'], target['alt'])
    
    # Run delivery
    stats = delivery_drone.run_delivery()
    
    # Summary
    print("\n" + "=" * 60)
    print("MISSION SUMMARY")
    print("=" * 60)
    print(f"Survey:")
    print(f"  Waypoints flown: {len(waypoints)}")
    print(f"  Humans detected: {len(targets)}")
    print(f"Delivery:")
    print(f"  Targets processed: {delivery_drone.drops_completed}")
    print(f"  Payloads dropped: {stats['drops_performed']}")
    print(f"  Payloads remaining: {stats['remaining']}")
    print("=" * 60)
    
    return targets, stats


def main():
    parser = argparse.ArgumentParser(description="Dual-Drone Mission Simulation")
    parser.add_argument('--detections', type=int, default=3,
                       help='Number of simulated human detections')
    
    args = parser.parse_args()
    
    try:
        run_dual_mission_test(args.detections)
    except KeyboardInterrupt:
        print("\nSimulation interrupted")


if __name__ == "__main__":
    main()
