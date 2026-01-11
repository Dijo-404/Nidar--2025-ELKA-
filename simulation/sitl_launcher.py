#!/usr/bin/env python3
"""
SITL Launcher - ArduPilot Software-In-The-Loop Simulation

Launches ArduPilot SITL instances for testing dual-drone missions
without real hardware.

Usage:
    python simulation/sitl_launcher.py              # Launch both drones
    python simulation/sitl_launcher.py --drone1     # Launch only drone 1
    python simulation/sitl_launcher.py --drone2     # Launch only drone 2
    python simulation/sitl_launcher.py --headless   # No console output
"""

import os
import sys
import time
import signal
import argparse
import subprocess
import logging
from pathlib import Path
from typing import Optional, List

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('SITL')


class SITLInstance:
    """Manages a single ArduPilot SITL instance."""
    
    def __init__(self, instance_id: int, home_lat: float, home_lon: float,
                 home_alt: float = 0, home_heading: float = 0):
        """
        Initialize SITL instance.
        
        Args:
            instance_id: Instance number (0, 1, etc.)
            home_lat: Home latitude
            home_lon: Home longitude
            home_alt: Home altitude (meters)
            home_heading: Initial heading (degrees)
        """
        self.instance_id = instance_id
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.home_alt = home_alt
        self.home_heading = home_heading
        self.process: Optional[subprocess.Popen] = None
        
        # Port calculation based on instance
        self.master_port = 5760 + (instance_id * 10)
        self.sitl_port = 5501 + instance_id
        self.mavproxy_port = 14550 + instance_id
        
    def start(self, ardupilot_path: str = None, vehicle: str = "copter",
              speedup: int = 1, headless: bool = False) -> bool:
        """
        Start the SITL instance.
        
        Args:
            ardupilot_path: Path to ArduPilot source directory
            vehicle: Vehicle type (copter, plane, rover)
            speedup: Simulation speed multiplier
            headless: Run without console output
            
        Returns:
            True if started successfully
        """
        # Find ArduPilot
        if not ardupilot_path:
            ardupilot_path = os.environ.get('ARDUPILOT_HOME')
            if not ardupilot_path:
                # Try common locations
                for path in ['~/ardupilot', '~/ArduPilot', '/opt/ardupilot']:
                    expanded = os.path.expanduser(path)
                    if os.path.exists(expanded):
                        ardupilot_path = expanded
                        break
        
        if not ardupilot_path or not os.path.exists(ardupilot_path):
            logger.error("ArduPilot not found. Set ARDUPILOT_HOME or provide path.")
            return False
        
        # Build command
        sim_vehicle = os.path.join(ardupilot_path, 'Tools', 'autotest', 'sim_vehicle.py')
        if not os.path.exists(sim_vehicle):
            logger.error(f"sim_vehicle.py not found at {sim_vehicle}")
            return False
        
        home_string = f"{self.home_lat},{self.home_lon},{self.home_alt},{self.home_heading}"
        
        cmd = [
            sys.executable, sim_vehicle,
            '-v', vehicle,
            '-I', str(self.instance_id),
            '--custom-location', home_string,
            '--speedup', str(speedup),
            '--out', f'udp:127.0.0.1:{self.mavproxy_port}',
            '-w',  # Wipe EEPROM for fresh start
        ]
        
        if headless:
            cmd.extend(['--no-mavproxy', '-w'])
        
        logger.info(f"Starting SITL instance {self.instance_id}...")
        logger.info(f"  Home: {home_string}")
        logger.info(f"  MAVLink port: {self.mavproxy_port}")
        
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE if headless else None,
                stderr=subprocess.PIPE if headless else None,
                cwd=ardupilot_path
            )
            
            # Wait for startup
            time.sleep(5)
            
            if self.process.poll() is None:
                logger.info(f"SITL instance {self.instance_id} started (PID: {self.process.pid})")
                return True
            else:
                logger.error(f"SITL instance {self.instance_id} failed to start")
                return False
                
        except Exception as e:
            logger.error(f"Failed to start SITL: {e}")
            return False
    
    def stop(self):
        """Stop the SITL instance."""
        if self.process:
            logger.info(f"Stopping SITL instance {self.instance_id}...")
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None
            logger.info(f"SITL instance {self.instance_id} stopped")
    
    def is_running(self) -> bool:
        """Check if instance is running."""
        return self.process is not None and self.process.poll() is None
    
    def get_connection_string(self) -> str:
        """Get MAVLink connection string for this instance."""
        return f"udp:127.0.0.1:{self.mavproxy_port}"


class DualDroneSITL:
    """Manages dual-drone SITL simulation."""
    
    def __init__(self, config: dict = None):
        """
        Initialize dual-drone SITL.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        
        # Default home positions (slightly separated)
        # Drone 1: Scout
        self.drone1 = SITLInstance(
            instance_id=0,
            home_lat=self.config.get('drone1_lat', 12.9716),
            home_lon=self.config.get('drone1_lon', 77.5946),
            home_alt=0,
            home_heading=0
        )
        
        # Drone 2: Delivery (50m away)
        self.drone2 = SITLInstance(
            instance_id=1,
            home_lat=self.config.get('drone2_lat', 12.9720),
            home_lon=self.config.get('drone2_lon', 77.5946),
            home_alt=0,
            home_heading=0
        )
        
        self.instances: List[SITLInstance] = []
    
    def start_drone1(self, **kwargs) -> bool:
        """Start only Drone 1 (Scout)."""
        if self.drone1.start(**kwargs):
            self.instances.append(self.drone1)
            return True
        return False
    
    def start_drone2(self, **kwargs) -> bool:
        """Start only Drone 2 (Delivery)."""
        if self.drone2.start(**kwargs):
            self.instances.append(self.drone2)
            return True
        return False
    
    def start_both(self, **kwargs) -> bool:
        """Start both drones."""
        success1 = self.start_drone1(**kwargs)
        time.sleep(3)  # Stagger startup
        success2 = self.start_drone2(**kwargs)
        return success1 and success2
    
    def stop_all(self):
        """Stop all running instances."""
        for instance in self.instances:
            instance.stop()
        self.instances.clear()
    
    def get_connection_strings(self) -> dict:
        """Get connection strings for all drones."""
        return {
            'drone1': self.drone1.get_connection_string(),
            'drone2': self.drone2.get_connection_string()
        }
    
    def print_status(self):
        """Print current status."""
        print("\n" + "=" * 50)
        print("SITL Status")
        print("=" * 50)
        print(f"Drone 1 (Scout):    {'RUNNING' if self.drone1.is_running() else 'STOPPED'}")
        print(f"  Connection: {self.drone1.get_connection_string()}")
        print(f"Drone 2 (Delivery): {'RUNNING' if self.drone2.is_running() else 'STOPPED'}")
        print(f"  Connection: {self.drone2.get_connection_string()}")
        print("=" * 50)


def main():
    parser = argparse.ArgumentParser(description="SITL Launcher for Nidar Dual-Drone System")
    parser.add_argument('--drone1', action='store_true', help='Launch only Drone 1 (Scout)')
    parser.add_argument('--drone2', action='store_true', help='Launch only Drone 2 (Delivery)')
    parser.add_argument('--headless', action='store_true', help='Run without console output')
    parser.add_argument('--speedup', type=int, default=1, help='Simulation speed multiplier')
    parser.add_argument('--ardupilot', type=str, default=None, help='Path to ArduPilot source')
    parser.add_argument('--lat', type=float, default=12.9716, help='Home latitude')
    parser.add_argument('--lon', type=float, default=77.5946, help='Home longitude')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("NIDAR - SITL Launcher")
    print("Dual-Drone Simulation Environment")
    print("=" * 60)
    
    config = {
        'drone1_lat': args.lat,
        'drone1_lon': args.lon,
        'drone2_lat': args.lat + 0.0004,  # ~40m offset
        'drone2_lon': args.lon
    }
    
    sitl = DualDroneSITL(config)
    
    # Handle shutdown
    def signal_handler(sig, frame):
        print("\nShutting down...")
        sitl.stop_all()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Start requested instances
    kwargs = {
        'ardupilot_path': args.ardupilot,
        'speedup': args.speedup,
        'headless': args.headless
    }
    
    if args.drone1:
        sitl.start_drone1(**kwargs)
    elif args.drone2:
        sitl.start_drone2(**kwargs)
    else:
        sitl.start_both(**kwargs)
    
    sitl.print_status()
    
    print("\nSITL running. Press Ctrl+C to stop.")
    print("\nTo connect missions:")
    print(f"  Drone 1: python missions/01_survey_leader.py --sitl")
    print(f"  Drone 2: python missions/02_delivery_follower.py --sitl")
    
    # Keep running
    try:
        while True:
            time.sleep(1)
            # Check if any instance died
            for instance in sitl.instances:
                if not instance.is_running():
                    logger.warning(f"Instance {instance.instance_id} stopped unexpectedly")
    except KeyboardInterrupt:
        pass
    finally:
        sitl.stop_all()


if __name__ == "__main__":
    main()
