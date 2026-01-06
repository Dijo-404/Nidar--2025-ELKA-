#!/usr/bin/env python3
"""
Ground Relay Server - Laptop Main Executable

This script runs the ground relay server on the Arch Linux laptop:
1. Start ZMQ relay server
2. Monitor connections from both drones
3. Forward coordinate messages between drones
4. Log all message traffic
5. Display real-time status

Usage:
    python 00_ground_relay.py [--config CONFIG_PATH] [--log-file LOG_PATH]
"""

import os
import sys
import time
import yaml
import logging
import argparse
import signal
from pathlib import Path
from datetime import datetime

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.comms.relay_server import RelayServer

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('GroundRelay')


class GroundRelayStation:
    """
    Ground relay station for drone-to-drone communication.
    
    Manages the relay server and provides monitoring/logging capabilities.
    """
    
    def __init__(self, config_path: str, log_file: str = None):
        """
        Initialize ground relay station.
        
        Args:
            config_path: Path to network_map.yaml
            log_file: Optional path to message log file
        """
        self.config = self._load_config(config_path)
        self.log_file = log_file
        
        # Initialize server
        relay_config = self.config.get('ground_relay', {})
        self.server = RelayServer(relay_config)
        
        # State
        self.is_running = False
        self.start_time = None
        
        # Statistics
        self.stats_history = []
    
    def _load_config(self, path: str) -> dict:
        """Load YAML configuration file."""
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    
    def start(self) -> bool:
        """
        Start the relay server.
        
        Returns:
            True if started successfully
        """
        relay_config = self.config.get('ground_relay', {})
        
        logger.info("="*60)
        logger.info("ANTIGRAVITY CORE - Ground Relay Station")
        logger.info("="*60)
        logger.info(f"Binding to: {relay_config.get('ip', '*')}:{relay_config.get('zmq_port', 5555)}")
        
        if self.log_file:
            logger.info(f"Logging to: {self.log_file}")
        
        if not self.server.start(log_file=self.log_file):
            logger.error("Failed to start relay server")
            return False
        
        self.is_running = True
        self.start_time = time.time()
        
        logger.info("Relay server started successfully")
        logger.info("Waiting for drone connections...")
        
        return True
    
    def stop(self):
        """Stop the relay server."""
        logger.info("Stopping relay server...")
        self.is_running = False
        self.server.stop()
        logger.info("Relay server stopped")
    
    def run_interactive(self):
        """
        Run in interactive mode with status display.
        
        Displays real-time status and handles Ctrl+C gracefully.
        """
        # Set up signal handler for graceful shutdown
        def signal_handler(signum, frame):
            logger.info("\nShutdown signal received")
            self.stop()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        logger.info("Running in interactive mode. Press Ctrl+C to stop.")
        print()
        
        try:
            while self.is_running:
                self._display_status()
                time.sleep(3.0)
                
        except KeyboardInterrupt:
            print("\n")
            logger.info("Interrupted by user")
        finally:
            self.stop()
    
    def _display_status(self):
        """Display current relay status."""
        stats = self.server.get_statistics()
        
        # Clear screen (ANSI escape)
        print("\033[H\033[J", end="")
        
        # Header
        uptime = stats['uptime_seconds']
        uptime_str = f"{int(uptime // 3600)}h {int((uptime % 3600) // 60)}m {int(uptime % 60)}s"
        
        print("╔" + "═"*58 + "╗")
        print("║" + " ANTIGRAVITY CORE - Ground Relay Station".center(58) + "║")
        print("╠" + "═"*58 + "╣")
        print(f"║  Uptime: {uptime_str:<47} ║")
        print(f"║  Status: {'RUNNING' if self.is_running else 'STOPPED':<47} ║")
        print("╠" + "═"*58 + "╣")
        
        # Statistics
        print(f"║  Total Messages:      {stats['total_messages']:<34} ║")
        print(f"║  Coordinates Relayed: {stats['total_coords_relayed']:<34} ║")
        print(f"║  Connected Clients:   {stats['connected_clients']:<34} ║")
        print("╠" + "═"*58 + "╣")
        
        # Client details
        clients = stats.get('clients', {})
        
        if clients:
            print("║  CONNECTED DRONES:" + " "*39 + "║")
            print("╟" + "─"*58 + "╢")
            
            for client_id, info in clients.items():
                status = info.get('status', 'UNKNOWN')
                battery = info.get('battery', -1)
                payload = info.get('payload', -1)
                messages = info.get('messages', 0)
                
                battery_str = f"{battery}%" if battery >= 0 else "N/A"
                payload_str = str(payload) if payload >= 0 else "N/A"
                
                print(f"║    {client_id:<12} │ {status:<10} │ Bat: {battery_str:>4} │ "
                      f"Pay: {payload_str:>3} │ Msg: {messages:<4} ║")
        else:
            print("║  No drones connected" + " "*37 + "║")
        
        print("╠" + "═"*58 + "╣")
        
        # Instructions
        print("║  Press Ctrl+C to stop the relay server" + " "*18 + "║")
        print("╚" + "═"*58 + "╝")
        print()
        
        # Log recent activity
        current_time = datetime.now().strftime("%H:%M:%S")
        print(f"[{current_time}] Listening for messages...")
    
    def broadcast_message(self, topic: str, data: dict):
        """
        Broadcast a message to all connected drones.
        
        Args:
            topic: Message topic
            data: Message data
        """
        self.server.broadcast(topic, data)
        logger.info(f"Broadcast sent: {topic}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Ground Relay Server for Antigravity Core"
    )
    parser.add_argument(
        '--config', 
        default='config/network_map.yaml',
        help='Path to network configuration file'
    )
    parser.add_argument(
        '--log-file',
        help='Path to message log file'
    )
    parser.add_argument(
        '--test',
        action='store_true',
        help='Run in test mode (loopback testing)'
    )
    
    args = parser.parse_args()
    
    # Resolve config path
    script_dir = Path(__file__).parent.parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = script_dir / config_path
    
    # Default log file location
    log_file = args.log_file
    if not log_file:
        log_dir = script_dir / 'logs'
        log_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = str(log_dir / f"relay_{timestamp}.log")
    
    # Create relay station
    relay = GroundRelayStation(str(config_path), log_file)
    
    try:
        if not relay.start():
            logger.error("Failed to start relay station")
            sys.exit(1)
        
        if args.test:
            logger.info("TEST MODE - Running for 10 seconds")
            time.sleep(10)
            relay.stop()
        else:
            relay.run_interactive()
        
    except Exception as e:
        logger.error(f"Relay station error: {e}")
        relay.stop()
        sys.exit(1)


if __name__ == "__main__":
    main()
