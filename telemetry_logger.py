#!/usr/bin/env python3
"""
Telemetry Logger for Quadrotor Flight Controller
Connects to the ESP32 WebServer and logs telemetry data to CSV
"""

import requests
import csv
import time
import argparse
from datetime import datetime
import sys

# Configuration
DEFAULT_CONTROLLER_IP = "192.168.100.19"  # Default ESP32 AP IP
DEFAULT_CONTROLLER_HOSTNAME = "hakeem"  # Try hostname if IP fails
DEFAULT_LOG_INTERVAL = 0.1  # seconds (10 Hz)
DEFAULT_LOG_FILE = "telemetry_log.csv"


class TelemetryLogger:
    def __init__(self, host, log_file, interval=0.1, timeout=5):
        """
        Initialize the telemetry logger
        
        Args:
            host: IP address or hostname of the ESP32
            log_file: CSV file to write telemetry data
            interval: Logging interval in seconds
            timeout: HTTP request timeout
        """
        self.host = host
        self.telemetry_url = f"http://{host}/telemetry"
        self.log_file = log_file
        self.interval = interval
        self.timeout = timeout
        self.csv_file = None
        self.csv_writer = None
        self.data_count = 0
        
    def connect(self):
        """Test connection to the flight controller"""
        try:
            print(f"Connecting to {self.host}...", end=" ", flush=True)
            response = requests.get(self.telemetry_url, timeout=self.timeout)
            if response.status_code == 200:
                print("✓ Connected")
                return True
        except requests.exceptions.ConnectionError:
            print("✗ Connection failed")
            return False
        except requests.exceptions.Timeout:
            print("✗ Timeout")
            return False
        except Exception as e:
            print(f"✗ Error: {e}")
            return False
    
    def start_logging(self):
        """Open CSV file and write header"""
        try:
            self.csv_file = open(self.log_file, 'w', newline='')
            self.csv_writer = csv.DictWriter(
                self.csv_file,
                fieldnames=['timestamp', 'elapsed_s', 'temp_c', 'pressure_pa', 'altitude_m', 'pitch_deg', 'roll_deg', 'throttle', 'throttle_percent', 'motor_fl', 'motor_fr', 'motor_rr', 'motor_rl']
            )
            self.csv_writer.writeheader()
            self.csv_file.flush()
            print(f"Logging to: {self.log_file}")
            return True
        except IOError as e:
            print(f"Error opening log file: {e}")
            return False
    
    def stop_logging(self):
        """Close CSV file"""
        if self.csv_file:
            self.csv_file.close()
            print(f"\nLogging complete. {self.data_count} samples written.")
    
    def fetch_telemetry(self):
        """Fetch one telemetry sample from the controller"""
        try:
            response = requests.get(self.telemetry_url, timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"\nError fetching telemetry: {e}")
            return None
    
    def run(self, duration=None):
        """
        Main logging loop
        
        Args:
            duration: Optional duration in seconds (None = infinite)
        """
        if not self.connect():
            return False
        
        if not self.start_logging():
            return False
        
        start_time = time.time()
        last_fetch = start_time
        
        try:
            print(f"Logging at {1/self.interval:.1f} Hz (press Ctrl+C to stop)")
            print("-" * 70)
            
            while True:
                # Check duration limit
                if duration and (time.time() - start_time) > duration:
                    break
                
                # Fetch on schedule
                current_time = time.time()
                if (current_time - last_fetch) >= self.interval:
                    telemetry = self.fetch_telemetry()
                    if telemetry:
                        elapsed = current_time - start_time
                        row = {
                            'timestamp': datetime.now().isoformat(),
                            'elapsed_s': f"{elapsed:.3f}",
                            'temp_c': f"{telemetry.get('temperature_c', 0):.2f}",
                            'pressure_pa': f"{telemetry.get('pressure_pa', 0):.2f}",
                            'altitude_m': f"{telemetry.get('altitude_m', 0):.3f}",
                            'pitch_deg': f"{telemetry.get('pitch_deg', 0):.2f}",
                            'roll_deg': f"{telemetry.get('roll_deg', 0):.2f}",
                            'throttle': f"{telemetry.get('throttle', 0):.0f}",
                            'throttle_percent': f"{telemetry.get('throttle_percent', 0):.2f}",
                            'motor_fl': f"{telemetry.get('motor_fl', 0):.0f}",
                            'motor_fr': f"{telemetry.get('motor_fr', 0):.0f}",
                            'motor_rr': f"{telemetry.get('motor_rr', 0):.0f}",
                            'motor_rl': f"{telemetry.get('motor_rl', 0):.0f}",
                        }
                        self.csv_writer.writerow(row)
                        self.csv_file.flush()
                        self.data_count += 1
                        
                        # Print sample (every 10 samples)
                        if self.data_count % 10 == 0:
                            print(
                                f"[{self.data_count:5d}] "
                                f"T={telemetry.get('temperature_c', 0):6.2f}°C  "
                                f"Alt={telemetry.get('altitude_m', 0):7.2f}m  "
                                f"Pitch={telemetry.get('pitch_deg', 0):6.2f}°  "
                                f"Roll={telemetry.get('roll_deg', 0):6.2f}°  "
                                f"Throttle={telemetry.get('throttle', 0):4.0f}  "
                                f"Motors=[{telemetry.get('motor_fl', 0):4.0f} {telemetry.get('motor_fr', 0):4.0f} {telemetry.get('motor_rr', 0):4.0f} {telemetry.get('motor_rl', 0):4.0f}]"
                            )
                        
                        last_fetch = current_time
                
                # Sleep to avoid busy-waiting
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\n\nLogging interrupted by user")
        except Exception as e:
            print(f"\nFatal error: {e}")
        finally:
            self.stop_logging()
        
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Log telemetry data from Quadrotor Flight Controller',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Log to default file at 10 Hz
  python telemetry_logger.py
  
  # Log to custom file
  python telemetry_logger.py -o my_flight.csv
  
  # Log at 50 Hz for 30 seconds to specific host
  python telemetry_logger.py -i 192.168.1.100 -f 50 -d 30
        '''
    )
    
    parser.add_argument(
        '-i', '--host',
        default=DEFAULT_CONTROLLER_IP,
        help=f'Flight controller IP or hostname (default: {DEFAULT_CONTROLLER_IP})'
    )
    parser.add_argument(
        '-o', '--output',
        default=DEFAULT_LOG_FILE,
        help=f'Output CSV file (default: {DEFAULT_LOG_FILE})'
    )
    parser.add_argument(
        '-f', '--frequency',
        type=float,
        default=1.0/DEFAULT_LOG_INTERVAL,
        help=f'Logging frequency in Hz (default: {1.0/DEFAULT_LOG_INTERVAL:.1f})'
    )
    parser.add_argument(
        '-d', '--duration',
        type=float,
        help='Logging duration in seconds (default: infinite)'
    )
    parser.add_argument(
        '-t', '--timeout',
        type=float,
        default=5,
        help='HTTP request timeout in seconds (default: 5)'
    )
    
    args = parser.parse_args()
    
    # Validate frequency
    if args.frequency <= 0:
        print("Error: Frequency must be positive")
        sys.exit(1)
    
    interval = 1.0 / args.frequency
    
    print("=" * 70)
    print("Quadrotor Telemetry Logger")
    print("=" * 70)
    print(f"Host:      {args.host}")
    print(f"Frequency: {args.frequency:.1f} Hz (interval: {interval*1000:.1f} ms)")
    print(f"Output:    {args.output}")
    if args.duration:
        print(f"Duration:  {args.duration:.1f} seconds")
    print("=" * 70)
    
    logger = TelemetryLogger(
        host=args.host,
        log_file=args.output,
        interval=interval,
        timeout=args.timeout
    )
    
    success = logger.run(duration=args.duration)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
