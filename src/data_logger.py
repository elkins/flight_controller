"""
Flight Data Logger

Logs flight controller data to CSV files for analysis with tools like PlotJuggler.
Compatible with ULog format structure for use with flight analysis tools.
"""

import time
import csv
import os
from typing import Optional, Dict, Any, List


class FlightDataLogger:
    """
    Flight data logger that records sensor data, PID outputs, and motor commands.
    
    Creates CSV files with timestamps for post-flight analysis.
    """
    
    def __init__(self, log_directory: str = "flight_logs", auto_timestamp: bool = True):
        """
        Initialize the flight data logger.
        
        Args:
            log_directory: Directory to store log files
            auto_timestamp: If True, add timestamp to log filename
        """
        self.log_directory = log_directory
        self.auto_timestamp = auto_timestamp
        self.log_file: Optional[str] = None
        self.csv_file = None
        self.csv_writer = None
        self.start_time: float = 0
        self.log_count: int = 0
        self.is_logging: bool = False
        
        # Create log directory if it doesn't exist
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
            print(f"Created log directory: {log_directory}")
    
    def start_logging(self, flight_name: str = "flight") -> str:
        """
        Start logging to a new file.
        
        Args:
            flight_name: Base name for the log file
            
        Returns:
            Path to the created log file
        """
        if self.is_logging:
            print("Warning: Already logging. Stop current log first.")
            return self.log_file
        
        # Generate filename with timestamp if enabled
        if self.auto_timestamp:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"{flight_name}_{timestamp}.csv"
        else:
            filename = f"{flight_name}.csv"
        
        self.log_file = os.path.join(self.log_directory, filename)
        
        # Open CSV file
        self.csv_file = open(self.log_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        header = [
            'timestamp_ms',           # Time since start in milliseconds
            'timestamp_us',           # Time since start in microseconds
            # IMU data
            'gyro_x', 'gyro_y', 'gyro_z',           # deg/s
            'accel_x', 'accel_y', 'accel_z',        # m/s²
            'roll', 'pitch', 'yaw',                  # degrees
            # RC inputs
            'rc_throttle', 'rc_roll', 'rc_pitch', 'rc_yaw',  # microseconds
            # PID outputs
            'pid_roll_output', 'pid_pitch_output', 'pid_yaw_output',
            # Motor commands
            'motor_0', 'motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5',  # microseconds
            # System state
            'battery_voltage', 'cpu_usage', 'loop_rate'
        ]
        self.csv_writer.writerow(header)
        
        self.start_time = time.time()
        self.log_count = 0
        self.is_logging = True
        
        print(f"Started logging to: {self.log_file}")
        return self.log_file
    
    def log_data(self, data: Dict[str, Any]):
        """
        Log a data frame.
        
        Args:
            data: Dictionary containing sensor data, RC inputs, PID outputs, etc.
                 Expected keys match the header columns.
        """
        if not self.is_logging:
            print("Warning: Not currently logging. Call start_logging() first.")
            return
        
        # Calculate timestamps
        current_time = time.time()
        elapsed = current_time - self.start_time
        timestamp_ms = int(elapsed * 1000)
        timestamp_us = int(elapsed * 1000000)
        
        # Build row from data dict (use 0 as default for missing values)
        row = [
            timestamp_ms,
            timestamp_us,
            # IMU
            data.get('gyro_x', 0),
            data.get('gyro_y', 0),
            data.get('gyro_z', 0),
            data.get('accel_x', 0),
            data.get('accel_y', 0),
            data.get('accel_z', 0),
            data.get('roll', 0),
            data.get('pitch', 0),
            data.get('yaw', 0),
            # RC
            data.get('rc_throttle', 1000),
            data.get('rc_roll', 1500),
            data.get('rc_pitch', 1500),
            data.get('rc_yaw', 1500),
            # PID
            data.get('pid_roll_output', 0),
            data.get('pid_pitch_output', 0),
            data.get('pid_yaw_output', 0),
            # Motors
            data.get('motor_0', 1000),
            data.get('motor_1', 1000),
            data.get('motor_2', 1000),
            data.get('motor_3', 1000),
            data.get('motor_4', 1000),
            data.get('motor_5', 1000),
            # System
            data.get('battery_voltage', 0),
            data.get('cpu_usage', 0),
            data.get('loop_rate', 0)
        ]
        
        self.csv_writer.writerow(row)
        self.log_count += 1
        
        # Flush periodically to ensure data is written
        if self.log_count % 100 == 0:
            self.csv_file.flush()
    
    def stop_logging(self):
        """Stop logging and close the file."""
        if not self.is_logging:
            print("Warning: Not currently logging.")
            return
        
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        
        elapsed = time.time() - self.start_time
        print(f"Stopped logging. {self.log_count} records written to {self.log_file}")
        print(f"Flight duration: {elapsed:.2f} seconds")
        
        self.is_logging = False
    
    def get_log_files(self) -> List[str]:
        """
        Get list of all log files in the log directory.
        
        Returns:
            List of log file paths
        """
        if not os.path.exists(self.log_directory):
            return []
        
        files = [
            os.path.join(self.log_directory, f)
            for f in os.listdir(self.log_directory)
            if f.endswith('.csv')
        ]
        return sorted(files)
    
    def __del__(self):
        """Ensure file is closed when logger is destroyed."""
        if self.is_logging:
            self.stop_logging()


class LoggerStats:
    """Calculate statistics from logged data for quick analysis."""
    
    @staticmethod
    def analyze_log(log_file: str) -> Dict[str, Any]:
        """
        Analyze a log file and return statistics.
        
        Args:
            log_file: Path to CSV log file
            
        Returns:
            Dictionary with statistics about the flight
        """
        import csv
        import statistics
        
        if not os.path.exists(log_file):
            return {"error": f"Log file not found: {log_file}"}
        
        stats = {
            "file": log_file,
            "record_count": 0,
            "duration_seconds": 0,
            "avg_loop_rate": 0,
            "motor_stats": {},
            "pid_stats": {},
            "attitude_stats": {}
        }
        
        try:
            with open(log_file, 'r') as f:
                reader = csv.DictReader(f)
                
                loop_rates = []
                motor_values = {i: [] for i in range(6)}
                roll_vals, pitch_vals, yaw_vals = [], [], []
                
                for row in reader:
                    stats["record_count"] += 1
                    
                    # Loop rate
                    if row.get('loop_rate'):
                        loop_rates.append(float(row['loop_rate']))
                    
                    # Motor values
                    for i in range(6):
                        if row.get(f'motor_{i}'):
                            motor_values[i].append(float(row[f'motor_{i}']))
                    
                    # Attitude
                    if row.get('roll'):
                        roll_vals.append(float(row['roll']))
                    if row.get('pitch'):
                        pitch_vals.append(float(row['pitch']))
                    if row.get('yaw'):
                        yaw_vals.append(float(row['yaw']))
                
                # Calculate duration from last timestamp
                f.seek(0)
                reader = csv.DictReader(f)
                rows = list(reader)
                if rows:
                    last_timestamp = float(rows[-1]['timestamp_ms'])
                    stats["duration_seconds"] = last_timestamp / 1000.0
                
                # Calculate averages
                if loop_rates:
                    stats["avg_loop_rate"] = statistics.mean(loop_rates)
                
                # Motor stats
                for i in range(6):
                    if motor_values[i]:
                        stats["motor_stats"][f"motor_{i}"] = {
                            "min": min(motor_values[i]),
                            "max": max(motor_values[i]),
                            "avg": statistics.mean(motor_values[i])
                        }
                
                # Attitude stats
                if roll_vals:
                    stats["attitude_stats"]["roll"] = {
                        "min": min(roll_vals),
                        "max": max(roll_vals),
                        "avg": statistics.mean(roll_vals),
                        "stdev": statistics.stdev(roll_vals) if len(roll_vals) > 1 else 0
                    }
                if pitch_vals:
                    stats["attitude_stats"]["pitch"] = {
                        "min": min(pitch_vals),
                        "max": max(pitch_vals),
                        "avg": statistics.mean(pitch_vals),
                        "stdev": statistics.stdev(pitch_vals) if len(pitch_vals) > 1 else 0
                    }
        
        except Exception as e:
            stats["error"] = str(e)
        
        return stats
