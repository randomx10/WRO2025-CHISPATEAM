#!/usr/bin/env python3
"""
LiDAR Obstacle Avoidance System - Headless Version
Designed to run automatically on system startup without user interaction.
Integrates LiDAR HTTP server, motor control (eng.py), and servo rotation (rot.py)
Automatically rotates when obstacle detected at 180 degrees with distance < 1500mm

Copyright (C) 2025  Alexis Martinez

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import time
import threading
import logging
import requests
import RPi.GPIO as GPIO
from typing import Optional, Dict, Any
import json
import subprocess
import os
import sys
import signal

# Configure logging for headless operation
log_dir = "/var/log/wro"
os.makedirs(log_dir, exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(f"{log_dir}/obstacle_avoidance.log"),
        logging.StreamHandler(sys.stdout)  # Also log to stdout for systemd journal
    ]
)
logger = logging.getLogger(__name__)

class LidarObstacleAvoidanceHeadless:
    """Headless obstacle avoidance system using LiDAR, motor control, and servo rotation"""
    
    def __init__(self, lidar_server_url: str = "http://localhost:5000"):
        self.lidar_server_url = lidar_server_url
        self.running = False
        self.monitor_thread = None
        self.lidar_server_process = None
        self.shutdown_requested = False
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Motor control setup (from eng.py)
        self.MOSFET_GPIO = 18
        self.PWM_FREQUENCY = 1000  # 1 kHz
        GPIO.setup(self.MOSFET_GPIO, GPIO.OUT)
        self.motor_pwm = GPIO.PWM(self.MOSFET_GPIO, self.PWM_FREQUENCY)
        self.motor_pwm.start(0)  # Start with motor off
        
        # Servo control setup (adapted from rot.py)
        # Note: rot.py uses BOARD mode pin 16, which is BCM pin 23
        self.SERVO_GPIO = 23  # BCM pin 23 (BOARD pin 16)
        GPIO.setup(self.SERVO_GPIO, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.SERVO_GPIO, 50)  # 50Hz for servo
        self.servo_pwm.start(0)
        
        # Configuration parameters
        self.OBSTACLE_THRESHOLD = 1500  # mm
        self.TARGET_ANGLE = 180  # degrees to monitor (front robot)
        self.ANGLE_TOLERANCE = 30.0  # degrees tolerance around target angle
        self.SCAN_INTERVAL = 0.001  # seconds between scans
        
        # Steering angles (using rot.py mapping)
        self.NORMAL_STEERING = 90  # Center position (0-180 range)
        self.AVOID_STEERING = 45  # Turn right to avoid obstacle
        
        # State variables
        self.obstacle_detected = False
        self.current_servo_angle = self.NORMAL_STEERING
        self.motor_speed = 0
        self.lap_count = 0
        self.max_laps = 7
        self.avoiding_obstacle = False
        self.avoidance_start_time = None
        
        logger.info("🤖 LiDAR Obstacle Avoidance System (Headless) initialized")
        logger.info(f"Motor GPIO: {self.MOSFET_GPIO}, Servo GPIO: {self.SERVO_GPIO}")
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        logger.info(f"🛑 Received signal {signum}, initiating graceful shutdown...")
        self.shutdown_requested = True
        self.stop()
    
    def _start_lidar_server(self) -> bool:
        """Start the LiDAR HTTP server as a subprocess"""
        try:
            # Check if server is already running
            try:
                response = requests.get(f"{self.lidar_server_url}/api/status", timeout=2)
                if response.status_code == 200:
                    logger.info("✅ LiDAR HTTP server already running")
                    return True
            except:
                pass  # Server not running, we'll start it
            
            # Start the LiDAR Flask server
            server_script = "/home/pi/wro/lidar_flask.py"
            if not os.path.exists(server_script):
                logger.error(f"❌ LiDAR server script not found: {server_script}")
                return False
            
            logger.info("🚀 Starting LiDAR HTTP server...")
            
            # Start the server as a subprocess
            self.lidar_server_process = subprocess.Popen(
                [sys.executable, server_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd="/home/pi/wro"
            )
            
            # Wait for server to start
            for attempt in range(30):  # Wait up to 30 seconds
                try:
                    response = requests.get(f"{self.lidar_server_url}/api/status", timeout=2)
                    if response.status_code == 200:
                        logger.info("✅ LiDAR HTTP server started successfully")
                        return True
                except:
                    time.sleep(1)
                    continue
            
            logger.error("❌ LiDAR HTTP server failed to start within 30 seconds")
            return False
            
        except Exception as e:
            logger.error(f"❌ Error starting LiDAR server: {e}")
            return False
    
    def _stop_lidar_server(self):
        """Stop the LiDAR HTTP server subprocess"""
        if self.lidar_server_process:
            try:
                logger.info("🛑 Stopping LiDAR HTTP server...")
                self.lidar_server_process.terminate()
                
                # Wait for graceful shutdown
                try:
                    self.lidar_server_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    logger.warning("⚠️ LiDAR server didn't stop gracefully, force killing...")
                    self.lidar_server_process.kill()
                    self.lidar_server_process.wait()
                
                logger.info("✅ LiDAR HTTP server stopped")
                self.lidar_server_process = None
            except Exception as e:
                logger.error(f"❌ Error stopping LiDAR server: {e}")
    
    def start(self):
        """Start the obstacle avoidance system"""
        if self.running:
            logger.warning("⚠️ System already running")
            return False
        
        logger.info("🚀 Starting LiDAR Obstacle Avoidance System (Headless Mode)")
        
        # Start LiDAR HTTP server
        if not self._start_lidar_server():
            logger.error("❌ Failed to start LiDAR server, aborting")
            return False
        
        # Ensure LiDAR is running
        self._ensure_lidar_running()
        
        self.running = True
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        # Initialize with normal operation
        self.set_servo_angle(self.NORMAL_STEERING)
        time.sleep(1)  # Allow servo to reach position
        self.set_motor_speed(70)  # Start with 70% motor speed
        
        logger.info("Obstacle avoidance system started")
        logger.info(f"Monitoring for obstacles at {self.TARGET_ANGLE}° (front)")
        logger.info(f"Will complete {self.max_laps} laps")
        
        return True
    
    def stop(self):
        """Stop the obstacle avoidance system"""
        if not self.running:
            return
        
        logger.info("🛑 Stopping obstacle avoidance system...")
        self.running = False
        
        # Stop motor and center servo
        self.set_motor_speed(0)
        self.set_servo_angle(self.NORMAL_STEERING)
        
        # Wait for monitoring thread to finish
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        
        # Stop LiDAR server
        self._stop_lidar_server()
        
        logger.info("⏹️ Obstacle avoidance system stopped")
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        try:
            self.motor_pwm.stop()
            self.servo_pwm.stop()
            GPIO.cleanup()
            logger.info("🧹 GPIO cleanup complete")
        except Exception as e:
            logger.error(f"❌ Error during cleanup: {e}")
    
    def _ensure_lidar_running(self):
        """Ensure LiDAR scanning is active"""
        max_retries = 5
        for retry in range(max_retries):
            try:
                # Check LiDAR status
                response = requests.get(f"{self.lidar_server_url}/api/status", timeout=3)
                if response.status_code == 200:
                    status = response.json()
                    if not status.get('running', False):
                        # Start LiDAR scanning
                        start_response = requests.post(f"{self.lidar_server_url}/api/start", timeout=5)
                        if start_response.status_code == 200:
                            logger.info("✅ LiDAR scanning started")
                            return
                        else:
                            logger.error(f"❌ Failed to start LiDAR: {start_response.status_code}")
                    else:
                        logger.info("✅ LiDAR already running")
                        return
                else:
                    logger.error(f"❌ Cannot check LiDAR status: {response.status_code}")
            except Exception as e:
                logger.error(f"❌ Error ensuring LiDAR is running (attempt {retry + 1}/{max_retries}): {e}")
                if retry < max_retries - 1:
                    time.sleep(2)
    
    def _get_lidar_data(self) -> Optional[Dict[str, Any]]:
        """Get current LiDAR scan data from HTTP server"""
        try:
            response = requests.get(f"{self.lidar_server_url}/api/scan", timeout=2)
            if response.status_code == 200:
                return response.json()
            else:
                logger.debug(f"Failed to get LiDAR data: {response.status_code}")
                return None
        except Exception as e:
            logger.debug(f"Error getting LiDAR data: {e}")
            return None
    
    def _find_obstacle_distance(self, scan_data: Dict[str, Any]) -> float:
        """Find closest obstacle distance at target angle (180°)"""
        if not scan_data or 'points' not in scan_data:
            return float('inf')
        
        points = scan_data.get('points', [])
        if not points:
            return float('inf')
        
        # Find points near the target angle (180°)
        target_distances = []
        for point in points:
            angle = point.get('angle', 0)
            distance = point.get('distance', 0)
            
            # Check if angle is within tolerance of 180°
            angle_diff = abs(angle - self.TARGET_ANGLE)
            # Handle angle wraparound (e.g., 179° to 181° spans 0°)
            if angle_diff > 180:
                angle_diff = 360 - angle_diff
            
            if angle_diff <= self.ANGLE_TOLERANCE and distance > 0:
                target_distances.append(distance)
        
        if not target_distances:
            return float('inf')
        
        # Return the closest (minimum) distance
        return min(target_distances)
    
    def _monitor_loop(self):
        """Main monitoring loop for obstacle detection"""
        logger.info("🔍 Monitoring loop started")
        
        consecutive_detections = 0
        consecutive_clears = 0
        detection_threshold = 2  # Require 2 consecutive detections to avoid false positives

        while self.running and self.lap_count < self.max_laps and not self.shutdown_requested:
            try:
                # Handle ongoing avoidance maneuver
                if self.avoiding_obstacle:
                    if time.time() - self.avoidance_start_time >= 2.0:
                        # Return to normal position after 2.0 seconds
                        self.set_servo_angle(self.NORMAL_STEERING)
                        self.avoiding_obstacle = False
                        self.avoidance_start_time = None
                        self.lap_count += 1
                        logger.info(f"🏁 Lap {self.lap_count}/{self.max_laps} completed!")
                        
                        if self.lap_count >= self.max_laps:
                            logger.info("🎉 All laps completed! Stopping...")
                            self.running = False
                            break
                
                # Get LiDAR scan data
                scan_data = self._get_lidar_data()
                
                if scan_data:
                    # Find closest obstacle at target angle
                    closest_distance = self._find_obstacle_distance(scan_data)
                    
                    # Check for obstacle (only if not currently avoiding)
                    if not self.avoiding_obstacle and closest_distance < self.OBSTACLE_THRESHOLD:
                        consecutive_detections += 1
                        consecutive_clears = 0
                        
                        if consecutive_detections >= detection_threshold and not self.obstacle_detected:
                            self.obstacle_detected = True
                            logger.warning(f"🚨 OBSTACLE DETECTED! Distance: {closest_distance:.1f}mm at {self.TARGET_ANGLE}°")
                            self._execute_avoidance_maneuver()
                    else:
                        consecutive_clears += 1
                        consecutive_detections = 0
                        
                        if consecutive_clears >= detection_threshold and self.obstacle_detected:
                            self.obstacle_detected = False
                            logger.info(f"✅ Obstacle cleared! Distance now: {closest_distance:.1f}mm")
                
                time.sleep(self.SCAN_INTERVAL)
                
            except Exception as e:
                logger.error(f"❌ Error in monitoring loop: {e}")
                time.sleep(1.0)
        
        logger.info("🔍 Monitoring loop stopped")
        if self.lap_count >= self.max_laps:
            logger.info("🏆 Challenge completed!")
        
        # Stop the system
        self.set_motor_speed(0)
        self.set_servo_angle(self.NORMAL_STEERING)
    
    def _execute_avoidance_maneuver(self):
        """Execute obstacle avoidance: keep motor speed and turn servo"""
        logger.info(f"🔄 EXECUTING AVOIDANCE: Keep motor speed + Turn to {self.AVOID_STEERING}°")
        
        # Keep motor speed (don't stop)
        # Turn servo to avoid obstacle
        self.set_servo_angle(self.AVOID_STEERING)
        
        # Mark as avoiding and record start time
        self.avoiding_obstacle = True
        self.avoidance_start_time = time.time()
        
        logger.info("⏱️ Avoidance maneuver started - will return to normal in 2.0 seconds")
    
    def set_motor_speed(self, speed_percent: float):
        """Set motor speed using PWM (from eng.py)"""
        if not (0 <= speed_percent <= 100):
            logger.error(f"❌ Invalid motor speed: {speed_percent}% (must be 0-100)")
            return
        
        try:
            self.motor_pwm.ChangeDutyCycle(speed_percent)
            self.motor_speed = speed_percent
            logger.info(f"🏎️ Motor speed: {speed_percent}%")
        except Exception as e:
            logger.error(f"❌ Error setting motor speed: {e}")
    
    def map_steering_angle(self, desired_angle: float) -> Optional[float]:
        """Map steering angle (0-180) to servo angle (60-120) - from rot.py"""
        if not 0 <= desired_angle <= 180:
            return None
        
        # Map the 0-180 range to the 60-120 range linearly
        servo_angle = ((desired_angle - 0) * (120 - 60)) / (180 - 0) + 60
        return servo_angle
    
    def set_servo_angle(self, angle_degrees: float):
        """Set servo angle using rot.py mapping and control logic"""
        if not (0 <= angle_degrees <= 180):
            logger.error(f"❌ Invalid servo angle: {angle_degrees}° (must be 0-180)")
            return
        
        try:
            # Map steering angle to servo range (from rot.py)
            servo_angle = self.map_steering_angle(angle_degrees)
            if servo_angle is None:
                logger.error(f"❌ Failed to map angle: {angle_degrees}°")
                return
            
            # Convert servo angle (60-120) to duty cycle (2-12 for 50Hz) - from rot.py
            duty_cycle = 2 + (servo_angle / 18)
            
            # Apply PWM signal
            self.servo_pwm.ChangeDutyCycle(duty_cycle)
            self.current_servo_angle = angle_degrees
            
            logger.info(f"🎯 Servo: {angle_degrees}° (mapped: {servo_angle:.1f}°, duty: {duty_cycle:.2f}%)")
            
            # Hold position briefly, then turn off signal (rot.py style)
            time.sleep(0.3)
            self.servo_pwm.ChangeDutyCycle(0)
            
        except Exception as e:
            logger.error(f"❌ Error setting servo angle: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current system status"""
        return {
            "running": self.running,
            "obstacle_detected": self.obstacle_detected,
            "current_servo_angle": self.current_servo_angle,
            "motor_speed": self.motor_speed,
            "target_angle": self.TARGET_ANGLE,
            "obstacle_threshold": self.OBSTACLE_THRESHOLD,
            "normal_steering": self.NORMAL_STEERING,
            "avoid_steering": self.AVOID_STEERING,
            "lap_count": self.lap_count,
            "max_laps": self.max_laps,
            "avoiding_obstacle": self.avoiding_obstacle,
            "lidar_server_running": self.lidar_server_process is not None
        }
    
    def run_until_complete(self):
        """Run the system until completion or shutdown signal"""
        if not self.start():
            logger.error("❌ Failed to start system")
            return False
        
        try:
            # Main loop - just monitor status
            last_status_time = 0
            while self.running and not self.shutdown_requested:
                current_time = time.time()
                
                # Log status every 10 seconds
                if current_time - last_status_time >= 10:
                    status = self.get_status()
                    logger.info(f"Status - Lap: {status['lap_count']}/{status['max_laps']}, "
                              f"Obstacle: {'🚨 YES' if status['obstacle_detected'] else '✅ NO'}, "
                              f"Servo: {status['current_servo_angle']}°, "
                              f"Motor: {status['motor_speed']}%, "
                              f"Avoiding: {'🔄 YES' if status['avoiding_obstacle'] else '✅ NO'}")
                    last_status_time = current_time
                
                time.sleep(1.0)
                
            if not self.shutdown_requested:
                logger.info("Challenge completed successfully!")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Error in main loop: {e}")
            return False
        
        finally:
            self.cleanup()

def main():
    """Main function for headless operation"""
    logger.info("=" * 60)
    logger.info("LiDAR Obstacle Avoidance System - HEADLESS MODE")
    logger.info("=" * 60)
    logger.info("🚀 Designed for automatic startup without user interaction")
    logger.info("📊 Features:")
    logger.info("   • Automatically starts LiDAR HTTP server")
    logger.info("   • Monitors LiDAR for obstacles at 180° (front of robot)")
    logger.info("   • When distance < 1500mm detected:")
    logger.info("     - Keeps motor speed unchanged")
    logger.info("     - Turns servo to 45° (right turn)")
    logger.info("     - After 2.0 seconds, returns to 90° (normal)")
    logger.info("   • Completes 7 laps automatically")
    logger.info("   • Logs all activity to /var/log/wro/obstacle_avoidance.log")
    logger.info("🔧 Hardware:")
    logger.info("   • Motor connected to GPIO 18")
    logger.info("   • Servo connected to GPIO 23 (BOARD pin 16)")
    logger.info("   • LiDAR on /dev/ttyUSB0")
    logger.info("=" * 60)
    
    avoidance_system = LidarObstacleAvoidanceHeadless()
    
    try:
        # Run the system until completion
        success = avoidance_system.run_until_complete()
        
        if success:
            logger.info("✅ System completed successfully")
            sys.exit(0)
        else:
            logger.error("❌ System failed")
            sys.exit(1)
            
    except Exception as e:
        logger.error(f"❌ Fatal error: {e}")
        sys.exit(1)
    
    finally:
        logger.info("🛑 System shutdown complete")

if __name__ == "__main__":
    main()
