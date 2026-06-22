#!/usr/bin/env python3
"""
PiRover with Web Server API
Run with: sudo python3 main.py
"""

import RPi.GPIO as GPIO
import time
import threading
import json
import logging
from flask import Flask, request, jsonify
from flask_cors import CORS
from collections import deque
from compass import HMC5883L

# ========== LOGGING SETUP ==========
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# ========== PIN CONFIGURATION ==========
PINS = {
    'L298N_IN1': 17, 'L298N_IN2': 18, 'L298N_IN3': 22, 'L298N_IN4': 23,
    'L298N_ENA': 12, 'L298N_ENB': 13,
    'ULTRASONIC_TRIG': 5, 'ULTRASONIC_ECHO': 6,
    'IR_TOP_LEFT': 26, 'IR_TOP_RIGHT': 20,
    'IR_BOTTOM_LEFT': 21, 'IR_BOTTOM_RIGHT': 16,
    'LED1': 24, 'LED2': 25, 'LED3': 8, 'BUZZER': 27
}

# ========== CONFIGURATION ==========
ROVER_SPEED = 20
OBSTACLE_THRESHOLD = 30
CRITICAL_DISTANCE = 15
TURN_DURATION = 0.8
SAFE_DISTANCE = 25
MISSION_SIDES = 4
START_SPEED = 10
SPEED_STEP = 2
SPEED_RAMP_INTERVAL = 0.35
TURN_TOLERANCE_DEG = 4.0
API_HOST = '0.0.0.0'
API_PORT = 5000
NAVIGATION_MODE = 'manual'  # 'manual' or 'auto'

# ========== FLASK APP ==========
app = Flask(__name__)
CORS(app)
nav_system = None  # Will be set during initialization


class MotorController:
    def __init__(self):
        self.current_speed = ROVER_SPEED
        self.pwm_a = None
        self.pwm_b = None
        self.running = False
        self._lock = threading.Lock()
    
    def setup(self):
        GPIO.setup(PINS['L298N_IN1'], GPIO.OUT)
        GPIO.setup(PINS['L298N_IN2'], GPIO.OUT)
        GPIO.setup(PINS['L298N_IN3'], GPIO.OUT)
        GPIO.setup(PINS['L298N_IN4'], GPIO.OUT)
        GPIO.setup(PINS['L298N_ENA'], GPIO.OUT)
        GPIO.setup(PINS['L298N_ENB'], GPIO.OUT)
        
        self.pwm_a = GPIO.PWM(PINS['L298N_ENA'], 1000)
        self.pwm_b = GPIO.PWM(PINS['L298N_ENB'], 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        self.running = True
        logger.info(f"✅ Motors ready ({self.current_speed}%)")
    
    def set_speed(self, speed):
        with self._lock:
            self.current_speed = max(0, min(100, speed))
            if self.running:
                self.pwm_a.ChangeDutyCycle(self.current_speed)
                self.pwm_b.ChangeDutyCycle(self.current_speed)
                logger.debug(f"Speed set to {self.current_speed}%")
    
    def forward(self):
        with self._lock:
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
            GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
            logger.debug("Motor: FORWARD")
    
    def backward(self):
        with self._lock:
            self.pwm_a.ChangeDutyCycle(35)
            self.pwm_b.ChangeDutyCycle(35)
            GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
            logger.debug("Motor: BACKWARD")
    
    def turn_left(self):
        with self._lock:
            self.pwm_a.ChangeDutyCycle(50)
            self.pwm_b.ChangeDutyCycle(50)
            GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
            logger.debug("Motor: TURN LEFT")
    
    def turn_right(self):
        with self._lock:
            self.pwm_a.ChangeDutyCycle(50)
            self.pwm_b.ChangeDutyCycle(50)
            GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
            logger.debug("Motor: TURN RIGHT")
    
    def stop(self):
        with self._lock:
            GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)
            logger.debug("Motor: STOP")
    
    def cleanup(self):
        self.running = False
        self.stop()
        if self.pwm_a:
            self.pwm_a.stop()
        if self.pwm_b:
            self.pwm_b.stop()
        logger.info("Motor cleanup complete")


class ObstacleDetector:
    def __init__(self):
        self.distance = 999
        self.last_ir = [0, 0, 0, 0]
        self._lock = threading.Lock()
        self.read_count = 0
        self.error_count = 0
    
    def setup(self):
        GPIO.setup(PINS['ULTRASONIC_TRIG'], GPIO.OUT)
        GPIO.setup(PINS['ULTRASONIC_ECHO'], GPIO.IN)
        GPIO.output(PINS['ULTRASONIC_TRIG'], GPIO.LOW)
        
        ir_pins = [PINS['IR_TOP_LEFT'], PINS['IR_TOP_RIGHT'], 
                   PINS['IR_BOTTOM_LEFT'], PINS['IR_BOTTOM_RIGHT']]
        for pin in ir_pins:
            GPIO.setup(pin, GPIO.IN)
        
        logger.info("✅ Sensors ready (Ultrasonic + IR)")
    
    def get_distance(self):
        """Get ultrasonic distance with detailed logging"""
        self.read_count += 1
        try:
            # Send trigger pulse
            GPIO.output(PINS['ULTRASONIC_TRIG'], False)
            time.sleep(0.05)
            GPIO.output(PINS['ULTRASONIC_TRIG'], True)
            time.sleep(0.00001)
            GPIO.output(PINS['ULTRASONIC_TRIG'], False)
            
            # Measure pulse start
            timeout = time.time() + 0.1
            start = time.time()
            while GPIO.input(PINS['ULTRASONIC_ECHO']) == 0 and time.time() < timeout:
                start = time.time()
            
            if time.time() >= timeout:
                logger.debug(f"Ultrasonic timeout (no pulse start) - read #{self.read_count}")
                self.error_count += 1
                return 999
            
            # Measure pulse end
            timeout = time.time() + 0.1
            end = time.time()
            while GPIO.input(PINS['ULTRASONIC_ECHO']) == 1 and time.time() < timeout:
                end = time.time()
            
            if time.time() >= timeout:
                logger.debug(f"Ultrasonic timeout (no pulse end) - read #{self.read_count}")
                self.error_count += 1
                return 999
            
            # Calculate distance
            dist = (end - start) * 17150
            
            # Validate reading
            if 2 < dist < 400:
                with self._lock:
                    self.distance = dist
                logger.debug(f"Ultrasonic distance: {dist:.1f}cm (read #{self.read_count})")
                return dist
            else:
                logger.debug(f"Ultrasonic out of range: {dist:.1f}cm (read #{self.read_count})")
                return 999
                
        except Exception as e:
            self.error_count += 1
            logger.error(f"Ultrasonic error: {e}")
            return 999
    
    def get_ir(self):
        """Get IR sensor states with logging"""
        try:
            with self._lock:
                self.last_ir = [
                    1 if GPIO.input(PINS['IR_TOP_LEFT']) == 0 else 0,
                    1 if GPIO.input(PINS['IR_TOP_RIGHT']) == 0 else 0,
                    1 if GPIO.input(PINS['IR_BOTTOM_LEFT']) == 0 else 0,
                    1 if GPIO.input(PINS['IR_BOTTOM_RIGHT']) == 0 else 0
                ]
            
            ir_str = ''.join(['X' if x else '.' for x in self.last_ir])
            logger.debug(f"IR sensors: [{ir_str}]")
            return self.last_ir.copy()
            
        except Exception as e:
            logger.error(f"IR sensor error: {e}")
            return [0, 0, 0, 0]
    
    def get_sensor_data(self):
        """Get all sensor data in one call"""
        distance = self.get_distance()
        ir = self.get_ir()
        
        data = {
            'distance': distance if distance < 999 else None,
            'ir_top_left': bool(ir[0]),
            'ir_top_right': bool(ir[1]),
            'ir_bottom_left': bool(ir[2]),
            'ir_bottom_right': bool(ir[3]),
            'sensor_stats': {
                'total_reads': self.read_count,
                'errors': self.error_count
            }
        }
        
        logger.info(f"Sensor Data - Distance: {data['distance']}cm, IR: [{''.join(['X' if x else '.' for x in ir])}]")
        return data


class NavigationSystem:
    def __init__(self):
        self.motors = MotorController()
        self.detector = ObstacleDetector()
        self.compass = None
        self.running = False
        self.auto_mode = False
        self.is_moving = False
        self.mission_active = False
        self.current_heading = None
        self.mission_progress = 0
        self._status_lock = threading.Lock()
        self.navigation_mode = 'manual'
        self.manual_command = 'stop'
        self.compass_initialized = False
        self.compass_error_count = 0
        self.compass_read_count = 0
        
        # LED state
        self.led2_blink_state = False
        self.last_led2_toggle = 0.0
        self.led2_blink_interval = 0.2
    
    def setup_indicators(self):
        for pin in [PINS['LED1'], PINS['LED2'], PINS['LED3']]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        GPIO.setup(PINS['BUZZER'], GPIO.OUT)
        logger.debug("Indicators setup complete")

    def set_moving(self, moving):
        self.is_moving = moving
        if not moving:
            self.led2_blink_state = False
            GPIO.output(PINS['LED2'], GPIO.LOW)
        logger.debug(f"Moving state: {moving}")

    def update_led2_blink(self):
        if not self.is_moving:
            return
        now = time.time()
        if now - self.last_led2_toggle >= self.led2_blink_interval:
            self.led2_blink_state = not self.led2_blink_state
            GPIO.output(PINS['LED2'], GPIO.HIGH if self.led2_blink_state else GPIO.LOW)
            self.last_led2_toggle = now
    
    def beep(self, duration=0.1):
        GPIO.output(PINS['BUZZER'], GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(PINS['BUZZER'], GPIO.LOW)
        logger.debug(f"Buzzer beep: {duration}s")
    
    def initialize(self):
        logger.info("="*50)
        logger.info("   PiRover Web Server System - Initializing")
        logger.info("="*50)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.setup_indicators()
        GPIO.output(PINS['LED1'], GPIO.HIGH)
        logger.info("LED1 ON - System starting")
        
        # Initialize hardware
        self.motors.setup()
        self.detector.setup()

        # Initialize compass with detailed logging
        logger.info("Initializing compass...")
        try:
            self.compass = HMC5883L()
            self.compass_initialized = True
            logger.info("✅ Compass ready")
            
            # Test compass reading
            logger.info("Testing compass reading...")
            test_data = self.compass.read_raw_data()
            if test_data:
                x, z, y = test_data
                logger.info(f"Compass test reading - X:{x:6d} Y:{y:6d} Z:{z:6d}")
                test_heading = self.compass.get_heading(x, y, z)
                logger.info(f"Compass test heading: {test_heading:.1f}°")
            else:
                logger.warning("⚠️ Compass test reading failed - no data")
                
        except Exception as e:
            self.compass = None
            self.compass_initialized = False
            logger.error(f"❌ Compass initialization failed: {e}")
            logger.error("   Check: I2C enabled, wiring, and compass module")
        
        # Ready signal
        self.beep(0.2)
        time.sleep(0.1)
        self.beep(0.2)
        
        logger.info(f"\n✅ System Ready! (Cruise speed: {ROVER_SPEED}%)")
        logger.info(f"🌐 API Server: http://{API_HOST}:{API_PORT}")
        logger.info("📡 Endpoints:")
        logger.info("   GET  /status              - Get all sensor data and status")
        logger.info("   GET  /sensors             - Get sensor readings only")
        logger.info("   POST /control             - Control rover movement")
        logger.info("   POST /speed               - Set motor speed")
        logger.info("   POST /mission/start       - Start square mission")
        logger.info("   POST /mission/stop        - Stop current mission")
        logger.info("   POST /mode                - Set auto/manual mode")
        logger.info("   POST /navigation/mode     - Set navigation mode (manual/auto)")
        logger.info("\nPress Ctrl+C to stop\n")
        
        return True
    
    def get_status(self):
        """Get comprehensive status for API with detailed logging"""
        logger.info("API Request: /status")
        sensor_data = self.detector.get_sensor_data()
        
        with self._status_lock:
            heading = self.current_heading
            mission_active = self.mission_active
            mission_progress = self.mission_progress
        
        # Test compass if available
        compass_test = None
        if self.compass and self.compass_initialized:
            try:
                test_data = self.compass.read_raw_data()
                if test_data:
                    x, z, y = test_data
                    compass_test = {
                        'raw': {'x': x, 'y': y, 'z': z},
                        'heading': self.compass.get_heading(x, y, z)
                    }
                    logger.debug(f"Compass test in status - X:{x} Y:{y} Z:{z}")
                else:
                    logger.warning("Compass test in status - no data")
            except Exception as e:
                logger.error(f"Compass test in status failed: {e}")
        
        response = {
            'status': {
                'running': self.running,
                'auto_mode': self.auto_mode,
                'navigation_mode': self.navigation_mode,
                'is_moving': self.is_moving,
                'mission_active': mission_active,
                'mission_progress': mission_progress,
                'manual_command': self.manual_command,
                'compass_initialized': self.compass_initialized,
                'compass_error_count': self.compass_error_count,
                'compass_read_count': self.compass_read_count
            },
            'sensors': sensor_data,
            'motors': {
                'current_speed': self.motors.current_speed,
                'speed_percent': self.motors.current_speed
            },
            'navigation': {
                'heading': heading,
                'heading_available': heading is not None,
                'compass_test': compass_test
            }
        }
        
        logger.info(f"Status Response - Heading: {heading}, Sensors: {sensor_data['distance']}cm")
        return response
    
    def read_heading(self, samples=5):
        """Read compass heading with detailed logging"""
        self.compass_read_count += 1
        
        if not self.compass or not self.compass_initialized:
            logger.warning(f"Compass not available (read #{self.compass_read_count})")
            return None

        headings = []
        logger.debug(f"Reading compass heading - attempt #{self.compass_read_count}")
        
        for i in range(samples):
            try:
                data = self.compass.read_raw_data()
                if not data:
                    logger.debug(f"Compass read #{i+1}/{samples} - no data")
                    continue
                
                x, z, y = data
                logger.debug(f"Compass raw #{i+1}: X:{x:6d} Y:{y:6d} Z:{z:6d}")
                heading = self.compass.get_heading(x, y, z)
                headings.append(heading)
                logger.debug(f"Compass heading #{i+1}: {heading:.1f}°")
                time.sleep(0.01)
                
            except Exception as e:
                self.compass_error_count += 1
                logger.error(f"Compass read error #{i+1}: {e}")
                continue

        if not headings:
            logger.warning(f"No valid heading readings (read #{self.compass_read_count})")
            return None
        
        # Calculate average
        avg_heading = sum(headings) / len(headings)
        logger.info(f"Compass heading: {avg_heading:.1f}° (samples: {len(headings)}/{samples})")
        
        with self._status_lock:
            self.current_heading = avg_heading
        
        return avg_heading

    @staticmethod
    def _normalize_heading(heading):
        return heading % 360.0

    @staticmethod
    def _angle_error(current, target):
        return ((target - current + 540.0) % 360.0) - 180.0

    def turn_right_90(self):
        logger.info("Starting 90-degree right turn")
        start_heading = self.read_heading()

        if start_heading is None:
            logger.warning("No compass heading - using timed turn")
            self.set_moving(True)
            self.motors.turn_right()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            self.set_moving(False)
            return False

        target_heading = self._normalize_heading(start_heading + 90.0)
        logger.info(f"Turn start: {start_heading:.1f}° -> target: {target_heading:.1f}°")

        in_tolerance_count = 0
        start_time = time.time()

        while self.running and time.time() - start_time < 8.0:
            self.update_led2_blink()
            heading = self.read_heading(samples=3)

            if heading is None:
                logger.debug("No heading during turn - continuing")
                continue

            error = self._angle_error(heading, target_heading)
            abs_error = abs(error)

            if abs_error <= TURN_TOLERANCE_DEG:
                in_tolerance_count += 1
                self.motors.stop()
                logger.debug(f"Within tolerance: {abs_error:.1f}° (count: {in_tolerance_count}/3)")
            else:
                in_tolerance_count = 0
                self.set_moving(True)
                self.motors.turn_right()
                logger.debug(f"Turning... heading:{heading:.1f}° err:{error:.1f}°")

            if in_tolerance_count >= 3:
                self.motors.stop()
                self.set_moving(False)
                logger.info(f"✅ Turn done - heading:{heading:.1f}° err:{error:.1f}°")
                return True

            time.sleep(0.04)

        self.motors.stop()
        self.set_moving(False)
        logger.warning("⚠️ Turn timeout - continuing mission")
        return False

    def run_square_mission(self):
        logger.info("Starting square mission")
        side_index = 0
        with self._status_lock:
            self.mission_active = True
            self.mission_progress = 0

        while self.running and side_index < MISSION_SIDES and self.mission_active:
            side_number = side_index + 1
            logger.info(f"Side {side_number}/{MISSION_SIDES} - starting")

            current_speed = START_SPEED
            self.motors.set_speed(current_speed)
            last_ramp = time.time()

            while self.running and self.mission_active:
                self.update_led2_blink()

                dist = self.detector.get_distance()
                ir = self.detector.get_ir()
                ir_str = ''.join(['X' if x else '.' for x in ir])

                # Check for obstacles
                if dist < SAFE_DISTANCE:
                    self.motors.stop()
                    self.set_moving(False)
                    self.beep(0.08)
                    logger.info(f"🛑 Safe distance reached ({int(dist)}cm). Preparing right turn...")
                    break

                # Speed ramping
                now = time.time()
                if now - last_ramp >= SPEED_RAMP_INTERVAL and current_speed < ROVER_SPEED:
                    current_speed = min(ROVER_SPEED, current_speed + SPEED_STEP)
                    self.motors.set_speed(current_speed)
                    last_ramp = now
                    logger.debug(f"Speed ramped to {current_speed}%")

                # Move forward
                self.set_moving(True)
                self.motors.forward()

                heading = self.read_heading(samples=2)
                heading_text = f"{heading:.1f}°" if heading is not None else "n/a"

                # Log every 10 iterations to avoid spam
                if side_index % 10 == 0:
                    logger.info(f"Side:{side_number} Dist:{int(dist):3d}cm IR:[{ir_str}] Speed:{self.motors.current_speed:2d}% Heading:{heading_text}")

                time.sleep(0.05)

            if not self.running or not self.mission_active:
                logger.info("Mission interrupted")
                break

            self.turn_right_90()
            side_index += 1
            with self._status_lock:
                self.mission_progress = (side_index / MISSION_SIDES) * 100
            logger.info(f"Side {side_number} complete - Progress: {self.mission_progress:.1f}%")

        self.motors.stop()
        self.set_moving(False)
        with self._status_lock:
            self.mission_active = False
            self.mission_progress = 100 if side_index >= MISSION_SIDES else 0
        
        if side_index >= MISSION_SIDES:
            logger.info("🏁 Square mission complete!")
        else:
            logger.info("Mission stopped early")
        return True

    def run_manual_navigation(self):
        """Run manual navigation based on commands received via API"""
        logger.debug("Manual navigation loop started")
        loop_count = 0
        
        while self.running and self.navigation_mode == 'manual':
            loop_count += 1
            
            # Execute manual command
            if self.manual_command == 'forward':
                dist = self.detector.get_distance()
                if dist < SAFE_DISTANCE:
                    self.motors.stop()
                    self.set_moving(False)
                    if loop_count % 20 == 0:  # Log every ~1 second
                        logger.info(f"🛑 Obstacle detected at {int(dist)}cm - stopping")
                else:
                    self.set_moving(True)
                    self.motors.forward()
                    if loop_count % 20 == 0:
                        logger.debug(f"Manual forward - distance: {int(dist)}cm")
                        
            elif self.manual_command == 'backward':
                self.set_moving(True)
                self.motors.backward()
                if loop_count % 20 == 0:
                    logger.debug("Manual backward")
                    
            elif self.manual_command == 'left':
                self.set_moving(True)
                self.motors.turn_left()
                if loop_count % 20 == 0:
                    logger.debug("Manual left turn")
                    
            elif self.manual_command == 'right':
                self.set_moving(True)
                self.motors.turn_right()
                if loop_count % 20 == 0:
                    logger.debug("Manual right turn")
                    
            elif self.manual_command == 'stop':
                self.motors.stop()
                self.set_moving(False)
                if loop_count % 20 == 0:
                    logger.debug("Manual stop")
            
            # Update LED blink
            self.update_led2_blink()
            
            # Short sleep to prevent CPU overload
            time.sleep(0.05)
        
        logger.info("Manual navigation loop ended")
    
    def run(self):
        self.running = True
        logger.info("Navigation system running")
        
        try:
            # Start the Flask server in a separate thread
            api_thread = threading.Thread(target=self.run_api_server, daemon=True)
            api_thread.start()
            logger.info(f"API server thread started on {API_HOST}:{API_PORT}")
            
            # Main loop - handle navigation modes
            while self.running:
                if self.navigation_mode == 'auto' and not self.mission_active:
                    logger.info("Auto mode - starting mission")
                    self.run_square_mission()
                elif self.navigation_mode == 'manual':
                    self.run_manual_navigation()
                else:
                    time.sleep(0.1)
                
        except KeyboardInterrupt:
            logger.info("\n🛑 Stopping...")
        finally:
            self.stop()
    
    def run_api_server(self):
        """Run Flask API server in a separate thread"""
        global nav_system
        nav_system = self
        logger.info("Flask API starting...")
        app.run(host=API_HOST, port=API_PORT, debug=False, use_reloader=False)
    
    def stop(self):
        self.running = False
        with self._status_lock:
            self.mission_active = False
        self.set_moving(False)
        self.motors.stop()
        GPIO.output(PINS['LED1'], GPIO.LOW)
        GPIO.cleanup()
        logger.info("✅ System stopped")


# ========== FLASK API ROUTES ==========

@app.route('/status', methods=['GET'])
def get_status():
    """Get complete rover status including all sensors"""
    if nav_system is None:
        logger.error("Status request - system not initialized")
        return jsonify({'error': 'System not initialized'}), 503
    return jsonify(nav_system.get_status())


@app.route('/sensors', methods=['GET'])
def get_sensors():
    """Get only sensor readings"""
    if nav_system is None:
        logger.error("Sensors request - system not initialized")
        return jsonify({'error': 'System not initialized'}), 503
    return jsonify(nav_system.detector.get_sensor_data())


@app.route('/control', methods=['POST'])
def control_rover():
    """Control rover movement (only works in manual mode)"""
    if nav_system is None:
        logger.error("Control request - system not initialized")
        return jsonify({'error': 'System not initialized'}), 503
    
    data = request.get_json()
    if not data:
        logger.warning("Control request - missing JSON body")
        return jsonify({'error': 'Missing JSON body'}), 400
    
    command = data.get('command', '').lower()
    valid_commands = ['forward', 'backward', 'left', 'right', 'stop']
    
    if command not in valid_commands:
        logger.warning(f"Control request - invalid command: {command}")
        return jsonify({'error': f'Unknown command: {command}'}), 400
    
    # Check if in manual mode
    if nav_system.navigation_mode != 'manual':
        logger.warning(f"Control request - not in manual mode: {nav_system.navigation_mode}")
        return jsonify({
            'error': f'Cannot control in {nav_system.navigation_mode} mode',
            'navigation_mode': nav_system.navigation_mode
        }), 403
    
    # Stop any running mission if auto mode was active
    if nav_system.auto_mode:
        with nav_system._status_lock:
            nav_system.mission_active = False
        nav_system.motors.stop()
        nav_system.set_moving(False)
        nav_system.auto_mode = False
        logger.info("Switched to manual control")
    
    # Store the manual command
    nav_system.manual_command = command
    logger.info(f"Manual control command: {command}")
    
    return jsonify({
        'status': 'ok',
        'command': command,
        'navigation_mode': nav_system.navigation_mode,
        'auto_mode': nav_system.auto_mode
    })


@app.route('/speed', methods=['POST'])
def set_speed():
    """Set motor speed (0-100)"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Missing JSON body'}), 400
    
    speed = data.get('speed')
    if speed is None:
        return jsonify({'error': 'Missing speed parameter'}), 400
    
    try:
        speed = int(speed)
        if speed < 0 or speed > 100:
            return jsonify({'error': 'Speed must be between 0 and 100'}), 400
    except ValueError:
        return jsonify({'error': 'Speed must be an integer'}), 400
    
    nav_system.motors.set_speed(speed)
    logger.info(f"Speed set to {speed}%")
    return jsonify({
        'status': 'ok',
        'speed': speed,
        'message': f'Speed set to {speed}%'
    })


@app.route('/mission/start', methods=['POST'])
def start_mission():
    """Start the square mission (only works in auto mode)"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    if nav_system.navigation_mode != 'auto':
        return jsonify({
            'error': f'Cannot start mission in {nav_system.navigation_mode} mode',
            'navigation_mode': nav_system.navigation_mode
        }), 403
    
    if nav_system.mission_active:
        return jsonify({'error': 'Mission already running'}), 409
    
    data = request.get_json() or {}
    sides = data.get('sides', MISSION_SIDES)
    
    # Update mission parameters
    MISSION_SIDES = sides
    
    nav_system.auto_mode = True
    with nav_system._status_lock:
        nav_system.mission_active = True
        nav_system.mission_progress = 0
    
    logger.info(f"Starting mission with {sides} sides")
    return jsonify({
        'status': 'ok',
        'message': f'Starting square mission with {sides} sides',
        'navigation_mode': nav_system.navigation_mode,
        'auto_mode': True
    })


@app.route('/mission/stop', methods=['POST'])
def stop_mission():
    """Stop the current mission"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    with nav_system._status_lock:
        nav_system.mission_active = False
        nav_system.mission_progress = 0
    
    nav_system.motors.stop()
    nav_system.set_moving(False)
    
    logger.info("Mission stopped")
    return jsonify({
        'status': 'ok',
        'message': 'Mission stopped',
        'navigation_mode': nav_system.navigation_mode,
        'auto_mode': nav_system.auto_mode
    })


@app.route('/mode', methods=['POST'])
def set_mode():
    """Set auto or manual mode (legacy compatibility)"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Missing JSON body'}), 400
    
    auto_mode = data.get('auto_mode')
    if auto_mode is None:
        return jsonify({'error': 'Missing auto_mode parameter'}), 400
    
    # Set navigation mode based on auto_mode
    if bool(auto_mode):
        nav_system.navigation_mode = 'auto'
        nav_system.auto_mode = True
        logger.info("Switched to AUTO mode (legacy)")
    else:
        nav_system.navigation_mode = 'manual'
        nav_system.auto_mode = False
        with nav_system._status_lock:
            nav_system.mission_active = False
        nav_system.motors.stop()
        nav_system.set_moving(False)
        nav_system.manual_command = 'stop'
        logger.info("Switched to MANUAL mode (legacy)")
    
    return jsonify({
        'status': 'ok',
        'navigation_mode': nav_system.navigation_mode,
        'auto_mode': nav_system.auto_mode,
        'message': f'Switched to {nav_system.navigation_mode.upper()} mode'
    })


@app.route('/navigation/mode', methods=['POST'])
def set_navigation_mode():
    """Set navigation mode to 'manual' or 'auto'"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Missing JSON body'}), 400
    
    mode = data.get('mode', '').lower()
    if mode not in ['manual', 'auto']:
        return jsonify({'error': 'Mode must be "manual" or "auto"'}), 400
    
    # Update mode
    nav_system.navigation_mode = mode
    
    if mode == 'auto':
        nav_system.auto_mode = True
        nav_system.manual_command = 'stop'
        logger.info("Switched to AUTO navigation mode")
    else:
        nav_system.auto_mode = False
        with nav_system._status_lock:
            nav_system.mission_active = False
        nav_system.motors.stop()
        nav_system.set_moving(False)
        nav_system.manual_command = 'stop'
        logger.info("Switched to MANUAL navigation mode")
    
    return jsonify({
        'status': 'ok',
        'navigation_mode': nav_system.navigation_mode,
        'auto_mode': nav_system.auto_mode,
        'message': f'Switched to {mode.upper()} navigation mode'
    })


@app.route('/navigation/mode', methods=['GET'])
def get_navigation_mode():
    """Get current navigation mode"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    return jsonify({
        'navigation_mode': nav_system.navigation_mode,
        'auto_mode': nav_system.auto_mode,
        'mission_active': nav_system.mission_active,
        'mission_progress': nav_system.mission_progress
    })


@app.route('/heading', methods=['GET'])
def get_heading():
    """Get current compass heading"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    heading = nav_system.read_heading()
    return jsonify({
        'heading': heading,
        'available': heading is not None,
        'read_count': nav_system.compass_read_count,
        'error_count': nav_system.compass_error_count
    })


@app.route('/compass/test', methods=['GET'])
def test_compass():
    """Test compass with raw data output"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    if nav_system.compass is None:
        return jsonify({
            'error': 'Compass not initialized',
            'compass_initialized': False
        }), 503
    
    try:
        data = nav_system.compass.read_raw_data()
        if data is None:
            return jsonify({
                'error': 'Failed to read compass data',
                'compass_initialized': nav_system.compass_initialized
            }), 500
        
        x, z, y = data
        heading = nav_system.compass.get_heading(x, y, z)
        
        return jsonify({
            'compass_initialized': True,
            'raw_data': {
                'x': x,
                'y': y,
                'z': z
            },
            'heading': heading,
            'timestamp': time.time()
        })
    except Exception as e:
        return jsonify({
            'error': str(e),
            'compass_initialized': nav_system.compass_initialized
        }), 500


@app.route('/ping', methods=['GET'])
def ping():
    """Health check endpoint"""
    logger.debug("Ping request")
    return jsonify({
        'status': 'ok',
        'message': 'PiRover API is running',
        'compass_available': nav_system.compass_initialized if nav_system else False
    })


def main():
    nav = NavigationSystem()
    if nav.initialize():
        nav.run()

if __name__ == "__main__":
    main()