#!/usr/bin/env python3
"""
PiRover with Web Server API
Run with: sudo python3 main.py
"""

import RPi.GPIO as GPIO
import time
import threading
import json
from flask import Flask, request, jsonify
from collections import deque
from compass import HMC5883L

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
SAFE_DISTANCE = 20
MISSION_SIDES = 4
START_SPEED = 10
SPEED_STEP = 2
SPEED_RAMP_INTERVAL = 0.35
TURN_TOLERANCE_DEG = 4.0
API_HOST = '0.0.0.0'
API_PORT = 5000

# ========== FLASK APP ==========
app = Flask(__name__)
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
        print(f"✅ Motors ready ({self.current_speed}%)")
    
    def set_speed(self, speed):
        with self._lock:
            self.current_speed = max(0, min(100, speed))
            if self.running:
                self.pwm_a.ChangeDutyCycle(self.current_speed)
                self.pwm_b.ChangeDutyCycle(self.current_speed)
    
    def forward(self):
        with self._lock:
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
            GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def backward(self):
        with self._lock:
            self.pwm_a.ChangeDutyCycle(35)
            self.pwm_b.ChangeDutyCycle(35)
            GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def turn_left(self):
        with self._lock:
            self.pwm_a.ChangeDutyCycle(50)
            self.pwm_b.ChangeDutyCycle(50)
            GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def turn_right(self):
        with self._lock:
            self.pwm_a.ChangeDutyCycle(50)
            self.pwm_b.ChangeDutyCycle(50)
            GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
            GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def stop(self):
        with self._lock:
            GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
            GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)
    
    def cleanup(self):
        self.running = False
        self.stop()
        if self.pwm_a:
            self.pwm_a.stop()
        if self.pwm_b:
            self.pwm_b.stop()


class ObstacleDetector:
    def __init__(self):
        self.distance = 999
        self.last_ir = [0, 0, 0, 0]
        self._lock = threading.Lock()
    
    def setup(self):
        GPIO.setup(PINS['ULTRASONIC_TRIG'], GPIO.OUT)
        GPIO.setup(PINS['ULTRASONIC_ECHO'], GPIO.IN)
        GPIO.output(PINS['ULTRASONIC_TRIG'], GPIO.LOW)
        
        ir_pins = [PINS['IR_TOP_LEFT'], PINS['IR_TOP_RIGHT'], 
                   PINS['IR_BOTTOM_LEFT'], PINS['IR_BOTTOM_RIGHT']]
        for pin in ir_pins:
            GPIO.setup(pin, GPIO.IN)
        print("✅ Sensors ready")
    
    def get_distance(self):
        try:
            GPIO.output(PINS['ULTRASONIC_TRIG'], False)
            time.sleep(0.05)
            GPIO.output(PINS['ULTRASONIC_TRIG'], True)
            time.sleep(0.00001)
            GPIO.output(PINS['ULTRASONIC_TRIG'], False)
            
            timeout = time.time() + 0.1
            while GPIO.input(PINS['ULTRASONIC_ECHO']) == 0 and time.time() < timeout:
                start = time.time()
            if time.time() >= timeout:
                return 999
            
            timeout = time.time() + 0.1
            while GPIO.input(PINS['ULTRASONIC_ECHO']) == 1 and time.time() < timeout:
                end = time.time()
            if time.time() >= timeout:
                return 999
            
            dist = (end - start) * 17150
            with self._lock:
                self.distance = dist if 2 < dist < 400 else 999
            return self.distance
        except:
            return 999
    
    def get_ir(self):
        with self._lock:
            self.last_ir = [
                1 if GPIO.input(PINS['IR_TOP_LEFT']) == 0 else 0,
                1 if GPIO.input(PINS['IR_TOP_RIGHT']) == 0 else 0,
                1 if GPIO.input(PINS['IR_BOTTOM_LEFT']) == 0 else 0,
                1 if GPIO.input(PINS['IR_BOTTOM_RIGHT']) == 0 else 0
            ]
            return self.last_ir.copy()
    
    def get_sensor_data(self):
        """Get all sensor data in one call"""
        distance = self.get_distance()
        ir = self.get_ir()
        return {
            'distance': distance if distance < 999 else None,
            'ir_top_left': bool(ir[0]),
            'ir_top_right': bool(ir[1]),
            'ir_bottom_left': bool(ir[2]),
            'ir_bottom_right': bool(ir[3])
        }


class NavigationSystem:
    def __init__(self):
        self.motors = MotorController()
        self.detector = ObstacleDetector()
        self.compass = None
        self.running = False
        self.auto_mode = True
        self.is_moving = False
        self.mission_active = False
        self.current_heading = None
        self.mission_progress = 0
        self._status_lock = threading.Lock()
        
        # LED state
        self.led2_blink_state = False
        self.last_led2_toggle = 0.0
        self.led2_blink_interval = 0.2
    
    def setup_indicators(self):
        for pin in [PINS['LED1'], PINS['LED2'], PINS['LED3']]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        GPIO.setup(PINS['BUZZER'], GPIO.OUT)

    def set_moving(self, moving):
        self.is_moving = moving
        if not moving:
            self.led2_blink_state = False
            GPIO.output(PINS['LED2'], GPIO.LOW)

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
    
    def initialize(self):
        print("\n" + "="*50)
        print("   PiRover Web Server System")
        print("="*50)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.setup_indicators()
        GPIO.output(PINS['LED1'], GPIO.HIGH)
        
        # Initialize hardware
        self.motors.setup()
        self.detector.setup()

        # Initialize compass
        try:
            self.compass = HMC5883L()
            print("✅ Compass ready")
        except Exception as e:
            self.compass = None
            print(f"⚠️ Compass unavailable ({e})")
        
        # Ready signal
        self.beep(0.2)
        time.sleep(0.1)
        self.beep(0.2)
        
        print(f"\n✅ System Ready! (Cruise speed: {ROVER_SPEED}%)")
        print(f"🌐 API Server: http://{API_HOST}:{API_PORT}")
        print(f"📡 Endpoints:")
        print("   GET  /status         - Get all sensor data and status")
        print("   GET  /sensors        - Get sensor readings only")
        print("   POST /control        - Control rover movement")
        print("   POST /speed          - Set motor speed")
        print("   POST /mission/start  - Start square mission")
        print("   POST /mission/stop   - Stop current mission")
        print("   POST /mode           - Set auto/manual mode")
        print("\nPress Ctrl+C to stop\n")
        
        return True
    
    def get_status(self):
        """Get comprehensive status for API"""
        sensor_data = self.detector.get_sensor_data()
        
        with self._status_lock:
            heading = self.current_heading
            mission_active = self.mission_active
            mission_progress = self.mission_progress
        
        return {
            'status': {
                'running': self.running,
                'auto_mode': self.auto_mode,
                'is_moving': self.is_moving,
                'mission_active': mission_active,
                'mission_progress': mission_progress
            },
            'sensors': sensor_data,
            'motors': {
                'current_speed': self.motors.current_speed,
                'speed_percent': self.motors.current_speed
            },
            'navigation': {
                'heading': heading,
                'heading_available': heading is not None
            }
        }
    
    def read_heading(self, samples=5):
        if not self.compass:
            return None

        headings = []
        for _ in range(samples):
            data = self.compass.read_raw_data()
            if not data:
                continue
            x, z, y = data
            headings.append(self.compass.get_heading(x, y, z))
            time.sleep(0.01)

        if not headings:
            return None
        
        heading = sum(headings) / len(headings)
        with self._status_lock:
            self.current_heading = heading
        return heading

    @staticmethod
    def _normalize_heading(heading):
        return heading % 360.0

    @staticmethod
    def _angle_error(current, target):
        return ((target - current + 540.0) % 360.0) - 180.0

    def turn_right_90(self):
        start_heading = self.read_heading()

        if start_heading is None:
            self.set_moving(True)
            self.motors.turn_right()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            self.set_moving(False)
            return False

        target_heading = self._normalize_heading(start_heading + 90.0)
        print(f"\n↪️ Turn start: {start_heading:6.1f}° -> target: {target_heading:6.1f}°")

        in_tolerance_count = 0
        start_time = time.time()

        while self.running and time.time() - start_time < 8.0:
            self.update_led2_blink()
            heading = self.read_heading(samples=3)

            if heading is None:
                continue

            error = self._angle_error(heading, target_heading)
            abs_error = abs(error)

            if abs_error <= TURN_TOLERANCE_DEG:
                in_tolerance_count += 1
                self.motors.stop()
            else:
                in_tolerance_count = 0
                self.set_moving(True)
                self.motors.turn_right()

            print(f"\r🧭 Turning... heading:{heading:6.1f}° err:{error:6.1f}°", end='')

            if in_tolerance_count >= 3:
                self.motors.stop()
                self.set_moving(False)
                print(f"\r✅ Turn done   heading:{heading:6.1f}° err:{error:6.1f}°")
                return True

            time.sleep(0.04)

        self.motors.stop()
        self.set_moving(False)
        print("\n⚠️ Turn timeout - continuing mission")
        return False

    def run_square_mission(self):
        side_index = 0
        with self._status_lock:
            self.mission_active = True
            self.mission_progress = 0

        while self.running and side_index < MISSION_SIDES and self.mission_active:
            side_number = side_index + 1
            print(f"\n\n🟩 Side {side_number}/{MISSION_SIDES} - moving forward")

            current_speed = START_SPEED
            self.motors.set_speed(current_speed)
            last_ramp = time.time()

            while self.running and self.mission_active:
                self.update_led2_blink()

                dist = self.detector.get_distance()
                ir = self.detector.get_ir()
                ir_str = ''.join(['X' if x else '.' for x in ir])

                if dist < SAFE_DISTANCE:
                    self.motors.stop()
                    self.set_moving(False)
                    self.beep(0.08)
                    print(f"\n🛑 Safe distance reached ({int(dist)}cm). Preparing right turn...")
                    break

                now = time.time()
                if now - last_ramp >= SPEED_RAMP_INTERVAL and current_speed < ROVER_SPEED:
                    current_speed = min(ROVER_SPEED, current_speed + SPEED_STEP)
                    self.motors.set_speed(current_speed)
                    last_ramp = now

                self.set_moving(True)
                self.motors.forward()

                heading = self.read_heading(samples=2)
                heading_text = f"{heading:6.1f}°" if heading is not None else "  n/a  "

                print(
                    f"\r📡 Side:{side_number} Dist:{int(dist):3d}cm "
                    f"IR:[{ir_str}] Speed:{self.motors.current_speed:2d}% "
                    f"Heading:{heading_text}",
                    end=''
                )

                time.sleep(0.05)

            if not self.running or not self.mission_active:
                break

            self.turn_right_90()
            side_index += 1
            with self._status_lock:
                self.mission_progress = (side_index / MISSION_SIDES) * 100

        self.motors.stop()
        self.set_moving(False)
        with self._status_lock:
            self.mission_active = False
            self.mission_progress = 100 if side_index >= MISSION_SIDES else 0
        
        if side_index >= MISSION_SIDES:
            print("\n\n🏁 Square mission complete. Returned to base path.")
        return True
    
    def run(self):
        self.running = True
        
        try:
            # Start the Flask server in a separate thread
            api_thread = threading.Thread(target=self.run_api_server, daemon=True)
            api_thread.start()
            
            # Main loop - run mission if in auto mode
            while self.running:
                if self.auto_mode and not self.mission_active:
                    self.run_square_mission()
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping...")
        finally:
            self.stop()
    
    def run_api_server(self):
        """Run Flask API server in a separate thread"""
        global nav_system
        nav_system = self
        app.run(host=API_HOST, port=API_PORT, debug=False, use_reloader=False)
    
    def stop(self):
        self.running = False
        with self._status_lock:
            self.mission_active = False
        self.set_moving(False)
        self.motors.stop()
        GPIO.output(PINS['LED1'], GPIO.LOW)
        GPIO.cleanup()
        print("✅ System stopped")


# ========== FLASK API ROUTES ==========

@app.route('/status', methods=['GET'])
def get_status():
    """Get complete rover status including all sensors"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    return jsonify(nav_system.get_status())


@app.route('/sensors', methods=['GET'])
def get_sensors():
    """Get only sensor readings"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    return jsonify(nav_system.detector.get_sensor_data())


@app.route('/control', methods=['POST'])
def control_rover():
    """Control rover movement"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Missing JSON body'}), 400
    
    command = data.get('command', '').lower()
    
    # If auto mode is on, stop auto mission first
    if nav_system.auto_mode:
        with nav_system._status_lock:
            nav_system.mission_active = False
        nav_system.motors.stop()
        nav_system.set_moving(False)
        # Switch to manual mode
        nav_system.auto_mode = False
        print("🔄 Switched to manual control")
    
    # Execute command
    if command == 'forward':
        nav_system.set_moving(True)
        nav_system.motors.forward()
    elif command == 'backward':
        nav_system.set_moving(True)
        nav_system.motors.backward()
    elif command == 'left':
        nav_system.set_moving(True)
        nav_system.motors.turn_left()
    elif command == 'right':
        nav_system.set_moving(True)
        nav_system.motors.turn_right()
    elif command == 'stop':
        nav_system.motors.stop()
        nav_system.set_moving(False)
    else:
        return jsonify({'error': f'Unknown command: {command}'}), 400
    
    return jsonify({
        'status': 'ok',
        'command': command,
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
    return jsonify({
        'status': 'ok',
        'speed': speed,
        'message': f'Speed set to {speed}%'
    })


@app.route('/mission/start', methods=['POST'])
def start_mission():
    """Start the square mission"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    if nav_system.mission_active:
        return jsonify({'error': 'Mission already running'}), 409
    
    data = request.get_json() or {}
    sides = data.get('sides', MISSION_SIDES)
    
    # Update mission parameters
    # global MISSION_SIDES
    MISSION_SIDES = sides
    
    nav_system.auto_mode = True
    with nav_system._status_lock:
        nav_system.mission_active = True
        nav_system.mission_progress = 0
    
    return jsonify({
        'status': 'ok',
        'message': f'Starting square mission with {sides} sides',
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
    
    return jsonify({
        'status': 'ok',
        'message': 'Mission stopped',
        'auto_mode': nav_system.auto_mode
    })


@app.route('/mode', methods=['POST'])
def set_mode():
    """Set auto or manual mode"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Missing JSON body'}), 400
    
    auto_mode = data.get('auto_mode')
    if auto_mode is None:
        return jsonify({'error': 'Missing auto_mode parameter'}), 400
    
    nav_system.auto_mode = bool(auto_mode)
    
    # If switching to manual, stop any ongoing mission
    if not nav_system.auto_mode:
        with nav_system._status_lock:
            nav_system.mission_active = False
        nav_system.motors.stop()
        nav_system.set_moving(False)
    
    return jsonify({
        'status': 'ok',
        'auto_mode': nav_system.auto_mode,
        'message': f'Switched to {"AUTO" if nav_system.auto_mode else "MANUAL"} mode'
    })


@app.route('/heading', methods=['GET'])
def get_heading():
    """Get current compass heading"""
    if nav_system is None:
        return jsonify({'error': 'System not initialized'}), 503
    
    heading = nav_system.read_heading()
    return jsonify({
        'heading': heading,
        'available': heading is not None
    })


@app.route('/ping', methods=['GET'])
def ping():
    """Health check endpoint"""
    return jsonify({
        'status': 'ok',
        'message': 'PiRover API is running'
    })


def main():
    nav = NavigationSystem()
    if nav.initialize():
        nav.run()

if __name__ == "__main__":
    main()