#!/usr/bin/env python3
"""
Intelligent Navigation System for Raspberry Pi with BLE Broadcasting
Run with: sudo python3 main.py
"""

import RPi.GPIO as GPIO
import time
import math
import threading
import json
import os
import re
import subprocess
import struct
from collections import deque
import serial
import serial.tools.list_ports

# ========== BLE CONFIGURATION ==========
BLE_SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
BLE_CHARACTERISTIC_UUID = "abcd1234-5678-90ab-cdef-1234567890ab"
DEVICE_NAME = 'PiRover'
BLE_ADVERTISING_INTERVAL_MS = 200  # 200ms = 5Hz broadcast rate

# ========== PIN CONFIGURATION ==========
PINS = {
    # Sensors
    'DHT11': 4,
    'MQ135': 7,
    'GY271_SDA': 2,
    'GY271_SCL': 3,
    
    # Actuators
    'L298N_IN1': 17,
    'L298N_IN2': 18,
    'L298N_IN3': 22,
    'L298N_IN4': 23,
    'L298N_ENA': 12,
    'L298N_ENB': 13,
    
    'ULTRASONIC_TRIG': 5,
    'ULTRASONIC_ECHO': 6,
    
    'SERVO': 19,
    
    # IR Sensors (4x at 45 deg angles)
    'IR_TOP_LEFT': 26,     # 45 deg top-left
    'IR_TOP_RIGHT': 20,    # 45 deg top-right
    'IR_BOTTOM_LEFT': 21,  # 45 deg bottom-left
    'IR_BOTTOM_RIGHT': 16, # 45 deg bottom-right
    
    # LEDs
    'LED1': 24,
    'LED2': 25,
    'LED3': 8,
    
    # Buzzer
    'BUZZER': 27
}

# ========== GLOBAL SPEED CONFIGURATION ==========
# ⚙️ ADJUST THIS VALUE TO CHANGE ROVER SPEED (0-100)
# 0 = stopped, 30 = slow, 60 = medium, 100 = maximum
ROVER_SPEED = 40  # Default: 40% speed

# Speed presets for different conditions
SPEED_PRESETS = {
    'CRUISE': 60,      # Normal cruising speed
    'SLOW': 40,        # Slow speed (obstacle near)
    'TURN': 50,        # Turning speed
    'BACKUP': 35,      # Backing up speed
    'MIN': 20,         # Minimum operational speed
    'MAX': 85          # Maximum safe speed
}

# ========== NAVIGATION CONSTANTS ==========
OBSTACLE_THRESHOLD = 30  # cm (start slowing down)
SAFE_DISTANCE = 40       # cm
CRITICAL_DISTANCE = 15   # cm (emergency stop)
TURN_DURATION = 0.8      # seconds

# Direction mapping for IR sensors (angles relative to robot)
SENSOR_ANGLES = {
    'IR_TOP_LEFT': 135,      # 135 deg (top-left quadrant)
    'IR_TOP_RIGHT': 45,      # 45 deg (top-right quadrant)
    'IR_BOTTOM_LEFT': 225,   # 225 deg (bottom-left quadrant)
    'IR_BOTTOM_RIGHT': 315   # 315 deg (bottom-right quadrant)
}

class BLEReceiver:
    """Handles BLE packet reception using bluetoothctl"""
    
    def __init__(self):
        self.running = False
        self.receive_thread = None
        self.command_queue = deque(maxlen=10)
        self.last_packet_time = 0
        
    def setup_ble_advertising(self):
        """Setup BLE advertising for broadcasting"""
        try:
            # Kill any existing bluetoothctl processes
            subprocess.run(['sudo', 'killall', 'bluetoothctl'], stderr=subprocess.DEVNULL)
            time.sleep(1)
            
            # Start bluetoothctl with commands
            proc = subprocess.Popen(['bluetoothctl'], 
                                   stdin=subprocess.PIPE,
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL,
                                   text=True)
            
            commands = [
                'power on',
                'discoverable on',
                'pairable off',  # No pairing required
                f'advertise on',
                f'advertise uuid {BLE_SERVICE_UUID}',
                f'advertise service {BLE_SERVICE_UUID}',
                f'advertise data 02 01 06',  # Flags: LE General Discoverable + BR/EDR not supported
                f'advertise name {DEVICE_NAME}',
                'advertise tx-power on',
                'advertise interval 200',  # 200ms interval
                'advertise on'
            ]
            
            for cmd in commands:
                proc.stdin.write(cmd + '\n')
                proc.stdin.flush()
                time.sleep(0.1)
            
            print(f"✅ BLE advertising started as '{DEVICE_NAME}' (No pairing required)")
            return True
            
        except Exception as e:
            print(f"⚠️ BLE advertising setup error: {e}")
            return False
    
    def setup_ble_scan(self):
        """Setup BLE scanning to receive packets"""
        try:
            # Start scanning for BLE packets
            scan_proc = subprocess.Popen(['sudo', 'btmgmt', 'scan', 'on'],
                                        stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL)
            
            # Start bluetoothctl for receiving
            self.receive_proc = subprocess.Popen(['bluetoothctl', 'scan', 'on'],
                                                stdout=subprocess.PIPE,
                                                stderr=subprocess.DEVNULL,
                                                text=True)
            
            print("✅ BLE scanning enabled for receiving commands")
            return True
        except Exception as e:
            print(f"⚠️ BLE scan setup error: {e}")
            return False
    
    def start_receiving(self):
        """Start BLE packet reception thread"""
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
    def _receive_loop(self):
        """Background thread for receiving BLE packets"""
        while self.running:
            try:
                if hasattr(self, 'receive_proc') and self.receive_proc:
                    # Read line from bluetoothctl output
                    line = self.receive_proc.stdout.readline()
                    if line:
                        # Parse BLE packet - look for manufacturer specific data
                        # Format: "Device XX:XX:XX:XX:XX:XX RSSI: -XX Data: [hex]"
                        packet = self._parse_ble_packet(line)
                        if packet:
                            self.command_queue.append(packet)
                            self.last_packet_time = time.time()
            except Exception as e:
                pass
            time.sleep(0.01)
    
    def _parse_ble_packet(self, line):
        """Parse BLE packet for command data"""
        try:
            # Look for data in the packet
            if 'Data:' in line:
                # Extract hex data
                data_part = line.split('Data:')[-1].strip()
                hex_bytes = data_part.split()
                
                # Convert hex to bytes
                packet_bytes = bytes([int(b, 16) for b in hex_bytes if len(b) == 2])
                
                # Try to decode as JSON command
                if packet_bytes:
                    try:
                        command_str = packet_bytes.decode('utf-8')
                        command = json.loads(command_str)
                        return command
                    except:
                        # Try as simple text command
                        return {'type': 'raw', 'command': packet_bytes.decode('utf-8', errors='ignore')}
        except:
            pass
        return None
    
    def get_command(self):
        """Get next received command"""
        if self.command_queue:
            return self.command_queue.popleft()
        return None
    
    def stop(self):
        """Stop BLE reception"""
        self.running = False
        if hasattr(self, 'receive_proc') and self.receive_proc:
            self.receive_proc.terminate()
        subprocess.run(['sudo', 'killall', 'bluetoothctl'], stderr=subprocess.DEVNULL)
        print("🔵 BLE stopped")

class BLEBroadcaster:
    """Handles BLE broadcasting of sensor data and rover specs"""
    
    def __init__(self):
        self.running = False
        self.broadcast_thread = None
        self.current_data = {}
        
    def start_broadcasting(self):
        """Start broadcasting sensor data"""
        self.running = True
        self.broadcast_thread = threading.Thread(target=self._broadcast_loop)
        self.broadcast_thread.daemon = True
        self.broadcast_thread.start()
        print("📡 BLE broadcasting started (5Hz)")
        
    def _broadcast_loop(self):
        """Background thread for broadcasting BLE packets"""
        while self.running:
            if self.current_data:
                # Prepare broadcast packet
                packet = self._encode_packet(self.current_data)
                if packet:
                    self._send_ble_packet(packet)
            time.sleep(BLE_ADVERTISING_INTERVAL_MS / 1000.0)
    
    def _encode_packet(self, data):
        """Encode sensor data into BLE packet format"""
        try:
            # Convert to JSON string
            json_str = json.dumps(data)
            # Encode to bytes
            return json_str.encode('utf-8')
        except Exception as e:
            print(f"⚠️ Packet encoding error: {e}")
            return None
    
    def _send_ble_packet(self, packet):
        """Send BLE packet via bluetoothctl"""
        try:
            # Convert packet to hex for advertising data
            hex_str = ' '.join(f'{b:02x}' for b in packet[:31])  # Max 31 bytes for advertising data
            
            # Update advertising data
            proc = subprocess.Popen(['bluetoothctl'], 
                                   stdin=subprocess.PIPE,
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL,
                                   text=True)
            
            # Set advertising data (manufacturer specific data)
            cmd = f'advertise data {len(packet):02x} ff {hex_str}'
            proc.stdin.write(cmd + '\n')
            proc.stdin.write('advertise on\n')
            proc.stdin.flush()
            proc.terminate()
            
        except Exception as e:
            pass  # Silent fail to avoid console spam
    
    def update_data(self, sensor_data):
        """Update the current sensor data to broadcast"""
        self.current_data = sensor_data
    
    def broadcast_rover_specs(self):
        """Broadcast rover specifications once"""
        specs = {
            'type': 'rover_specs',
            'name': DEVICE_NAME,
            'sensors': {
                'ultrasonic': {'range_cm': [2, 400], 'fov_deg': 15},
                'ir_sensors': {
                    'count': 4,
                    'angles_deg': [135, 45, 225, 315],
                    'type': 'obstacle_detection'
                },
                'compass': {'type': 'GY-271', 'accuracy_deg': 1}
            },
            'actuators': {
                'motors': {'type': 'L298N', 'max_speed_percent': 100},
                'leds': 3,
                'buzzer': True
            },
            'capabilities': {
                'modes': ['auto', 'manual'],
                'max_speed_cm_s': 30,
                'turn_radius_cm': 0,  # Zero-turn capable
                'obstacle_threshold_cm': OBSTACLE_THRESHOLD,
                'critical_distance_cm': CRITICAL_DISTANCE
            },
            'firmware_version': '2.0',
            'timestamp': time.time()
        }
        
        # Broadcast specs packet
        packet = self._encode_packet(specs)
        if packet:
            self._send_ble_packet(packet)
            print("📡 Rover specifications broadcasted")
    
    def stop(self):
        """Stop broadcasting"""
        self.running = False
        print("📡 BLE broadcasting stopped")

class Compass:
    """GY-271 Compass sensor interface"""
    def __init__(self):
        self.reference_angle = 0
        self.current_angle = 0
        self.calibrated = False
    
    def initialize(self):
        """Initialize compass and set reference"""
        try:
            import smbus2
            self.bus = smbus2.SMBus(1)
            
            # Initialize HMC5883L
            self.bus.write_byte_data(0x1e, 0x02, 0x00)  # Continuous mode
            
            time.sleep(0.1)
            
            # Get initial reading for reference
            self.reference_angle = self.get_angle()
            self.calibrated = True
            print(f"🧭 Compass initialized. Reference angle: {self.reference_angle:.1f}°")
            return True
        except ImportError:
            print("⚠️  smbus2 not installed. Compass disabled.")
            return False
        except Exception as e:
            print(f"⚠️  Compass error: {e}")
            return False
    
    def get_angle(self):
        """Get current angle in degrees (0-360)"""
        if not self.calibrated:
            return 0
        
        try:
            # Read from HMC5883L
            data = self.bus.read_i2c_block_data(0x1e, 0x03, 6)
            
            # Convert to signed integers
            x = (data[0] << 8) | data[1]
            if x > 32767:
                x -= 65536
            z = (data[4] << 8) | data[5]
            if z > 32767:
                z -= 65536
            
            # Calculate angle
            angle = math.atan2(z, x) * 180 / math.pi
            if angle < 0:
                angle += 360
            
            self.current_angle = angle
            return angle
        except:
            return self.current_angle
    
    def get_relative_angle(self):
        """Get angle relative to reference (0 = reference direction)"""
        raw_angle = self.get_angle()
        relative = raw_angle - self.reference_angle
        if relative < 0:
            relative += 360
        return relative

class MotorController:
    """L298N Motor controller"""
    def __init__(self, base_speed=ROVER_SPEED):
        self.running = False
        self.base_speed = base_speed
        self.current_speed = base_speed
        self.pwm_a = None
        self.pwm_b = None
    
    def set_speed(self, speed):
        """Dynamically change motor speed (0-100)"""
        self.current_speed = max(0, min(100, speed))
        if self.running and self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
        print(f"  ⚡ Speed set to {self.current_speed}%")
    
    def get_speed(self):
        """Get current motor speed"""
        return self.current_speed
    
    def setup(self):
        """Setup motor pins and PWM"""
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
        print(f"✅ Motor controller initialized (Base speed: {self.base_speed}%)")
    
    def stop(self):
        """Stop all motors"""
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)
    
    def forward(self, speed=None):
        """Move forward"""
        use_speed = speed if speed is not None else self.current_speed
        use_speed = max(0, min(100, use_speed))
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(use_speed)
            self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def backward(self, speed=None):
        """Move backward"""
        use_speed = speed if speed is not None else SPEED_PRESETS['BACKUP']
        use_speed = max(0, min(100, use_speed))
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(use_speed)
            self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def turn_left(self, speed=None):
        """Turn left (counter-rotate)"""
        use_speed = speed if speed is not None else SPEED_PRESETS['TURN']
        use_speed = max(0, min(100, use_speed))
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(use_speed)
            self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def turn_right(self, speed=None):
        """Turn right (counter-rotate)"""
        use_speed = speed if speed is not None else SPEED_PRESETS['TURN']
        use_speed = max(0, min(100, use_speed))
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(use_speed)
            self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def cleanup(self):
        """Cleanup motor controller"""
        if self.running:
            self.stop()
            if self.pwm_a:
                self.pwm_a.stop()
            if self.pwm_b:
                self.pwm_b.stop()

class ObstacleDetector:
    """Obstacle detection using ultrasonic and IR sensors"""
    def __init__(self):
        self.obstacle_history = deque(maxlen=5)
        self.last_distance = 999
    
    def setup(self):
        """Setup ultrasonic pins"""
        GPIO.setup(PINS['ULTRASONIC_TRIG'], GPIO.OUT)
        GPIO.setup(PINS['ULTRASONIC_ECHO'], GPIO.IN)
        GPIO.output(PINS['ULTRASONIC_TRIG'], GPIO.LOW)
        
        # Setup IR sensors (4x at 45 deg angles)
        ir_pins = [PINS['IR_TOP_LEFT'], PINS['IR_TOP_RIGHT'], 
                   PINS['IR_BOTTOM_LEFT'], PINS['IR_BOTTOM_RIGHT']]
        for pin in ir_pins:
            GPIO.setup(pin, GPIO.IN)
        
        print("✅ Obstacle detector initialized")
    
    def get_ultrasonic_distance(self):
        """Get distance from ultrasonic sensor"""
        trig = PINS['ULTRASONIC_TRIG']
        echo = PINS['ULTRASONIC_ECHO']
        
        try:
            # Send trigger pulse
            GPIO.output(trig, False)
            time.sleep(0.05)
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)
            
            # Wait for echo
            timeout = time.time() + 0.1
            while GPIO.input(echo) == 0 and time.time() < timeout:
                pulse_start = time.time()
            
            if time.time() >= timeout:
                self.last_distance = 999
                return 999
            
            timeout = time.time() + 0.1
            while GPIO.input(echo) == 1 and time.time() < timeout:
                pulse_end = time.time()
            
            if time.time() >= timeout:
                self.last_distance = 999
                return 999
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            
            self.last_distance = distance if 2 < distance < 400 else 999
            return self.last_distance
        except:
            self.last_distance = 999
            return 999
    
    def get_ir_readings(self):
        """Get all IR sensor readings"""
        readings = {
            'top_left': GPIO.input(PINS['IR_TOP_LEFT']),
            'top_right': GPIO.input(PINS['IR_TOP_RIGHT']),
            'bottom_left': GPIO.input(PINS['IR_BOTTOM_LEFT']),
            'bottom_right': GPIO.input(PINS['IR_BOTTOM_RIGHT'])
        }
        # IR sensors return 0 when object detected
        return {k: (v == 0) for k, v in readings.items()}
    
    def analyze_obstacles(self):
        """Analyze obstacle situation and return best action"""
        front_distance = self.get_ultrasonic_distance()
        ir_readings = self.get_ir_readings()
        
        # Store history
        self.obstacle_history.append({
            'front': front_distance,
            'ir': ir_readings
        })
        
        # Critical obstacle in front
        if front_distance < CRITICAL_DISTANCE:
            return 'STOP_AND_BACK'
        
        # Obstacle in front
        if front_distance < OBSTACLE_THRESHOLD:
            # Check IR sensors for escape path
            if not ir_readings['top_left'] and not ir_readings['bottom_left']:
                return 'TURN_LEFT'
            elif not ir_readings['top_right'] and not ir_readings['bottom_right']:
                return 'TURN_RIGHT'
            elif not ir_readings['top_left']:
                return 'TURN_LEFT'
            elif not ir_readings['top_right']:
                return 'TURN_RIGHT'
            else:
                return 'TURN_AROUND'
        
        # No immediate obstacle
        return 'FORWARD'
    
    def get_recommended_speed(self, front_distance):
        """Get recommended speed based on obstacle distance"""
        if front_distance < CRITICAL_DISTANCE:
            return 0  # Stop
        elif front_distance < OBSTACLE_THRESHOLD:
            # Slow down as obstacle gets closer
            ratio = (front_distance - CRITICAL_DISTANCE) / (OBSTACLE_THRESHOLD - CRITICAL_DISTANCE)
            speed = SPEED_PRESETS['MIN'] + (SPEED_PRESETS['SLOW'] - SPEED_PRESETS['MIN']) * ratio
            return int(speed)
        elif front_distance < SAFE_DISTANCE:
            # Slightly reduced speed
            ratio = (front_distance - OBSTACLE_THRESHOLD) / (SAFE_DISTANCE - OBSTACLE_THRESHOLD)
            speed = SPEED_PRESETS['SLOW'] + (ROVER_SPEED - SPEED_PRESETS['SLOW']) * ratio
            return int(speed)
        else:
            # Full speed
            return ROVER_SPEED

class NavigationSystem:
    """Main navigation system"""
    def __init__(self):
        self.compass = Compass()
        self.motors = MotorController(ROVER_SPEED)
        self.detector = ObstacleDetector()
        self.ble_broadcaster = BLEBroadcaster()
        self.ble_receiver = BLEReceiver()
        self.running = False
        self.navigation_thread = None
        self.data_publish_thread = None
        
        # Control modes - AUTO is default
        self.auto_mode = True  # Default to AUTO control
        self.manual_command = None
        
        # LED pins
        self.led_pins = [PINS['LED1'], PINS['LED2'], PINS['LED3']]
        
        # Buzzer pin
        self.buzzer_pin = PINS['BUZZER']
        
        # Speed control
        self.current_speed = ROVER_SPEED
        
        # Sensor data for broadcasting
        self.sensor_data = {
            'type': 'sensor_data',
            'front_distance': 999,
            'ir_sensors': {},
            'compass_angle': 0,
            'motor_speed': 0,
            'motor_speed_percent': 0,
            'auto_mode': True,
            'obstacle_action': 'FORWARD',
            'timestamp': 0
        }
    
    def setup_indicators(self):
        """Setup LED and buzzer pins"""
        for pin in self.led_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def led_on(self, led_num):
        """Turn on specific LED (1-3)"""
        if 1 <= led_num <= 3:
            GPIO.output(self.led_pins[led_num - 1], GPIO.HIGH)
    
    def led_off(self, led_num):
        """Turn off specific LED"""
        if 1 <= led_num <= 3:
            GPIO.output(self.led_pins[led_num - 1], GPIO.LOW)
    
    def led_all_off(self):
        """Turn off all LEDs"""
        for pin in self.led_pins:
            GPIO.output(pin, GPIO.LOW)
    
    def beep(self, duration=0.1):
        """Make a beep sound"""
        GPIO.output(self.buzzer_pin, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def alert_pattern(self):
        """Alert pattern for obstacles"""
        for _ in range(2):
            self.beep(0.05)
            time.sleep(0.05)
    
    def broadcast_sensor_data(self):
        """Continuously broadcast sensor data via BLE"""
        while self.running:
            # Update sensor data
            self.sensor_data['front_distance'] = self.detector.last_distance
            self.sensor_data['ir_sensors'] = self.detector.get_ir_readings()
            self.sensor_data['compass_angle'] = self.compass.get_angle()
            self.sensor_data['motor_speed_percent'] = self.current_speed
            self.sensor_data['auto_mode'] = self.auto_mode
            self.sensor_data['timestamp'] = time.time()
            
            # Get current action for broadcast
            if self.auto_mode:
                action = self.detector.analyze_obstacles()
                self.sensor_data['obstacle_action'] = action
            
            # Broadcast via BLE
            self.ble_broadcaster.update_data(self.sensor_data)
            
            time.sleep(0.2)  # Broadcast at 5Hz
    
    def process_ble_commands(self):
        """Process incoming BLE commands"""
        command = self.ble_receiver.get_command()
        
        if not command:
            return
        
        print(f"\n📱 Received BLE command: {command}")
        
        # Handle different command types
        if command.get('type') == 'mode':
            # Change control mode
            mode = command.get('mode', 'auto')
            if mode == 'auto':
                self.auto_mode = True
                print("🤖 Switched to AUTO navigation mode")
                self.beep(0.2)
                self.led_on(3)
                self.led_off(2)
            else:
                self.auto_mode = False
                print("🎮 Switched to MANUAL control mode")
                self.beep(0.1)
                self.led_off(3)
                self.led_on(2)
                
        elif command.get('type') == 'speed':
            # Change rover speed
            global ROVER_SPEED
            new_speed = command.get('speed', ROVER_SPEED)
            ROVER_SPEED = max(0, min(100, new_speed))
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            print(f"⚡ Speed set to {ROVER_SPEED}%")
            
        elif command.get('type') == 'control' and not self.auto_mode:
            # Manual control commands (only in manual mode)
            action = command.get('action', 'stop')
            
            if action == 'forward':
                self.motors.forward(self.current_speed)
                print("  ⬆️  Moving forward")
            elif action == 'backward':
                self.motors.backward()
                print("  ⬇️  Moving backward")
            elif action == 'left':
                self.motors.turn_left()
                print("  ⬅️  Turning left")
            elif action == 'right':
                self.motors.turn_right()
                print("  ➡️  Turning right")
            elif action == 'stop':
                self.motors.stop()
                print("  🛑 Stopped")
                
        elif command.get('type') == 'command':
            # Direct command
            cmd = command.get('command', '').lower()
            if cmd == 'stop':
                self.motors.stop()
            elif cmd == 'beep':
                self.beep(0.3)
    
    def initialize(self):
        """Initialize all systems"""
        print("\n🚀 Initializing Navigation System...")
        print(f"⚙️  Global speed setting: {ROVER_SPEED}%")
        print(f"   - Cruise speed: {SPEED_PRESETS['CRUISE']}%")
        print(f"   - Slow speed: {SPEED_PRESETS['SLOW']}%")
        print(f"   - Turn speed: {SPEED_PRESETS['TURN']}%")
        print(f"   - Backup speed: {SPEED_PRESETS['BACKUP']}%")
        print(f"\n🤖 Default mode: AUTO NAVIGATION")
        print("📡 BLE broadcasting enabled - No pairing required")
        
        # Setup GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup indicators
        self.setup_indicators()
        
        # Turn on LED 1 - System ready
        self.led_on(1)
        print("🔆 LED 1 ON - System ready")
        
        # Setup BLE broadcasting
        if self.ble_broadcaster.start_broadcasting():
            print("📡 BLE broadcaster ready")
            # Broadcast rover specs once
            self.ble_broadcaster.broadcast_rover_specs()
        
        # Setup BLE receiving
        if self.ble_receiver.setup_ble_scan():
            self.ble_receiver.start_receiving()
            print("📡 BLE receiver ready - Listening for commands")
        
        # Initialize components
        self.motors.setup()
        self.detector.setup()
        
        # Initialize compass
        if not self.compass.initialize():
            print("⚠️  Compass not available - using dead reckoning")
        
        # Beep to indicate ready
        self.beep(0.3)
        time.sleep(0.1)
        self.beep(0.3)
        
        print("\n✅ Navigation System Ready!")
        print("📡 Obstacle detection active")
        print(f"🤖 Control mode: AUTO (default)")
        print("\n📱 BLE Info:")
        print(f"   - Device Name: {DEVICE_NAME}")
        print(f"   - Service UUID: {BLE_SERVICE_UUID}")
        print(f"   - No pairing required")
        print(f"   - Broadcast rate: 5Hz")
        
        return True
    
    def manual_control_loop(self):
        """Manual control loop - processes BLE commands only"""
        while self.running and not self.auto_mode:
            self.process_ble_commands()
            time.sleep(0.05)
    
    def auto_navigation_loop(self):
        """Automatic navigation loop with obstacle avoidance"""
        while self.running and self.auto_mode:
            # Check for mode change command
            self.process_ble_commands()
            
            # Analyze obstacles
            action = self.detector.analyze_obstacles()
            
            # Get front distance for display
            front_distance = self.detector.get_ultrasonic_distance()
            
            # Update sensor data with current action
            self.sensor_data['obstacle_action'] = action
            
            # Display status
            if front_distance < 100:
                status = f"Front: {front_distance:.0f}cm"
            else:
                status = "Front: Clear"
            
            status += f" | Speed: {self.current_speed}%"
            
            ir_readings = self.detector.get_ir_readings()
            if any(ir_readings.values()):
                ir_active = [k for k, v in ir_readings.items() if v]
                status += f" | IR: {', '.join(ir_active)}"
            
            print(f"\r📍 {status} | Action: {action}", end="", flush=True)
            
            # Execute action
            self.execute_action(action, front_distance)
            
            # Small delay for stability
            time.sleep(0.05)
    
    def execute_action(self, action, front_distance):
        """Execute navigation action based on obstacle analysis"""
        if action == 'FORWARD':
            # Dynamic speed based on obstacle distance
            recommended_speed = self.detector.get_recommended_speed(front_distance)
            if recommended_speed != self.current_speed:
                self.current_speed = recommended_speed
                self.motors.set_speed(self.current_speed)
            
            self.motors.forward(self.current_speed)
            self.led_off(2)
            self.led_off(3)
            
        elif action == 'TURN_LEFT':
            print("  ↪️  Turning left to avoid obstacle")
            self.led_on(2)
            self.led_off(3)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.turn_left()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            self.alert_pattern()
            # Reset speed after turn
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            
        elif action == 'TURN_RIGHT':
            print("  ↩️  Turning right to avoid obstacle")
            self.led_off(2)
            self.led_on(3)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.turn_right()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            self.alert_pattern()
            # Reset speed after turn
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            
        elif action == 'TURN_AROUND':
            print("  🔄 Turning around - path blocked")
            self.led_on(2)
            self.led_on(3)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.turn_left()
            time.sleep(TURN_DURATION * 2)
            self.motors.stop()
            for _ in range(3):
                self.beep(0.1)
                time.sleep(0.1)
            # Reset speed
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            
        elif action == 'STOP_AND_BACK':
            print("  ⚠️  CRITICAL! Backing up...")
            for _ in range(3):
                self.beep(0.2)
                time.sleep(0.1)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.backward()
            time.sleep(1.0)
            self.motors.stop()
            time.sleep(0.1)
            # Random turn after backing up
            import random
            if random.choice([True, False]):
                self.motors.turn_left()
            else:
                self.motors.turn_right()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            # Reset speed
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
    
    def start(self):
        """Start navigation system"""
        # Start BLE broadcast thread
        self.ble_broadcaster.start_broadcasting()
        
        # Start sensor broadcast thread
        self.data_publish_thread = threading.Thread(target=self.broadcast_sensor_data)
        self.data_publish_thread.daemon = True
        self.data_publish_thread.start()
        
        print("\n🎯 System Active!")
        print("🤖 Default mode: AUTO NAVIGATION")
        print("📱 BLE commands accepted (no pairing required)")
        print("   - Mode control: {'type': 'mode', 'mode': 'auto' or 'manual'}")
        print("   - Speed control: {'type': 'speed', 'speed': 0-100}")
        print("   - Manual control: {'type': 'control', 'action': 'forward/backward/left/right/stop'}")
        print("   - Simple commands: {'type': 'command', 'command': 'stop/beep'}")
        print("\n📡 Sensor data broadcasted via BLE at 5Hz")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.running:
                if self.auto_mode:
                    self.auto_navigation_loop()
                else:
                    self.manual_control_loop()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping navigation...")
            self.stop()
    
    def stop(self):
        """Stop navigation system"""
        self.running = False
        
        self.motors.stop()
        self.led_all_off()
        
        # Cleanup
        self.motors.cleanup()
        self.ble_broadcaster.stop()
        self.ble_receiver.stop()
        GPIO.cleanup()
        
        print("✅ Navigation system stopped")
        print("👋 Goodbye!")

def main():
    """Main entry point"""
    print("="*60)
    print("   INTELLIGENT NAVIGATION SYSTEM v2.0")
    print("   - BLE Broadcasting (No Pairing Required)")
    print("   - 4x IR Sensors at 45° angles")
    print("   - Ultrasonic Sensor (forward)")
    print("   - AUTO Navigation Mode (Default)")
    print("   - Manual Mode via BLE Commands")
    print("="*60)
    print(f"\n⚙️  Current global speed setting: {ROVER_SPEED}%")
    print("   To change speed, edit ROVER_SPEED variable or send BLE command")
    print(f"\n📡 BLE Device Name: {DEVICE_NAME}")
    print(f"   Service UUID: {BLE_SERVICE_UUID}")
    print("   No pairing required - just listen for broadcasts")
    print()
    
    nav_system = NavigationSystem()
    
    try:
        if nav_system.initialize():
            nav_system.running = True
            nav_system.start()
    except Exception as e:
        print(f"\n❌ System error: {e}")
        nav_system.stop()

if __name__ == "__main__":
    main()