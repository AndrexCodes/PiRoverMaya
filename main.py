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
ROVER_SPEED = 40  # Default: 40% speed

# Speed presets for different conditions
SPEED_PRESETS = {
    'CRUISE': 60,
    'SLOW': 40,
    'TURN': 50,
    'BACKUP': 35,
    'MIN': 20,
    'MAX': 85
}

# ========== NAVIGATION CONSTANTS ==========
OBSTACLE_THRESHOLD = 30  # cm
SAFE_DISTANCE = 40       # cm
CRITICAL_DISTANCE = 15   # cm
TURN_DURATION = 0.8      # seconds

# Direction mapping for IR sensors
SENSOR_ANGLES = {
    'IR_TOP_LEFT': 135,
    'IR_TOP_RIGHT': 45,
    'IR_BOTTOM_LEFT': 225,
    'IR_BOTTOM_RIGHT': 315
}

class BLEManager:
    """Handles BLE setup, advertising, and packet reception"""
    
    def __init__(self):
        self.running = False
        self.advertising_process = None
        self.command_queue = deque(maxlen=10)
        self.current_ad_data = None
        self.advertising_pid = None
        
    def setup_bluetooth_interface(self):
        """Initialize and configure Bluetooth interface"""
        print("\n🔵 Setting up Bluetooth interface...")
        
        try:
            # Check if Bluetooth is available
            result = subprocess.run(['hciconfig'], capture_output=True, text=True)
            if 'No such file' in result.stderr or not result.stdout.strip():
                print("❌ Bluetooth adapter not found!")
                print("   Please ensure:")
                print("   1. Bluetooth dongle is connected")
                print("   2. Run: sudo apt install bluetooth bluez bluez-tools")
                return False
            
            # Find Bluetooth interface (usually hci0)
            hci_result = subprocess.run(['hcitool', 'dev'], capture_output=True, text=True)
            lines = hci_result.stdout.strip().split('\n')
            if len(lines) < 2:
                print("❌ No Bluetooth device found")
                return False
            
            self.bt_interface = lines[1].split()[0]
            print(f"✅ Found Bluetooth interface: {self.bt_interface}")
            
            # Bring up the interface
            subprocess.run(['sudo', 'hciconfig', self.bt_interface, 'up'], capture_output=True)
            subprocess.run(['sudo', 'hciconfig', self.bt_interface, 'piscan'], capture_output=True)
            
            # Set device name
            subprocess.run(['sudo', 'hciconfig', self.bt_interface, 'name', DEVICE_NAME], capture_output=True)
            
            # Check status
            status = subprocess.run(['hciconfig', self.bt_interface], capture_output=True, text=True)
            print(f"✅ Bluetooth interface configured")
            print(f"   Device: {DEVICE_NAME}")
            print(f"   Address: {self.get_bt_address()}")
            
            return True
            
        except Exception as e:
            print(f"❌ Bluetooth setup error: {e}")
            return False
    
    def get_bt_address(self):
        """Get Bluetooth MAC address"""
        try:
            result = subprocess.run(['hcitool', 'dev'], capture_output=True, text=True)
            lines = result.stdout.strip().split('\n')
            if len(lines) >= 2:
                return lines[1].split()[1]
        except:
            pass
        return "Unknown"
    
    def install_bluez_tools(self):
        """Install required Bluetooth tools if missing"""
        print("\n📦 Checking required packages...")
        
        # Check for bluetoothctl
        result = subprocess.run(['which', 'bluetoothctl'], capture_output=True)
        if result.returncode != 0:
            print("Installing bluez...")
            subprocess.run(['sudo', 'apt', 'update'], capture_output=True)
            subprocess.run(['sudo', 'apt', 'install', '-y', 'bluez', 'bluez-tools'], capture_output=True)
        
        # Check for Python BLE library
        try:
            import bluetooth
        except ImportError:
            print("Installing PyBluez...")
            subprocess.run(['sudo', 'pip3', 'install', 'pybluez'], capture_output=True)
        
        print("✅ Required packages installed")
        return True
    
    def start_ble_advertising(self):
        """Start BLE advertising using hcitool (most reliable method)"""
        print("\n📡 Starting BLE advertising...")
        
        # Stop any existing advertising
        self.stop_ble_advertising()
        
        # Set up advertising packet format
        # Format: AD Type | AD Data
        # 0x01 = Flags, 0x06 = LE General Discoverable + BR/EDR not supported
        # 0x09 = Complete Local Name
        # 0xFF = Manufacturer Specific Data
        
        try:
            # Create advertising data using hcitool
            # First, set the advertising data
            cmd_set_ad = f"sudo hcitool -i {self.bt_interface} cmd 0x08 0x0008 1e 02 01 06 0a 09 50 69 52 6f 76 65 72 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
            subprocess.run(cmd_set_ad, shell=True, capture_output=True)
            
            # Enable advertising
            cmd_enable_ad = f"sudo hcitool -i {self.bt_interface} cmd 0x08 0x000a 01"
            subprocess.run(cmd_enable_ad, shell=True, capture_output=True)
            
            print(f"✅ BLE advertising started on {self.bt_interface}")
            print(f"   Device name: {DEVICE_NAME}")
            print("   No pairing required - advertising as beacon")
            return True
            
        except Exception as e:
            print(f"⚠️ HCI tool advertising failed: {e}")
            return self.start_ble_advertising_alt()
    
    def start_ble_advertising_alt(self):
        """Alternative method using bluetoothctl"""
        try:
            # Start bluetoothctl in background
            self.advertising_process = subprocess.Popen(
                ['bluetoothctl'],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True
            )
            
            if self.advertising_process:
                commands = [
                    'power on',
                    'discoverable on',
                    'pairable off',
                    f'advertise on',
                    f'advertise name {DEVICE_NAME}',
                    'advertise service 12345678-1234-1234-1234-123456789abc',
                    'advertise interval 200',
                    'advertise tx-power on',
                ]
                
                for cmd in commands:
                    if self.advertising_process.stdin:
                        self.advertising_process.stdin.write(cmd + '\n')
                        self.advertising_process.stdin.flush()
                        time.sleep(0.1)
                
                print(f"✅ BLE advertising started (alt method)")
                return True
        except Exception as e:
            print(f"⚠️ Alt advertising failed: {e}")
        
        return False
    
    def stop_ble_advertising(self):
        """Stop BLE advertising"""
        try:
            # Disable advertising via hcitool
            subprocess.run(f"sudo hcitool -i {self.bt_interface} cmd 0x08 0x000a 00", shell=True, capture_output=True)
        except:
            pass
        
        if self.advertising_process:
            try:
                if self.advertising_process.stdin:
                    self.advertising_process.stdin.write('quit\n')
                    self.advertising_process.stdin.flush()
                self.advertising_process.terminate()
            except:
                pass
            self.advertising_process = None
    
    def broadcast_data(self, data_dict):
        """Broadcast data using manufacturer specific data"""
        if not self.running:
            return
        
        try:
            # Convert data to JSON string
            json_str = json.dumps(data_dict)
            # Limit to 25 bytes to fit in advertising packet
            if len(json_str) > 25:
                json_str = json_str[:25]
            
            # Convert to bytes
            data_bytes = json_str.encode('utf-8')
            
            # Create manufacturer specific data packet
            # Format: 0x08 0x0008 [length] 0xFF [company ID (2 bytes)] [data]
            # Using 0xFFFF as test company ID
            length = len(data_bytes) + 3  # +3 for type(1) + company ID(2)
            
            # Build command
            cmd_parts = ['sudo', 'hcitool', '-i', self.bt_interface, 'cmd', '0x08', '0x0008']
            
            # Add length and data
            cmd_parts.append(f'{length:02x}')
            cmd_parts.append('ff')  # Manufacturer specific data type
            cmd_parts.append('ff')  # Company ID low byte
            cmd_parts.append('ff')  # Company ID high byte
            
            # Add data bytes
            for byte in data_bytes:
                cmd_parts.append(f'{byte:02x}')
            
            # Pad to minimum length
            while len(cmd_parts) < 15:
                cmd_parts.append('00')
            
            # Execute command
            subprocess.run(' '.join(cmd_parts), shell=True, capture_output=True)
            
        except Exception as e:
            pass  # Silent fail to avoid console spam
    
    def setup_rfcomm_server(self):
        """Setup RFCOMM server for receiving commands (no pairing)"""
        try:
            # Kill existing rfcomm processes
            subprocess.run(['sudo', 'killall', 'rfcomm'], stderr=subprocess.DEVNULL)
            
            # Create a simple socket server
            import socket
            
            self.server_socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to any channel
            self.server_socket.bind((self.get_bt_address(), 1))
            self.server_socket.listen(1)
            
            # Start listening thread
            self.running = True
            self.rfcomm_thread = threading.Thread(target=self._rfcomm_listener)
            self.rfcomm_thread.daemon = True
            self.rfcomm_thread.start()
            
            print("✅ RFCOMM server started on channel 1")
            return True
            
        except Exception as e:
            print(f"⚠️ RFCOMM server error: {e}")
            return False
    
    def _rfcomm_listener(self):
        """Listen for incoming RFCOMM connections"""
        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                client, address = self.server_socket.accept()
                
                # Receive data
                data = client.recv(1024)
                if data:
                    try:
                        command = json.loads(data.decode('utf-8'))
                        self.command_queue.append(command)
                        print(f"\n📱 Received: {command}")
                    except:
                        pass
                
                client.close()
            except socket.timeout:
                continue
            except Exception as e:
                pass
    
    def get_command(self):
        """Get next received command"""
        if self.command_queue:
            return self.command_queue.popleft()
        return None
    
    def start(self):
        """Start BLE services"""
        self.running = True
        return self.setup_rfcomm_server()
    
    def stop(self):
        """Stop BLE services"""
        self.running = False
        self.stop_ble_advertising()
        
        if hasattr(self, 'server_socket'):
            try:
                self.server_socket.close()
            except:
                pass
        
        print("🔵 BLE services stopped")

class Compass:
    """GY-271 Compass sensor interface"""
    def __init__(self):
        self.reference_angle = 0
        self.current_angle = 0
        self.calibrated = False
    
    def initialize(self):
        try:
            import smbus2
            self.bus = smbus2.SMBus(1)
            self.bus.write_byte_data(0x1e, 0x02, 0x00)
            time.sleep(0.1)
            self.reference_angle = self.get_angle()
            self.calibrated = True
            print(f"🧭 Compass initialized. Reference: {self.reference_angle:.1f}°")
            return True
        except:
            print("⚠️ Compass not available")
            return False
    
    def get_angle(self):
        if not self.calibrated:
            return 0
        try:
            import smbus2
            bus = smbus2.SMBus(1)
            data = bus.read_i2c_block_data(0x1e, 0x03, 6)
            x = (data[0] << 8) | data[1]
            if x > 32767:
                x -= 65536
            z = (data[4] << 8) | data[5]
            if z > 32767:
                z -= 65536
            angle = math.atan2(z, x) * 180 / math.pi
            if angle < 0:
                angle += 360
            self.current_angle = angle
            return angle
        except:
            return self.current_angle

class MotorController:
    """L298N Motor controller"""
    def __init__(self, base_speed=ROVER_SPEED):
        self.running = False
        self.base_speed = base_speed
        self.current_speed = base_speed
        self.pwm_a = None
        self.pwm_b = None
    
    def set_speed(self, speed):
        self.current_speed = max(0, min(100, speed))
        if self.running and self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
        print(f"  ⚡ Speed: {self.current_speed}%")
    
    def get_speed(self):
        return self.current_speed
    
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
        print(f"✅ Motors initialized (speed: {self.base_speed}%)")
    
    def stop(self):
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)
    
    def forward(self, speed=None):
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
        GPIO.setup(PINS['ULTRASONIC_TRIG'], GPIO.OUT)
        GPIO.setup(PINS['ULTRASONIC_ECHO'], GPIO.IN)
        GPIO.output(PINS['ULTRASONIC_TRIG'], GPIO.LOW)
        
        ir_pins = [PINS['IR_TOP_LEFT'], PINS['IR_TOP_RIGHT'], 
                   PINS['IR_BOTTOM_LEFT'], PINS['IR_BOTTOM_RIGHT']]
        for pin in ir_pins:
            GPIO.setup(pin, GPIO.IN)
        
        print("✅ Obstacle detector initialized")
    
    def get_ultrasonic_distance(self):
        trig = PINS['ULTRASONIC_TRIG']
        echo = PINS['ULTRASONIC_ECHO']
        
        try:
            GPIO.output(trig, False)
            time.sleep(0.05)
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)
            
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
            
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            
            self.last_distance = distance if 2 < distance < 400 else 999
            return self.last_distance
        except:
            self.last_distance = 999
            return 999
    
    def get_ir_readings(self):
        readings = {
            'top_left': GPIO.input(PINS['IR_TOP_LEFT']),
            'top_right': GPIO.input(PINS['IR_TOP_RIGHT']),
            'bottom_left': GPIO.input(PINS['IR_BOTTOM_LEFT']),
            'bottom_right': GPIO.input(PINS['IR_BOTTOM_RIGHT'])
        }
        return {k: (v == 0) for k, v in readings.items()}
    
    def analyze_obstacles(self):
        front_distance = self.get_ultrasonic_distance()
        ir_readings = self.get_ir_readings()
        
        self.obstacle_history.append({
            'front': front_distance,
            'ir': ir_readings
        })
        
        if front_distance < CRITICAL_DISTANCE:
            return 'STOP_AND_BACK'
        
        if front_distance < OBSTACLE_THRESHOLD:
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
        
        return 'FORWARD'
    
    def get_recommended_speed(self, front_distance):
        if front_distance < CRITICAL_DISTANCE:
            return 0
        elif front_distance < OBSTACLE_THRESHOLD:
            ratio = (front_distance - CRITICAL_DISTANCE) / (OBSTACLE_THRESHOLD - CRITICAL_DISTANCE)
            speed = SPEED_PRESETS['MIN'] + (SPEED_PRESETS['SLOW'] - SPEED_PRESETS['MIN']) * ratio
            return int(speed)
        elif front_distance < SAFE_DISTANCE:
            ratio = (front_distance - OBSTACLE_THRESHOLD) / (SAFE_DISTANCE - OBSTACLE_THRESHOLD)
            speed = SPEED_PRESETS['SLOW'] + (ROVER_SPEED - SPEED_PRESETS['SLOW']) * ratio
            return int(speed)
        else:
            return ROVER_SPEED

class NavigationSystem:
    """Main navigation system"""
    def __init__(self):
        self.compass = Compass()
        self.motors = MotorController(ROVER_SPEED)
        self.detector = ObstacleDetector()
        self.ble = BLEManager()
        self.running = False
        self.data_publish_thread = None
        
        # Control modes - AUTO is default
        self.auto_mode = True
        
        # LED pins
        self.led_pins = [PINS['LED1'], PINS['LED2'], PINS['LED3']]
        self.buzzer_pin = PINS['BUZZER']
        self.current_speed = ROVER_SPEED
        
        # Sensor data for broadcasting
        self.sensor_data = {
            'type': 'sensor_data',
            'front_distance': 999,
            'ir_sensors': {},
            'compass_angle': 0,
            'motor_speed': 0,
            'auto_mode': True,
            'obstacle_action': 'FORWARD',
            'timestamp': 0
        }
    
    def setup_indicators(self):
        for pin in self.led_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def led_on(self, led_num):
        if 1 <= led_num <= 3:
            GPIO.output(self.led_pins[led_num - 1], GPIO.HIGH)
    
    def led_off(self, led_num):
        if 1 <= led_num <= 3:
            GPIO.output(self.led_pins[led_num - 1], GPIO.LOW)
    
    def led_all_off(self):
        for pin in self.led_pins:
            GPIO.output(pin, GPIO.LOW)
    
    def beep(self, duration=0.1):
        GPIO.output(self.buzzer_pin, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def alert_pattern(self):
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
            self.sensor_data['motor_speed'] = self.current_speed
            self.sensor_data['auto_mode'] = self.auto_mode
            self.sensor_data['timestamp'] = time.time()
            
            if self.auto_mode:
                action = self.detector.analyze_obstacles()
                self.sensor_data['obstacle_action'] = action
            
            # Broadcast via BLE
            self.ble.broadcast_data(self.sensor_data)
            
            time.sleep(0.2)  # 5Hz broadcast
    
    def process_ble_commands(self):
        """Process incoming BLE commands"""
        command = self.ble.get_command()
        
        if not command:
            return
        
        print(f"\n📱 BLE Command: {command}")
        
        if command.get('type') == 'mode':
            mode = command.get('mode', 'auto')
            if mode == 'auto':
                self.auto_mode = True
                print("🤖 AUTO mode")
                self.beep(0.2)
                self.led_on(3)
                self.led_off(2)
            else:
                self.auto_mode = False
                print("🎮 MANUAL mode")
                self.beep(0.1)
                self.led_off(3)
                self.led_on(2)
                
        elif command.get('type') == 'speed':
            global ROVER_SPEED
            new_speed = command.get('speed', ROVER_SPEED)
            ROVER_SPEED = max(0, min(100, new_speed))
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            print(f"⚡ Speed: {ROVER_SPEED}%")
            
        elif command.get('type') == 'control' and not self.auto_mode:
            action = command.get('action', 'stop')
            
            if action == 'forward':
                self.motors.forward(self.current_speed)
                print("  ⬆️ Forward")
            elif action == 'backward':
                self.motors.backward()
                print("  ⬇️ Backward")
            elif action == 'left':
                self.motors.turn_left()
                print("  ⬅️ Left")
            elif action == 'right':
                self.motors.turn_right()
                print("  ➡️ Right")
            elif action == 'stop':
                self.motors.stop()
                print("  🛑 Stop")
                
        elif command.get('type') == 'command':
            cmd = command.get('command', '').lower()
            if cmd == 'stop':
                self.motors.stop()
            elif cmd == 'beep':
                self.beep(0.3)
    
    def initialize(self):
        """Initialize all systems"""
        print("\n" + "="*60)
        print("   INTELLIGENT NAVIGATION SYSTEM v2.0")
        print("   BLE Broadcasting - No Pairing Required")
        print("="*60)
        print(f"\n⚙️ Speed: {ROVER_SPEED}%")
        print("🤖 Default: AUTO NAVIGATION")
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.setup_indicators()
        self.led_on(1)
        
        # Install and setup BLE
        self.ble.install_bluez_tools()
        
        if not self.ble.setup_bluetooth_interface():
            print("⚠️ Running without BLE")
        else:
            if self.ble.start_ble_advertising():
                print("✅ BLE advertising active")
            if self.ble.start():
                print("✅ BLE command receiver active")
        
        # Initialize hardware
        self.motors.setup()
        self.detector.setup()
        self.compass.initialize()
        
        # Startup beep
        self.beep(0.3)
        time.sleep(0.1)
        self.beep(0.3)
        
        print("\n✅ System Ready!")
        print(f"📡 BLE Device: {DEVICE_NAME}")
        print("   Scan with any BLE scanner to see data")
        print("   No pairing required!\n")
        
        return True
    
    def manual_control_loop(self):
        while self.running and not self.auto_mode:
            self.process_ble_commands()
            time.sleep(0.05)
    
    def auto_navigation_loop(self):
        while self.running and self.auto_mode:
            self.process_ble_commands()
            
            action = self.detector.analyze_obstacles()
            front_distance = self.detector.get_ultrasonic_distance()
            
            # Update sensor data
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
            
            self.execute_action(action, front_distance)
            time.sleep(0.05)
    
    def execute_action(self, action, front_distance):
        if action == 'FORWARD':
            recommended_speed = self.detector.get_recommended_speed(front_distance)
            if recommended_speed != self.current_speed:
                self.current_speed = recommended_speed
                self.motors.set_speed(self.current_speed)
            self.motors.forward(self.current_speed)
            self.led_off(2)
            self.led_off(3)
            
        elif action == 'TURN_LEFT':
            print("  ↪️ Turning left")
            self.led_on(2)
            self.led_off(3)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.turn_left()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            self.alert_pattern()
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            
        elif action == 'TURN_RIGHT':
            print("  ↩️ Turning right")
            self.led_off(2)
            self.led_on(3)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.turn_right()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            self.alert_pattern()
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            
        elif action == 'TURN_AROUND':
            print("  🔄 Turning around")
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
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            
        elif action == 'STOP_AND_BACK':
            print("  ⚠️ CRITICAL! Backing up...")
            for _ in range(3):
                self.beep(0.2)
                time.sleep(0.1)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.backward()
            time.sleep(1.0)
            self.motors.stop()
            time.sleep(0.1)
            import random
            if random.choice([True, False]):
                self.motors.turn_left()
            else:
                self.motors.turn_right()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
    
    def start(self):
        """Start navigation system"""
        # Start broadcast thread
        self.data_publish_thread = threading.Thread(target=self.broadcast_sensor_data)
        self.data_publish_thread.daemon = True
        self.data_publish_thread.start()
        
        print("🎯 System Active!")
        print("🤖 AUTO NAVIGATION mode")
        print("📱 Send commands via BLE (no pairing)")
        print("   - Mode: {'type':'mode','mode':'auto/manual'}")
        print("   - Speed: {'type':'speed','speed':0-100}")
        print("   - Control: {'type':'control','action':'forward/backward/left/right/stop'}")
        print("\n📡 Sensor data broadcasted via BLE")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.running:
                if self.auto_mode:
                    self.auto_navigation_loop()
                else:
                    self.manual_control_loop()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping...")
            self.stop()
    
    def stop(self):
        self.running = False
        self.motors.stop()
        self.led_all_off()
        self.motors.cleanup()
        self.ble.stop()
        GPIO.cleanup()
        print("✅ System stopped")

def main():
    nav_system = NavigationSystem()
    
    try:
        if nav_system.initialize():
            nav_system.running = True
            nav_system.start()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        nav_system.stop()

if __name__ == "__main__":
    main()