#!/usr/bin/env python3
"""
Intelligent Navigation System for Raspberry Pi with Bluetooth Control
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
from collections import deque
import serial
import serial.tools.list_ports

# ========== BLUETOOTH CONFIGURATION ==========
BLUETOOTH_BAUDRATE = 9600
# Common Bluetooth serial ports on Raspberry Pi
BLUETOOTH_PORTS = ['/dev/rfcomm0', '/dev/ttyAMA0', '/dev/ttyS0']
DEVICE_NAME = 'PiRover'

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

class BluetoothController:
    """Handles Bluetooth setup, pairing, and serial communication."""
    
    def __init__(self):
        self.serial_connection = None
        self.connected = False
        self.connection_thread = None
        self.running = False
        self.last_command = None
        self.command_lock = threading.Lock()
        self.agent_process = None

    def run_command(self, cmd, shell=False, timeout=15):
        """Run a system command and return stdout on success."""
        try:
            result = subprocess.run(
                cmd,
                shell=shell,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
            if result.returncode != 0:
                command_text = cmd if isinstance(cmd, str) else ' '.join(cmd)
                stderr = result.stderr.strip() if result.stderr else ''
                print(f"⚠️  Command failed: {command_text}")
                if stderr:
                    print(f"   {stderr}")
            return result.stdout.strip()
        except Exception as e:
            print(f"❌ Command error: {e}")
            return None

    def configure_bluetooth_daemon(self):
        """Make Bluetooth discoverable and pairable by default."""
        try:
            config_path = '/etc/bluetooth/main.conf'
            if not os.path.exists(config_path):
                print(f"⚠️  Bluetooth config not found: {config_path}")
                return False

            with open(config_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()

            def upsert(key, value, current_lines):
                pattern = re.compile(rf'^\s*#?\s*{re.escape(key)}\s*=.*$', re.IGNORECASE)
                replaced = False
                updated = []
                for line in current_lines:
                    if pattern.match(line):
                        updated.append(f'{key} = {value}\n')
                        replaced = True
                    else:
                        updated.append(line)
                if not replaced:
                    if updated and not updated[-1].endswith('\n'):
                        updated[-1] = updated[-1] + '\n'
                    updated.append(f'{key} = {value}\n')
                return updated

            lines = upsert('DiscoverableTimeout', 0, lines)
            lines = upsert('PairableTimeout', 0, lines)

            with open(config_path, 'w', encoding='utf-8') as f:
                f.writelines(lines)

            print('✅ Bluetooth daemon configured for persistent discoverability/pairability')
            return True
        except Exception as e:
            print(f"⚠️  Could not update Bluetooth daemon config: {e}")
            return False

    def start_bluetooth_agent(self):
        """Start a persistent bluetoothctl agent with NoInputNoOutput pairing."""
        try:
            if self.agent_process and self.agent_process.poll() is None:
                return True

            self.agent_process = subprocess.Popen(
                ['bluetoothctl'],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True,
            )

            commands = [
                'agent NoInputNoOutput',
                'default-agent',
                'power on',
                'pairable on',
                'pairable-timeout 0',
                'discoverable on',
                'discoverable-timeout 0',
                f'name {DEVICE_NAME}',
            ]
            for command in commands:
                self.agent_process.stdin.write(command + '\n')
            self.agent_process.stdin.flush()

            print('✅ Bluetooth agent started (NoInputNoOutput)')
            return True
        except Exception as e:
            print(f"⚠️  Failed to start bluetooth agent: {e}")
            return False

    def add_serial_profile(self):
        """Register the Serial Port Profile (SPP)."""
        result = self.run_command(['sdptool', 'add', 'SP'])
        if result is not None:
            print('✅ Serial Port Profile registered')
            return True
        return False

    def release_rfcomm(self):
        """Release any existing RFCOMM bindings."""
        self.run_command(['rfcomm', 'release', '/dev/rfcomm0'])
        self.run_command(['rfcomm', 'release', '/dev/rfcomm1'])

    def wait_for_connection(self, timeout=60):
        """Wait for a device to connect and then bind RFCOMM."""
        print(f"\n📱 Waiting for device connection ({timeout} seconds max)...")
        print("   Pair from your phone/PC — no PIN should be required")

        start_time = time.time()
        while time.time() - start_time < timeout:
            connected = self.run_command(['bluetoothctl', 'devices', 'Connected'])
            if connected:
                for line in connected.splitlines():
                    parts = line.split()
                    if len(parts) >= 2 and parts[0] == 'Device':
                        mac = parts[1]
                        print(f"✅ Device connected: {mac}")
                        self.run_command(['rfcomm', 'bind', '/dev/rfcomm0', mac, '1'], timeout=20)
                        time.sleep(2)
                        if os.path.exists('/dev/rfcomm0'):
                            print('✅ RFCOMM bound to /dev/rfcomm0')
                            return True

            elapsed = int(time.time() - start_time)
            if elapsed and elapsed % 10 == 0:
                print(f"   Still waiting... ({elapsed}s)")
            time.sleep(2)

        print('⚠️  No device connected within timeout')
        return False

    def find_and_connect_serial(self):
        """Open the Bluetooth serial port once RFCOMM is bound."""
        ports_to_try = ['/dev/rfcomm0'] + BLUETOOTH_PORTS

        for port in ports_to_try:
            if os.path.exists(port):
                try:
                    self.serial_connection = serial.Serial(
                        port,
                        BLUETOOTH_BAUDRATE,
                        timeout=1,
                        write_timeout=1,
                    )
                    print(f"✅ Serial connection opened on {port}")
                    self.connected = True
                    return True
                except Exception as e:
                    print(f"⚠️  Failed to open {port}: {e}")

        print('❌ Could not open any Bluetooth serial port')
        return False

    def setup_bluetooth(self):
        """Complete Bluetooth setup before opening the serial port."""
        print('\n🔵 Setting up Bluetooth...')
        self.configure_bluetooth_daemon()
        self.run_command(['systemctl', 'restart', 'bluetooth'], timeout=30)
        time.sleep(2)
        self.release_rfcomm()

        if not self.start_bluetooth_agent():
            return False

        time.sleep(2)
        self.add_serial_profile()
        print(f"✅ Bluetooth configured as '{DEVICE_NAME}'")
        print('   → Discoverable')
        print('   → Pairable without PIN')
        return True
        
    def find_bluetooth_port(self):
        """Find available Bluetooth serial port"""
        # First check common Bluetooth ports
        for port in BLUETOOTH_PORTS:
            try:
                test_serial = serial.Serial(port, BLUETOOTH_BAUDRATE, timeout=0.5)
                test_serial.close()
                return port
            except:
                continue
        
        # If not found, scan all serial ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'bluetooth' in port.description.lower() or 'rfcomm' in port.device:
                return port.device
        
        return None
    
    def connect(self):
        """Establish Bluetooth setup and connection."""
        print("\n🔵 Initializing Bluetooth...")

        if not self.setup_bluetooth():
            print("⚠️  Bluetooth setup failed - continuing in standalone mode")
            return False

        if not self.wait_for_connection(timeout=90):
            print("⚠️  Bluetooth connected device not detected yet")
            return False

        if self.find_and_connect_serial():
            try:
                self.serial_connection.write(b"ROVER_READY\n")
            except Exception:
                pass
            print('📱 Bluetooth serial link ready')
            return True

        return False
    
    def send_data(self, data_dict):
        """Send sensor data to PC"""
        if not self.connected or not self.serial_connection:
            return False
        
        try:
            json_data = json.dumps(data_dict) + "\n"
            self.serial_connection.write(json_data.encode('utf-8'))
            return True
        except Exception as e:
            print(f"⚠️ Bluetooth send error: {e}")
            self.connected = False
            return False
    
    def receive_command(self):
        """Check for and receive commands from PC"""
        if not self.connected or not self.serial_connection:
            return None
        
        try:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().decode('utf-8').strip()
                if data:
                    try:
                        command = json.loads(data)
                        with self.command_lock:
                            self.last_command = command
                        return command
                    except json.JSONDecodeError:
                        # Handle plain text commands
                        return {'type': 'raw', 'command': data}
        except Exception as e:
            print(f"⚠️ Bluetooth receive error: {e}")
            self.connected = False
        
        return None
    
    def get_last_command(self):
        """Get and clear last command"""
        with self.command_lock:
            command = self.last_command
            self.last_command = None
            return command
    
    def close(self):
        """Close Bluetooth connection"""
        self.running = False
        if self.serial_connection:
            self.serial_connection.close()
        if self.agent_process:
            try:
                if self.agent_process.stdin:
                    self.agent_process.stdin.write('quit\n')
                    self.agent_process.stdin.flush()
                    self.agent_process.stdin.close()
            except Exception:
                pass
            try:
                self.agent_process.terminate()
            except Exception:
                pass
            self.agent_process = None
        self.release_rfcomm()
        self.connected = False
        print("🔵 Bluetooth disconnected")

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
        self.bluetooth = BluetoothController()
        self.running = False
        self.navigation_thread = None
        self.data_publish_thread = None
        
        # Control modes
        self.auto_mode = False  # Default to manual control
        self.manual_command = None
        
        # LED pins
        self.led_pins = [PINS['LED1'], PINS['LED2'], PINS['LED3']]
        
        # Buzzer pin
        self.buzzer_pin = PINS['BUZZER']
        
        # Speed control
        self.current_speed = ROVER_SPEED
        
        # Sensor data for publishing
        self.sensor_data = {
            'front_distance': 999,
            'ir_sensors': {},
            'compass_angle': 0,
            'motor_speed': 0,
            'auto_mode': False,
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
    
    def publish_sensor_data(self):
        """Continuously publish sensor data via Bluetooth"""
        while self.running:
            if self.bluetooth.connected:
                # Update sensor data
                self.sensor_data['front_distance'] = self.detector.last_distance
                self.sensor_data['ir_sensors'] = self.detector.get_ir_readings()
                self.sensor_data['compass_angle'] = self.compass.get_angle()
                self.sensor_data['motor_speed'] = self.current_speed
                self.sensor_data['auto_mode'] = self.auto_mode
                self.sensor_data['timestamp'] = time.time()
                
                # Send data
                self.bluetooth.send_data(self.sensor_data)
            
            time.sleep(0.2)  # Publish at 5Hz
    
    def process_commands(self):
        """Process incoming Bluetooth commands"""
        global ROVER_SPEED
        self.bluetooth.receive_command()
        command = self.bluetooth.get_last_command()
        
        if not command:
            return
        
        print(f"\n📱 Received command: {command}")
        
        # Handle different command types
        if command.get('type') == 'mode':
            # Change control mode
            mode = command.get('mode', 'manual')
            if mode == 'auto':
                self.auto_mode = True
                print("🤖 Switched to AUTO navigation mode")
                self.beep(0.2)
                self.led_on(3)
            else:
                self.auto_mode = False
                print("🎮 Switched to MANUAL control mode")
                self.beep(0.1)
                self.led_off(3)
                
        elif command.get('type') == 'speed':
            # Change rover speed
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
        print(f"\n🎮 Default mode: MANUAL CONTROL")
        
        # Setup GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup indicators
        self.setup_indicators()
        
        # Turn on LED 1 - Waiting for Bluetooth
        self.led_on(1)
        print("🔆 LED 1 ON - Waiting for Bluetooth connection")
        
        # Initialize Bluetooth
        if not self.bluetooth.connect():
            print("⚠️  Bluetooth not available - running in standalone mode")
        else:
            # Turn on LED 2 when Bluetooth connected
            self.led_on(2)
            print("🔆 LED 2 ON - Bluetooth connected")
        
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
        print(f"🎮 Control mode: {'AUTO' if self.auto_mode else 'MANUAL'}")
        
        return True
    
    def manual_control_loop(self):
        """Manual control loop - processes Bluetooth commands only"""
        while self.running and not self.auto_mode:
            self.process_commands()
            time.sleep(0.05)
    
    def auto_navigation_loop(self):
        """Automatic navigation loop with obstacle avoidance"""
        while self.running and self.auto_mode:
            # Check for mode change command
            self.process_commands()
            
            # Analyze obstacles
            action = self.detector.analyze_obstacles()
            
            # Get front distance for display
            front_distance = self.detector.get_ultrasonic_distance()
            
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
        # Start data publishing thread
        self.data_publish_thread = threading.Thread(target=self.publish_sensor_data)
        self.data_publish_thread.daemon = True
        self.data_publish_thread.start()
        
        print("\n🎯 System Active!")
        print("🎮 Default mode: MANUAL CONTROL")
        print("📱 Send 'mode' command with 'auto' or 'manual' to switch")
        print("   - Manual: forward, backward, left, right, stop")
        print("   - Auto: Autonomous obstacle avoidance")
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
        self.bluetooth.close()
        GPIO.cleanup()
        
        print("✅ Navigation system stopped")
        print("👋 Goodbye!")

def main():
    """Main entry point"""
    print("="*60)
    print("   INTELLIGENT NAVIGATION SYSTEM")
    print("   - Bluetooth Control (2-way)")
    print("   - 4x IR Sensors at 45° angles")
    print("   - Ultrasonic Sensor (forward)")
    print("   - Manual & Auto Navigation Modes")
    print("="*60)
    print(f"\n⚙️  Current global speed setting: {ROVER_SPEED}%")
    print("   To change speed, edit ROVER_SPEED variable or send speed command")
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