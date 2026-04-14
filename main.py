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
from collections import deque
import bluetooth
import struct

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

# ========== BLUETOOTH CONFIGURATION ==========
BT_SERVICE_NAME = "RoverNavSystem"
BT_UUID = "00001101-0000-1000-8000-00805F9B34FB"  # Standard Serial Port Profile UUID

# ========== GLOBAL SPEED CONFIGURATION ==========
# ⚙️ ADJUST THIS VALUE TO CHANGE ROVER SPEED (0-100)
# 0 = stopped, 30 = slow, 60 = medium, 100 = maximum
global ROVER_SPEED  # Default: 40% speed
ROVER_SPEED = 40

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
        if self.pwm_a:
            self.pwm_a.ChangeDutyCycle(0)
        if self.pwm_b:
            self.pwm_b.ChangeDutyCycle(0)
    
    def forward(self, speed=None):
        """Move forward"""
        use_speed = speed if speed is not None else self.current_speed
        use_speed = max(0, min(100, use_speed))
        if self.pwm_a:
            self.pwm_a.ChangeDutyCycle(use_speed)
        if self.pwm_b:
            self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def backward(self, speed=None):
        """Move backward"""
        use_speed = speed if speed is not None else SPEED_PRESETS['BACKUP']
        use_speed = max(0, min(100, use_speed))
        if self.pwm_a:
            self.pwm_a.ChangeDutyCycle(use_speed)
        if self.pwm_b:
            self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def turn_left(self, speed=None):
        """Turn left (counter-rotate)"""
        use_speed = speed if speed is not None else SPEED_PRESETS['TURN']
        use_speed = max(0, min(100, use_speed))
        if self.pwm_a:
            self.pwm_a.ChangeDutyCycle(use_speed)
        if self.pwm_b:
            self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def turn_right(self, speed=None):
        """Turn right (counter-rotate)"""
        use_speed = speed if speed is not None else SPEED_PRESETS['TURN']
        use_speed = max(0, min(100, use_speed))
        if self.pwm_a:
            self.pwm_a.ChangeDutyCycle(use_speed)
        if self.pwm_b:
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
                return 999  # No obstacle
            
            timeout = time.time() + 0.1
            while GPIO.input(echo) == 1 and time.time() < timeout:
                pulse_end = time.time()
            
            if time.time() >= timeout:
                return 999
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            
            return distance if 2 < distance < 400 else 999
        except:
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

class BluetoothServer:
    """Bluetooth communication server"""
    def __init__(self):
        self.server_sock = None
        self.client_sock = None
        self.client_address = None
        self.connected = False
        self.running = False
        self.receive_thread = None
        
    def setup(self):
        """Setup Bluetooth server"""
        try:
            # Create server socket
            self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.server_sock.bind(("", bluetooth.PORT_ANY))
            self.server_sock.listen(1)
            
            port = self.server_sock.getsockname()[1]
            
            # Advertise service
            bluetooth.advertise_service(self.server_sock, BT_SERVICE_NAME,
                                       service_id=BT_UUID,
                                       service_classes=[BT_UUID, bluetooth.SERIAL_PORT_CLASS],
                                       profiles=[bluetooth.SERIAL_PORT_PROFILE])
            
            print(f"📡 Bluetooth server setup complete. Waiting for connection on port {port}...")
            print(f"   Service Name: {BT_SERVICE_NAME}")
            print("   Pair with your PC and connect using a serial terminal or custom app")
            return True
            
        except Exception as e:
            print(f"❌ Bluetooth setup failed: {e}")
            print("   Make sure Bluetooth is enabled: sudo hciconfig hci0 up")
            return False
    
    def accept_connection(self):
        """Accept incoming connection"""
        if not self.server_sock:
            return False
        
        print("📱 Waiting for PC to connect...")
        self.client_sock, self.client_address = self.server_sock.accept()
        self.connected = True
        print(f"✅ Connected to {self.client_address}")
        return True
    
    def send_data(self, data):
        """Send data to connected PC"""
        if self.connected and self.client_sock:
            try:
                json_data = json.dumps(data)
                self.client_sock.send((json_data + "\n").encode('utf-8'))
                return True
            except Exception as e:
                print(f"⚠️  Send error: {e}")
                self.connected = False
                return False
        return False
    
    def receive_data(self):
        """Receive data from PC"""
        if self.connected and self.client_sock:
            try:
                data = self.client_sock.recv(1024).decode('utf-8').strip()
                if data:
                    return json.loads(data)
            except Exception as e:
                print(f"⚠️  Receive error: {e}")
                self.connected = False
                return None
        return None
    
    def close(self):
        """Close connections"""
        self.connected = False
        if self.client_sock:
            self.client_sock.close()
        if self.server_sock:
            self.server_sock.close()
        print("🔌 Bluetooth connection closed")

class NavigationSystem:
    """Main navigation system"""
    def __init__(self):
        self.compass = Compass()
        self.motors = MotorController(ROVER_SPEED)
        self.detector = ObstacleDetector()
        self.bt_server = BluetoothServer()
        self.running = False
        self.navigation_thread = None
        self.publish_thread = None
        self.control_mode = "MANUAL"  # MANUAL or AUTO
        self.manual_command = None
        
        # LED pins
        self.led_pins = [PINS['LED1'], PINS['LED2'], PINS['LED3']]
        
        # Buzzer pin
        self.buzzer_pin = PINS['BUZZER']
        
        # Speed control
        self.current_speed = ROVER_SPEED
        
        # Sensor data
        self.sensor_data = {
            'front_distance': 999,
            'ir_readings': {},
            'compass_angle': 0,
            'motor_speed': 0,
            'control_mode': 'MANUAL',
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
    
    def update_sensor_data(self):
        """Update sensor readings"""
        self.sensor_data['front_distance'] = self.detector.get_ultrasonic_distance()
        self.sensor_data['ir_readings'] = self.detector.get_ir_readings()
        self.sensor_data['compass_angle'] = self.compass.get_angle()
        self.sensor_data['motor_speed'] = self.current_speed
        self.sensor_data['control_mode'] = self.control_mode
        self.sensor_data['timestamp'] = time.time()
    
    def publish_sensor_data(self):
        """Publish sensor data via Bluetooth"""
        self.update_sensor_data()
        if self.bt_server.connected:
            self.bt_server.send_data({
                'type': 'telemetry',
                'data': self.sensor_data
            })
    
    def process_command(self, command):
        """Process commands from PC"""
        cmd_type = command.get('type', '')
        
        if cmd_type == 'mode':
            # Change control mode
            new_mode = command.get('mode', 'MANUAL')
            if new_mode in ['MANUAL', 'AUTO']:
                self.control_mode = new_mode
                print(f"\n🎮 Control mode changed to: {self.control_mode}")
                if self.control_mode == 'MANUAL':
                    self.motors.stop()
                # LED3 indicates AUTO mode
                if self.control_mode == 'AUTO':
                    self.led_on(3)
                else:
                    self.led_off(3)
                return True
                
        elif cmd_type == 'speed':
            # Set global speed
            new_speed = command.get('speed', ROVER_SPEED)
            global ROVER_SPEED
            ROVER_SPEED = max(0, min(100, new_speed))
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            print(f"\n⚡ Speed set to: {ROVER_SPEED}%")
            return True
            
        elif cmd_type == 'manual_control' and self.control_mode == 'MANUAL':
            # Manual control commands
            action = command.get('action', 'STOP')
            duration = command.get('duration', 0.5)
            
            if action == 'FORWARD':
                print("  🏃 Manual: Forward")
                self.motors.forward(self.current_speed)
            elif action == 'BACKWARD':
                print("  🔄 Manual: Backward")
                self.motors.backward()
            elif action == 'TURN_LEFT':
                print("  ↪️  Manual: Turn Left")
                self.motors.turn_left()
                time.sleep(duration)
                self.motors.stop()
            elif action == 'TURN_RIGHT':
                print("  ↩️  Manual: Turn Right")
                self.motors.turn_right()
                time.sleep(duration)
                self.motors.stop()
            elif action == 'STOP':
                print("  🛑 Manual: Stop")
                self.motors.stop()
            return True
            
        elif cmd_type == 'get_status':
            # Send current status
            self.publish_sensor_data()
            return True
            
        return False
    
    def bluetooth_receive_loop(self):
        """Thread for receiving Bluetooth commands"""
        while self.running and self.bt_server.connected:
            command = self.bt_server.receive_data()
            if command:
                self.process_command(command)
            time.sleep(0.05)
    
    def bluetooth_publish_loop(self):
        """Thread for publishing sensor data"""
        while self.running and self.bt_server.connected:
            self.publish_sensor_data()
            time.sleep(0.5)  # Publish every 500ms
    
    def navigation_loop(self):
        """Main navigation loop (only active in AUTO mode)"""
        while self.running:
            if self.control_mode == 'AUTO':
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
                self.execute_auto_action(action, front_distance)
                
                # Small delay for stability
                time.sleep(0.05)
            else:
                # In MANUAL mode, just sleep
                time.sleep(0.1)
    
    def execute_auto_action(self, action, front_distance):
        """Execute navigation action based on obstacle analysis"""
        if action == 'FORWARD':
            # Dynamic speed based on obstacle distance
            recommended_speed = self.detector.get_recommended_speed(front_distance)
            if recommended_speed != self.current_speed:
                self.current_speed = recommended_speed
                self.motors.set_speed(self.current_speed)
            
            self.motors.forward(self.current_speed)
            self.led_off(2)
            
        elif action == 'TURN_LEFT':
            print("  ↪️  Turning left to avoid obstacle")
            self.led_on(2)
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
            self.led_on(2)
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
    
    def initialize(self):
        """Initialize all systems"""
        print("\n🚀 Initializing Navigation System...")
        print(f"⚙️  Global speed setting: {ROVER_SPEED}%")
        print(f"   - Cruise speed: {SPEED_PRESETS['CRUISE']}%")
        print(f"   - Slow speed: {SPEED_PRESETS['SLOW']}%")
        print(f"   - Turn speed: {SPEED_PRESETS['TURN']}%")
        print(f"   - Backup speed: {SPEED_PRESETS['BACKUP']}%")
        
        # Setup GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup indicators
        self.setup_indicators()
        
        # Turn on LED 1 - Waiting for Bluetooth connection
        self.led_on(1)
        print("🔆 LED 1 ON - Waiting for Bluetooth connection")
        
        # Initialize components
        self.motors.setup()
        self.detector.setup()
        
        # Initialize compass
        if not self.compass.initialize():
            print("⚠️  Compass not available - using dead reckoning")
        
        # Setup Bluetooth
        if not self.bt_server.setup():
            print("⚠️  Bluetooth not available - running in standalone mode")
        
        # Wait for Bluetooth connection
        print("\n📱 Waiting for PC to connect via Bluetooth...")
        print("   On your PC:")
        print("   1. Pair with the Raspberry Pi")
        print("   2. Connect using a Bluetooth terminal or custom application")
        print("   3. Default control mode is MANUAL")
        
        # Wait for connection
        connected = False
        wait_time = 0
        while not connected and self.bt_server.server_sock:
            connected = self.bt_server.accept_connection()
            if not connected:
                wait_time += 1
                if wait_time % 10 == 0:
                    print(f"   Still waiting... ({wait_time} seconds)")
                time.sleep(1)
        
        if connected:
            # Turn on LED 2 - Bluetooth connected
            self.led_on(2)
            print("🔆 LED 2 ON - Bluetooth connected")
            self.beep(0.3)
            time.sleep(0.1)
            self.beep(0.3)
        else:
            print("⚠️  No Bluetooth connection - running in standalone mode")
        
        # Start communication threads
        self.running = True
        if self.bt_server.connected:
            self.receive_thread = threading.Thread(target=self.bluetooth_receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            self.publish_thread = threading.Thread(target=self.bluetooth_publish_loop)
            self.publish_thread.daemon = True
            self.publish_thread.start()
        
        print("\n✅ Navigation System Ready!")
        print(f"🎮 Control Mode: {self.control_mode}")
        if self.control_mode == 'MANUAL':
            print("   Use PC to send manual commands or switch to AUTO mode")
        print("📡 Telemetry publishing active")
        
        return True
    
    def start(self):
        """Start navigation system"""
        self.navigation_thread = threading.Thread(target=self.navigation_loop)
        self.navigation_thread.daemon = True
        self.navigation_thread.start()
        
        print("\n🎯 System Active!")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping navigation...")
            self.stop()
    
    def stop(self):
        """Stop navigation system"""
        self.running = False
        if self.navigation_thread:
            self.navigation_thread.join(timeout=2)
        
        self.motors.stop()
        self.led_all_off()
        
        # Close Bluetooth
        self.bt_server.close()
        
        # Cleanup
        self.motors.cleanup()
        GPIO.cleanup()
        
        print("✅ Navigation system stopped")
        print("👋 Goodbye!")

def main():
    """Main entry point"""
    print("="*60)
    print("   INTELLIGENT NAVIGATION SYSTEM WITH BLUETOOTH")
    print("   - 4x IR Sensors at 45° angles")
    print("   - Ultrasonic Sensor (forward)")
    print("   - Compass reference set at boot")
    print("   - Bluetooth control & telemetry")
    print("="*60)
    print(f"\n⚙️  Current global speed setting: {ROVER_SPEED}%")
    print("   To change default speed, edit ROVER_SPEED variable at top of file")
    print("   Range: 0 (stop) to 100 (maximum)")
    print()
    
    nav_system = NavigationSystem()
    
    try:
        if nav_system.initialize():
            nav_system.start()
    except Exception as e:
        print(f"\n❌ System error: {e}")
        nav_system.stop()

if __name__ == "__main__":
    main()