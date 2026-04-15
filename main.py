#!/usr/bin/env python3
"""
PiRover with BLE Broadcasting including DHT11 and MQ135
Run with: sudo python3 main.py
"""

import RPi.GPIO as GPIO
import time
import subprocess
import struct
from collections import deque
import threading

# Try to import DHT11 library
try:
    import Adafruit_DHT
    DHT_AVAILABLE = True
except ImportError:
    DHT_AVAILABLE = False
    print("⚠️ Adafruit_DHT not installed. DHT11 disabled.")

# ========== PIN CONFIGURATION ==========
PINS = {
    'L298N_IN1': 17, 'L298N_IN2': 18, 'L298N_IN3': 22, 'L298N_IN4': 23,
    'L298N_ENA': 12, 'L298N_ENB': 13,
    'ULTRASONIC_TRIG': 5, 'ULTRASONIC_ECHO': 6,
    'IR_TOP_LEFT': 26, 'IR_TOP_RIGHT': 20,
    'IR_BOTTOM_LEFT': 21, 'IR_BOTTOM_RIGHT': 16,
    'LED1': 24, 'LED2': 25, 'LED3': 8, 'BUZZER': 27,
    'DHT11': 4,      # Add DHT11 pin
    'MQ135': 7       # Add MQ135 pin
}

# ========== CONFIGURATION ==========
DEVICE_NAME = 'PiRover'
ROVER_SPEED = 40
OBSTACLE_THRESHOLD = 30
CRITICAL_DISTANCE = 15
TURN_DURATION = 0.8
SENSOR_UPDATE_INTERVAL = 2  # Update DHT/MQ135 every 2 seconds

class BLEBeacon:
    """Simple BLE beacon using hcitool - Fixed version"""
    
    def __init__(self):
        self.running = False
        self.last_data = ""
        
    def setup(self):
        """Initialize Bluetooth"""
        try:
            # Bring up Bluetooth
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'], capture_output=True)
            
            # Set device name
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'name', DEVICE_NAME], capture_output=True)
            
            # Stop any existing advertising
            subprocess.run(['sudo', 'hcitool', 'cmd', '0x08', '0x000a', '00'], capture_output=True)
            
            # Extra settings for better compatibility
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'lestate'], capture_output=True)
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'txpower', '6'], capture_output=True)

            subprocess.run([
                'sudo', 'hcitool', 'cmd', '0x08', '0x0006',
                'a0', '00',   # interval min (100ms)
                'a0', '00',   # interval max
                '00',         # connectable undirected advertising
                '00',         # own address type
                '00',         # direct addr type
                '00','00','00','00','00','00',  # direct addr
                '07',         # channel map
                '00'          # filter policy
            ], capture_output=True)

            print(f"✅ BLE beacon ready on hci0 as '{DEVICE_NAME}'")
            return True
        except Exception as e:
            print(f"⚠️ BLE setup error: {e}")
            return False
    
    def start(self):
        """Start the beacon"""
        self.running = True
        return self.setup()
    
    def stop(self):
        """Stop advertising"""
        self.running = False
        try:
            subprocess.run(['sudo', 'hcitool', 'cmd', '0x08', '0x000a', '00'], capture_output=True)
        except:
            pass

    def broadcast(self, distance, speed, auto_mode, ir_list, temperature, humidity, gas_detected):
        """Broadcast sensor data as manufacturer-specific data"""
        if not self.running:
            return

        # Enhanced data format: D{distance}S{speed}M{mode}I{ir_bits}T{temp}H{humidity}G{gas}
        ir_bits = ''.join([str(x) for x in ir_list])
        data_str = f"D{distance}S{speed}M{1 if auto_mode else 0}I{ir_bits}T{temperature:.1f}H{humidity:.1f}G{1 if gas_detected else 0}"
        
        if data_str == self.last_data:
            return
        self.last_data = data_str

        # Convert string to bytes
        data_bytes = data_str.encode('utf-8')
        
        # Create manufacturer specific data
        company_id = 0xFFFF  # Use this for testing
        
        # Build the advertising packet
        adv_data = bytearray()
        
        # Flags (required)
        adv_data.extend([0x02, 0x01, 0x06])  # LE General Discoverable, BR/EDR Not Supported
        
        # Local Name (optional but helpful)
        name_bytes = DEVICE_NAME.encode('utf-8')
        adv_data.append(len(name_bytes) + 1)
        adv_data.append(0x09)  # Complete Local Name
        adv_data.extend(name_bytes)
        
        # Manufacturer Specific Data with custom ID
        manuf_len = len(data_bytes) + 2
        adv_data.append(manuf_len + 1)  # +1 for the type byte
        adv_data.append(0xFF)  # Manufacturer Specific Data type
        adv_data.extend([company_id & 0xFF, (company_id >> 8) & 0xFF])  # Company ID (little endian)
        adv_data.extend(data_bytes)
        
        # Pad to 31 bytes as required
        while len(adv_data) < 31:
            adv_data.append(0x00)
        
        # Send the advertising packet
        try:
            # Stop current advertising
            subprocess.run(['sudo', 'hcitool', 'cmd', '0x08', '0x000a', '00'], 
                        capture_output=True)
            
            # Set advertising data
            real_length = len(adv_data)
            cmd = ['sudo', 'hcitool', 'cmd', '0x08', '0x0008', f'{real_length:02x}']
            for b in adv_data:
                cmd.append(f'{b:02x}')
            subprocess.run(' '.join(cmd), shell=True, capture_output=True)
            
            # Start advertising
            subprocess.run(['sudo', 'hcitool', 'cmd', '0x08', '0x000a', '01'], 
                        capture_output=True)
            
        except Exception as e:
            print(f"BLE send error: {e}")

class EnvironmentSensor:
    """Handle DHT11 and MQ135 sensors"""
    
    def __init__(self):
        self.temperature = 0.0
        self.humidity = 0.0
        self.gas_detected = False
        self.last_update = 0
        
    def setup(self):
        """Setup environment sensors"""
        if DHT_AVAILABLE:
            print("✅ DHT11 sensor ready")
        else:
            print("⚠️ DHT11 unavailable - install Adafruit_DHT")
        
        GPIO.setup(PINS['MQ135'], GPIO.IN)
        print("✅ MQ135 gas sensor ready")
    
    def read_dht11(self):
        """Read DHT11 sensor"""
        if not DHT_AVAILABLE:
            return None, None
        
        try:
            humidity, temperature = Adafruit_DHT.read_retry(
                Adafruit_DHT.DHT11, 
                PINS['DHT11']
            )
            
            if humidity is not None and temperature is not None:
                return temperature, humidity
            else:
                return None, None
        except Exception as e:
            print(f"DHT11 read error: {e}")
            return None, None
    
    def read_mq135(self):
        """Read MQ135 gas sensor (digital output)"""
        try:
            # Digital output: 0 = gas detected, 1 = normal
            gas_status = GPIO.input(PINS['MQ135'])
            return gas_status == 0  # True if gas detected
        except Exception as e:
            print(f"MQ135 read error: {e}")
            return False
    
    def update(self):
        """Update all environment sensors"""
        current_time = time.time()
        
        # Only update every SENSOR_UPDATE_INTERVAL seconds
        if current_time - self.last_update >= SENSOR_UPDATE_INTERVAL:
            # Read DHT11
            temp, hum = self.read_dht11()
            if temp is not None and hum is not None:
                self.temperature = temp
                self.humidity = hum
            
            # Read MQ135
            self.gas_detected = self.read_mq135()
            
            self.last_update = current_time
            
            # Print status occasionally
            if self.gas_detected:
                print(f"\n⚠️ GAS DETECTED! Temp: {self.temperature:.1f}°C Hum: {self.humidity:.1f}%")
        
        return self.temperature, self.humidity, self.gas_detected

class MotorController:
    def __init__(self):
        self.current_speed = ROVER_SPEED
        self.pwm_a = None
        self.pwm_b = None
        self.running = False
    
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
        self.current_speed = max(0, min(100, speed))
        if self.running:
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
    
    def forward(self):
        self.pwm_a.ChangeDutyCycle(self.current_speed)
        self.pwm_b.ChangeDutyCycle(self.current_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def backward(self):
        self.pwm_a.ChangeDutyCycle(35)
        self.pwm_b.ChangeDutyCycle(35)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def turn_left(self):
        self.pwm_a.ChangeDutyCycle(50)
        self.pwm_b.ChangeDutyCycle(50)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def turn_right(self):
        self.pwm_a.ChangeDutyCycle(50)
        self.pwm_b.ChangeDutyCycle(50)
        GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def stop(self):
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
    
    def setup(self):
        GPIO.setup(PINS['ULTRASONIC_TRIG'], GPIO.OUT)
        GPIO.setup(PINS['ULTRASONIC_ECHO'], GPIO.IN)
        GPIO.output(PINS['ULTRASONIC_TRIG'], GPIO.LOW)
        
        ir_pins = [PINS['IR_TOP_LEFT'], PINS['IR_TOP_RIGHT'], 
                   PINS['IR_BOTTOM_LEFT'], PINS['IR_BOTTOM_RIGHT']]
        for pin in ir_pins:
            GPIO.setup(pin, GPIO.IN)
        print("✅ Obstacle sensors ready")
    
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
            self.distance = dist if 2 < dist < 400 else 999
            return self.distance
        except:
            return 999
    
    def get_ir(self):
        return [
            1 if GPIO.input(PINS['IR_TOP_LEFT']) == 0 else 0,
            1 if GPIO.input(PINS['IR_TOP_RIGHT']) == 0 else 0,
            1 if GPIO.input(PINS['IR_BOTTOM_LEFT']) == 0 else 0,
            1 if GPIO.input(PINS['IR_BOTTOM_RIGHT']) == 0 else 0
        ]
    
    def analyze(self):
        dist = self.get_distance()
        ir = self.get_ir()
        
        if dist < CRITICAL_DISTANCE:
            return 'BACK'
        if dist < OBSTACLE_THRESHOLD:
            if ir[0] == 0:
                return 'LEFT'
            if ir[1] == 0:
                return 'RIGHT'
            return 'TURN'
        return 'FWD'

class NavigationSystem:
    def __init__(self):
        self.motors = MotorController()
        self.detector = ObstacleDetector()
        self.environment = EnvironmentSensor()
        self.ble = BLEBeacon()
        self.running = False
        self.auto_mode = True
    
    def setup_indicators(self):
        for pin in [PINS['LED1'], PINS['LED2'], PINS['LED3']]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        GPIO.setup(PINS['BUZZER'], GPIO.OUT)
    
    def beep(self, duration=0.1):
        GPIO.output(PINS['BUZZER'], GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(PINS['BUZZER'], GPIO.LOW)
    
    def initialize(self):
        print("\n" + "="*50)
        print("   PiRover Navigation System")
        print("   BLE Beacon Broadcasting")
        print("   with DHT11 & MQ135")
        print("="*50)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.setup_indicators()
        
        # Initialize BLE beacon
        if not self.ble.start():
            print("⚠️ BLE not available - running without beacon")
        
        # Initialize hardware
        self.motors.setup()
        self.detector.setup()
        self.environment.setup()
        
        # Ready signal
        self.beep(0.2)
        time.sleep(0.1)
        self.beep(0.2)
        
        print(f"\n✅ System Ready! (Speed: {ROVER_SPEED}%)")
        print(f"🤖 Mode: AUTO (default)")
        print(f"📡 BLE Beacon: {DEVICE_NAME}")
        print("   Broadcasting sensor data including:")
        print("   - Distance & IR sensors")
        print("   - Temperature & Humidity (DHT11)")
        print("   - Gas detection (MQ135)")
        print("\nPress Ctrl+C to stop\n")
        
        return True
    
    def broadcast_sensor_data(self):
        """Broadcast sensor data via BLE beacon"""
        if not self.ble.running:
            return
        
        distance = int(self.detector.distance) if self.detector.distance < 999 else 999
        ir = self.detector.get_ir()
        speed = self.motors.current_speed
        
        # Get environment data
        temp, hum, gas = self.environment.update()
        
        self.ble.broadcast(distance, speed, self.auto_mode, ir, temp, hum, gas)
    
    def run(self):
        self.running = True
        last_broadcast = time.time()
        last_env_print = time.time()
        
        try:
            while self.running:
                if self.auto_mode:
                    dist = self.detector.get_distance()
                    action = self.detector.analyze()
                    
                    ir = self.detector.get_ir()
                    ir_str = ''.join(['X' if x else '.' for x in ir])
                    
                    # Get environment data for display
                    temp, hum, gas = self.environment.update()
                    
                    # Display status with environment data
                    gas_marker = "⚠️GAS! " if gas else ""
                    print(f"\r{gas_marker}📡 Dist:{int(dist):3d}cm IR:[{ir_str}] {action:5} Speed:{self.motors.current_speed}% T:{temp:.1f}°C H:{hum:.1f}%", end='')
                    
                    # Gas alert
                    if gas and time.time() - last_env_print > 5:
                        self.beep(0.3)
                        last_env_print = time.time()
                    
                    if action == 'FWD':
                        self.motors.forward()
                    elif action == 'LEFT':
                        self.motors.stop()
                        time.sleep(0.1)
                        self.motors.turn_left()
                        time.sleep(TURN_DURATION)
                        self.motors.stop()
                        self.beep(0.05)
                    elif action == 'RIGHT':
                        self.motors.stop()
                        time.sleep(0.1)
                        self.motors.turn_right()
                        time.sleep(TURN_DURATION)
                        self.motors.stop()
                        self.beep(0.05)
                    elif action == 'BACK':
                        self.motors.backward()
                        time.sleep(0.8)
                        self.motors.stop()
                        self.beep(0.2)
                    elif action == 'TURN':
                        self.motors.stop()
                        time.sleep(0.1)
                        self.motors.turn_left()
                        time.sleep(TURN_DURATION * 1.5)
                        self.motors.stop()
                
                # Broadcast every 0.2 seconds
                if time.time() - last_broadcast >= 0.2:
                    self.broadcast_sensor_data()
                    last_broadcast = time.time()
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping...")
            self.stop()
    
    def stop(self):
        self.running = False
        self.motors.stop()
        self.ble.stop()
        GPIO.cleanup()
        print("✅ System stopped")

def main():
    nav = NavigationSystem()
    if nav.initialize():
        nav.run()

if __name__ == "__main__":
    main()