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
import socket  # ADD THIS IMPORT
from collections import deque

# ========== BLE CONFIGURATION ==========
DEVICE_NAME = 'PiRover'
BLE_ENABLED = True

# ========== PIN CONFIGURATION ==========
PINS = {
    'DHT11': 4,
    'MQ135': 7,
    'GY271_SDA': 2,
    'GY271_SCL': 3,
    'L298N_IN1': 17,
    'L298N_IN2': 18,
    'L298N_IN3': 22,
    'L298N_IN4': 23,
    'L298N_ENA': 12,
    'L298N_ENB': 13,
    'ULTRASONIC_TRIG': 5,
    'ULTRASONIC_ECHO': 6,
    'SERVO': 19,
    'IR_TOP_LEFT': 26,
    'IR_TOP_RIGHT': 20,
    'IR_BOTTOM_LEFT': 21,
    'IR_BOTTOM_RIGHT': 16,
    'LED1': 24,
    'LED2': 25,
    'LED3': 8,
    'BUZZER': 27
}

# ========== SPEED CONFIGURATION ==========
ROVER_SPEED = 40
SPEED_PRESETS = {
    'CRUISE': 60, 'SLOW': 40, 'TURN': 50,
    'BACKUP': 35, 'MIN': 20, 'MAX': 85
}

# ========== NAVIGATION CONSTANTS ==========
OBSTACLE_THRESHOLD = 30
SAFE_DISTANCE = 40
CRITICAL_DISTANCE = 15
TURN_DURATION = 0.8

class BLEManager:
    """Handles BLE advertising and command reception"""
    
    def __init__(self):
        self.running = False
        self.command_queue = deque(maxlen=10)
        self.bt_interface = None
        self.has_bluetooth = False
        self.server_socket = None
        self.rfcomm_thread = None
        
    def detect_bluetooth(self):
        """Detect if Bluetooth hardware is available"""
        print("\n🔍 Detecting Bluetooth hardware...")
        
        try:
            result = subprocess.run(['hciconfig'], capture_output=True, text=True)
            if result.stdout and 'hci' in result.stdout:
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'hci' in line:
                        self.bt_interface = line.split(':')[0].strip()
                        self.has_bluetooth = True
                        print(f"✅ Bluetooth adapter found: {self.bt_interface}")
                        return True
        except:
            pass
        
        print("❌ No Bluetooth adapter found")
        return False
    
    def setup_bluetooth(self):
        """Setup Bluetooth interface for advertising"""
        if not self.has_bluetooth:
            return False
        
        print("\n🔵 Configuring Bluetooth...")
        
        try:
            # Bring interface up
            subprocess.run(['sudo', 'hciconfig', self.bt_interface, 'up'], 
                          capture_output=True, text=True)
            
            # Set discoverable and pairable
            subprocess.run(['sudo', 'hciconfig', self.bt_interface, 'piscan'], 
                          capture_output=True, text=True)
            
            # Set device name
            subprocess.run(['sudo', 'hciconfig', self.bt_interface, 'name', DEVICE_NAME], 
                          capture_output=True, text=True)
            
            # Get MAC address
            result = subprocess.run(['hcitool', 'dev'], capture_output=True, text=True)
            mac = result.stdout.split('\n')[1].split()[1] if len(result.stdout.split('\n')) > 1 else "Unknown"
            
            print(f"✅ Bluetooth configured")
            print(f"   Interface: {self.bt_interface}")
            print(f"   MAC: {mac}")
            print(f"   Name: {DEVICE_NAME}")
            
            return True
            
        except Exception as e:
            print(f"⚠️ Bluetooth setup error: {e}")
            return False
    
    def start_advertising(self):
        """Start BLE advertising using hcitool"""
        if not self.has_bluetooth:
            return False
        
        try:
            # Create advertising packet
            name_bytes = DEVICE_NAME.encode('utf-8')
            
            # Build advertising data
            adv_data = bytearray()
            
            # Flags
            adv_data.append(0x02)
            adv_data.append(0x01)
            adv_data.append(0x06)
            
            # Device name
            adv_data.append(len(name_bytes) + 1)
            adv_data.append(0x09)
            adv_data.extend(name_bytes)
            
            # Pad to minimum length
            while len(adv_data) < 31:
                adv_data.append(0x00)
            
            # Send command to set advertising data
            cmd_parts = ['sudo', 'hcitool', '-i', self.bt_interface, 'cmd', '0x08', '0x0008']
            for byte in adv_data[:31]:
                cmd_parts.append(f'{byte:02x}')
            
            subprocess.run(' '.join(cmd_parts), shell=True, capture_output=True)
            
            # Enable advertising
            subprocess.run(f'sudo hcitool -i {self.bt_interface} cmd 0x08 0x000a 01', 
                          shell=True, capture_output=True)
            
            print("✅ BLE advertising started")
            return True
            
        except Exception as e:
            print(f"⚠️ Advertising error: {e}")
            return False
    
    def update_advertising_data(self, data_dict):
        """Update advertising data with sensor readings"""
        if not self.has_bluetooth:
            return
        
        try:
            # Convert to JSON and limit size
            json_str = json.dumps(data_dict)
            if len(json_str) > 22:
                json_str = json_str[:22]
            
            data_bytes = json_str.encode('utf-8')
            
            # Build advertising packet
            adv_data = bytearray()
            
            # Flags
            adv_data.append(0x02)
            adv_data.append(0x01)
            adv_data.append(0x06)
            
            # Device name
            name_bytes = DEVICE_NAME.encode('utf-8')[:8]
            adv_data.append(len(name_bytes) + 1)
            adv_data.append(0x09)
            adv_data.extend(name_bytes)
            
            # Manufacturer specific data
            if data_bytes:
                adv_data.append(len(data_bytes) + 2)
                adv_data.append(0xFF)
                adv_data.append(0xFF)
                adv_data.extend(data_bytes)
            
            # Pad to 31 bytes
            while len(adv_data) < 31:
                adv_data.append(0x00)
            
            # Update advertising data
            cmd_parts = ['sudo', 'hcitool', '-i', self.bt_interface, 'cmd', '0x08', '0x0008']
            for byte in adv_data[:31]:
                cmd_parts.append(f'{byte:02x}')
            
            subprocess.run(' '.join(cmd_parts), shell=True, capture_output=True)
            
        except Exception as e:
            pass
    
    def setup_command_server(self):
        """Setup RFCOMM server for receiving commands"""
        if not self.has_bluetooth:
            return False
        
        try:
            # Kill any existing RFCOMM
            subprocess.run(['sudo', 'killall', 'rfcomm'], stderr=subprocess.DEVNULL)
            
            # Create socket
            self.server_socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            
            # Get MAC address
            result = subprocess.run(['hcitool', 'dev'], capture_output=True, text=True)
            lines = result.stdout.strip().split('\n')
            if len(lines) < 2:
                return False
            
            mac = lines[1].split()[1]
            
            # Bind and listen
            self.server_socket.bind((mac, 1))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)  # Set timeout for accept
            
            # Start listening thread
            self.running = True
            self.rfcomm_thread = threading.Thread(target=self._rfcomm_listener)
            self.rfcomm_thread.daemon = True
            self.rfcomm_thread.start()
            
            print("✅ BLE command server ready on channel 1")
            return True
            
        except Exception as e:
            print(f"⚠️ Command server error: {e}")
            return False
    
    def _rfcomm_listener(self):
        """Listen for incoming commands"""
        while self.running:
            try:
                if self.server_socket:
                    self.server_socket.settimeout(1.0)
                    client, address = self.server_socket.accept()
                    
                    # Receive data
                    data = client.recv(1024)
                    if data:
                        try:
                            command = json.loads(data.decode('utf-8'))
                            self.command_queue.append(command)
                            print(f"\n📱 Command: {command}")
                        except:
                            pass
                    
                    client.close()
            except socket.timeout:
                # Timeout is expected - just continue
                continue
            except Exception as e:
                if self.running:
                    pass  # Silent fail
                break
    
    def get_command(self):
        """Get next command from queue"""
        if self.command_queue:
            return self.command_queue.popleft()
        return None
    
    def start(self):
        """Start BLE services"""
        if not self.detect_bluetooth():
            print("⚠️ BLE disabled - running without Bluetooth")
            return False
        
        if not self.setup_bluetooth():
            return False
        
        if not self.start_advertising():
            print("⚠️ Advertising failed")
        
        self.setup_command_server()
        return True
    
    def broadcast_sensor_data(self, sensor_data):
        """Broadcast sensor data via BLE"""
        if self.has_bluetooth:
            self.update_advertising_data(sensor_data)
    
    def stop(self):
        """Stop BLE services"""
        self.running = False
        
        # Close socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        # Wait for thread to finish
        if self.rfcomm_thread and self.rfcomm_thread.is_alive():
            time.sleep(0.5)
        
        # Release RFCOMM
        try:
            subprocess.run(['sudo', 'rfcomm', 'release', '/dev/rfcomm0'], 
                          stderr=subprocess.DEVNULL)
        except:
            pass
        
        print("🔵 BLE stopped")

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
            print(f"🧭 Compass ready")
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
    def __init__(self, base_speed=ROVER_SPEED):
        self.running = False
        self.current_speed = base_speed
        self.pwm_a = None
        self.pwm_b = None
    
    def set_speed(self, speed):
        self.current_speed = max(0, min(100, speed))
        if self.running and self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
    
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
        print(f"✅ Motors ready (speed: {self.current_speed}%)")
    
    def stop(self):
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)
    
    def forward(self, speed=None):
        s = speed if speed is not None else self.current_speed
        s = max(0, min(100, s))
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(s)
            self.pwm_b.ChangeDutyCycle(s)
        GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def backward(self, speed=None):
        s = speed if speed is not None else SPEED_PRESETS['BACKUP']
        s = max(0, min(100, s))
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(s)
            self.pwm_b.ChangeDutyCycle(s)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def turn_left(self, speed=None):
        s = speed if speed is not None else SPEED_PRESETS['TURN']
        s = max(0, min(100, s))
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(s)
            self.pwm_b.ChangeDutyCycle(s)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def turn_right(self, speed=None):
        s = speed if speed is not None else SPEED_PRESETS['TURN']
        s = max(0, min(100, s))
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(s)
            self.pwm_b.ChangeDutyCycle(s)
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
    def __init__(self):
        self.last_distance = 999
    
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
                return 999
            
            timeout = time.time() + 0.1
            while GPIO.input(echo) == 1 and time.time() < timeout:
                pulse_end = time.time()
            
            if time.time() >= timeout:
                return 999
            
            distance = (pulse_end - pulse_start) * 17150
            self.last_distance = distance if 2 < distance < 400 else 999
            return self.last_distance
        except:
            return 999
    
    def get_ir_readings(self):
        return {
            'TL': GPIO.input(PINS['IR_TOP_LEFT']) == 0,
            'TR': GPIO.input(PINS['IR_TOP_RIGHT']) == 0,
            'BL': GPIO.input(PINS['IR_BOTTOM_LEFT']) == 0,
            'BR': GPIO.input(PINS['IR_BOTTOM_RIGHT']) == 0
        }
    
    def analyze(self):
        dist = self.get_distance()
        ir = self.get_ir_readings()
        
        if dist < CRITICAL_DISTANCE:
            return 'BACK'
        if dist < OBSTACLE_THRESHOLD:
            if not ir['TL']:
                return 'LEFT'
            if not ir['TR']:
                return 'RIGHT'
            return 'TURN'
        return 'FWD'
    
    def get_speed(self, dist):
        if dist < CRITICAL_DISTANCE:
            return 0
        if dist < OBSTACLE_THRESHOLD:
            ratio = (dist - CRITICAL_DISTANCE) / (OBSTACLE_THRESHOLD - CRITICAL_DISTANCE)
            return int(SPEED_PRESETS['MIN'] + (SPEED_PRESETS['SLOW'] - SPEED_PRESETS['MIN']) * ratio)
        if dist < SAFE_DISTANCE:
            ratio = (dist - OBSTACLE_THRESHOLD) / (SAFE_DISTANCE - OBSTACLE_THRESHOLD)
            return int(SPEED_PRESETS['SLOW'] + (ROVER_SPEED - SPEED_PRESETS['SLOW']) * ratio)
        return ROVER_SPEED

class NavigationSystem:
    def __init__(self):
        self.compass = Compass()
        self.motors = MotorController(ROVER_SPEED)
        self.detector = ObstacleDetector()
        self.ble = BLEManager()
        self.running = False
        self.auto_mode = True
        self.current_speed = ROVER_SPEED
        
        # LED pins
        self.led_pins = [PINS['LED1'], PINS['LED2'], PINS['LED3']]
        self.buzzer_pin = PINS['BUZZER']
    
    def setup_indicators(self):
        for pin in self.led_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def led_on(self, num):
        if 1 <= num <= 3:
            GPIO.output(self.led_pins[num-1], GPIO.HIGH)
    
    def led_off(self, num):
        if 1 <= num <= 3:
            GPIO.output(self.led_pins[num-1], GPIO.LOW)
    
    def beep(self, duration=0.1):
        GPIO.output(self.buzzer_pin, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def broadcast_loop(self):
        """Broadcast sensor data via BLE"""
        while self.running:
            data = {
                'd': self.detector.last_distance,
                'tl': self.detector.get_ir_readings()['TL'],
                'tr': self.detector.get_ir_readings()['TR'],
                'bl': self.detector.get_ir_readings()['BL'],
                'br': self.detector.get_ir_readings()['BR'],
                'spd': self.current_speed,
                'auto': self.auto_mode
            }
            self.ble.broadcast_sensor_data(data)
            time.sleep(0.2)
    
    def initialize(self):
        print("\n" + "="*60)
        print("   INTELLIGENT NAVIGATION SYSTEM v2.0")
        print("   BLE Broadcasting - No Pairing Required")
        print("="*60)
        print(f"\n⚙️ Speed: {ROVER_SPEED}%")
        print("🤖 Default: AUTO NAVIGATION")
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.setup_indicators()
        self.led_on(1)
        
        # Initialize BLE
        self.ble.start()
        
        # Initialize hardware
        self.motors.setup()
        self.detector.setup()
        self.compass.initialize()
        
        # Ready indication
        self.beep(0.2)
        time.sleep(0.1)
        self.beep(0.2)
        
        print("\n✅ System Ready!")
        print(f"📡 BLE Device: {DEVICE_NAME}")
        print("   Scan with any BLE scanner to see data")
        print("   No pairing required!")
        print("\n🎯 System Active!")
        print("🤖 AUTO NAVIGATION mode")
        print("📱 Send commands via BLE (no pairing)")
        print("   - Mode: {'type':'mode','mode':'auto/manual'}")
        print("   - Speed: {'type':'speed','speed':0-100}")
        print("   - Control: {'type':'control','action':'forward/backward/left/right/stop'}")
        print("\n📡 Sensor data broadcasted via BLE")
        print("Press Ctrl+C to stop\n")
        
        return True
    
    def process_commands(self):
        cmd = self.ble.get_command()
        if not cmd:
            return
        
        if cmd.get('type') == 'mode':
            self.auto_mode = cmd.get('mode') == 'auto'
            print(f"\n🤖 Mode: {'AUTO' if self.auto_mode else 'MANUAL'}")
            self.beep(0.1)
            
        elif cmd.get('type') == 'speed':
            global ROVER_SPEED
            ROVER_SPEED = max(0, min(100, cmd.get('speed', 40)))
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            print(f"\n⚡ Speed: {ROVER_SPEED}%")
            
        elif cmd.get('type') == 'control' and not self.auto_mode:
            action = cmd.get('action', 'stop')
            if action == 'forward':
                self.motors.forward(self.current_speed)
                print(f"\n⬆️ Forward")
            elif action == 'backward':
                self.motors.backward()
                print(f"\n⬇️ Backward")
            elif action == 'left':
                self.motors.turn_left()
                print(f"\n⬅️ Left")
            elif action == 'right':
                self.motors.turn_right()
                print(f"\n➡️ Right")
            elif action == 'stop':
                self.motors.stop()
                print(f"\n🛑 Stop")
    
    def run(self):
        self.running = True
        
        # Start broadcast thread
        broadcast_thread = threading.Thread(target=self.broadcast_loop)
        broadcast_thread.daemon = True
        broadcast_thread.start()
        
        try:
            while self.running:
                self.process_commands()
                
                if self.auto_mode:
                    dist = self.detector.get_distance()
                    action = self.detector.analyze()
                    speed = self.detector.get_speed(dist)
                    ir = self.detector.get_ir_readings()
                    
                    # Update speed if needed
                    if speed != self.current_speed:
                        self.current_speed = speed
                        self.motors.set_speed(speed)
                    
                    # Show status
                    ir_str = ''
                    ir_str += 'L' if ir['TL'] else '.'
                    ir_str += 'R' if ir['TR'] else '.'
                    ir_str += 'l' if ir['BL'] else '.'
                    ir_str += 'r' if ir['BR'] else '.'
                    
                    print(f"\r📍 Dist:{dist:3d}cm IR:[{ir_str}] {action:6} Speed:{speed}%", end='', flush=True)
                    
                    # Execute action
                    if action == 'FWD':
                        self.motors.forward(speed)
                        self.led_off(2)
                        self.led_off(3)
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
                else:
                    time.sleep(0.05)
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping...")
            self.stop()
    
    def stop(self):
        self.running = False
        self.motors.stop()
        for pin in self.led_pins:
            GPIO.output(pin, GPIO.LOW)
        self.motors.cleanup()
        self.ble.stop()
        GPIO.cleanup()
        print("✅ System stopped")

def main():
    nav = NavigationSystem()
    if nav.initialize():
        nav.run()

if __name__ == "__main__":
    main()