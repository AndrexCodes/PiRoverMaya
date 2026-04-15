#!/usr/bin/env python3
"""
PiRover with Proper BLE Broadcasting
Run with: sudo python3 main.py
"""

import RPi.GPIO as GPIO
import time
import math
import threading
import json
import subprocess
import struct
from collections import deque

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
DEVICE_NAME = 'PiRover'
ROVER_SPEED = 40
OBSTACLE_THRESHOLD = 30
CRITICAL_DISTANCE = 15
TURN_DURATION = 0.8

class BLEBeacon:
    """Broadcasts BLE beacon with sensor data"""
    
    def __init__(self):
        self.running = False
        self.bt_interface = "hci0"
        
    def setup(self):
        """Setup Bluetooth for beacon broadcasting"""
        try:
            # Bring up interface
            subprocess.run(['sudo', 'hciconfig', self.bt_interface, 'up'], capture_output=True)
            subprocess.run(['sudo', 'hciconfig', self.bt_interface, 'leadv', '3'], capture_output=True)
            
            # Set device name
            subprocess.run(['sudo', 'hciconfig', self.bt_interface, 'name', DEVICE_NAME], capture_output=True)
            
            # Reset advertising
            subprocess.run(['sudo', 'hcitool', 'cmd', '0x08', '0x000a', '00'], capture_output=True)
            
            print(f"✅ BLE beacon ready on {self.bt_interface}")
            return True
        except Exception as e:
            print(f"⚠️ BLE setup error: {e}")
            return False
    
    def broadcast(self, distance, speed, auto_mode, ir_list):
        """Broadcast sensor data as BLE beacon"""
        if not self.running:
            return
        
        # Create simple data packet as JSON-like string
        # Format: {d:45,s:40,m:1,i:1010}
        # Where i is 4 bits for IR sensors (TL,TR,BL,BR)
        ir_bits = 0
        if ir_list[0]: ir_bits |= 1  # TL
        if ir_list[1]: ir_bits |= 2  # TR  
        if ir_list[2]: ir_bits |= 4  # BL
        if ir_list[3]: ir_bits |= 8  # BR
        
        # Create compact data string
        data_str = f"d:{distance},s:{speed},m:{1 if auto_mode else 0},i:{ir_bits}"
        data_bytes = data_str.encode('utf-8')[:25]  # Max 25 bytes
        
        # Build advertising packet
        adv_data = bytearray()
        adv_data.append(0x02)  # Flags length
        adv_data.append(0x01)  # Flags type
        adv_data.append(0x06)  # LE General Discoverable
        
        # Add local name
        name_bytes = DEVICE_NAME.encode('utf-8')
        adv_data.append(len(name_bytes) + 1)
        adv_data.append(0x09)  # Complete Local Name
        adv_data.extend(name_bytes)
        
        # Add manufacturer specific data with sensor readings
        adv_data.append(len(data_bytes) + 2)
        adv_data.append(0xFF)  # Manufacturer specific
        adv_data.append(0x4C)  # Company ID (Apple)
        adv_data.append(0x00)
        adv_data.extend(data_bytes)
        
        # Pad to 31 bytes
        while len(adv_data) < 31:
            adv_data.append(0x00)
        
        # Send via hcitool
        try:
            cmd = ['sudo', 'hcitool', '-i', self.bt_interface, 'cmd', '0x08', '0x0008']
            for byte in adv_data[:31]:
                cmd.append(f'{byte:02x}')
            subprocess.run(' '.join(cmd), shell=True, capture_output=True)
            
            # Enable advertising
            subprocess.run(['sudo', 'hcitool', '-i', self.bt_interface, 'cmd', '0x08', '0x000a', '01'], 
                          capture_output=True)
        except Exception as e:
            pass
    
    def start(self):
        self.running = True
        return self.setup()
    
    def stop(self):
        self.running = False
        subprocess.run(['sudo', 'hcitool', 'cmd', '0x08', '0x000a', '00'], capture_output=True)

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
            self.distance = dist if 2 < dist < 400 else 999
            return self.distance
        except:
            return 999
    
    def get_ir(self):
        return [
            1 if GPIO.input(PINS['IR_TOP_LEFT']) == 0 else 0,    # TL
            1 if GPIO.input(PINS['IR_TOP_RIGHT']) == 0 else 0,   # TR
            1 if GPIO.input(PINS['IR_BOTTOM_LEFT']) == 0 else 0, # BL
            1 if GPIO.input(PINS['IR_BOTTOM_RIGHT']) == 0 else 0 # BR
        ]
    
    def analyze(self):
        dist = self.get_distance()
        ir = self.get_ir()
        
        if dist < CRITICAL_DISTANCE:
            return 'BACK'
        if dist < OBSTACLE_THRESHOLD:
            if ir[0] == 0:  # Top-Left clear
                return 'LEFT'
            if ir[1] == 0:  # Top-Right clear
                return 'RIGHT'
            return 'TURN'
        return 'FWD'

class NavigationSystem:
    def __init__(self):
        self.motors = MotorController()
        self.detector = ObstacleDetector()
        self.ble = BLEBeacon()
        self.running = False
        self.auto_mode = True  # Default AUTO
        self.last_broadcast = 0
    
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
        
        # Ready signal
        self.beep(0.2)
        time.sleep(0.1)
        self.beep(0.2)
        
        print(f"\n✅ System Ready! (Speed: {ROVER_SPEED}%)")
        print(f"🤖 Mode: AUTO (default)")
        print(f"📡 BLE Beacon: {DEVICE_NAME}")
        print("   Broadcasting sensor data every 0.2s")
        print("\nPress Ctrl+C to stop\n")
        
        return True
    
    def broadcast_sensor_data(self):
        """Broadcast sensor data via BLE beacon"""
        if not self.ble.running:
            return
        
        # Get current data
        distance = int(self.detector.distance) if self.detector.distance < 999 else 999
        ir = self.detector.get_ir()
        speed = self.motors.current_speed
        
        # Broadcast with correct parameter order: (distance, speed, auto_mode, ir_list)
        self.ble.broadcast(distance, speed, self.auto_mode, ir)
    
    def run(self):
        self.running = True
        last_broadcast = time.time()
        
        try:
            while self.running:
                # Auto navigation
                if self.auto_mode:
                    dist = self.detector.get_distance()
                    action = self.detector.analyze()
                    
                    # Show status
                    ir = self.detector.get_ir()
                    ir_str = ''.join(['X' if x else '.' for x in ir])
                    print(f"\r📡 Dist:{int(dist):3d}cm IR:[{ir_str}] {action:5} Speed:{self.motors.current_speed}%", end='')
                    
                    # Execute action
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
                
                # Broadcast sensor data via BLE (5 times per second)
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