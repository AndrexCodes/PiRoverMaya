#!/usr/bin/env python3
"""
PiRover with Proper BLE Broadcasting using bluetoothctl
Run with: sudo python3 main.py
"""

import RPi.GPIO as GPIO
import time
import subprocess
import threading
import json
import os
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
DEVICE_NAME = 'raspberrypi'
ROVER_SPEED = 40
OBSTACLE_THRESHOLD = 30
CRITICAL_DISTANCE = 15
TURN_DURATION = 0.8

class BLEBeacon:
    """Broadcasts BLE beacon with sensor data using hcitool"""
    
    def __init__(self):
        self.running = False
        self.current_data = ""
        self.hci_device = 'hci0'
        
    def setup(self):
        """Setup Bluetooth for beacon broadcasting"""
        try:
            # Enable Bluetooth and set as discoverable
            subprocess.run(['sudo', 'hciconfig', self.hci_device, 'up'], check=True)
            subprocess.run(['sudo', 'hciconfig', self.hci_device, 'piscan'], check=True)
            time.sleep(0.5)
            
            print(f"✅ BLE beacon ready - Advertising as '{DEVICE_NAME}'")
            return True
            
        except Exception as e:
            print(f"⚠️ BLE setup error: {e}")
            return False
    
    def broadcast(self, distance, speed, auto_mode, ir_list):
        """Broadcast sensor data as BLE manufacturer data using hcitool"""
        if not self.running:
            return
        
        # Create data string
        ir_bits = ''.join([str(x) for x in ir_list])
        data_str = f"d={distance},s={speed},m={'A' if auto_mode else 'M'},ir={ir_bits}"
        
        # Only update if data changed (reduce writes)
        if data_str == self.current_data:
            return
        self.current_data = data_str
        
        # Convert to hex for manufacturer data (Apple MFG ID: 0x004c)
        hex_data = ''.join([f'{ord(c):02x}' for c in data_str[:20]])
        
        try:
            # Build advertisement data: length(1) + type(1) + mfg_id(2 LE) + data
            ad_data = f'1b ff 4c 00 {" ".join([hex_data[i:i+2] for i in range(0, len(hex_data), 2)])}'
            
            # Use hcitool to set advertisement data
            cmd = f"sudo hcitool -i {self.hci_device} cmd 0x08 0x0008 " + ad_data.replace(' ', ' ')
            subprocess.run(cmd, shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
            
        except Exception as e:
            pass
    
    def start(self):
        self.running = True
        return self.setup()
    
    def stop(self):
        self.running = False
        try:
            subprocess.run(['sudo', 'hciconfig', self.hci_device, 'noscan'], stderr=subprocess.DEVNULL)
        except:
            pass

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
        
        distance = int(self.detector.distance) if self.detector.distance < 999 else 999
        ir = self.detector.get_ir()
        speed = self.motors.current_speed
        
        self.ble.broadcast(distance, speed, self.auto_mode, ir)
    
    def run(self):
        self.running = True
        last_broadcast = time.time()
        
        try:
            while self.running:
                if self.auto_mode:
                    dist = self.detector.get_distance()
                    action = self.detector.analyze()
                    
                    ir = self.detector.get_ir()
                    ir_str = ''.join(['X' if x else '.' for x in ir])
                    print(f"\r📡 Dist:{int(dist):3d}cm IR:[{ir_str}] {action:5} Speed:{self.motors.current_speed}%", end='')
                    
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