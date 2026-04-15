#!/usr/bin/env python3
"""
PiRover with BLE Broadcasting using hcitool
Run with: sudo python3 main.py
"""

import RPi.GPIO as GPIO
import time
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

    """Simple BLE beacon using hcitool - Fixed version"""
    
    def __init__(self):
        self.running = False
        self.last_data = ""
        
    def setup(self):

        """Initialize Bluetooth"""
        try:

            # Bring up Bluetooth
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'], capture_output=True)
            
            # Set device name so both system scanner and LE apps show "PiRover"
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'name', DEVICE_NAME], capture_output=True)
            
            # Stop any existing advertising
            subprocess.run(['sudo', 'hcitool', 'cmd', '0x08', '0x000a', '00'], capture_output=True)
            
            # Extra settings for better compatibility
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'lestate'], capture_output=True)
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'txpower', '6'], capture_output=True)

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

        def broadcast(self, distance, speed, auto_mode, ir_list):
        """Improved broadcast - much better compatibility with Bleak/BlueZ"""
        if not self.running:
            return

        ir_bits = ''.join([str(x) for x in ir_list])
        data_str = f"D{distance}S{speed}M{1 if auto_mode else 0}I{ir_bits}"

        if data_str == self.last_data:
            return
        self.last_data = data_str

        adv_data = bytearray()

        # Flags (must come first)
        adv_data.extend([0x02, 0x01, 0x06])   # LE General Discoverable, BR/EDR Not Supported

        # Local Name
        name_bytes = DEVICE_NAME.encode('utf-8')
        adv_data.extend([len(name_bytes) + 1, 0x09])
        adv_data.extend(name_bytes)

        # Manufacturer Specific Data
        data_bytes = data_str.encode('utf-8')
        adv_data.extend([len(data_bytes) + 2, 0xFF, 0x4C, 0x00])  # Apple company ID
        adv_data.extend(data_bytes)

        # === CRITICAL: Calculate real length and pad correctly ===
        real_length = len(adv_data)
        while len(adv_data) < 31:
            adv_data.append(0x00)

        try:
            # Command with correct length byte
            cmd = ['sudo', 'hcitool', 'cmd', '0x08', '0x0008', f'{real_length:02x}']
            for b in adv_data:
                cmd.append(f'{b:02x}')

            subprocess.run(' '.join(cmd), shell=True, capture_output=True)

            # Enable advertising
            subprocess.run(['sudo', 'hcitool', 'cmd', '0x08', '0x000a', '01'], 
                          capture_output=True)

        except Exception as e:
            print(f"BLE send error: {e}")

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