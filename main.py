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
DEVICE_NAME = 'PiRover'
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

    def broadcast(self, distance, speed, auto_mode, ir_list):
        """Broadcast sensor data as manufacturer-specific data"""
        if not self.running:
            return

        ir_bits = ''.join([str(x) for x in ir_list])
        data_str = f"D{distance}S{speed}M{1 if auto_mode else 0}I{ir_bits}"
        
        if data_str == self.last_data:
            return
        self.last_data = data_str

        # Convert string to bytes
        data_bytes = data_str.encode('utf-8')
        
        # Create manufacturer specific data
        # Format: [Length][Type: 0xFF][Company ID (2 bytes)][Data]
        # Using a custom company ID (0xFFFF is reserved for testing)
        company_id = 0xFFFF  # Use this for testing, or get your own
        
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
        # Calculate total length: 2 bytes for company ID + data bytes
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
        self.compass = None
        self.running = False
        self.auto_mode = True
        self.is_moving = False
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
        print("   PiRover Navigation System")
        print("   BLE Beacon Broadcasting")
        print("="*50)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.setup_indicators()
        GPIO.output(PINS['LED1'], GPIO.HIGH)
        
        # Initialize BLE beacon
        if not self.ble.start():
            print("⚠️ BLE not available - running without beacon")
        
        # Initialize hardware
        self.motors.setup()
        self.detector.setup()

        # Initialize compass for mission turns
        try:
            self.compass = HMC5883L()
            print("✅ Compass ready")
        except Exception as e:
            self.compass = None
            print(f"⚠️ Compass unavailable ({e}) - falling back to timed turns")
        
        # Ready signal
        self.beep(0.2)
        time.sleep(0.1)
        self.beep(0.2)
        
        print(f"\n✅ System Ready! (Cruise speed: {ROVER_SPEED}%)")
        print(f"🤖 Mode: AUTO SQUARE MISSION (default)")
        print(f"📡 BLE Beacon: {DEVICE_NAME}")
        print("   Broadcasting sensor data every 0.2s")
        print(f"   Mission: {MISSION_SIDES} sides, right 90° at < {SAFE_DISTANCE}cm")
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

    @staticmethod
    def _normalize_heading(heading):
        return heading % 360.0

    @staticmethod
    def _angle_error(current, target):
        return ((target - current + 540.0) % 360.0) - 180.0

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
        return sum(headings) / len(headings)

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

            self.broadcast_sensor_data()
            time.sleep(0.04)

        self.motors.stop()
        self.set_moving(False)
        print("\n⚠️ Turn timeout - continuing mission")
        return False

    def run_square_mission(self):
        side_index = 0
        last_broadcast = time.time()

        while self.running and side_index < MISSION_SIDES:
            side_number = side_index + 1
            print(f"\n\n🟩 Side {side_number}/{MISSION_SIDES} - moving forward")

            current_speed = START_SPEED
            self.motors.set_speed(current_speed)
            last_ramp = time.time()

            while self.running:
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

                if now - last_broadcast >= 0.2:
                    self.broadcast_sensor_data()
                    last_broadcast = now

                time.sleep(0.05)

            if not self.running:
                return False

            self.turn_right_90()
            side_index += 1

        self.motors.stop()
        self.set_moving(False)
        print("\n\n🏁 Square mission complete. Returned to base path.")
        return True
    
    def run(self):
        self.running = True
        
        try:
            while self.running:
                if self.auto_mode:
                    self.run_square_mission()
                    self.running = False
                else:
                    self.set_moving(False)

                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping...")
        finally:
            self.stop()
    
    def stop(self):
        self.running = False
        self.set_moving(False)
        self.motors.stop()
        self.ble.stop()
        GPIO.output(PINS['LED1'], GPIO.LOW)
        GPIO.cleanup()
        print("✅ System stopped")

def main():
    nav = NavigationSystem()
    if nav.initialize():
        nav.run()

if __name__ == "__main__":
    main()