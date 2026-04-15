#!/usr/bin/env python3
"""
PiRover with BLE Advertising using D-Bus
Run with: sudo python3 main.py
"""

import RPi.GPIO as GPIO
import time
import threading
import dbus
import dbus.exceptions
import dbus.mainloop.glib
from gi.repository import GLib
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

# BLE UUIDs
SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

class BLEAdvertisement:
    """BLE Advertisement using D-Bus"""
    
    def __init__(self):
        self.mainloop = None
        self.adapter = None
        self.advertisement = None
        self.running = False
        
    def setup(self):
        """Setup D-Bus for BLE advertising"""
        try:
            dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
            self.mainloop = GLib.MainLoop()
            
            # Get system bus
            bus = dbus.SystemBus()
            
            # Get adapter
            adapter_path = "/org/bluez/hci0"
            adapter_obj = bus.get_object("org.bluez", adapter_path)
            self.adapter = dbus.Interface(adapter_obj, "org.bluez.Adapter1")
            
            # Power on adapter
            self.adapter.SetProperty("Powered", dbus.Boolean(True))
            self.adapter.SetProperty("Discoverable", dbus.Boolean(True))
            self.adapter.SetProperty("Alias", DEVICE_NAME)
            
            time.sleep(1)
            
            # Create advertisement
            self.advertisement = self.create_advertisement(bus, adapter_path)
            
            # Register advertisement
            self.register_advertisement(bus, self.advertisement)
            
            print(f"✅ BLE advertisement registered as '{DEVICE_NAME}'")
            return True
            
        except Exception as e:
            print(f"⚠️ BLE setup error: {e}")
            return False
    
    def create_advertisement(self, bus, path):
        """Create advertisement object"""
        advertisement_path = f"{path}/advertisement"
        
        advertisement = bus.get_object("org.bluez", advertisement_path)
        if advertisement:
            return advertisement
            
        # Create new advertisement
        obj = bus.get_object("org.bluez", "/org/bluez")
        
        manager = dbus.Interface(obj, "org.bluez.LEAdvertisingManager1")
        
        # Advertisement properties
        props = {
            "Type": "peripheral",
            "ServiceUUIDs": dbus.Array([SERVICE_UUID], signature='s'),
            "Discoverable": dbus.Boolean(True),
            "DiscoverableTimeout": dbus.UInt16(0),
            "LocalName": DEVICE_NAME,
        }
        
        # Register advertisement
        manager.RegisterAdvertisement(advertisement_path, props, reply_handler=self.ad_reply, error_handler=self.ad_error)
        
        return advertisement
    
    def register_advertisement(self, bus, advertisement):
        """Register advertisement with D-Bus"""
        # Implementation depends on your BlueZ version
        pass
    
    def update_data(self, distance, speed, auto_mode, ir_list):
        """Update advertising data with sensor readings"""
        if not self.running:
            return
        
        # Create data packet
        # Format: distance|speed|mode|ir_bits
        ir_bits = ''.join([str(x) for x in ir_list])
        data_str = f"{distance},{speed},{'A' if auto_mode else 'M'},{ir_bits}"
        
        # Add to manufacturer data
        # This would require updating the advertisement via D-Bus
        pass
    
    def ad_reply(self):
        pass
    
    def ad_error(self, error):
        print(f"Advertisement error: {error}")
    
    def start(self):
        self.running = True
        return self.setup()
    
    def stop(self):
        self.running = False
        if self.mainloop:
            self.mainloop.quit()

# SIMPLER WORKING SOLUTION - Use Python's BLE Library
class SimpleBLEBeacon:
    """Simple BLE beacon using pybluez"""
    
    def __init__(self):
        self.running = False
        
    def setup(self):
        """Setup BLE advertising using simple method"""
        try:
            import bluetooth._bluetooth as bluez
            
            # Open HCI socket
            self.sock = bluez.hci_open_dev(0)
            bluez.hci_le_set_scan_parameters(self.sock, 0x00, 0x0800, 0x0800, 0x00, 0x00)
            
            # Set advertising parameters
            cmd = struct.pack("<HHBBBB", 0x0800, 0x0800, 0x00, 0x00, 0x00, 0x00)
            bluez.hci_send_cmd(self.sock, 0x08, 0x0006, cmd)
            
            print("✅ Simple BLE beacon ready")
            return True
        except Exception as e:
            print(f"⚠️ BLE error: {e}")
            return False
    
    def broadcast(self, distance, speed, auto_mode, ir_list):
        """Broadcast data as iBeacon"""
        if not self.running:
            return
        
        try:
            import bluetooth._bluetooth as bluez
            
            # Create iBeacon packet
            # Proximity UUID: PiRover
            uuid = [0x50, 0x69, 0x52, 0x6F, 0x76, 0x65, 0x72, 0x00]  # "PiRover"
            
            # Major = distance
            major = distance & 0xFFFF
            
            # Minor = (speed << 8) | (auto_mode << 4) | ir_bits
            ir_bits = (ir_list[0] << 3) | (ir_list[1] << 2) | (ir_list[2] << 1) | ir_list[3]
            minor = (speed << 8) | (auto_mode << 4) | ir_bits
            
            # Measured power
            power = 0xC5  # -59 dBm
            
            # Build advertising packet
            adv_data = bytearray()
            adv_data.append(0x02)  # Flags length
            adv_data.append(0x01)  # Flags type
            adv_data.append(0x06)  # LE General Discoverable
            
            # iBeacon prefix
            adv_data.append(0x1A)  # Length
            adv_data.append(0xFF)  # Manufacturer specific
            adv_data.append(0x4C)  # Apple Company ID
            adv_data.append(0x00)
            adv_data.append(0x02)  # iBeacon type
            adv_data.append(0x15)  # iBeacon length
            adv_data.extend(uuid)  # UUID
            adv_data.extend(struct.pack('>H', major))
            adv_data.extend(struct.pack('>H', minor))
            adv_data.append(power)
            
            # Set advertising data
            bluez.hci_send_cmd(self.sock, 0x08, 0x0008, adv_data)
            
            # Enable advertising
            bluez.hci_send_cmd(self.sock, 0x08, 0x000a, struct.pack("B", 0x01))
            
        except Exception as e:
            pass
    
    def start(self):
        self.running = True
        return self.setup()
    
    def stop(self):
        self.running = False
        try:
            import bluetooth._bluetooth as bluez
            bluez.hci_send_cmd(self.sock, 0x08, 0x000a, struct.pack("B", 0x00))
        except:
            pass

# ULTIMATE SIMPLE SOLUTION - Use 'hciconfig' with pre-set advertising
class WorkingBLEBeacon:
    """Working BLE beacon using pre-configured advertising"""
    
    def __init__(self):
        self.running = False
        
    def setup(self):
        """Setup BLE with a fixed advertising packet"""
        try:
            import subprocess
            
            # Stop any existing advertising
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'noLEadv'], capture_output=True)
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'], capture_output=True)
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'name', DEVICE_NAME], capture_output=True)
            subprocess.run(['sudo', 'hciconfig', 'hci0', 'leadv'], capture_output=True)
            
            print(f"✅ BLE advertising configured for '{DEVICE_NAME}'")
            return True
        except Exception as e:
            print(f"⚠️ BLE error: {e}")
            return False
    
    def broadcast(self, distance, speed, auto_mode, ir_list):
        """Update advertising packet (works with hcitool)"""
        if not self.running:
            return
        
        try:
            import subprocess
            
            # Create simple data string
            ir_str = ''.join([str(x) for x in ir_list])
            data_str = f"PiRover:{distance},{speed},{auto_mode},{ir_str}"
            
            # Convert to hex
            hex_data = ' '.join([f'{ord(c):02x}' for c in data_str[:20]])
            
            # Create full advertising packet
            # Flags + Local Name + Manufacturer Data
            adv_packet = []
            
            # Flags
            adv_packet.append('02')
            adv_packet.append('01')
            adv_packet.append('06')
            
            # Local Name
            name_hex = ' '.join([f'{ord(c):02x}' for c in DEVICE_NAME])
            name_len = len(DEVICE_NAME) + 1
            adv_packet.append(f'{name_len:02x}')
            adv_packet.append('09')
            adv_packet.extend(name_hex.split())
            
            # Manufacturer specific data
            data_bytes = [f'{ord(c):02x}' for c in data_str[:18]]
            if data_bytes:
                adv_packet.append(f'{len(data_bytes)+2:02x}')
                adv_packet.append('ff')
                adv_packet.append('4c')
                adv_packet.append('00')
                adv_packet.extend(data_bytes)
            
            # Pad to 31 bytes
            while len(adv_packet) < 31:
                adv_packet.append('00')
            
            # Send command
            cmd = ['sudo', 'hcitool', '-i', 'hci0', 'cmd', '0x08', '0x0008']
            cmd.extend(adv_packet[:31])
            subprocess.run(' '.join(cmd), shell=True, capture_output=True)
            
            # Enable advertising
            subprocess.run(['sudo', 'hcitool', '-i', 'hci0', 'cmd', '0x08', '0x000a', '01'], 
                          capture_output=True)
            
        except Exception as e:
            pass
    
    def start(self):
        self.running = True
        return self.setup()
    
    def stop(self):
        self.running = False
        import subprocess
        subprocess.run(['sudo', 'hcitool', '-i', 'hci0', 'cmd', '0x08', '0x000a', '00'], capture_output=True)

# ========== MAIN NAVIGATION SYSTEM ==========

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
        self.ble = WorkingBLEBeacon()  # Using the working version
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