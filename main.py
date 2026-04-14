#!/usr/bin/env python3
"""
Intelligent Navigation System for Raspberry Pi
Now with FULL Bluetooth setup inside Python
"""

import RPi.GPIO as GPIO
import time
import math
import threading
import json
import subprocess
import os
from collections import deque
import serial
import serial.tools.list_ports

# ========== BLUETOOTH CONFIGURATION ==========
BLUETOOTH_BAUDRATE = 9600
BLUETOOTH_PORTS = ['/dev/rfcomm0', '/dev/ttyAMA0', '/dev/ttyS0']
DEVICE_NAME = "PiRover"   # You can change this

class BluetoothManager:
    """Handles ALL Bluetooth setup + communication"""
    
    def __init__(self):
        self.serial_connection = None
        self.connected = False
        self.agent_process = None
        self.running = False
    
    def run_command(self, cmd, shell=False):
        """Run system command with logging"""
        try:
            result = subprocess.run(cmd, shell=shell, capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                print(f"⚠️  Command failed: {' '.join(cmd) if isinstance(cmd, list) else cmd}")
            return result.stdout.strip()
        except Exception as e:
            print(f"❌ Command error: {e}")
            return None

    def setup_bluetooth(self):
        """Complete Bluetooth setup (No PIN, Discoverable, etc.)"""
        print("\n🔵 Setting up Bluetooth...")

        try:
            # 1. Configure main.conf
            self.run_command("sudo sed -i 's/#*DiscoverableTimeout.*/DiscoverableTimeout = 0/' /etc/bluetooth/main.conf", shell=True)
            self.run_command("sudo sed -i 's/#*PairableTimeout.*/PairableTimeout = 0/' /etc/bluetooth/main.conf", shell=True)
            
            # 2. Restart Bluetooth service
            self.run_command(["sudo", "systemctl", "restart", "bluetooth"])
            time.sleep(2)

            # 3. Configure via bluetoothctl
            bt_commands = f"""
                agent NoInputNoOutput
                default-agent
                power on
                pairable on
                discoverable on
                discoverable-timeout 0
                pairable-timeout 0
                name {DEVICE_NAME}
                exit
            """
            self.run_command(f"echo '{bt_commands}' | sudo bluetoothctl", shell=True)
            time.sleep(2)

            # 4. Add Serial Port Profile
            self.run_command(["sudo", "sdptool", "add", "SP"])
            print("✅ Serial Port Profile registered")

            # 5. Release old bindings
            self.run_command(["sudo", "rfcomm", "release", "0"], shell=True)
            self.run_command(["sudo", "rfcomm", "release", "1"], shell=True)

            print(f"✅ Bluetooth fully configured as '{DEVICE_NAME}'")
            print("   → No PIN pairing enabled")
            print("   → Discoverable to all devices")
            return True

        except Exception as e:
            print(f"❌ Bluetooth setup failed: {e}")
            return False

    def wait_for_connection(self, timeout=60):
        """Wait for a device to connect and bind RFCOMM"""
        print("\n📱 Waiting for device to connect (60 seconds max)...")
        print("   Pair from your phone/PC - it should NOT ask for PIN")

        start_time = time.time()
        attempt = 0

        while time.time() - start_time < timeout:
            # Check connected devices
            connected = self.run_command("bluetoothctl devices Connected", shell=True)
            if connected and "Device" in connected:
                mac = connected.split()[-1] if len(connected.split()) > 1 else None
                if mac:
                    print(f"✅ Device connected: {mac}")
                    
                    # Bind RFCOMM
                    result = self.run_command(["sudo", "rfcomm", "bind", "/dev/rfcomm0", mac, "1"])
                    time.sleep(2)
                    
                    if os.path.exists("/dev/rfcomm0"):
                        print("✅ RFCOMM bound to /dev/rfcomm0")
                        return True
            
            attempt += 1
            if attempt % 6 == 0:
                print(f"   Still waiting... ({int(time.time()-start_time)}s)")
            time.sleep(5)

        print("⚠️  No device connected within timeout")
        return False

    def find_and_connect_serial(self):
        """Find and open serial port"""
        ports_to_try = ['/dev/rfcomm0'] + BLUETOOTH_PORTS
        
        for port in ports_to_try:
            if os.path.exists(port):
                try:
                    self.serial_connection = serial.Serial(port, BLUETOOTH_BAUDRATE, timeout=1, write_timeout=1)
                    print(f"✅ Serial connection opened on {port}")
                    self.connected = True
                    return True
                except Exception as e:
                    print(f"⚠️  Failed to open {port}: {e}")
        
        print("❌ Could not open any Bluetooth serial port")
        return False

    def send_data(self, data_dict):
        if not self.connected or not self.serial_connection:
            return False
        try:
            json_data = json.dumps(data_dict) + "\n"
            self.serial_connection.write(json_data.encode('utf-8'))
            return True
        except:
            self.connected = False
            return False

    def receive_command(self):
        if not self.connected or not self.serial_connection:
            return None
        try:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().decode('utf-8').strip()
                if data:
                    try:
                        return json.loads(data)
                    except:
                        return {'type': 'raw', 'command': data}
        except:
            self.connected = False
        return None

    def close(self):
        self.running = False
        if self.serial_connection:
            self.serial_connection.close()
        self.connected = False
        print("🔵 Bluetooth connection closed")


# ================== REST OF YOUR CODE (Only small changes) ==================

class NavigationSystem:
    def __init__(self):
        self.bluetooth = BluetoothManager()
        # ... (rest of your original __init__ remains the same)
        self.compass = Compass()
        self.motors = MotorController(ROVER_SPEED)
        self.detector = ObstacleDetector()
        self.running = False
        self.auto_mode = False
        # ... keep all your other variables

    def initialize(self):
        print("\n🚀 Initializing Navigation System...")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # === FULL BLUETOOTH SETUP ===
        if self.bluetooth.setup_bluetooth():
            if self.bluetooth.wait_for_connection(timeout=90):
                if self.bluetooth.find_and_connect_serial():
                    print("✅ Bluetooth Fully Ready!")
                else:
                    print("⚠️  Bluetooth setup done but serial failed")
            else:
                print("⚠️  Running without Bluetooth connection")
        else:
            print("⚠️  Bluetooth setup failed - continuing in standalone mode")

        # Setup other components (LEDs, motors, sensors)...
        self.setup_indicators()
        self.motors.setup()
        self.detector.setup()
        self.compass.initialize()

        print("\n✅ System Initialized!")
        return True

    # Keep all your other methods (process_commands, auto_navigation_loop, etc.)
    # Just make sure to use self.bluetooth instead of self.bluetooth_controller

# ================== MAIN ==================
def main():
    print("="*70)
    print("   PiRover - Full Bluetooth + Navigation System")
    print("="*70)

    nav = NavigationSystem()
    try:
        if nav.initialize():
            nav.running = True
            nav.start()
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        nav.stop()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        nav.stop()

if __name__ == "__main__":
    main()