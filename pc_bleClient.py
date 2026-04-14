#!/usr/bin/env python3
"""
PC Controller for Raspberry Pi Rover
Run this on your PC to control the rover via Bluetooth
"""

import serial
import json
import time
import threading
import sys

# ========== CONFIGURATION ==========
BLUETOOTH_PORT = 'COM8'  # Change this to your Bluetooth COM port (Windows)
# For Linux/Mac: '/dev/rfcomm0' or '/dev/tty.Rover'
BLUETOOTH_BAUDRATE = 9600

class RoverController:
    def __init__(self, port=BLUETOOTH_PORT, baudrate=BLUETOOTH_BAUDRATE):
        self.serial_conn = None
        self.connected = False
        self.running = False
        self.receive_thread = None
        self.last_sensor_data = None
        
    def connect(self):
        """Connect to rover via Bluetooth"""
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            self.connected = True
            print(f"✅ Connected to rover on {port}")
            
            # Wait for rover ready message
            time.sleep(1)
            if self.serial_conn.in_waiting:
                data = self.serial_conn.readline().decode('utf-8').strip()
                print(f"📱 Rover says: {data}")
            
            # Send connection confirmation
            self.send_command("PC_CONNECTED")
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def send_command(self, command):
        """Send command to rover"""
        if not self.connected or not self.serial_conn:
            return False
        
        try:
            if isinstance(command, dict):
                data = json.dumps(command) + "\n"
            else:
                data = command + "\n"
            self.serial_conn.write(data.encode('utf-8'))
            return True
        except Exception as e:
            print(f"⚠️ Send error: {e}")
            return False
    
    def receive_data(self):
        """Receive sensor data from rover"""
        while self.running and self.connected:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.readline().decode('utf-8').strip()
                    if data:
                        try:
                            sensor_data = json.loads(data)
                            self.last_sensor_data = sensor_data
                            self.display_sensor_data(sensor_data)
                        except json.JSONDecodeError:
                            print(f"Raw: {data}")
            except Exception as e:
                print(f"⚠️ Receive error: {e}")
                break
            time.sleep(0.05)
    
    def display_sensor_data(self, data):
        """Display received sensor data"""
        print("\r" + " " * 80, end="")  # Clear line
        front_dist = data.get('front_distance', 'N/A')
        if front_dist != 999:
            dist_str = f"{front_dist:.0f}cm"
        else:
            dist_str = "Clear"
        
        ir = data.get('ir_sensors', {})
        ir_active = [k for k, v in ir.items() if v]
        ir_str = ", ".join(ir_active) if ir_active else "None"
        
        mode = "AUTO" if data.get('auto_mode') else "MANUAL"
        
        print(f"\r📊 Front: {dist_str:6} | IR: {ir_str:20} | Mode: {mode} | Speed: {data.get('motor_speed', 0)}%", end="", flush=True)
    
    def switch_mode(self, mode):
        """Switch between manual and auto mode"""
        command = {'type': 'mode', 'mode': mode}
        self.send_command(command)
        print(f"\n🔄 Switching to {mode.upper()} mode...")
    
    def set_speed(self, speed):
        """Set rover speed (0-100)"""
        speed = max(0, min(100, speed))
        command = {'type': 'speed', 'speed': speed}
        self.send_command(command)
        print(f"\n⚡ Setting speed to {speed}%")
    
    def manual_control(self, action):
        """Send manual control command"""
        command = {'type': 'control', 'action': action}
        self.send_command(command)
    
    def start(self):
        """Start the controller"""
        self.running = True
        
        # Start receive thread
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        print("\n" + "="*60)
        print("   ROVER CONTROL INTERFACE")
        print("="*60)
        print("\nCONTROLS:")
        print("  Arrow Keys / WASD - Manual movement")
        print("  M - Switch to Manual Mode")
        print("  A - Switch to Auto Mode")
        print("  +/- - Increase/Decrease Speed")
        print("  S - Stop")
        print("  Q - Quit")
        print("="*60)
        print("\n📡 Receiving sensor data...\n")
        
        try:
            while self.running:
                # Get keyboard input
                if sys.platform == 'win32':
                    import msvcrt
                    if msvcrt.kbhit():
                        key = msvcrt.getch().decode('utf-8').lower()
                        self.handle_keypress(key)
                else:
                    # For Linux/Mac
                    import select
                    if select.select([sys.stdin], [], [], 0)[0]:
                        key = sys.stdin.read(1).lower()
                        self.handle_keypress(key)
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            self.stop()
    
    def handle_keypress(self, key):
        """Handle keyboard input"""
        if key == 'w' or key == 'up':
            self.manual_control('forward')
            print("\n⬆️  Moving forward")
        elif key == 's' or key == 'down':
            self.manual_control('backward')
            print("\n⬇️  Moving backward")
        elif key == 'a' or key == 'left':
            self.manual_control('left')
            print("\n⬅️  Turning left")
        elif key == 'd' or key == 'right':
            self.manual_control('right')
            print("\n➡️  Turning right")
        elif key == ' ' or key == 's':
            self.manual_control('stop')
            print("\n🛑 Stopped")
        elif key == 'm':
            self.switch_mode('manual')
        elif key == 'a':
            self.switch_mode('auto')
        elif key == '+':
            new_speed = min(100, (self.last_sensor_data or {}).get('motor_speed', 0) + 10)
            self.set_speed(new_speed)
        elif key == '-':
            new_speed = max(0, (self.last_sensor_data or {}).get('motor_speed', 0) - 10)
            self.set_speed(new_speed)
        elif key == 'q':
            print("\n👋 Quitting...")
            self.stop()
    
    def stop(self):
        """Stop the controller"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        print("\n✅ Disconnected from rover")

def main():
    print("="*60)
    print("   PC ROVER CONTROLLER")
    print("="*60)
    print(f"\n📱 Connecting to rover on {BLUETOOTH_PORT}...")
    print("   Make sure Bluetooth is paired and connected")
    
    controller = RoverController()
    
    if controller.connect():
        controller.start()
    else:
        print("\n⚠️  Could not connect to rover")
        print("   Check:")
        print("   1. Bluetooth is enabled on PC")
        print("   2. Rover is powered on")
        print("   3. Bluetooth is paired")
        print("   4. COM port is correct (edit BLUETOOTH_PORT)")

if __name__ == "__main__":
    main()