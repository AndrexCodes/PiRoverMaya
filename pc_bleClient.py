#!/usr/bin/env python3
"""
PC Controller for Raspberry Pi Rover
Run with: python3 pc_controller.py
"""

import bluetooth
import json
import time
import threading
import sys
import termios
import tty
import select

# Bluetooth configuration
ROVER_NAME = "RoverNavSystem"  # Or use MAC address: "XX:XX:XX:XX:XX:XX"
BT_UUID = "00001101-0000-1000-8000-00805F9B34FB"

class RoverController:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.running = False
        self.receive_thread = None
        
    def connect(self):
        """Connect to Raspberry Pi via Bluetooth"""
        print(f"🔍 Searching for {ROVER_NAME}...")
        
        try:
            # Find devices
            devices = bluetooth.discover_devices(duration=5, lookup_names=True)
            
            rover_address = None
            for addr, name in devices:
                if name and ROVER_NAME in name:
                    rover_address = addr
                    break
            
            if not rover_address:
                print("❌ Rover not found. Make sure it's powered on and Bluetooth is enabled")
                print("   You can also specify the MAC address directly in the code")
                return False
            
            print(f"✅ Found rover at {rover_address}")
            print("🔗 Connecting...")
            
            # Connect
            self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.sock.connect((rover_address, 1))
            self.connected = True
            print("✅ Connected to Rover!")
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def send_command(self, command):
        """Send command to rover"""
        if self.connected and self.sock:
            try:
                json_data = json.dumps(command)
                self.sock.send((json_data + "\n").encode('utf-8'))
                return True
            except Exception as e:
                print(f"⚠️  Send error: {e}")
                self.connected = False
                return False
        return False
    
    def receive_data(self):
        """Receive data from rover"""
        if self.connected and self.sock: