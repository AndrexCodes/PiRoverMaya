#!/usr/bin/env python3
"""
PC Bluetooth Controller for Raspberry Pi Rover
Auto-connects via MAC address and sends commands
"""

import serial
import serial.tools.list_ports
import json
import time
import threading
import subprocess
import platform
import re
from typing import Optional, Dict, Any

# ========== CONFIGURATION ==========
# Update this with your Raspberry Pi's Bluetooth MAC address
# Format: "XX:XX:XX:XX:XX:XX" or "XX-XX-XX-XX-XX-XX"
ROVER_MAC_ADDRESS = "B8:27:EB:89:C1:AA"  # ← CHANGE THIS!

# Alternative: Auto-discover by name
ROVER_NAME = "raspberrypi"  # The name set in the Raspberry Pi code

BLUETOOTH_BAUDRATE = 9600
RECONNECT_DELAY = 5  # seconds between reconnection attempts


class PCBluetoothController:
    """PC-side Bluetooth controller for Raspberry Pi Rover"""
    
    def __init__(self, mac_address: str = None, device_name: str = None):
        self.mac_address = mac_address or ROVER_MAC_ADDRESS
        self.device_name = device_name or ROVER_NAME
        self.serial_connection = None
        self.connected = False
        self.running = False
        self.receive_thread = None
        self.os_type = platform.system()
        
        # Command callbacks
        self.on_data_received = None
        self.on_connected = None
        self.on_disconnected = None
        
        # Data buffer
        self.last_sensor_data = None
        
    def get_bluetooth_mac_from_name(self, timeout: int = 10) -> Optional[str]:
        """Discover Bluetooth device by name and return its MAC address"""
        print(f"🔍 Searching for device named '{self.device_name}'...")
        
        if self.os_type == "Windows":
            return self._discover_windows(timeout)
        elif self.os_type == "Linux":
            return self._discover_linux(timeout)
        elif self.os_type == "Darwin":  # macOS
            return self._discover_macos(timeout)
        else:
            print(f"❌ Unsupported OS: {self.os_type}")
            return None
    
    def _discover_linux(self, timeout: int) -> Optional[str]:
        """Discover Bluetooth device on Linux using bluetoothctl"""
        try:
            # Start bluetoothctl and scan
            scan_cmd = ['bluetoothctl', 'scan', 'on']
            scan_process = subprocess.Popen(scan_cmd, stdout=subprocess.DEVNULL, 
                                           stderr=subprocess.DEVNULL)
            
            print("   Scanning for Bluetooth devices...")
            time.sleep(3)  # Allow time for scan to start
            
            # Get devices list
            result = subprocess.run(['bluetoothctl', 'devices'], 
                                   capture_output=True, text=True, timeout=timeout)
            
            scan_process.terminate()
            
            # Parse output
            for line in result.stdout.split('\n'):
                # Format: "Device XX:XX:XX:XX:XX:XX Device Name"
                match = re.match(r'Device\s+([0-9A-F:]+)\s+(.+)', line.strip(), re.IGNORECASE)
                if match:
                    mac = match.group(1)
                    name = match.group(2)
                    if self.device_name.lower() in name.lower():
                        print(f"✅ Found {name} at {mac}")
                        return mac
            
            print(f"⚠️  Device '{self.device_name}' not found")
            return None
            
        except Exception as e:
            print(f"❌ Discovery error: {e}")
            return None
    
    def _discover_windows(self, timeout: int) -> Optional[str]:
        """Discover Bluetooth device on Windows"""
        try:
            # Use PowerShell to query Bluetooth devices
            ps_script = '''
            Add-Type -AssemblyName System.Runtime.WindowsRuntime
            $asTaskGeneric = ([System.WindowsRuntimeSystemExtensions].GetMethods() | ? { $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]
            Function Await($WinRtTask, $ResultType) {
                $asTask = $asTaskGeneric.MakeGenericMethod($ResultType)
                $netTask = $asTask.Invoke($null, @($WinRtTask))
                $netTask.Wait(-1) | Out-Null
                $netTask.Result
            }
            [Windows.Devices.Enumeration.DeviceInformation, Windows.Devices.Enumeration, ContentType = WindowsRuntime] | Out-Null
            $selector = [Windows.Devices.Bluetooth.BluetoothDevice]::GetDeviceSelector()
            $devices = Await ([Windows.Devices.Enumeration.DeviceInformation]::FindAllAsync($selector)) ([System.Collections.Generic.IReadOnlyList[Windows.Devices.Enumeration.DeviceInformation]])
            foreach ($device in $devices) {
                $name = $device.Name
                $id = $device.Id
                if ($name -like "*$env:DEVICE_NAME*") {
                    $macMatch = [regex]::Match($id, '[0-9A-F]{12}')
                    if ($macMatch.Success) {
                        $mac = $macMatch.Value -replace '(..)(..)(..)(..)(..)(..)', '$1:$2:$3:$4:$5:$6'
                        Write-Output "$name|$mac"
                    }
                }
            }
            '''
            
            env = os.environ.copy()
            env['DEVICE_NAME'] = self.device_name
            
            result = subprocess.run(['powershell', '-Command', ps_script],
                                   capture_output=True, text=True, timeout=timeout,
                                   env=env)
            
            if result.stdout.strip():
                name, mac = result.stdout.strip().split('|')
                print(f"✅ Found {name} at {mac}")
                return mac
            
            print(f"⚠️  Device '{self.device_name}' not found")
            return None
            
        except Exception as e:
            print(f"❌ Windows discovery error: {e}")
            print("   Make sure Bluetooth is enabled and paired manually first")
            return None
    
    def _discover_macos(self, timeout: int) -> Optional[str]:
        """Discover Bluetooth device on macOS"""
        try:
            # Use system_profiler to get Bluetooth info
            result = subprocess.run(['system_profiler', 'SPBluetoothDataType'],
                                   capture_output=True, text=True, timeout=timeout)
            
            # Parse output
            current_device = {}
            for line in result.stdout.split('\n'):
                line = line.strip()
                if 'Address:' in line:
                    mac = line.split('Address:')[-1].strip()
                    current_device['mac'] = mac
                elif 'Name:' in line and 'Connected' not in line:
                    name = line.split('Name:')[-1].strip()
                    current_device['name'] = name
                    if self.device_name.lower() in name.lower():
                        print(f"✅ Found {name} at {current_device['mac']}")
                        return current_device['mac']
            
            print(f"⚠️  Device '{self.device_name}' not found")
            return None
            
        except Exception as e:
            print(f"❌ macOS discovery error: {e}")
            return None
    
    def pair_device_linux(self, mac_address: str) -> bool:
        """Pair with device on Linux using bluetoothctl"""
        print(f"🔗 Pairing with {mac_address}...")
        
        commands = [
            f'pair {mac_address}',
            f'trust {mac_address}',
            f'connect {mac_address}'
        ]
        
        try:
            # Start bluetoothctl process
            process = subprocess.Popen(['bluetoothctl'], 
                                      stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE,
                                      text=True)
            
            for cmd in commands:
                process.stdin.write(cmd + '\n')
                process.stdin.flush()
                time.sleep(2)
            
            process.stdin.write('quit\n')
            process.stdin.flush()
            process.wait(timeout=5)
            
            # Check if RFCOMM device exists
            time.sleep(2)
            if self.find_rfcomm_device(mac_address):
                print(f"✅ Successfully paired and connected to {mac_address}")
                return True
            else:
                print("⚠️  Paired but RFCOMM device not found")
                return False
                
        except Exception as e:
            print(f"❌ Pairing error: {e}")
            return False
    
    def find_rfcomm_device(self, mac_address: str = None) -> Optional[str]:
        """Find RFCOMM device for given MAC address"""
        # Standard RFCOMM port
        if os.path.exists('/dev/rfcomm0'):
            return '/dev/rfcomm0'
        
        # Try to bind RFCOMM
        if mac_address:
            try:
                subprocess.run(['rfcomm', 'bind', '/dev/rfcomm0', mac_address, '1'],
                             timeout=10, capture_output=True)
                if os.path.exists('/dev/rfcomm0'):
                    return '/dev/rfcomm0'
            except:
                pass
        
        # Look for any serial ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'rfcomm' in port.device or 'bluetooth' in port.description.lower():
                return port.device
        
        return None
    
    def connect(self, mac_address: str = None, use_discovery: bool = True) -> bool:
        """Connect to Raspberry Pi rover"""
        
        # Use provided MAC or discover
        target_mac = mac_address or self.mac_address
        
        if use_discovery and target_mac == ROVER_MAC_ADDRESS:
            print("🔍 Attempting to auto-discover rover...")
            discovered_mac = self.get_bluetooth_mac_from_name()
            if discovered_mac:
                target_mac = discovered_mac
                self.mac_address = target_mac
        
        if not target_mac:
            print("❌ No MAC address specified and auto-discovery failed")
            return False
        
        print(f"\n🔵 Connecting to Raspberry Pi at {target_mac}...")
        
        # Platform-specific connection
        if self.os_type == "Linux":
            # On Linux, pair and connect via bluetoothctl
            if not self.pair_device_linux(target_mac):
                return False
            
            # Find serial port
            port = self.find_rfcomm_device(target_mac)
            if not port:
                print("❌ No RFCOMM device found")
                return False
                
        elif self.os_type == "Windows":
            # On Windows, need a paired COM port
            port = self.find_bluetooth_com_port(target_mac)
            if not port:
                print("❌ No Bluetooth COM port found")
                print("   Make sure the device is paired in Windows Bluetooth settings")
                return False
                
        elif self.os_type == "Darwin":  # macOS
            port = self.find_bluetooth_port_macos()
            if not port:
                print("❌ No Bluetooth serial port found")
                return False
        else:
            print(f"❌ Unsupported OS: {self.os_type}")
            return False
        
        # Open serial connection
        try:
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=BLUETOOTH_BAUDRATE,
                timeout=1,
                write_timeout=1
            )
            self.connected = True
            print(f"✅ Connected to rover on {port}")
            
            # Start receive thread
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            if self.on_connected:
                self.on_connected()
            
            return True
            
        except Exception as e:
            print(f"❌ Serial connection error: {e}")
            return False
    
    def find_bluetooth_com_port(self, mac_address: str) -> Optional[str]:
        """Find Bluetooth COM port on Windows"""
        try:
            # Use PowerShell to find Bluetooth COM port
            ps_script = f'''
            $mac = "{mac_address}" -replace ":", ""
            $comPorts = Get-WmiObject -Query "SELECT * FROM Win32_SerialPort WHERE Name LIKE '%Bluetooth%'"
            foreach ($port in $comPorts) {{
                $portName = $port.Name
                $deviceId = $port.DeviceID
                if ($portName -match $mac -or $deviceId -match $mac) {{
                    Write-Output $deviceId
                    break
                }}
            }}
            '''
            result = subprocess.run(['powershell', '-Command', ps_script],
                                   capture_output=True, text=True, timeout=10)
            
            if result.stdout.strip():
                return result.stdout.strip()
                
        except Exception as e:
            print(f"⚠️  COM port detection error: {e}")
        
        # Fallback: try common COM ports
        for i in range(1, 10):
            port = f'COM{i}'
            try:
                test = serial.Serial(port, timeout=0.5)
                test.close()
                return port
            except:
                continue
        
        return None
    
    def find_bluetooth_port_macos(self) -> Optional[str]:
        """Find Bluetooth serial port on macOS"""
        # macOS typically uses /dev/cu.Bluetooth-Incoming-Port or similar
        import glob
        patterns = ['/dev/cu.*', '/dev/tty.*']
        for pattern in patterns:
            for port in glob.glob(pattern):
                if 'bluetooth' in port.lower() or 'rfcomm' in port.lower():
                    return port
        return None
    
    def _receive_loop(self):
        """Background thread to receive data from rover"""
        while self.running and self.connected:
            try:
                if self.serial_connection and self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    if line:
                        try:
                            data = json.loads(line)
                            self.last_sensor_data = data
                            if self.on_data_received:
                                self.on_data_received(data)
                        except json.JSONDecodeError:
                            # Handle plain text messages
                            if self.on_data_received:
                                self.on_data_received({'raw': line})
            except Exception as e:
                if self.running:
                    print(f"⚠️  Receive error: {e}")
                    self.connected = False
                    if self.on_disconnected:
                        self.on_disconnected()
            time.sleep(0.01)
    
    def send_command(self, command: Dict[str, Any]) -> bool:
        """Send a command to the rover"""
        if not self.connected or not self.serial_connection:
            print("❌ Not connected to rover")
            return False
        
        try:
            json_data = json.dumps(command) + "\n"
            self.serial_connection.write(json_data.encode('utf-8'))
            return True
        except Exception as e:
            print(f"❌ Send error: {e}")
            self.connected = False
            return False
    
    # ========== Convenience Methods ==========
    
    def set_mode(self, mode: str) -> bool:
        """Set control mode: 'auto' or 'manual'"""
        return self.send_command({'type': 'mode', 'mode': mode})
    
    def set_speed(self, speed: int) -> bool:
        """Set rover speed (0-100)"""
        speed = max(0, min(100, speed))
        return self.send_command({'type': 'speed', 'speed': speed})
    
    def manual_control(self, action: str) -> bool:
        """Send manual control command"""
        valid_actions = ['forward', 'backward', 'left', 'right', 'stop']
        if action not in valid_actions:
            print(f"Invalid action. Use: {valid_actions}")
            return False
        return self.send_command({'type': 'control', 'action': action})
    
    def send_raw_command(self, command: str) -> bool:
        """Send raw command string"""
        return self.send_command({'type': 'command', 'command': command})
    
    def disconnect(self):
        """Disconnect from rover"""
        self.running = False
        self.connected = False
        
        if self.serial_connection:
            try:
                self.serial_connection.close()
            except:
                pass
        
        # Clean up RFCOMM on Linux
        if self.os_type == "Linux":
            subprocess.run(['rfcomm', 'release', '/dev/rfcomm0'], 
                          capture_output=True)
        
        print("🔌 Disconnected from rover")


class RoverControllerGUI:
    """Simple GUI controller for the rover"""
    
    def __init__(self):
        self.rover = PCBluetoothController()
        self.setup_callbacks()
        
    def setup_callbacks(self):
        """Setup callback functions"""
        self.rover.on_data_received = self.on_sensor_data
        self.rover.on_connected = self.on_connected
        self.rover.on_disconnected = self.on_disconnected
    
    def on_sensor_data(self, data):
        """Handle incoming sensor data"""
        print(f"\r📊 Sensor: Dist={data.get('front_distance', '?')}cm | "
              f"Angle={data.get('compass_angle', 0):.0f}° | "
              f"Mode={'AUTO' if data.get('auto_mode') else 'MANUAL'}", end="")
    
    def on_connected(self):
        print("\n✅ Rover connected and ready!")
    
    def on_disconnected(self):
        print("\n⚠️  Rover disconnected!")
    
    def console_control(self):
        """Interactive console control interface"""
        print("\n" + "="*50)
        print("   ROVER CONTROL INTERFACE")
        print("="*50)
        print("\nCommands:")
        print("  m auto      - Switch to AUTO mode")
        print("  m manual    - Switch to MANUAL mode")
        print("  speed N     - Set speed to N% (0-100)")
        print("  f           - Move forward (manual mode)")
        print("  b           - Move backward (manual mode)")
        print("  l           - Turn left (manual mode)")
        print("  r           - Turn right (manual mode)")
        print("  s           - Stop")
        print("  status      - Show rover status")
        print("  q           - Quit")
        print("-"*50)
        
        while True:
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 'f':
                    self.rover.manual_control('forward')
                elif cmd == 'b':
                    self.rover.manual_control('backward')
                elif cmd == 'l':
                    self.rover.manual_control('left')
                elif cmd == 'r':
                    self.rover.manual_control('right')
                elif cmd == 's':
                    self.rover.manual_control('stop')
                elif cmd.startswith('speed'):
                    parts = cmd.split()
                    if len(parts) == 2:
                        try:
                            speed = int(parts[1])
                            self.rover.set_speed(speed)
                        except ValueError:
                            print("Invalid speed value")
                elif cmd == 'm auto':
                    self.rover.set_mode('auto')
                elif cmd == 'm manual':
                    self.rover.set_mode('manual')
                elif cmd == 'status':
                    if self.rover.last_sensor_data:
                        data = self.rover.last_sensor_data
                        print(f"\n  Distance: {data.get('front_distance', '?')} cm")
                        print(f"  Compass: {data.get('compass_angle', 0):.1f}°")
                        print(f"  Speed: {data.get('motor_speed', 0)}%")
                        print(f"  Mode: {'AUTO' if data.get('auto_mode') else 'MANUAL'}")
                        print(f"  IR Sensors: {data.get('ir_sensors', {})}")
                    else:
                        print("  No data received yet")
                else:
                    print("Unknown command")
                    
            except KeyboardInterrupt:
                break
        
        self.rover.disconnect()
        print("\nGoodbye!")


def main():
    """Main entry point"""
    print("="*60)
    print("   PC BLUETOOTH CONTROLLER FOR RASPBERRY PI ROVER")
    print("="*60)
    
    # Configuration
    print("\n📝 Configuration:")
    print(f"   Target MAC: {ROVER_MAC_ADDRESS}")
    print(f"   Target Name: {ROVER_NAME}")
    print("\n   To change MAC address, edit ROVER_MAC_ADDRESS variable")
    print("   Or run: python pc_controller.py --mac XX:XX:XX:XX:XX:XX")
    print()
    
    # Parse command line args
    import sys
    mac_address = ROVER_MAC_ADDRESS
    if len(sys.argv) > 2 and sys.argv[1] == '--mac':
        mac_address = sys.argv[2]
    
    # Create controller
    controller = PCBluetoothController(mac_address=mac_address)
    
    # Connect to rover
    if not controller.connect():
        print("\n❌ Failed to connect to rover")
        print("\nTroubleshooting:")
        print("  1. Make sure Raspberry Pi is powered on")
        print("  2. Check that rover is running (sudo python3 main.py)")
        print("  3. Verify Bluetooth is enabled on both devices")
        print("  4. Try pairing manually first:")
        print("     - On Linux: bluetoothctl pair <MAC>")
        print("     - On Windows: Add Bluetooth device in Settings")
        print("  5. Update ROVER_MAC_ADDRESS with the correct MAC")
        return
    
    # Start GUI/Console control
    gui = RoverControllerGUI()
    gui.rover = controller
    gui.setup_callbacks()
    gui.console_control()


if __name__ == "__main__":
    main()