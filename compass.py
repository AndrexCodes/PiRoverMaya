#!/usr/bin/env python3
"""
GY-271 Digital Compass Live Visualizer
Displays heading, direction, and a live compass rose
Run with: python3 compass_visualizer.py
"""

import smbus2
import time
import math
import sys
from datetime import datetime

# ========== GY-271 (HMC5883L) REGISTERS ==========
HMC5883L_ADDR = 0x1E
HMC5883L_REG_CONFIG_A = 0x00
HMC5883L_REG_CONFIG_B = 0x01
HMC5883L_REG_MODE = 0x02
HMC5883L_REG_DATA_X_MSB = 0x03
HMC5883L_REG_DATA_X_LSB = 0x04
HMC5883L_REG_DATA_Z_MSB = 0x05
HMC5883L_REG_DATA_Z_LSB = 0x06
HMC5883L_REG_DATA_Y_MSB = 0x07
HMC5883L_REG_DATA_Y_LSB = 0x08
HMC5883L_REG_STATUS = 0x09

# ========== CALIBRATION OFFSETS (adjust these!) ==========
# Run calibration mode first to find your offsets
CALIBRATION_OFFSETS = {
    'x_offset': 0,
    'y_offset': 0,
    'z_offset': 0
}

# ========== COMPASS CLASS ==========
class HMC5883L:
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)
        self.initialize_sensor()
        
    def initialize_sensor(self):
        """Initialize the HMC5883L sensor"""
        try:
            # Configuration Register A
            # 0x70 = 8 samples averaged, 15Hz output rate, normal measurement
            self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70)
            
            # Configuration Register B (gain = 1090 LSB/Gauss)
            self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 0x20)
            
            # Mode Register (continuous measurement mode)
            self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00)
            
            print("✅ GY-271 initialized successfully")
            return True
        except Exception as e:
            print(f"❌ Failed to initialize GY-271: {e}")
            return False
    
    def read_raw_data(self):
        """Read raw 16-bit values from all axes"""
        try:
            # Read X axis (MSB + LSB)
            x_msb = self.bus.read_byte_data(HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB)
            x_lsb = self.bus.read_byte_data(HMC5883L_ADDR, HMC5883L_REG_DATA_X_LSB)
            x = (x_msb << 8) | x_lsb
            if x > 32767:
                x = x - 65536
            
            # Read Z axis (MSB + LSB)
            z_msb = self.bus.read_byte_data(HMC5883L_ADDR, HMC5883L_REG_DATA_Z_MSB)
            z_lsb = self.bus.read_byte_data(HMC5883L_ADDR, HMC5883L_REG_DATA_Z_LSB)
            z = (z_msb << 8) | z_lsb
            if z > 32767:
                z = z - 65536
            
            # Read Y axis (MSB + LSB)
            y_msb = self.bus.read_byte_data(HMC5883L_ADDR, HMC5883L_REG_DATA_Y_MSB)
            y_lsb = self.bus.read_byte_data(HMC5883L_ADDR, HMC5883L_REG_DATA_Y_LSB)
            y = (y_msb << 8) | y_lsb
            if y > 32767:
                y = y - 65536
            
            return (x, z, y)  # Note: X, Z, Y order matches sensor orientation
            
        except Exception as e:
            print(f"❌ Error reading sensor: {e}")
            return None
    
    def get_heading(self, x, y, z):
        """Calculate heading in degrees from raw magnetometer data"""
        # Apply calibration offsets
        x_adjusted = x - CALIBRATION_OFFSETS['x_offset']
        y_adjusted = y - CALIBRATION_OFFSETS['y_offset']
        
        # Calculate heading (atan2 returns angle in radians)
        heading_rad = math.atan2(y_adjusted, x_adjusted)
        
        # Convert to degrees and adjust for declination
        heading_deg = math.degrees(heading_rad)
        
        # Normalize to 0-360°
        if heading_deg < 0:
            heading_deg = 360 + heading_deg
        
        return heading_deg
    
    def get_direction_name(self, heading):
        """Convert heading to cardinal/intercardinal direction"""
        directions = [
            (0, "N"), (22.5, "NNE"), (45, "NE"), (67.5, "ENE"),
            (90, "E"), (112.5, "ESE"), (135, "SE"), (157.5, "SSE"),
            (180, "S"), (202.5, "SSW"), (225, "SW"), (247.5, "WSW"),
            (270, "W"), (292.5, "WNW"), (315, "NW"), (337.5, "NNW")
        ]
        
        for angle, name in directions:
            if heading >= angle:
                closest_angle = angle
                closest_name = name
        
        return closest_name
    
    def calibrate(self, duration=10):
        """Auto-calibration: rotate sensor in a figure-8 pattern"""
        print("\n🧭 CALIBRATION MODE")
        print("Rotate the sensor in a figure-8 pattern for", duration, "seconds...")
        print("Starting in 3 seconds...")
        
        for i in range(3, 0, -1):
            print(f"{i}...")
            time.sleep(1)
        
        x_min = x_max = y_min = y_max = None
        start_time = time.time()
        samples = 0
        
        while time.time() - start_time < duration:
            data = self.read_raw_data()
            if data:
                x, z, y = data
                samples += 1
                
                if x_min is None or x < x_min: x_min = x
                if x_max is None or x > x_max: x_max = x
                if y_min is None or y < y_min: y_min = y
                if y_max is None or y > y_max: y_max = y
                
                # Progress indicator
                progress = int((time.time() - start_time) / duration * 40)
                sys.stdout.write(f"\r{'█' * progress}{'░' * (40-progress)} {int((time.time() - start_time)/duration*100)}%")
                sys.stdout.flush()
                time.sleep(0.05)
        
        print("\n\n📊 Calibration complete!")
        
        if samples > 0:
            # Calculate offsets (center of the min/max range)
            x_offset = (x_max + x_min) // 2
            y_offset = (y_max + y_min) // 2
            
            print(f"  X range: {x_min} to {x_max} (offset: {x_offset})")
            print(f"  Y range: {y_min} to {y_max} (offset: {y_offset})")
            print(f"  Samples collected: {samples}")
            
            return x_offset, y_offset
        else:
            print("  ❌ No data collected!")
            return 0, 0

# ========== VISUALIZATION FUNCTIONS ==========
def clear_screen():
    """Clear terminal screen"""
    print("\033[2J\033[H", end="")

def draw_compass_rose(heading):
    """Draw an ASCII compass rose with current heading highlighted"""
    # Compass directions in ASCII art
    points = [
        (0, "N", "↑"),
        (45, "NE", "↗"),
        (90, "E", "→"),
        (135, "SE", "↘"),
        (180, "S", "↓"),
        (225, "SW", "↙"),
        (270, "W", "←"),
        (315, "NW", "↖")
    ]
    
    # Find which direction is currently pointing up (north)
    # For simplicity, we'll just show a static compass with the needle
    needle_angle = heading
    needle_rad = math.radians(needle_angle)
    
    # Calculate needle position (length 10)
    needle_x = int(math.sin(needle_rad) * 10)
    needle_y = int(-math.cos(needle_rad) * 10)  # Negative because screen Y goes down
    
    # Create compass canvas
    canvas = [[' ' for _ in range(25)] for _ in range(21)]
    center_x, center_y = 12, 10
    
    # Draw circle
    radius = 9
    for angle in range(0, 360, 10):
        rad = math.radians(angle)
        x = int(center_x + math.cos(rad) * radius)
        y = int(center_y + math.sin(rad) * radius)
        if 0 <= x < 25 and 0 <= y < 21:
            canvas[y][x] = '·'
    
    # Draw cardinal points
    for deg, name, symbol in points:
        rad = math.radians(deg)
        x = int(center_x + math.cos(rad) * (radius - 1))
        y = int(center_y + math.sin(rad) * (radius - 1))
        if 0 <= x < 25 and 0 <= y < 21:
            canvas[y][x] = name[0]
    
    # Draw center
    canvas[center_y][center_x] = '●'
    
    # Draw needle
    needle_end_x = center_x + needle_x
    needle_end_y = center_y + needle_y
    if 0 <= needle_end_x < 25 and 0 <= needle_end_y < 21:
        canvas[needle_end_y][needle_end_x] = '▶'
    
    # Draw small dot at 180° opposite
    opposite_x = center_x - needle_x
    opposite_y = center_y - needle_y
    if 0 <= opposite_x < 25 and 0 <= opposite_y < 21:
        if canvas[opposite_y][opposite_x] == ' ':
            canvas[opposite_y][opposite_x] = '○'
    
    # Print canvas
    print("┌" + "─" * 25 + "┐")
    for row in canvas:
        print("│" + "".join(row) + "│")
    print("└" + "─" * 25 + "┘")

def draw_big_arrow(heading):
    """Draw a large arrow pointing in the current direction"""
    # Determine which octant the heading is in
    if heading < 22.5 or heading >= 337.5:
        arrow = "   ↑   \n  N↑N  \n ↑ N ↑ \n↑  N  ↑"
    elif heading < 67.5:
        arrow = "   ↗   \n  ↗ ↗  \n ↗   ↗ \n↗     ↗"
    elif heading < 112.5:
        arrow = "   →   \n  E→E  \n → E → \n→  E  →"
    elif heading < 157.5:
        arrow = "   ↘   \n  ↘ ↘  \n ↘   ↘ \n↘     ↘"
    elif heading < 202.5:
        arrow = "   ↓   \n  S↓S  \n ↓ S ↓ \n↓  S  ↓"
    elif heading < 247.5:
        arrow = "   ↙   \n  ↙ ↙  \n ↙   ↙ \n↙     ↙"
    elif heading < 292.5:
        arrow = "   ←   \n  W←W  \n ← W ← \n←  W  ←"
    else:
        arrow = "   ↖   \n  ↖ ↖  \n ↖   ↖ \n↖     ↖"
    
    print("\n" + arrow)

def display_heading_style(heading, direction):
    """Display heading with nice formatting"""
    # Create a circular progress bar
    bar_length = 40
    position = int(heading / 360 * bar_length)
    
    bar = "[" + "=" * position + " " * (bar_length - position) + "]"
    
    print("\n" + "═" * 50)
    print(f"  🧭 HEADING: {heading:06.1f}°  |  {direction:3s}")
    print("═" * 50)
    print(f"  {bar}")
    
    # Compass rose at bottom of bar
    tick_positions = [0, 90, 180, 270, 360]
    tick_line = "  "
    for i in range(bar_length + 2):
        if i in [int(p/360*bar_length) for p in tick_positions]:
            tick_line += "|"
        else:
            tick_line += " "
    print(tick_line)
    print("  0°" + " " * 35 + "90°" + " " * 33 + "180°" + " " * 33 + "270°" + " " * 33 + "360°")

def plot_live_data(samples=100):
    """Real-time plotting using terminal"""
    import curses
    import threading
    
    # For simplicity, we'll use a non-curses approach that's easier
    pass

# ========== MAIN VISUALIZER ==========
def run_visualizer():
    """Main live visualizer loop"""
    compass = HMC5883L()
    
    if not compass.initialize_sensor():
        print("\n❌ Cannot continue. Check wiring:")
        print("   - VCC → 3.3V (Pin 1)")
        print("   - GND → GND")
        print("   - SDA → GPIO 2 (Pin 3)")
        print("   - SCL → GPIO 3 (Pin 5)")
        print("\nAlso enable I2C: sudo raspi-config → Interface Options → I2C → Enable")
        return
    
    # Ask for calibration
    print("\n" + "="*50)
    print("   LIVE COMPASS VISUALIZER")
    print("="*50)
    
    calibrate_choice = input("\nRun calibration first? (y/n): ").strip().lower()
    if calibrate_choice == 'y':
        x_off, y_off = compass.calibrate(duration=10)
        CALIBRATION_OFFSETS['x_offset'] = x_off
        CALIBRATION_OFFSETS['y_offset'] = y_off
        
        # Save offsets for future use
        print(f"\n💾 Add these to your code:")
        print(f"   CALIBRATION_OFFSETS = {{'x_offset': {x_off}, 'y_offset': {y_off}, 'z_offset': 0}}")
    
    print("\n🎯 Starting live display... Press Ctrl+C to exit\n")
    time.sleep(2)
    
    try:
        while True:
            # Read sensor data
            data = compass.read_raw_data()
            if data:
                x, z, y = data
                heading = compass.get_heading(x, y, z)
                direction = compass.get_direction_name(heading)
                
                # Clear screen and display
                clear_screen()
                
                print("\n" + "═" * 50)
                print("   🧭 GY-271 DIGITAL COMPASS - LIVE DATA")
                print("═" * 50)
                
                # Raw values
                print(f"\n📊 RAW DATA:")
                print(f"   X: {x:6d}  |  Y: {y:6d}  |  Z: {z:6d}")
                
                # Heading display
                display_heading_style(heading, direction)
                
                # ASCII Compass Rose
                print("\n🗺️  COMPASS ROSE:")
                draw_compass_rose(heading)
                
                # Big arrow
                draw_big_arrow(heading)
                
                # Direction card
                print("\n" + "═" * 50)
                print(f"   📍 DIRECTION: {direction}  |  ANGLE: {heading:.1f}°")
                
                # Additional info
                print(f"\n⏱️  Time: {datetime.now().strftime('%H:%M:%S')}")
                print(f"🔄 Refresh rate: ~10 Hz")
                
                print("\n" + "═" * 50)
                print("Press Ctrl+C to exit")
                
            else:
                print("⚠️  Waiting for sensor data...")
            
            time.sleep(0.1)  # ~10Hz refresh rate
            
    except KeyboardInterrupt:
        print("\n\n👋 Visualizer stopped!")
        print("Exiting...")

def run_simple_mode():
    """Simple text-only mode for low-resource environments"""
    compass = HMC5883L()
    
    if not compass.initialize_sensor():
        return
    
    print("\n📡 SIMPLE COMPASS MODE (Ctrl+C to exit)")
    print("=" * 40)
    
    try:
        while True:
            data = compass.read_raw_data()
            if data:
                x, z, y = data
                heading = compass.get_heading(x, y, z)
                direction = compass.get_direction_name(heading)
                
                # Simple one-line display
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"\r[{timestamp}] 🧭 {heading:06.1f}°  {direction:3s}  |  X:{x:5d} Y:{y:5d} Z:{z:5d}", end="")
                
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n👋 Exited!")

# ========== ENTRY POINT ==========
if __name__ == "__main__":
    # Check if smbus2 is installed
    try:
        import smbus2
    except ImportError:
        print("❌ smbus2 library not installed!")
        print("Install with: sudo pip3 install smbus2")
        sys.exit(1)
    
    print("\n" + "═" * 50)
    print("   GY-271 DIGITAL COMPASS VISUALIZER")
    print("═" * 50)
    print("\nSelect display mode:")
    print("  1. 🎨 Full visualizer (compass rose + arrow)")
    print("  2. 📊 Simple mode (text-only, faster)")
    
    choice = input("\n👉 Choose (1/2): ").strip()
    
    if choice == '2':
        run_simple_mode()
    else:
        run_visualizer()