#!/usr/bin/env python3
"""
Intelligent Navigation System for Raspberry Pi
Run with: sudo python3 main.py
"""

import RPi.GPIO as GPIO
import time
import math
import threading
from collections import deque

# ========== PIN CONFIGURATION ==========
PINS = {
    # Sensors
    'DHT11': 4,
    'MQ135': 7,
    'GY271_SDA': 2,
    'GY271_SCL': 3,
    
    # Actuators
    'L298N_IN1': 17,
    'L298N_IN2': 18,
    'L298N_IN3': 22,
    'L298N_IN4': 23,
    'L298N_ENA': 12,
    'L298N_ENB': 13,
    
    'ULTRASONIC_TRIG': 5,
    'ULTRASONIC_ECHO': 6,
    
    'SERVO': 19,
    
    # IR Sensors (4x at 45 deg angles)
    'IR_TOP_LEFT': 26,     # 45 deg top-left
    'IR_TOP_RIGHT': 20,    # 45 deg top-right
    'IR_BOTTOM_LEFT': 21,  # 45 deg bottom-left
    'IR_BOTTOM_RIGHT': 16, # 45 deg bottom-right
    
    # LEDs
    'LED1': 24,
    'LED2': 25,
    'LED3': 8,
    
    # Buzzer
    'BUZZER': 27
}

# ========== GLOBAL SPEED CONFIGURATION ==========
# ⚙️ ADJUST THIS VALUE TO CHANGE ROVER SPEED (0-100)
# 0 = stopped, 30 = slow, 60 = medium, 100 = maximum
ROVER_SPEED = 40  # Default: 60% speed

# Speed presets for different conditions
SPEED_PRESETS = {
    'CRUISE': 60,      # Normal cruising speed
    'SLOW': 40,        # Slow speed (obstacle near)
    'TURN': 50,        # Turning speed
    'BACKUP': 35,      # Backing up speed
    'MIN': 20,         # Minimum operational speed
    'MAX': 85          # Maximum safe speed
}

# ========== NAVIGATION CONSTANTS ==========
OBSTACLE_THRESHOLD = 30  # cm (start slowing down)
SAFE_DISTANCE = 40       # cm
CRITICAL_DISTANCE = 15   # cm (emergency stop)
TURN_DURATION = 0.8      # seconds

# Speed will be dynamically adjusted based on obstacles
# but ROVER_SPEED sets the base maximum speed

# Direction mapping for IR sensors (angles relative to robot)
# Sensors positioned at 45 deg angles
SENSOR_ANGLES = {
    'IR_TOP_LEFT': 135,      # 135 deg (top-left quadrant)
    'IR_TOP_RIGHT': 45,      # 45 deg (top-right quadrant)
    'IR_BOTTOM_LEFT': 225,   # 225 deg (bottom-left quadrant)
    'IR_BOTTOM_RIGHT': 315   # 315 deg (bottom-right quadrant)
}

class Compass:
    """GY-271 Compass sensor interface"""
    def __init__(self):
        self.reference_angle = 0
        self.current_angle = 0
        self.calibrated = False
    
    def initialize(self):
        """Initialize compass and set reference"""
        try:
            import smbus2
            self.bus = smbus2.SMBus(1)
            
            # Initialize HMC5883L
            self.bus.write_byte_data(0x1e, 0x02, 0x00)  # Continuous mode
            
            time.sleep(0.1)
            
            # Get initial reading for reference
            self.reference_angle = self.get_angle()
            self.calibrated = True
            print(f"🧭 Compass initialized. Reference angle: {self.reference_angle:.1f}°")
            return True
        except ImportError:
            print("⚠️  smbus2 not installed. Compass disabled.")
            return False
        except Exception as e:
            print(f"⚠️  Compass error: {e}")
            return False
    
    def get_angle(self):
        """Get current angle in degrees (0-360)"""
        if not self.calibrated:
            return 0
        
        try:
            # Read from HMC5883L
            data = self.bus.read_i2c_block_data(0x1e, 0x03, 6)
            
            # Convert to signed integers
            x = (data[0] << 8) | data[1]
            if x > 32767:
                x -= 65536
            z = (data[4] << 8) | data[5]
            if z > 32767:
                z -= 65536
            
            # Calculate angle
            angle = math.atan2(z, x) * 180 / math.pi
            if angle < 0:
                angle += 360
            
            self.current_angle = angle
            return angle
        except:
            return self.current_angle
    
    def get_relative_angle(self):
        """Get angle relative to reference (0 = reference direction)"""
        raw_angle = self.get_angle()
        relative = raw_angle - self.reference_angle
        if relative < 0:
            relative += 360
        return relative

class MotorController:
    """L298N Motor controller"""
    def __init__(self, base_speed=ROVER_SPEED):
        self.running = False
        self.base_speed = base_speed
        self.current_speed = base_speed
    
    def set_speed(self, speed):
        """Dynamically change motor speed (0-100)"""
        self.current_speed = max(0, min(100, speed))
        if self.running:
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
        print(f"  ⚡ Speed set to {self.current_speed}%")
    
    def get_speed(self):
        """Get current motor speed"""
        return self.current_speed
    
    def setup(self):
        """Setup motor pins and PWM"""
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
        print(f"✅ Motor controller initialized (Base speed: {self.base_speed}%)")
    
    def stop(self):
        """Stop all motors"""
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
    
    def forward(self, speed=None):
        """Move forward"""
        use_speed = speed if speed is not None else self.current_speed
        use_speed = max(0, min(100, use_speed))
        self.pwm_a.ChangeDutyCycle(use_speed)
        self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def backward(self, speed=None):
        """Move backward"""
        use_speed = speed if speed is not None else SPEED_PRESETS['BACKUP']
        use_speed = max(0, min(100, use_speed))
        self.pwm_a.ChangeDutyCycle(use_speed)
        self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def turn_left(self, speed=None):
        """Turn left (counter-rotate)"""
        use_speed = speed if speed is not None else SPEED_PRESETS['TURN']
        use_speed = max(0, min(100, use_speed))
        self.pwm_a.ChangeDutyCycle(use_speed)
        self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN2'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN3'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN4'], GPIO.LOW)
    
    def turn_right(self, speed=None):
        """Turn right (counter-rotate)"""
        use_speed = speed if speed is not None else SPEED_PRESETS['TURN']
        use_speed = max(0, min(100, use_speed))
        self.pwm_a.ChangeDutyCycle(use_speed)
        self.pwm_b.ChangeDutyCycle(use_speed)
        GPIO.output(PINS['L298N_IN1'], GPIO.HIGH)
        GPIO.output(PINS['L298N_IN2'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN3'], GPIO.LOW)
        GPIO.output(PINS['L298N_IN4'], GPIO.HIGH)
    
    def cleanup(self):
        """Cleanup motor controller"""
        if self.running:
            self.stop()
            self.pwm_a.stop()
            self.pwm_b.stop()

class ObstacleDetector:
    """Obstacle detection using ultrasonic and IR sensors"""
    def __init__(self):
        self.obstacle_history = deque(maxlen=5)
    
    def setup(self):
        """Setup ultrasonic pins"""
        GPIO.setup(PINS['ULTRASONIC_TRIG'], GPIO.OUT)
        GPIO.setup(PINS['ULTRASONIC_ECHO'], GPIO.IN)
        GPIO.output(PINS['ULTRASONIC_TRIG'], GPIO.LOW)
        
        # Setup IR sensors (4x at 45 deg angles)
        ir_pins = [PINS['IR_TOP_LEFT'], PINS['IR_TOP_RIGHT'], 
                   PINS['IR_BOTTOM_LEFT'], PINS['IR_BOTTOM_RIGHT']]
        for pin in ir_pins:
            GPIO.setup(pin, GPIO.IN)
        
        print("✅ Obstacle detector initialized")
    
    def get_ultrasonic_distance(self):
        """Get distance from ultrasonic sensor"""
        trig = PINS['ULTRASONIC_TRIG']
        echo = PINS['ULTRASONIC_ECHO']
        
        try:
            # Send trigger pulse
            GPIO.output(trig, False)
            time.sleep(0.05)
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)
            
            # Wait for echo
            timeout = time.time() + 0.1
            while GPIO.input(echo) == 0 and time.time() < timeout:
                pulse_start = time.time()
            
            if time.time() >= timeout:
                return 999  # No obstacle
            
            timeout = time.time() + 0.1
            while GPIO.input(echo) == 1 and time.time() < timeout:
                pulse_end = time.time()
            
            if time.time() >= timeout:
                return 999
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            
            return distance if 2 < distance < 400 else 999
        except:
            return 999
    
    def get_ir_readings(self):
        """Get all IR sensor readings"""
        readings = {
            'top_left': GPIO.input(PINS['IR_TOP_LEFT']),
            'top_right': GPIO.input(PINS['IR_TOP_RIGHT']),
            'bottom_left': GPIO.input(PINS['IR_BOTTOM_LEFT']),
            'bottom_right': GPIO.input(PINS['IR_BOTTOM_RIGHT'])
        }
        # IR sensors return 0 when object detected
        return {k: (v == 0) for k, v in readings.items()}
    
    def analyze_obstacles(self):
        """Analyze obstacle situation and return best action"""
        front_distance = self.get_ultrasonic_distance()
        ir_readings = self.get_ir_readings()
        
        # Store history
        self.obstacle_history.append({
            'front': front_distance,
            'ir': ir_readings
        })
        
        # Critical obstacle in front
        if front_distance < CRITICAL_DISTANCE:
            return 'STOP_AND_BACK'
        
        # Obstacle in front
        if front_distance < OBSTACLE_THRESHOLD:
            # Check IR sensors for escape path
            if not ir_readings['top_left'] and not ir_readings['bottom_left']:
                return 'TURN_LEFT'
            elif not ir_readings['top_right'] and not ir_readings['bottom_right']:
                return 'TURN_RIGHT'
            elif not ir_readings['top_left']:
                return 'TURN_LEFT'
            elif not ir_readings['top_right']:
                return 'TURN_RIGHT'
            else:
                return 'TURN_AROUND'
        
        # No immediate obstacle
        return 'FORWARD'
    
    def get_recommended_speed(self, front_distance):
        """Get recommended speed based on obstacle distance"""
        if front_distance < CRITICAL_DISTANCE:
            return 0  # Stop
        elif front_distance < OBSTACLE_THRESHOLD:
            # Slow down as obstacle gets closer
            # Linear interpolation between SLOW and MIN speeds
            ratio = (front_distance - CRITICAL_DISTANCE) / (OBSTACLE_THRESHOLD - CRITICAL_DISTANCE)
            speed = SPEED_PRESETS['MIN'] + (SPEED_PRESETS['SLOW'] - SPEED_PRESETS['MIN']) * ratio
            return int(speed)
        elif front_distance < SAFE_DISTANCE:
            # Slightly reduced speed
            ratio = (front_distance - OBSTACLE_THRESHOLD) / (SAFE_DISTANCE - OBSTACLE_THRESHOLD)
            speed = SPEED_PRESETS['SLOW'] + (ROVER_SPEED - SPEED_PRESETS['SLOW']) * ratio
            return int(speed)
        else:
            # Full speed
            return ROVER_SPEED

class NavigationSystem:
    """Main navigation system"""
    def __init__(self):
        self.compass = Compass()
        self.motors = MotorController(ROVER_SPEED)
        self.detector = ObstacleDetector()
        self.running = False
        self.navigation_thread = None
        
        # LED pins
        self.led_pins = [PINS['LED1'], PINS['LED2'], PINS['LED3']]
        
        # Buzzer pin
        self.buzzer_pin = PINS['BUZZER']
        
        # Speed control
        self.current_speed = ROVER_SPEED
    
    def setup_indicators(self):
        """Setup LED and buzzer pins"""
        for pin in self.led_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def led_on(self, led_num):
        """Turn on specific LED (1-3)"""
        if 1 <= led_num <= 3:
            GPIO.output(self.led_pins[led_num - 1], GPIO.HIGH)
    
    def led_off(self, led_num):
        """Turn off specific LED"""
        if 1 <= led_num <= 3:
            GPIO.output(self.led_pins[led_num - 1], GPIO.LOW)
    
    def led_all_off(self):
        """Turn off all LEDs"""
        for pin in self.led_pins:
            GPIO.output(pin, GPIO.LOW)
    
    def beep(self, duration=0.1):
        """Make a beep sound"""
        GPIO.output(self.buzzer_pin, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def alert_pattern(self):
        """Alert pattern for obstacles"""
        for _ in range(2):
            self.beep(0.05)
            time.sleep(0.05)
    
    def initialize(self):
        """Initialize all systems"""
        print("\n🚀 Initializing Navigation System...")
        print(f"⚙️  Global speed setting: {ROVER_SPEED}%")
        print(f"   - Cruise speed: {SPEED_PRESETS['CRUISE']}%")
        print(f"   - Slow speed: {SPEED_PRESETS['SLOW']}%")
        print(f"   - Turn speed: {SPEED_PRESETS['TURN']}%")
        print(f"   - Backup speed: {SPEED_PRESETS['BACKUP']}%")
        
        # Setup GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup indicators
        self.setup_indicators()
        
        # Turn on LED 1
        self.led_on(1)
        print("🔆 LED 1 ON - System starting")
        
        # Wait 5 seconds
        print("⏳ Waiting 5 seconds before starting navigation...")
        for i in range(5):
            time.sleep(1)
            print(f"   {5 - i} seconds remaining...")
        
        # Initialize components
        self.motors.setup()
        self.detector.setup()
        
        # Initialize compass
        if not self.compass.initialize():
            print("⚠️  Compass not available - using dead reckoning")
        
        # Beep to indicate ready
        self.beep(0.3)
        time.sleep(0.1)
        self.beep(0.3)
        
        print("\n✅ Navigation System Ready!")
        print("📡 Obstacle detection active")
        print("🧭 Navigation starting...")
        
        return True
    
    def execute_action(self, action, front_distance):
        """Execute navigation action based on obstacle analysis"""
        if action == 'FORWARD':
            # Dynamic speed based on obstacle distance
            recommended_speed = self.detector.get_recommended_speed(front_distance)
            if recommended_speed != self.current_speed:
                self.current_speed = recommended_speed
                self.motors.set_speed(self.current_speed)
            
            self.motors.forward(self.current_speed)
            self.led_off(2)
            self.led_off(3)
            
        elif action == 'TURN_LEFT':
            print("  ↪️  Turning left to avoid obstacle")
            self.led_on(2)
            self.led_off(3)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.turn_left()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            self.alert_pattern()
            # Reset speed after turn
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            
        elif action == 'TURN_RIGHT':
            print("  ↩️  Turning right to avoid obstacle")
            self.led_off(2)
            self.led_on(3)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.turn_right()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            self.alert_pattern()
            # Reset speed after turn
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            
        elif action == 'TURN_AROUND':
            print("  🔄 Turning around - path blocked")
            self.led_on(2)
            self.led_on(3)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.turn_left()
            time.sleep(TURN_DURATION * 2)
            self.motors.stop()
            for _ in range(3):
                self.beep(0.1)
                time.sleep(0.1)
            # Reset speed
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
            
        elif action == 'STOP_AND_BACK':
            print("  ⚠️  CRITICAL! Backing up...")
            for _ in range(3):
                self.beep(0.2)
                time.sleep(0.1)
            self.motors.stop()
            time.sleep(0.1)
            self.motors.backward()
            time.sleep(1.0)
            self.motors.stop()
            time.sleep(0.1)
            # Random turn after backing up
            import random
            if random.choice([True, False]):
                self.motors.turn_left()
            else:
                self.motors.turn_right()
            time.sleep(TURN_DURATION)
            self.motors.stop()
            # Reset speed
            self.current_speed = ROVER_SPEED
            self.motors.set_speed(self.current_speed)
    
    def navigation_loop(self):
        """Main navigation loop"""
        self.running = True
        
        while self.running:
            # Analyze obstacles
            action = self.detector.analyze_obstacles()
            
            # Get front distance for display
            front_distance = self.detector.get_ultrasonic_distance()
            
            # Display status
            if front_distance < 100:
                status = f"Front: {front_distance:.0f}cm"
            else:
                status = "Front: Clear"
            
            # Show current speed
            status += f" | Speed: {self.current_speed}%"
            
            ir_readings = self.detector.get_ir_readings()
            if any(ir_readings.values()):
                ir_active = [k for k, v in ir_readings.items() if v]
                status += f" | IR: {', '.join(ir_active)}"
            
            print(f"\r📍 {status} | Action: {action}", end="", flush=True)
            
            # Execute action
            self.execute_action(action, front_distance)
            
            # Small delay for stability
            time.sleep(0.05)
    
    def start(self):
        """Start navigation system"""
        self.navigation_thread = threading.Thread(target=self.navigation_loop)
        self.navigation_thread.daemon = True
        self.navigation_thread.start()
        
        print("\n🎯 Navigation Active!")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping navigation...")
            self.stop()
    
    def stop(self):
        """Stop navigation system"""
        self.running = False
        if self.navigation_thread:
            self.navigation_thread.join(timeout=2)
        
        self.motors.stop()
        self.led_all_off()
        
        # Cleanup
        self.motors.cleanup()
        GPIO.cleanup()
        
        print("✅ Navigation system stopped")
        print("👋 Goodbye!")

def main():
    """Main entry point"""
    print("="*60)
    print("   INTELLIGENT NAVIGATION SYSTEM")
    print("   - 4x IR Sensors at 45° angles")
    print("   - Ultrasonic Sensor (forward)")
    print("   - Compass reference set at boot")
    print("="*60)
    print(f"\n⚙️  Current global speed setting: {ROVER_SPEED}%")
    print("   To change speed, edit ROVER_SPEED variable at top of file")
    print("   Range: 0 (stop) to 100 (maximum)")
    print()
    
    nav_system = NavigationSystem()
    
    try:
        if nav_system.initialize():
            nav_system.start()
    except Exception as e:
        print(f"\n❌ System error: {e}")
        nav_system.stop()

if __name__ == "__main__":
    main()
