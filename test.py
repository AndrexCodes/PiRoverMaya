#!/usr/bin/env python3
"""
Raspberry Pi Hardware Test Suite
Tests all sensors and actuators from the pinout table
Run with: sudo python3 hardware_test.py
"""

import RPi.GPIO as GPIO
import time
import sys
import subprocess
from datetime import datetime

# ========== PIN CONFIGURATION (Based on your pinout table) ==========
PINS = {
    # Sensors
    'DHT11': 4,
    'MQ135': 7,  # D0 pin (digital output)
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
    
    # IR Sensors (4x)
    'IR1': 26,
    'IR2': 20,
    'IR3': 21,
    'IR4': 16,
    
    # LEDs (3x)
    'LED1': 24,
    'LED2': 25,
    'LED3': 8,
    
    # Buzzer
    'BUZZER': 27
}

# ========== SETUP ==========
def setup_gpio():
    """Initialize GPIO settings"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    global servo_pwm, motor_pwm_a, motor_pwm_b
    
    # Setup outputs
    outputs = [
        PINS['L298N_IN1'], PINS['L298N_IN2'], 
        PINS['L298N_IN3'], PINS['L298N_IN4'],
        PINS['L298N_ENA'], PINS['L298N_ENB'],
        PINS['LED1'], PINS['LED2'], PINS['LED3'],
        PINS['BUZZER']
    ]
    
    for pin in outputs:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    # Setup inputs
    inputs = [
        PINS['MQ135'],
        PINS['IR1'], PINS['IR2'], PINS['IR3'], PINS['IR4']
    ]
    
    for pin in inputs:
        GPIO.setup(pin, GPIO.IN)
    
    # Setup PWM for servo and motor speed
    global servo_pwm, motor_pwm_a, motor_pwm_b
    servo_pwm = GPIO.PWM(PINS['SERVO'], 50)  # 50Hz for servo
    motor_pwm_a = GPIO.PWM(PINS['L298N_ENA'], 1000)  # 1kHz for motor
    motor_pwm_b = GPIO.PWM(PINS['L298N_ENB'], 1000)
    
    servo_pwm.start(0)
    motor_pwm_a.start(0)
    motor_pwm_b.start(0)

def cleanup():
    """Clean up GPIO and PWM"""
    # Stop PWM objects only if they were created
    for name in ('servo_pwm', 'motor_pwm_a', 'motor_pwm_b'):
        pwm = globals().get(name)
        if pwm is not None:
            try:
                pwm.stop()
            except Exception:
                pass

    try:
        GPIO.cleanup()
    except Exception:
        pass

    print("\n✅ GPIO cleaned up")

# ========== TEST FUNCTIONS ==========
def test_leds():
    """Test 3 LED indicators"""
    print("\n🔆 TESTING LEDS (3x)")
    print("Watch the LEDs - they should blink in sequence")
    
    leds = [PINS['LED1'], PINS['LED2'], PINS['LED3']]
    names = ["LED 1 (Red)", "LED 2 (Green)", "LED 3 (Blue)"]
    
    for i, (led, name) in enumerate(zip(leds, names)):
        print(f"  💡 {name} ON")
        GPIO.output(led, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(led, GPIO.LOW)
        time.sleep(0.5)
    
    print("✅ LED test complete")

def test_buzzer():
    """Test buzzer"""
    print("\n🔊 TESTING BUZZER")
    print("You should hear a beep pattern")
    
    for _ in range(3):
        GPIO.output(PINS['BUZZER'], GPIO.HIGH)
        time.sleep(0.3)
        GPIO.output(PINS['BUZZER'], GPIO.LOW)
        time.sleep(0.2)
    
    print("✅ Buzzer test complete")

def test_ir_sensors():
    """Test all 4 IR sensors"""
    print("\n📡 TESTING IR SENSORS (4x)")
    print("Place an object in front of each sensor")
    
    ir_pins = [
        (PINS['IR1'], "IR Sensor 1"),
        (PINS['IR2'], "IR Sensor 2"),
        (PINS['IR3'], "IR Sensor 3"),
        (PINS['IR4'], "IR Sensor 4")
    ]
    
    for pin, name in ir_pins:
        status = GPIO.input(pin)
        if status == 0:  # Most IR sensors go LOW when object detected
            print(f"  ✅ {name}: OBJECT DETECTED")
        else:
            print(f"  ⚪ {name}: No object")
        time.sleep(0.5)
    
    print("✅ IR sensor test complete")

def test_mq135():
    """Test MQ-135 gas sensor (D0 pin)"""
    print("\n🌫️  TESTING MQ-135 GAS SENSOR")
    print("Expose sensor to gas (lighter without flame, alcohol, etc.)")
    
    for i in range(5):
        status = GPIO.input(PINS['MQ135'])
        if status == 0:  # Gas detected (LOW output)
            print(f"  ⚠️  [{i+1}] GAS DETECTED! Threshold exceeded")
        else:
            print(f"  ✓ [{i+1}] Air quality normal")
        time.sleep(1)
    
    print("✅ MQ-135 test complete")

def test_ultrasonic():
    """Test HC-SR04 ultrasonic sensor"""
    print("\n📏 TESTING ULTRASONIC SENSOR")
    print("Point sensor at an object (wall, hand, etc.)")
    
    trig = PINS['ULTRASONIC_TRIG']
    echo = PINS['ULTRASONIC_ECHO']
    
    try:
        for _ in range(3):
            # Send trigger pulse
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)
            
            # Measure echo
            pulse_start = time.time()
            pulse_end = time.time()
            
            timeout_start = time.time()
            while GPIO.input(echo) == 0:
                pulse_start = time.time()
                if time.time() - timeout_start > 0.1:
                    print("  ⚠️  Timeout - no echo received")
                    break
            
            timeout_start = time.time()
            while GPIO.input(echo) == 1:
                pulse_end = time.time()
                if time.time() - timeout_start > 0.1:
                    print("  ⚠️  Timeout - echo too long")
                    break
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound / 2
            distance = round(distance, 2)
            
            if 2 < distance < 400:
                print(f"  📏 Distance: {distance} cm")
            else:
                print("  ⚠️  Out of range (2-400cm)")
            
            time.sleep(1)
            
    except Exception as e:
        print(f"  ❌ Error: {e}")
    
    print("✅ Ultrasonic test complete")

def test_servo():
    """Test servo motor"""
    print("\n🔄 TESTING SERVO MOTOR")
    print("Watch the servo rotate")
    
    angles = [2.5, 7.5, 12.5]  # Duty cycles for 0, 90, 180 degrees
    
    for duty in angles:
        servo_pwm.ChangeDutyCycle(duty)
        print(f"  Moving to angle: {int((duty-2.5)/10*180)}°")
        time.sleep(1)
    
    servo_pwm.ChangeDutyCycle(0)  # Stop signal
    print("✅ Servo test complete")

def test_motor_driver():
    """Test L298N motor driver"""
    print("\n⚙️  TESTING L298N MOTOR DRIVER")
    print("Connect motors to verify movement")
    
    in1 = PINS['L298N_IN1']
    in2 = PINS['L298N_IN2']
    in3 = PINS['L298N_IN3']
    in4 = PINS['L298N_IN4']
    
    # Test Motor A (forward/backward)
    print("\n  Testing Motor A:")
    print("    Forward (slow)")
    motor_pwm_a.ChangeDutyCycle(50)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    time.sleep(2)
    
    print("    Backward (slow)")
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    time.sleep(2)
    
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    motor_pwm_a.ChangeDutyCycle(0)
    
    # Test Motor B
    print("\n  Testing Motor B:")
    print("    Forward (slow)")
    motor_pwm_b.ChangeDutyCycle(50)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    time.sleep(2)
    
    print("    Backward (slow)")
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    time.sleep(2)
    
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    motor_pwm_b.ChangeDutyCycle(0)
    
    print("✅ Motor driver test complete")

def test_gy271():
    """Test GY-271 compass (I2C)"""
    print("\n🧭 TESTING GY-271 COMPASS")
    
    try:
        # Check if I2C is enabled
        result = subprocess.run(['i2cdetect', '-y', '1'], 
                              capture_output=True, text=True)
        
        if '1e' in result.stdout or '0x1e' in result.stdout:
            print("  ✅ GY-271 detected at address 0x1e")
            print("  📡 I2C communication successful")
        else:
            print("  ⚠️  GY-271 not detected!")
            print("  Check wiring: VCC→3.3V, GND→GND, SDA→GPIO2, SCL→GPIO3")
            print("  Also ensure I2C is enabled: sudo raspi-config")
            
    except Exception as e:
        print(f"  ❌ Error checking I2C: {e}")
        print("  Install i2c-tools: sudo apt-get install i2c-tools")
    
    print("✅ Compass test complete")

def test_dht11():
    """Test DHT11 temperature/humidity sensor"""
    print("\n🌡️  TESTING DHT11 SENSOR")
    
    try:
        import Adafruit_DHT
        
        sensor = Adafruit_DHT.DHT11
        pin = PINS['DHT11']
        
        humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
        
        if humidity is not None and temperature is not None:
            print(f"  🌡️  Temperature: {temperature:.1f}°C")
            print(f"  💧 Humidity: {humidity:.1f}%")
        else:
            print("  ❌ Failed to read from DHT11 sensor")
            print("  Check wiring and pull-up resistor")
            
    except ImportError:
        print("  ❌ Adafruit_DHT library not installed")
        print("  Install with: sudo pip3 install Adafruit_DHT")
    except Exception as e:
        print(f"  ❌ Error: {e}")
    
    print("✅ DHT11 test complete")

# ========== MAIN MENU ==========
def print_menu():
    """Display the test selection menu"""
    print("\n" + "="*50)
    print("   RASPBERRY PI HARDWARE TEST SUITE")
    print("="*50)
    print("\n📋 Available Tests:")
    print("  1. 🔆 LEDs (3x)")
    print("  2. 🔊 Buzzer")
    print("  3. 📡 IR Sensors (4x)")
    print("  4. 🌫️  MQ-135 Gas Sensor")
    print("  5. 📏 Ultrasonic Sensor")
    print("  6. 🔄 Servo Motor")
    print("  7. ⚙️  Motor Driver (L298N)")
    print("  8. 🧭 Compass (GY-271)")
    print("  9. 🌡️  DHT11 Sensor")
    print(" 10. 🎯 RUN ALL TESTS")
    print("  0. ❌ Exit")
    print("-"*50)

def run_all_tests():
    """Execute all tests in sequence"""
    print("\n🎯 RUNNING COMPLETE TEST SUITE")
    print("="*50)
    
    tests = [
        ("LEDs", test_leds),
        ("Buzzer", test_buzzer),
        ("IR Sensors", test_ir_sensors),
        ("MQ-135", test_mq135),
        ("Ultrasonic", test_ultrasonic),
        ("Servo", test_servo),
        ("Motor Driver", test_motor_driver),
        ("Compass", test_gy271),
        ("DHT11", test_dht11)
    ]
    
    for name, test_func in tests:
        print(f"\n▶️  Testing {name}...")
        test_func()
        time.sleep(1)
    
    print("\n" + "="*50)
    print("✅ ALL TESTS COMPLETE!")
    print("="*50)

# ========== MAIN ==========
def main():
    """Main program entry point"""
    try:
        setup_gpio()
        print("\n🔧 GPIO initialized successfully")
        
        while True:
            print_menu()
            choice = input("\n👉 Select test (0-10): ").strip()
            
            if choice == '0':
                print("\n👋 Exiting...")
                break
            elif choice == '1':
                test_leds()
            elif choice == '2':
                test_buzzer()
            elif choice == '3':
                test_ir_sensors()
            elif choice == '4':
                test_mq135()
            elif choice == '5':
                test_ultrasonic()
            elif choice == '6':
                test_servo()
            elif choice == '7':
                test_motor_driver()
            elif choice == '8':
                test_gy271()
            elif choice == '9':
                test_dht11()
            elif choice == '10':
                run_all_tests()
            else:
                print("❌ Invalid choice. Please select 0-10")
            
            input("\n⏎ Press Enter to continue...")
            
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    finally:
        cleanup()

if __name__ == "__main__":
    main()