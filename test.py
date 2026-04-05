#!/usr/bin/env python3
"""
Raspberry Pi Hardware Test Suite - FIXED VERSION
Run with: sudo python3 test.py
"""

import RPi.GPIO as GPIO
import time
import sys
import subprocess

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

# Global PWM variables
servo_pwm = None
motor_pwm_a = None
motor_pwm_b = None

# ========== SETUP ==========
def setup_gpio():
    """Initialize GPIO settings"""
    global servo_pwm, motor_pwm_a, motor_pwm_b
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Setup outputs (only pins that are strictly outputs)
    outputs = [
        PINS['L298N_IN1'], PINS['L298N_IN2'], 
        PINS['L298N_IN3'], PINS['L298N_IN4'],
        PINS['LED1'], PINS['LED2'], PINS['LED3'],
        PINS['BUZZER']
    ]
    
    for pin in outputs:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    # Setup PWM pins as outputs
    GPIO.setup(PINS['L298N_ENA'], GPIO.OUT)
    GPIO.setup(PINS['L298N_ENB'], GPIO.OUT)
    GPIO.setup(PINS['SERVO'], GPIO.OUT)
    
    # Setup inputs
    inputs = [
        PINS['MQ135'],
        PINS['IR1'], PINS['IR2'], PINS['IR3'], PINS['IR4']
    ]
    
    for pin in inputs:
        GPIO.setup(pin, GPIO.IN)
    
    # Setup ultrasonic pins
    GPIO.setup(PINS['ULTRASONIC_TRIG'], GPIO.OUT)
    GPIO.setup(PINS['ULTRASONIC_ECHO'], GPIO.IN)
    GPIO.output(PINS['ULTRASONIC_TRIG'], GPIO.LOW)
    
    # Initialize PWM
    try:
        servo_pwm = GPIO.PWM(PINS['SERVO'], 50)
        servo_pwm.start(0)
        time.sleep(0.5)
        servo_pwm.ChangeDutyCycle(0)
        
        motor_pwm_a = GPIO.PWM(PINS['L298N_ENA'], 1000)
        motor_pwm_b = GPIO.PWM(PINS['L298N_ENB'], 1000)
        motor_pwm_a.start(0)
        motor_pwm_b.start(0)
        
        print("✅ GPIO and PWM initialized successfully")
    except Exception as e:
        print(f"⚠️  PWM setup warning: {e}")

def cleanup():
    """Clean up GPIO and PWM"""
    global servo_pwm, motor_pwm_a, motor_pwm_b
    
    # Stop PWM
    for pwm in [servo_pwm, motor_pwm_a, motor_pwm_b]:
        if pwm is not None:
            try:
                pwm.stop()
            except:
                pass
    
    # Cleanup GPIO
    try:
        GPIO.cleanup()
    except:
        pass
    
    print("\n✅ GPIO cleaned up")

# ========== TEST FUNCTIONS ==========
def test_leds():
    """Test 3 LED indicators"""
    print("\n🔆 TESTING LEDS (3x)")
    print("Watch the LEDs - they should blink in sequence")
    
    leds = [PINS['LED1'], PINS['LED2'], PINS['LED3']]
    names = ["LED 1", "LED 2", "LED 3"]
    
    for led, name in zip(leds, names):
        print(f"  💡 {name} ON")
        GPIO.output(led, GPIO.HIGH)
        time.sleep(0.8)
        GPIO.output(led, GPIO.LOW)
        time.sleep(0.3)
    
    print("✅ LED test complete")

def test_buzzer():
    """Test buzzer"""
    print("\n🔊 TESTING BUZZER")
    print("You should hear a beep pattern")
    
    for _ in range(3):
        GPIO.output(PINS['BUZZER'], GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(PINS['BUZZER'], GPIO.LOW)
        time.sleep(0.1)
    
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
        try:
            status = GPIO.input(pin)
            if status == 0:
                print(f"  ✅ {name}: OBJECT DETECTED")
            else:
                print(f"  ⚪ {name}: No object")
        except Exception as e:
            print(f"  ❌ {name}: Error - {e}")
        time.sleep(0.3)
    
    print("✅ IR sensor test complete")

def test_mq135():
    """Test MQ-135 gas sensor"""
    print("\n🌫️  TESTING MQ-135 GAS SENSOR")
    print("Expose sensor to gas or alcohol vapor")
    
    detected = 0
    for i in range(5):
        try:
            status = GPIO.input(PINS['MQ135'])
            if status == 0:
                print(f"  ⚠️  [{i+1}] GAS DETECTED!")
                detected += 1
            else:
                print(f"  ✓ [{i+1}] Air normal")
        except Exception as e:
            print(f"  ❌ Error: {e}")
        time.sleep(0.5)
    
    if detected > 0:
        print(f"  📊 Gas detected in {detected}/5 readings")
    print("✅ MQ-135 test complete")

def test_ultrasonic():
    """Test ultrasonic sensor"""
    print("\n📏 TESTING ULTRASONIC SENSOR")
    print("Hold your hand ~30cm in front of the sensor")
    
    trig = PINS['ULTRASONIC_TRIG']
    echo = PINS['ULTRASONIC_ECHO']
    
    for reading in range(3):
        try:
            # Ensure trigger is low
            GPIO.output(trig, False)
            time.sleep(0.05)
            
            # Send trigger pulse
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)
            
            # Wait for echo with timeout
            timeout = time.time() + 0.1
            while GPIO.input(echo) == 0 and time.time() < timeout:
                pulse_start = time.time()
            
            if time.time() >= timeout:
                print(f"  [{reading+1}] ⚠️  No echo - check wiring")
                continue
            
            timeout = time.time() + 0.1
            while GPIO.input(echo) == 1 and time.time() < timeout:
                pulse_end = time.time()
            
            if time.time() >= timeout:
                print(f"  [{reading+1}] ⚠️  Echo timeout")
                continue
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            
            if 2 < distance < 400:
                print(f"  [{reading+1}] 📏 Distance: {distance:.1f} cm")
            else:
                print(f"  [{reading+1}] ⚠️  Out of range: {distance:.1f} cm")
            
        except Exception as e:
            print(f"  [{reading+1}] ❌ Error: {e}")
        
        time.sleep(0.5)
    
    print("✅ Ultrasonic test complete")

def test_servo():
    """Test servo motor"""
    global servo_pwm
    
    if servo_pwm is None:
        print("\n❌ Servo not initialized")
        return
    
    print("\n🔄 TESTING SERVO MOTOR")
    print("Watch the servo rotate")
    
    positions = [
        (2.5, "0° (left)"),
        (7.5, "90° (center)"),
        (12.5, "180° (right)"),
        (7.5, "90° (center)"),
        (2.5, "0° (left)")
    ]
    
    for duty, desc in positions:
        try:
            print(f"  Moving to {desc}")
            servo_pwm.ChangeDutyCycle(duty)
            time.sleep(0.8)
        except Exception as e:
            print(f"  ❌ Error: {e}")
    
    try:
        servo_pwm.ChangeDutyCycle(0)
    except:
        pass
    
    print("✅ Servo test complete")

def test_motor_driver():
    """Test motor driver"""
    global motor_pwm_a, motor_pwm_b
    
    if motor_pwm_a is None or motor_pwm_b is None:
        print("\n❌ Motor PWM not initialized")
        return
    
    print("\n⚙️  TESTING MOTOR DRIVER")
    print("Connect motors to see movement")
    
    in1, in2 = PINS['L298N_IN1'], PINS['L298N_IN2']
    in3, in4 = PINS['L298N_IN3'], PINS['L298N_IN4']
    
    try:
        # Test Motor A
        print("\n  Motor A - Forward")
        motor_pwm_a.ChangeDutyCycle(60)
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        time.sleep(1.5)
        
        print("  Motor A - Stop")
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        motor_pwm_a.ChangeDutyCycle(0)
        time.sleep(0.5)
        
        print("  Motor A - Reverse")
        motor_pwm_a.ChangeDutyCycle(60)
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        time.sleep(1.5)
        
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        motor_pwm_a.ChangeDutyCycle(0)
        
        # Test Motor B
        print("\n  Motor B - Forward")
        motor_pwm_b.ChangeDutyCycle(60)
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)
        time.sleep(1.5)
        
        print("  Motor B - Stop")
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
        motor_pwm_b.ChangeDutyCycle(0)
        time.sleep(0.5)
        
        print("  Motor B - Reverse")
        motor_pwm_b.ChangeDutyCycle(60)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.HIGH)
        time.sleep(1.5)
        
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
        motor_pwm_b.ChangeDutyCycle(0)
        
    except Exception as e:
        print(f"  ❌ Error: {e}")
    
    print("✅ Motor driver test complete")

def test_gy271():
    """Test GY-271 compass"""
    print("\n🧭 TESTING GY-271 COMPASS")
    
    try:
        result = subprocess.run(['i2cdetect', '-y', '1'], 
                              capture_output=True, text=True, timeout=5)
        
        if '1e' in result.stdout or '0x1e' in result.stdout:
            print("  ✅ GY-271 detected at address 0x1e")
        else:
            print("  ⚠️  GY-271 not detected")
            print("  Check: VCC→3.3V, GND→GND, SDA→GPIO2, SCL→GPIO3")
            
    except subprocess.TimeoutExpired:
        print("  ❌ I2C scan timeout")
    except FileNotFoundError:
        print("  ❌ Install: sudo apt-get install i2c-tools")
    except Exception as e:
        print(f"  ❌ Error: {e}")
    
    print("✅ Compass test complete")

def test_dht11():
    """Test DHT11 sensor"""
    print("\n🌡️  TESTING DHT11 SENSOR")
    
    try:
        import Adafruit_DHT
        
        humidity, temperature = Adafruit_DHT.read_retry(
            Adafruit_DHT.DHT11, 
            PINS['DHT11']
        )
        
        if humidity is not None and temperature is not None:
            print(f"  🌡️  Temperature: {temperature:.1f}°C")
            print(f"  💧 Humidity: {humidity:.1f}%")
        else:
            print("  ❌ Failed to read DHT11")
            print("  Check wiring and 10kΩ pull-up resistor")
            
    except ImportError:
        print("  ❌ Install: sudo pip3 install Adafruit_DHT")
    except Exception as e:
        print(f"  ❌ Error: {e}")
    
    print("✅ DHT11 test complete")

# ========== MENU ==========
def print_menu():
    print("\n" + "="*50)
    print("   HARDWARE TEST SUITE")
    print("="*50)
    print("\n 1. 🔆 LEDs")
    print(" 2. 🔊 Buzzer")
    print(" 3. 📡 IR Sensors")
    print(" 4. 🌫️  MQ-135")
    print(" 5. 📏 Ultrasonic")
    print(" 6. 🔄 Servo")
    print(" 7. ⚙️  Motor Driver")
    print(" 8. 🧭 Compass")
    print(" 9. 🌡️  DHT11")
    print("10. 🎯 RUN ALL")
    print(" 0. ❌ Exit")
    print("-"*50)

def run_all():
    print("\n🎯 RUNNING ALL TESTS\n")
    tests = [test_leds, test_buzzer, test_ir_sensors, test_mq135,
             test_ultrasonic, test_servo, test_motor_driver, 
             test_gy271, test_dht11]
    
    for test in tests:
        test()
        time.sleep(0.5)
    
    print("\n✅ ALL TESTS COMPLETE!")

# ========== MAIN ==========
def main():
    try:
        setup_gpio()
        
        while True:
            print_menu()
            choice = input("\n👉 Select (0-10): ").strip()
            
            if choice == '0':
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
                run_all()
            else:
                print("❌ Invalid choice")
            
            if choice != '0':
                input("\nPress Enter to continue...")
                
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        cleanup()

if __name__ == "__main__":
    main()