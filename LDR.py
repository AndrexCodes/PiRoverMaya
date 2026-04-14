#!/usr/bin/env python3
"""
Live LDR Tester (4 channels)

Wiring (BCM / physical):
- OUT 1 -> GPIO 26 (Pin 37)
- OUT 2 -> GPIO 20 (Pin 38)
- OUT 3 -> GPIO 21 (Pin 40)
- OUT 4 -> GPIO 16 (Pin 36)

Run on Raspberry Pi with:
	sudo python3 LDR.py
"""

import time
import RPi.GPIO as GPIO


LDR_PINS = {
	"LDR1": 26,
	"LDR2": 20,
	"LDR3": 21,
	"LDR4": 16,
}


def setup_gpio() -> None:
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)

	for pin in LDR_PINS.values():
		GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def clear_screen() -> None:
	print("\033[2J\033[H", end="")


def read_ldr_state(pin: int) -> tuple[int, str]:
	value = GPIO.input(pin)

	# Common digital LDR modules are active-low:
	# 0 = dark/object detected, 1 = bright/no detection
	label = "DARK/DETECTED" if value == 0 else "BRIGHT/CLEAR"
	return value, label


def run_live_monitor(interval_seconds: float = 0.2) -> None:
	print("Starting live LDR monitor... Press Ctrl+C to stop.")
	time.sleep(1)

	while True:
		clear_screen()
		print("=" * 44)
		print("         LIVE LDR TEST (GPIO INPUTS)")
		print("=" * 44)
		print("OUT1: GPIO26 (Pin37) | OUT2: GPIO20 (Pin38)")
		print("OUT3: GPIO21 (Pin40) | OUT4: GPIO16 (Pin36)")
		print("-" * 44)

		for name, pin in LDR_PINS.items():
			value, label = read_ldr_state(pin)
			print(f"{name:>4} | GPIO {pin:>2} | RAW={value} | {label}")

		print("-" * 44)
		print("Tip: cover/uncover each LDR and watch state changes.")
		time.sleep(interval_seconds)


def main() -> None:
	try:
		setup_gpio()
		run_live_monitor()
	except KeyboardInterrupt:
		print("\nStopping LDR monitor...")
	finally:
		GPIO.cleanup()
		print("GPIO cleaned up.")


if __name__ == "__main__":
	main()
