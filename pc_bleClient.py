#!/usr/bin/env python3
"""
PiRover Dedicated Scanner - Optimized for BlueZ / hcitool beacons
"""

import asyncio
import json
from pathlib import Path
from datetime import datetime
from bleak import BleakScanner, AdvertisementData, BLEDevice


OUTPUT_FILE = Path("ble_devices.json")
device_store = {}


def decode_pirover_data(data: bytes):
    try:
        text = data.decode('utf-8', errors='ignore')
        if text.startswith('D') and 'S' in text:
            return text
        return text
    except:
        return data.hex()


def callback(device: BLEDevice, advertisement: AdvertisementData):
    # if not device.name or "PiRover" not in device.name:
        # return  # Only show PiRover (remove this line if you want everything)

    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    print(f"\n{'='*85}")
    print(f"[{timestamp}] ✅ PiRover DETECTED on PC!")
    print(f"{'='*85}")
    print(f"Name      : {device.name}")
    print(f"Address   : {device.address}")
    print(f"RSSI      : {advertisement.rssi} dBm")

    # Manufacturer Data (this is where your payload lives)
    if advertisement.manufacturer_data:
        print(f"\n📦 Manufacturer Data:")
        for cid, data in advertisement.manufacturer_data.items():
            print(f"   Company ID : 0x{cid:04x}")
            print(f"   Raw        : {data.hex()}")
            print(f"   Payload    : {decode_pirover_data(data)}")

    # Save to file
    device_store[device.address] = {
        "name": device.name,
        "address": device.address,
        "rssi": advertisement.rssi,
        "payload": decode_pirover_data(next(iter(advertisement.manufacturer_data.values()), b'')),
        "last_seen": timestamp
    }
    with OUTPUT_FILE.open("w", encoding="utf-8") as f:
        json.dump(device_store, f, indent=2)

    print(f"{'='*85}\n")


async def main():
    global device_store
    print("🔍 Starting Advanced PiRover Scanner on PC...")
    print("Waiting for PiRover beacon...\n")

    # Important options for better detection
    scanner = BleakScanner(
        detection_callback=callback,
        scanning_mode="active",           # Very important!
        # adapter="hci0"                  # Uncomment if you have multiple adapters
    )

    try:
        await scanner.start()
        print("📡 Scanning... (Active mode)")
        await asyncio.sleep(3600)         # Run for 1 hour
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user")
    finally:
        await scanner.stop()
        print("✅ Scanner stopped.")


if __name__ == "__main__":
    asyncio.run(main())