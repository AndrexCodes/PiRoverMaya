#!/usr/bin/env python3
"""
PiRover BLE Scanner - Optimized for your custom beacon
"""

import asyncio
import json
from pathlib import Path
from datetime import datetime
from bleak import BleakScanner


OUTPUT_FILE = Path("ble_devices.json")
device_store = {}


def _load_existing_store():
    global device_store
    if OUTPUT_FILE.exists():
        try:
            with OUTPUT_FILE.open("r", encoding="utf-8") as f:
                loaded = json.load(f)
                if isinstance(loaded, dict):
                    device_store = loaded
        except Exception:
            device_store = {}


def _save_store():
    with OUTPUT_FILE.open("w", encoding="utf-8") as f:
        json.dump(device_store, f, indent=2)


def decode_pirover_payload(data: bytes) -> str:
    """Decode your custom DxxSxxMxxIxxxx payload"""
    try:
        text = data.decode('utf-8', errors='ignore')
        if text.startswith('D'):
            return text
        return text
    except:
        return data.hex()


def simple_callback(device, advertisement_data):

    """Enhanced callback focused on PiRover"""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    # Filter to show only relevant devices
    name = device.name or "Unknown"
    if "PiRover" not in name and "pirov" not in name.lower():
        return  # Optional: remove this line if you want ALL devices

    print(f"\n{'='*80}")
    print(f"[{timestamp}] 🛠️  PiRover DETECTED!")
    print(f"{'='*80}")
    print(f"📱 Name       : {device.name}")
    print(f"🔗 Address    : {device.address}")
    print(f"📶 RSSI       : {advertisement_data.rssi} dBm")

    # Save to JSON
    device_store[device.address] = {
        "name": device.name,
        "address": device.address,
        "rssi": advertisement_data.rssi,
        "payload": {
            "manufacturer_data": {
                f"0x{cid:04x}": data.hex() 
                for cid, data in advertisement_data.manufacturer_data.items()
            },
            "local_name": advertisement_data.local_name,
            "service_data": {
                str(uuid): data.hex() 
                for uuid, data in advertisement_data.service_data.items()
            }
        },
        "last_seen": timestamp,
    }
    _save_store()

    # === Manufacturer Data (Most Important for PiRover) ===
    if advertisement_data.manufacturer_data:
        print(f"\n📦 Manufacturer Data:")
        for company_id, data in advertisement_data.manufacturer_data.items():
            print(f"   Company ID : 0x{company_id:04x}")
            print(f"   Raw Hex    : {data.hex()}")
            decoded = decode_pirover_payload(data)
            print(f"   Decoded    : {decoded}")

    # Local Name
    if advertisement_data.local_name:
        print(f"🏷️  Local Name : {advertisement_data.local_name}")

    # Service Data (if any)
    if advertisement_data.service_data:
        print(f"\n🔧 Service Data:")
        for uuid, data in advertisement_data.service_data.items():
            print(f"   UUID : {uuid}")
            print(f"   Data : {data.hex()}")

    print(f"{'='*80}\n")


async def scan():
    _load_existing_store()
    print("🔍 Starting PiRover BLE Scanner...")
    print("Looking for 'PiRover' beacon with custom payload...\n")
    print("Press Ctrl+C to stop\n")

    scanner = BleakScanner(detection_callback=simple_callback)

    try:
        await scanner.start()
        await asyncio.sleep(3600)   # Run for 1 hour
    except KeyboardInterrupt:
        print("\n🛑 Stopping scanner...")
    finally:
        await scanner.stop()
        print("✅ Scan stopped.")


if __name__ == "__main__":
    asyncio.run(scan())