#!/usr/bin/env python3
"""
Simple BLE Scanner - Prints all nearby BLE device payloads

Installation:
    pip install bleak

Usage:
    python ble_scanner.py
"""

import asyncio
import json
from pathlib import Path
from bleak import BleakScanner
from datetime import datetime


OUTPUT_FILE = Path("ble_devices.json")
device_store = {}


def _load_existing_store():
    """Load existing device store from JSON file if it exists."""
    global device_store
    if OUTPUT_FILE.exists():
        try:
            with OUTPUT_FILE.open("r", encoding="utf-8") as file:
                loaded = json.load(file)
                if isinstance(loaded, dict):
                    device_store = loaded
        except (json.JSONDecodeError, OSError):
            device_store = {}


def _save_store():
    """Persist current device store to JSON file."""
    with OUTPUT_FILE.open("w", encoding="utf-8") as file:
        json.dump(device_store, file, indent=2)


def _build_payload(advertisement_data):
    """Build serializable payload details from advertisement data."""
    manufacturer_payload = {
        f"0x{company_id:04x}": data.hex()
        for company_id, data in advertisement_data.manufacturer_data.items()
    }
    service_payload = {
        uuid: data.hex()
        for uuid, data in advertisement_data.service_data.items()
    }

    return {
        "manufacturer_data": manufacturer_payload,
        "service_data": service_payload,
        "local_name": advertisement_data.local_name,
        "service_uuids": advertisement_data.service_uuids,
        "tx_power": advertisement_data.tx_power,
    }

def simple_callback(device, advertisement_data):
    """Simple callback to print device data"""
    
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    print(f"\n{'='*70}")
    print(f"[{timestamp}] Device Found!")
    print(f"{'='*70}")
    print(f"📱 Name: {device.name if device.name else 'Unknown'}")
    print(f"🔗 Address: {device.address}")
    print(f"📶 RSSI: {advertisement_data.rssi} dBm")

    device_store[device.address] = {
        "name": device.name if device.name else "Unknown",
        "address": device.address,
        "rssi": advertisement_data.rssi,
        "payload": _build_payload(advertisement_data),
        "last_seen": timestamp,
    }
    _save_store()
    
    # Print manufacturer data
    if advertisement_data.manufacturer_data:
        print(f"\n📦 Manufacturer Data:")
        for company_id, data in advertisement_data.manufacturer_data.items():
            print(f"   Company ID: 0x{company_id:04x}")
            print(f"   Data (hex): {data.hex()}")
            print(f"   Data (str): {data.decode('utf-8', errors='ignore')}")
    
    # Print service data
    if advertisement_data.service_data:
        print(f"\n🔧 Service Data:")
        for uuid, data in advertisement_data.service_data.items():
            print(f"   UUID: {uuid}")
            print(f"   Data (hex): {data.hex()}")
            print(f"   Data (str): {data.decode('utf-8', errors='ignore')}")
    
    # Print local name (if in advertisement)
    if advertisement_data.local_name:
        print(f"\n🏷️  Local Name: {advertisement_data.local_name}")
    
    # Print service UUIDs
    if advertisement_data.service_uuids:
        print(f"\n📋 Service UUIDs:")
        for uuid in advertisement_data.service_uuids:
            print(f"   - {uuid}")
    
    # Print tx power
    if advertisement_data.tx_power is not None:
        print(f"\n⚡ TX Power: {advertisement_data.tx_power} dBm")
    
    print(f"{'='*70}\n")

async def scan():
    """Start scanning for BLE devices"""
    _load_existing_store()
    print("🔍 Scanning for BLE devices...")
    print("Press Ctrl+C to stop\n")
    
    scanner = BleakScanner(detection_callback=simple_callback)
    
    try:
        await scanner.start()
        await asyncio.sleep(3600)  # Scan for 1 hour (or until Ctrl+C)
    except KeyboardInterrupt:
        print("\n🛑 Stopping scan...")
    finally:
        await scanner.stop()

if __name__ == "__main__":
    asyncio.run(scan())