#!/usr/bin/env python3
"""Quick test - just check if PiRover is broadcasting"""

import asyncio
from bleak import BleakScanner

async def quick_check():
    print("Scanning for 5 seconds...")
    devices = await BleakScanner.discover(timeout=5, return_adv=True)
    
    found = False
    for addr, (device, adv_data) in devices.items():
        if device.name and 'PiRover' in device.name:
            found = True
            print(f"\n✅ PiRover FOUND!")
            print(f"   Address: {addr}")
            print(f"   RSSI: {adv_data.rssi} dBm")
            
            if adv_data.manufacturer_data:
                print(f"\n   Payload data:")
                for company_id, data in adv_data.manufacturer_data.items():
                    print(f"   Hex: {data.hex()}")
                    try:
                        print(f"   Text: {data.decode('utf-8', errors='ignore')}")
                    except:
                        pass
    
    if not found:
        print("\n❌ PiRover not found!")
        print("   Make sure:")
        print("   1. Rover is running: sudo python3 main.py")
        print("   2. Bluetooth is enabled on both devices")
        print("   3. Run: sudo hciconfig hci0 up on rover")

if __name__ == "__main__":
    asyncio.run(quick_check())