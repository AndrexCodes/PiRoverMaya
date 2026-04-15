#!/usr/bin/env python3
"""
Windows-Optimized PiRover Scanner using Bleak
"""
import asyncio
import json
from pathlib import Path
from datetime import datetime
from bleak import BleakScanner, AdvertisementData, BLEDevice

OUTPUT_FILE = Path("ble_devices.json")

def decode_payload(data: bytes):
    try:
        text = data.decode("utf-8", errors="ignore")
        return text if text.startswith("D") else data.hex()
    except:
        return data.hex()

async def main():
    print("🔍 Windows PiRover Scanner (Bleak + WinRT)")
    print("Scanning for 60 seconds... (Active mode for better detection)\n")

    def callback(device: BLEDevice, adv: AdvertisementData):
        # if "PiRover" not in (device.name or "") and device.name != "raspberrypi":
            # return

        print(f"\n{'='*90}")
        print(f"[{datetime.now().strftime('%H:%M:%S')}] ✅ PiRover FOUND!")
        print(f"{'='*90}")
        print(f"Name     : {device.name or 'raspberrypi (cached)'}")
        print(f"Address  : {device.address}")
        print(f"RSSI     : {adv.rssi} dBm")

        if adv.manufacturer_data:
            for cid, data in adv.manufacturer_data.items():
                print(f"📦 Manufacturer Data 0x{cid:04x}: {decode_payload(data)}")

        # Save
        with OUTPUT_FILE.open("w", encoding="utf-8") as f:
            json.dump({
                device.address: {
                    "name": device.name or "raspberrypi",
                    "address": device.address,
                    "rssi": adv.rssi,
                    "payload": decode_payload(next(iter(adv.manufacturer_data.values()), b"")),
                    "last_seen": datetime.now().strftime("%H:%M:%S")
                }
            }, f, indent=2)

    scanner = BleakScanner(
        detection_callback=callback,
        scanning_mode="active",      # Important for Windows
    )

    await scanner.start()
    await asyncio.sleep(60)          # Scan for 60 seconds
    await scanner.stop()
    print("\n✅ Scan finished.")

if __name__ == "__main__":
    asyncio.run(main())