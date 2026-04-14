#!/usr/bin/env python3
import subprocess
import sys
import os

def run_cmd(cmd, check=True):
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if check and result.returncode != 0:
        print(f"Error: {result.stderr}")
        sys.exit(1)
    return result.stdout

def create_hotspot_nm():
    ssid = input("Enter Hotspot SSID: ").strip() or "Pi-Hotspot"
    password = input("Enter Password (min 8 chars): ").strip()
    if len(password) < 8:
        password = "raspberry123"

    print("Creating hotspot with NetworkManager...")
    
    # Delete existing connection if it exists to avoid conflicts
    run_cmd("nmcli connection delete Hotspot 2>/dev/null", check=False)
    
    # Create the hotspot on wlan0 (your Wi-Fi adapter)
    run_cmd(f"""nmcli connection add type wifi ifname wlan0 con-name Hotspot autoconnect yes ssid {ssid}""")
    run_cmd(f"nmcli connection modify Hotspot 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared")
    run_cmd(f"nmcli connection modify Hotspot wifi-sec.key-mgmt wpa-psk")
    run_cmd(f"nmcli connection modify Hotspot wifi-sec.psk '{password}'")
    
    # Bring the hotspot up
    run_cmd("nmcli connection up Hotspot")
    
    print(f"\n✓ Hotspot '{ssid}' created! Password: {password}")
    print("Your USB gadget SSH connection should remain active on usb0")

def main():
    if os.geteuid() != 0:
        print("Please run with sudo")
        sys.exit(1)
    
    choice = input("1. Create Hotspot (Safe for USB Gadget)\n2. Connect to WiFi\nChoice: ")
    if choice == "1":
        create_hotspot_nm()
    elif choice == "2":
        # Add standard nmcli wifi connect logic here if needed
        print("Use: nmcli device wifi connect 'SSID' password 'PASS'")

if __name__ == "__main__":
    main()
