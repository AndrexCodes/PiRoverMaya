#!/usr/bin/env python3
import subprocess
import sys
import os
import time

def run_command(command, sudo=True):
    """Run a shell command and return output"""
    try:
        if sudo:
            command = f"sudo {command}"
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        return result.returncode, result.stdout, result.stderr
    except Exception as e:
        return 1, "", str(e)

def connect_to_wifi():
    """Connect to an existing WiFi network"""
    print("\n=== Connect to WiFi Network ===")
    
    # Scan for available networks
    print("Scanning for WiFi networks...")
    run_command("iwlist wlan0 scan | grep 'ESSID'", sudo=True)
    
    # Get network details
    ssid = input("Enter WiFi SSID: ").strip()
    password = input("Enter WiFi password: ").strip()
    
    # Create wpa_supplicant configuration
    wpa_conf = f'''ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=US

network={{
    ssid="{ssid}"
    psk="{password}"
    key_mgmt=WPA-PSK
}}'''
    
    # Write configuration
    with open("/tmp/wpa_supplicant.conf", "w") as f:
        f.write(wpa_conf)
    
    # Apply configuration
    print(f"Connecting to {ssid}...")
    run_command("cp /tmp/wpa_supplicant.conf /etc/wpa_supplicant/")
    run_command("chmod 600 /etc/wpa_supplicant/wpa_supplicant.conf")
    run_command("wpa_cli -i wlan0 reconfigure")
    
    # Restart networking
    run_command("systemctl restart networking")
    run_command("systemctl restart wpa_supplicant")
    
    # Get IP address
    time.sleep(5)
    _, ip_output, _ = run_command("hostname -I", sudo=False)
    
    print(f"\n✓ Connected to {ssid}!")
    print(f"IP Address: {ip_output.strip()}")
    
    # Test connection
    print("Testing internet connection...")
    ret, _, _ = run_command("ping -c 3 google.com", sudo=False)
    if ret == 0:
        print("✓ Internet connection successful!")
    else:
        print("⚠ Connected to network but no internet access")

def create_hotspot():
    """Create a WiFi hotspot from Raspberry Pi"""
    print("\n=== Create WiFi Hotspot ===")
    
    # Get hotspot details
    ssid = input("Enter Hotspot SSID (name): ").strip()
    if not ssid:
        ssid = "RaspberryPi-Hotspot"
    
    password = input("Enter Hotspot password (min 8 characters): ").strip()
    if len(password) < 8:
        print("Password too short! Using default: raspberry123")
        password = "raspberry123"
    
    # Install required packages
    print("Installing required packages...")
    run_command("apt-get update")
    run_command("apt-get install -y hostapd dnsmasq")
    
    # Stop services
    run_command("systemctl stop hostapd")
    run_command("systemctl stop dnsmasq")
    
    # Configure static IP for wlan0
    dhcpcd_conf = '''interface wlan0
    static ip_address=192.168.4.1/24
    nohook wpa_supplicant'''
    
    with open("/tmp/dhcpcd.conf", "w") as f:
        f.write(dhcpcd_conf)
    run_command("cat /tmp/dhcpcd.conf >> /etc/dhcpcd.conf")
    
    # Restart dhcpcd
    run_command("systemctl restart dhcpcd")
    time.sleep(2)
    
    # Configure hostapd
    hostapd_conf = f'''interface=wlan0
driver=nl80211
ssid={ssid}
hw_mode=g
channel=7
wmm_enabled=1
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase={password}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP'''
    
    with open("/tmp/hostapd.conf", "w") as f:
        f.write(hostapd_conf)
    run_command("cp /tmp/hostapd.conf /etc/hostapd/hostapd.conf")
    
    # Configure dnsmasq
    dnsmasq_conf = '''interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
dhcp-option=3,192.168.4.1
dhcp-option=6,192.168.4.1
server=8.8.8.8
log-queries
log-dhcp
listen-address=127.0.0.1'''
    
    with open("/tmp/dnsmasq.conf", "w") as f:
        f.write(dnsmasq_conf)
    
    # Backup and replace dnsmasq config
    run_command("mv /etc/dnsmasq.conf /etc/dnsmasq.conf.bak")
    run_command("cp /tmp/dnsmasq.conf /etc/dnsmasq.conf")
    
    # Enable IP forwarding
    with open("/tmp/sysctl.conf", "w") as f:
        f.write("net.ipv4.ip_forward=1")
    run_command("cat /tmp/sysctl.conf >> /etc/sysctl.conf")
    run_command("sysctl -p")
    
    # Configure NAT with iptables
    run_command("iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE")
    run_command("iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT")
    run_command("iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT")
    run_command("sh -c 'iptables-save > /etc/iptables.ipv4.nat'")
    
    # Add iptables restore to rc.local
    run_command("sed -i '/^exit 0/d' /etc/rc.local")
    run_command("echo 'iptables-restore < /etc/iptables.ipv4.nat' >> /etc/rc.local")
    run_command("echo 'exit 0' >> /etc/rc.local")
    
    # Start services
    run_command("systemctl unmask hostapd")
    run_command("systemctl enable hostapd")
    run_command("systemctl enable dnsmasq")
    run_command("systemctl start hostapd")
    run_command("systemctl start dnsmasq")
    
    print(f"\n✓ Hotspot '{ssid}' created successfully!")
    print(f"Password: {password}")
    print("IP Address: 192.168.4.1")
    print("\nYou can now connect other devices to your Raspberry Pi hotspot")

def check_wifi_interface():
    """Check if WiFi interface exists"""
    ret, output, _ = run_command("iwconfig 2>&1 | grep -o 'wlan[0-9]'", sudo=False)
    if not output.strip():
        print("Error: No WiFi interface found!")
        print("Make sure your Raspberry Pi has a WiFi adapter")
        return False
    return True

def main():
    """Main menu"""
    print("=" * 50)
    print("Raspberry Pi WiFi Manager")
    print("=" * 50)
    
    # Check if running as root
    if os.geteuid() != 0:
        print("This script requires root privileges!")
        print("Please run with: sudo python3 wifi_manager.py")
        sys.exit(1)
    
    # Check WiFi interface
    if not check_wifi_interface():
        sys.exit(1)
    
    while True:
        print("\nOptions:")
        print("1. Connect to WiFi network")
        print("2. Create WiFi hotspot")
        print("3. Exit")
        
        choice = input("\nEnter your choice (1-3): ").strip()
        
        if choice == "1":
            connect_to_wifi()
            break
        elif choice == "2":
            create_hotspot()
            break
        elif choice == "3":
            print("Goodbye!")
            sys.exit(0)
        else:
            print("Invalid choice! Please enter 1, 2, or 3")

if __name__ == "__main__":
    main()