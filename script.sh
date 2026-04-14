#!/bin/bash

# ============================================
# Rover Bluetooth Auto-Start Script
# ============================================

# Log file for debugging
LOG_FILE="/home/raspberrypi/rover_boot.log"
exec 1> >(tee -a "$LOG_FILE")
exec 2>&1

echo "========================================="
echo "Rover Startup Script Started: $(date)"
echo "========================================="

# Wait for system to fully boot
sleep 10

# ============================================
# 1. Configure Bluetooth for Auto-Connection
# ============================================
echo "Configuring Bluetooth..."

# Kill any existing bluetooth agents
sudo pkill -f bluetoothd 2>/dev/null
sleep 2

# Start bluetooth daemon with serial profile support
sudo bluetoothd -C &
sleep 2

# Add Serial Port Profile
sudo sdptool add SP

# Configure bluetoothctl for auto-discovery and pairing
expect << EOF
set timeout 10
spawn bluetoothctl
send "agent on\r"
send "default-agent\r"
send "discoverable on\r"
send "pairable on\r"
send "discoverable-timeout 0\r"
send "pairable-timeout 0\r"
send "scan on\r"
sleep 2
                                                                             [ Read 140 lines ]
^G Help          ^O Write Out     ^F Where Is      ^K Cut           ^T Execute       ^C Location      M-U Undo         M-A Set Mark     M-] To Bracket   M-B Previous
^X Exit      