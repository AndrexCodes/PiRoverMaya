#!/bin/bash

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
expect eof
EOF

echo "✅ Bluetooth configured and discoverable"

# ============================================
# 2. Remove any existing RFCOMM binding
# ============================================
sudo rfcomm release /dev/rfcomm0 2>/dev/null
sudo rfcomm release /dev/rfcomm1 2>/dev/null

# ============================================
# 3. Wait for PC connection (blocking)
# ============================================
echo "========================================="
echo "Rover is now discoverable as 'raspberrypi'"
echo "Waiting for PC to connect..."
echo "Pair and connect from your PC"
echo "========================================="

CONNECTED=0
ATTEMPTS=0

while [ $CONNECTED -eq 0 ] && [ $ATTEMPTS -lt 60 ]; do  # Wait up to 5 minutes
    # Check if any Bluetooth device is connected
    CONNECTED_DEVICES=$(bluetoothctl devices Connected | wc -l)
    
    if [ $CONNECTED_DEVICES -gt 0 ]; then
        CONNECTED=1
        echo "✅ Device connected!"
        
        # Get the MAC address of connected device
        CONNECTED_MAC=$(bluetoothctl devices Connected | head -1 | awk '{print $2}')
        echo "Connected to: $CONNECTED_MAC"
        
        # Bind RFCOMM to the connected device
        echo "Binding RFCOMM to $CONNECTED_MAC"
        sudo rfcomm bind /dev/rfcomm0 $CONNECTED_MAC 1
        
        # Verify binding
        if [ -e /dev/rfcomm0 ]; then
            echo "✅ RFCOMM bound successfully to /dev/rfcomm0"
        else
            echo "⚠️  RFCOMM binding failed, but continuing..."
        fi
        break
    fi
    
    # Show discovery status
    if [ $((ATTEMPTS % 6)) -eq 0 ]; then  # Every 30 seconds
        echo "Waiting for connection... (${ATTEMPTS}/60 attempts)"
        bluetoothctl show | grep "Discoverable: yes"
    fi
    
    ATTEMPTS=$((ATTEMPTS + 1))
    sleep 5
done

if [ $CONNECTED -eq 0 ]; then
    echo "⚠️  No device connected after 5 minutes"
    echo "Starting rover anyway (will retry connection in background)"
fi

# ============================================
# 4. Activate virtual environment and run script
# ============================================
cd /home/raspberrypi/Desktop/PiRoverMaya

if [ -d "venv" ]; then
    source venv/bin/activate
    echo "✅ Virtual environment activated"
else
    echo "⚠️  Virtual environment not found"
fi

echo "========================================="
echo "Starting Python navigation script..."
echo "========================================="

# Run the script with sudo (required for GPIO)
sudo python3 main.py

# If script exits, cleanup
echo "========================================="
echo "Rover script stopped: $(date)"
echo "========================================="

# Cleanup RFCOMM
sudo rfcomm release /dev/rfcomm0 2>/dev/null

# Keep terminal open if running manually
read -p "Press Enter to exit..."
