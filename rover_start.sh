#!/bin/bash

# ============================================
# Rover Bluetooth Auto-Start Script
# ============================================

LOG_FILE="/home/raspberrypi/rover_boot.log"
exec 1> >(tee -a "$LOG_FILE")
exec 2>&1

echo "========================================="
echo "Rover Startup Script Started: $(date)"
echo "========================================="

sleep 10

# ============================================
# Configure Bluetooth for No-PIN Pairing
# ============================================
echo "Configuring Bluetooth for No-PIN pairing..."

# Configure Bluetooth daemon
sudo sed -i 's/^#*\(DiscoverableTimeout\)=.*/\1=0/' /etc/bluetooth/main.conf 2>/dev/null || echo "DiscoverableTimeout = 0" | sudo tee -a /etc/bluetooth/main.conf
sudo sed -i 's/^#*\(PairableTimeout\)=.*/\1=0/' /etc/bluetooth/main.conf 2>/dev/null || echo "PairableTimeout = 0" | sudo tee -a /etc/bluetooth/main.conf

# Restart Bluetooth
sudo systemctl restart bluetooth
sleep 3

# IMPORTANT: Use a subshell to keep agent alive
(
    # Wait for bluetoothd to be fully ready
    sleep 2
    
    # Set up the agent with NoInputNoOutput
    bluetoothctl << 'EOF'
agent NoInputNoOutput
default-agent
power on
pairable on
pairable-timeout 0
discoverable on
discoverable-timeout 0
EOF
    
    # Keep this subshell alive to maintain the agent
    while true; do
        sleep 30
        # Refresh discoverability if needed
        if ! bluetoothctl show | grep -q "Discoverable: yes"; then
            bluetoothctl discoverable on
        fi
    done
) &

# Give the agent time to register
sleep 3

# Add Serial Port Profile
sudo sdptool add SP

echo "✅ Bluetooth configured - Pairing should NOT require PIN"
echo "========================================="

# ============================================
# Remove existing RFCOMM binding
# ============================================
sudo rfcomm release /dev/rfcomm0 2>/dev/null
sudo rfcomm release /dev/rfcomm1 2>/dev/null

# ============================================
# Wait for PC/Phone connection
# ============================================
echo "Rover is discoverable as 'raspberrypi'"
echo "Pair from your phone - it should NOT ask for a PIN"
echo "========================================="

CONNECTED=0
ATTEMPTS=0

while [ $CONNECTED -eq 0 ] && [ $ATTEMPTS -lt 60 ]; do
    CONNECTED_DEVICES=$(bluetoothctl devices Connected | wc -l)
    
    if [ $CONNECTED_DEVICES -gt 0 ]; then
        CONNECTED=1
        echo "✅ Device connected!"
        CONNECTED_MAC=$(bluetoothctl devices Connected | head -1 | awk '{print $2}')
        echo "Connected to: $CONNECTED_MAC"
        
        sudo rfcomm bind /dev/rfcomm0 $CONNECTED_MAC 1
        sleep 2
        
        if [ -e /dev/rfcomm0 ]; then
            echo "✅ RFCOMM bound to /dev/rfcomm0"
        fi
        break
    fi
    
    if [ $((ATTEMPTS % 6)) -eq 0 ]; then
        echo "Waiting for connection... (${ATTEMPTS}/60 attempts)"
    fi
    
    ATTEMPTS=$((ATTEMPTS + 1))
    sleep 5
done

# ============================================
# Run Python script
# ============================================
cd /home/raspberrypi/Desktop/PiRoverMaya

if [ -d "venv" ]; then
    source venv/bin/activate
fi

echo "Starting Python navigation script..."
sudo python3 main.py

# Cleanup
sudo rfcomm release /dev/rfcomm0 2>/dev/null
read -p "Press Enter to exit..."