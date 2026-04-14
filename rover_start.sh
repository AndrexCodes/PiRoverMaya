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

# Use service manager for reliable daemon state
sudo systemctl restart bluetooth
sleep 2

# Ensure BlueZ keeps discoverability available longer-term
sudo mkdir -p /etc/bluetooth
if ! grep -q '^\s*DiscoverableTimeout\s*=' /etc/bluetooth/main.conf 2>/dev/null; then
    echo 'DiscoverableTimeout = 0' | sudo tee -a /etc/bluetooth/main.conf >/dev/null
else
    sudo sed -i 's/^\s*DiscoverableTimeout\s*=.*/DiscoverableTimeout = 0/' /etc/bluetooth/main.conf
fi

if ! grep -q '^\s*PairableTimeout\s*=' /etc/bluetooth/main.conf 2>/dev/null; then
    echo 'PairableTimeout = 0' | sudo tee -a /etc/bluetooth/main.conf >/dev/null
else
    sudo sed -i 's/^\s*PairableTimeout\s*=.*/PairableTimeout = 0/' /etc/bluetooth/main.conf
fi

sudo systemctl restart bluetooth
sleep 2

# Make sure adapter is unblocked
sudo rfkill unblock bluetooth
sleep 1

# ============================================
# 1b. Setup Auto-Pairing Agent (No PIN Required)
# ============================================
echo "Setting up auto-pairing agent..."

# Kill any existing bt-agent
sudo killall bt-agent 2>/dev/null

# Create bt-agent script if it doesn't exist
if [ ! -f /usr/local/bin/bt-autopair ]; then
    sudo tee /usr/local/bin/bt-autopair > /dev/null << 'EOF'
#!/bin/bash
killall bt-agent 2>/dev/null
bt-agent -c NoInputNoOutput &
EOF
    sudo chmod 755 /usr/local/bin/bt-autopair
fi

# Start bt-agent with NoInputNoOutput capability
sudo /usr/local/bin/bt-autopair

# Create systemd service for auto-pairing if it doesn't exist
if [ ! -f /etc/systemd/system/bt-agent.service ]; then
    sudo tee /etc/systemd/system/bt-agent.service > /dev/null << 'EOF'
[Unit]
Description=Bluetooth Auto Pairing Agent
After=bluetooth.service

[Service]
Type=forking
ExecStart=/usr/local/bin/bt-autopair
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
    sudo systemctl enable bt-agent.service
fi

# Add Serial Port Profile
sudo sdptool add SP

# Configure bluetoothctl for persistent discoverable + pairable
bluetoothctl power on
bluetoothctl agent DisplayOnly
bluetoothctl default-agent
bluetoothctl pairable on
bluetoothctl pairable-timeout 0
bluetoothctl discoverable on
bluetoothctl discoverable-timeout 0

# Verify current adapter state
echo "Bluetooth adapter status:"
bluetoothctl show | grep -E "Powered:|Discoverable:|Pairable:|DiscoverableTimeout:|PairableTimeout:" || true

# Keep discoverability enabled in case BlueZ drops it after the bluetoothctl session exits
(
    while true; do
        if ! bluetoothctl show | grep -q "Discoverable: yes"; then
            bluetoothctl << EOF
discoverable on
discoverable-timeout 0
pairable on
pairable-timeout 0
agent NoInputNoOutput
default-agent
EOF
        fi
        sleep 20
    done
) &

echo "✅ Bluetooth configured for auto-pairing (No PIN required)"
echo "✅ Device is discoverable as 'raspberrypi'"

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