#!/bin/bash
# start_rover.sh - Startup script for PiRover

# Wait for system to fully boot and Bluetooth to initialize
sleep 10

# Log file
LOG_FILE="/home/raspberrypi/Desktop/PiRoverMaya/pirover.log"
ERROR_LOG="/home/raspberrypi/Desktop/PiRoverMaya/pirover_error.log"

# Change to project directory
cd /home/raspberrypi/Desktop/PiRoverMaya

# Log start
echo "========================================" >> $LOG_FILE
echo "PiRover Started: $(date)" >> $LOG_FILE
echo "========================================" >> $LOG_FILE

# Ensure Bluetooth is up
sudo hciconfig hci0 up 2>> $ERROR_LOG

# Activate virtual environment and run
source /home/raspberrypi/Desktop/PiRoverMaya/venv/bin/activate
python3 /home/raspberrypi/Desktop/PiRoverMaya/main.py >> $LOG_FILE 2>> $ERROR_LOG

# Log stop
echo "PiRover Stopped: $(date)" >> $LOG_FILE