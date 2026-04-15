#!/usr/bin/env python3
"""
Flask Server Wrapper for PiRover BLE Scanner
Includes DHT11 temperature/humidity and MQ135 gas sensor support
"""
import asyncio
import json
import threading
import time
from pathlib import Path
from datetime import datetime
from flask import Flask, jsonify, render_template_string, request
from flask_socketio import SocketIO, emit
from bleak import BleakScanner, AdvertisementData, BLEDevice
from queue import Queue

app = Flask(__name__)
app.config['SECRET_KEY'] = 'pirover_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

OUTPUT_FILE = Path("ble_devices.json")
CURRENT_DATA = {
    "device_info": {},
    "last_update": None,
    "status": "waiting",
    "history": []
}

# Queue for thread-safe communication
data_queue = Queue()
scanning_active = False

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>PiRover Dashboard - Enhanced</title>
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        .header {
            background: white;
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        h1 {
            color: #667eea;
            margin-bottom: 10px;
        }
        .status {
            display: inline-block;
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 14px;
            font-weight: bold;
        }
        .status.active { background: #4caf50; color: white; }
        .status.waiting { background: #ff9800; color: white; }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        .card {
            background: white;
            border-radius: 15px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        .card h2 {
            color: #667eea;
            margin-bottom: 15px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        .sensor-value {
            font-size: 48px;
            font-weight: bold;
            text-align: center;
            margin: 20px 0;
        }
        .distance-critical { color: #f44336; }
        .distance-warning { color: #ff9800; }
        .distance-safe { color: #4caf50; }
        .environment-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin: 20px 0;
        }
        .env-sensor {
            text-align: center;
            padding: 15px;
            border-radius: 10px;
            background: #f5f5f5;
        }
        .env-value {
            font-size: 32px;
            font-weight: bold;
            margin-top: 10px;
        }
        .gas-alert {
            background: #f44336;
            color: white;
            padding: 15px;
            border-radius: 10px;
            text-align: center;
            font-weight: bold;
            margin-top: 15px;
            animation: pulse 1s ease-in-out infinite;
        }
        .ir-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin: 20px 0;
        }
        .ir-sensor {
            text-align: center;
            padding: 15px;
            border-radius: 10px;
            background: #f5f5f5;
        }
        .ir-sensor.detected {
            background: #f44336;
            color: white;
        }
        .warning {
            background: #fff3cd;
            border-left: 4px solid #ffc107;
            padding: 15px;
            margin-top: 15px;
            border-radius: 5px;
        }
        .suggestion {
            background: #d1ecf1;
            border-left: 4px solid #17a2b8;
            padding: 15px;
            margin-top: 15px;
            border-radius: 5px;
        }
        .history {
            max-height: 400px;
            overflow-y: auto;
        }
        .history-item {
            padding: 10px;
            border-bottom: 1px solid #eee;
            font-size: 14px;
        }
        .history-item:hover {
            background: #f5f5f5;
        }
        .button {
            background: #667eea;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            margin: 5px;
            font-size: 16px;
        }
        .button:hover {
            background: #764ba2;
        }
        @keyframes pulse {
            0% { transform: scale(1); }
            50% { transform: scale(1.05); }
            100% { transform: scale(1); }
        }
        .pulse {
            animation: pulse 1s ease-in-out;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 PiRover Enhanced Telemetry Dashboard</h1>
            <div id="statusBadge" class="status waiting">Waiting for data...</div>
            <div style="margin-top: 10px;">
                <button class="button" onclick="refreshData()">Refresh</button>
                <button class="button" onclick="clearHistory()">Clear History</button>
            </div>
        </div>

        <div class="grid">
            <div class="card">
                <h2>📊 Current Status</h2>
                <div class="sensor-value" id="distanceValue">-- cm</div>
                <div>⚡ Speed: <span id="speedValue">--</span>%</div>
                <div>🎮 Mode: <span id="modeValue">--</span></div>
                <div>📡 RSSI: <span id="rssiValue">--</span> dBm</div>
                <div>🕐 Last Update: <span id="timestamp">--</span></div>
                <div id="warningDiv"></div>
                <div id="suggestionDiv"></div>
            </div>

            <div class="card">
                <h2>🌡️ Environment Sensors</h2>
                <div class="environment-grid">
                    <div class="env-sensor">
                        <div>🌡️ Temperature</div>
                        <div class="env-value" id="tempValue">--°C</div>
                    </div>
                    <div class="env-sensor">
                        <div>💧 Humidity</div>
                        <div class="env-value" id="humValue">--%</div>
                    </div>
                </div>
                <div id="gasAlert"></div>
            </div>
        </div>

        <div class="grid">
            <div class="card">
                <h2>📡 IR Sensors</h2>
                <div class="ir-grid">
                    <div class="ir-sensor" id="irTL">
                        <div>⬆️ Top Left</div>
                        <div id="irTLVal">CLEAR</div>
                    </div>
                    <div class="ir-sensor" id="irTR">
                        <div>⬆️ Top Right</div>
                        <div id="irTRVal">CLEAR</div>
                    </div>
                    <div class="ir-sensor" id="irBL">
                        <div>⬇️ Bottom Left</div>
                        <div id="irBLVal">CLEAR</div>
                    </div>
                    <div class="ir-sensor" id="irBR">
                        <div>⬇️ Bottom Right</div>
                        <div id="irBRVal">CLEAR</div>
                    </div>
                </div>
                <div>Raw IR: <span id="irRaw">----</span></div>
            </div>

            <div class="card">
                <h2>📈 Quick Stats</h2>
                <div id="quickStats">
                    <p>🚗 <strong>Movement:</strong> <span id="movementStatus">--</span></p>
                    <p>🛑 <strong>Obstacles:</strong> <span id="obstacleStatus">--</span></p>
                    <p>🌫️ <strong>Air Quality:</strong> <span id="airQuality">Normal</span></p>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>📜 Telemetry History (Last 20 entries)</h2>
            <div class="history" id="historyDiv">
                <div style="color: #999; text-align: center;">No data yet...</div>
            </div>
        </div>
    </div>

    <script>
        const socket = io();
        
        socket.on('connect', function() {
            console.log('WebSocket connected');
        });
        
        socket.on('rover_data', function(data) {
            updateDashboard(data);
        });
        
        socket.on('status_update', function(status) {
            const badge = document.getElementById('statusBadge');
            badge.className = 'status ' + status.status;
            badge.textContent = status.message;
        });
        
        function updateDashboard(data) {
            // Animate
            document.getElementById('distanceValue').classList.add('pulse');
            setTimeout(() => {
                document.getElementById('distanceValue').classList.remove('pulse');
            }, 1000);
            
            // Distance
            const distance = data.parsed_data.distance_cm;
            const distanceElem = document.getElementById('distanceValue');
            distanceElem.textContent = distance + ' cm';
            distanceElem.className = 'sensor-value';
            if (distance < 15) distanceElem.classList.add('distance-critical');
            else if (distance < 30) distanceElem.classList.add('distance-warning');
            else distanceElem.classList.add('distance-safe');
            
            // Temperature & Humidity
            document.getElementById('tempValue').textContent = data.parsed_data.temperature + '°C';
            document.getElementById('humValue').textContent = data.parsed_data.humidity + '%';
            
            // Gas alert
            const gasAlertDiv = document.getElementById('gasAlert');
            if (data.parsed_data.gas_detected) {
                gasAlertDiv.innerHTML = '<div class="gas-alert">⚠️ GAS DETECTED! ⚠️<br>Dangerous gases present!</div>';
                document.getElementById('airQuality').innerHTML = '<span style="color: #f44336;">⚠️ GAS DETECTED!</span>';
            } else {
                gasAlertDiv.innerHTML = '<div class="env-sensor" style="background: #e8f5e9;">✅ Air quality normal</div>';
                document.getElementById('airQuality').innerHTML = 'Normal';
            }
            
            // Basic info
            document.getElementById('speedValue').textContent = data.parsed_data.speed_percent;
            document.getElementById('modeValue').textContent = data.parsed_data.mode;
            document.getElementById('rssiValue').textContent = data.rssi;
            document.getElementById('timestamp').textContent = new Date(data.timestamp).toLocaleTimeString();
            document.getElementById('irRaw').textContent = data.parsed_data.ir_raw;
            
            // Movement status
            if (data.parsed_data.speed_percent > 0) {
                document.getElementById('movementStatus').innerHTML = '<span style="color: #4caf50;">Moving</span>';
            } else {
                document.getElementById('movementStatus').innerHTML = '<span style="color: #ff9800;">Stopped</span>';
            }
            
            // Obstacle status
            if (data.parsed_data.critical_distance) {
                document.getElementById('obstacleStatus').innerHTML = '<span style="color: #f44336;">Critical!</span>';
            } else if (data.parsed_data.obstacle_detected) {
                document.getElementById('obstacleStatus').innerHTML = '<span style="color: #ff9800;">Warning</span>';
            } else {
                document.getElementById('obstacleStatus').innerHTML = '<span style="color: #4caf50;">Clear</span>';
            }
            
            // IR Sensors
            const ir = data.parsed_data.ir_sensors;
            updateIRSensor('irTL', ir.top_left);
            updateIRSensor('irTR', ir.top_right);
            updateIRSensor('irBL', ir.bottom_left);
            updateIRSensor('irBR', ir.bottom_right);
            
            // Warnings and suggestions
            const warningDiv = document.getElementById('warningDiv');
            if (data.parsed_data.critical_distance) {
                warningDiv.innerHTML = '<div class="warning">⚠️ CRITICAL: Very close obstacle! (' + distance + 'cm)</div>';
            } else if (data.parsed_data.obstacle_detected) {
                warningDiv.innerHTML = '<div class="warning">⚠️ Obstacle detected at ' + distance + 'cm</div>';
            } else {
                warningDiv.innerHTML = '';
            }
            
            const suggestionDiv = document.getElementById('suggestionDiv');
            if (ir.top_left && !ir.top_right) {
                suggestionDiv.innerHTML = '<div class="suggestion">💡 Suggestion: Turn RIGHT (obstacle on left)</div>';
            } else if (ir.top_right && !ir.top_left) {
                suggestionDiv.innerHTML = '<div class="suggestion">💡 Suggestion: Turn LEFT (obstacle on right)</div>';
            } else if (ir.top_left && ir.top_right) {
                suggestionDiv.innerHTML = '<div class="suggestion">💡 Suggestion: Back up or stop (obstacle ahead)</div>';
            } else {
                suggestionDiv.innerHTML = '';
            }
        }
        
        function updateIRSensor(elementId, detected) {
            const elem = document.getElementById(elementId);
            const valElem = document.getElementById(elementId + 'Val');
            if (detected) {
                elem.classList.add('detected');
                valElem.textContent = 'DETECTED ⬛';
            } else {
                elem.classList.remove('detected');
                valElem.textContent = 'CLEAR ⬜';
            }
        }
        
        function refreshData() {
            fetch('/api/latest')
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'success') {
                        updateDashboard(data.data);
                    }
                });
        }
        
        function clearHistory() {
            fetch('/api/clear_history', { method: 'POST' })
                .then(() => {
                    document.getElementById('historyDiv').innerHTML = '<div style="color: #999; text-align: center;">History cleared</div>';
                });
        }
        
        // Auto-refresh every 5 seconds
        setInterval(refreshData, 5000);
    </script>
</body>
</html>
"""

def decode_payload(data: bytes):
    try:
        text = data.decode("utf-8", errors="ignore")
        return text if text.startswith("D") else data.hex()
    except:
        return data.hex()

def parse_rover_data(data_string):
    """Parse PiRover data string format: D{distance}S{speed}M{mode}I{ir_bits}T{temp}H{humidity}G{gas}"""
    try:
        # Extract all values
        d_pos = data_string.find('D')
        s_pos = data_string.find('S')
        m_pos = data_string.find('M')
        i_pos = data_string.find('I')
        t_pos = data_string.find('T')
        h_pos = data_string.find('H')
        g_pos = data_string.find('G')
        
        if d_pos == -1 or s_pos == -1 or m_pos == -1 or i_pos == -1:
            return None
        
        distance = int(data_string[d_pos+1:s_pos])
        speed = int(data_string[s_pos+1:m_pos])
        mode = int(data_string[m_pos+1:i_pos])
        ir_bits = data_string[i_pos+1:t_pos] if t_pos != -1 else data_string[i_pos+1:]
        
        # Parse environment data if available
        temperature = 0.0
        humidity = 0.0
        gas_detected = False
        
        if t_pos != -1 and h_pos != -1 and g_pos != -1:
            try:
                temperature = float(data_string[t_pos+1:h_pos])
                humidity = float(data_string[h_pos+1:g_pos])
                gas_detected = bool(int(data_string[g_pos+1:]))
            except:
                pass
        
        ir_sensors = {
            'top_left': int(ir_bits[0]) if len(ir_bits) > 0 else 0,
            'top_right': int(ir_bits[1]) if len(ir_bits) > 1 else 0,
            'bottom_left': int(ir_bits[2]) if len(ir_bits) > 2 else 0,
            'bottom_right': int(ir_bits[3]) if len(ir_bits) > 3 else 0
        }
        
        return {
            'distance_cm': distance,
            'speed_percent': speed,
            'mode': 'AUTO' if mode == 1 else 'MANUAL',
            'mode_raw': mode,
            'ir_sensors': ir_sensors,
            'ir_raw': ir_bits,
            'temperature': temperature,
            'humidity': humidity,
            'gas_detected': gas_detected,
            'obstacle_detected': distance < 30,
            'critical_distance': distance < 15
        }
    except Exception as e:
        print(f"Error parsing data: {e}")
        return None

def ble_scanner_thread():
    """Run BLE scanner in a separate thread"""
    global scanning_active, CURRENT_DATA
    
    def callback(device: BLEDevice, adv: AdvertisementData):
        if "PiRover" not in (device.name or ""):
            return
        
        # Get the payload
        payload = ""
        if adv.manufacturer_data:
            for cid, data in adv.manufacturer_data.items():
                payload = decode_payload(data)
                break
        
        # Parse the data
        parsed = parse_rover_data(payload) if payload.startswith('D') else None
        
        if parsed:
            # Prepare data packet
            data_packet = {
                "device_info": {
                    "name": device.name or "PiRover",
                    "address": device.address,
                    "rssi": adv.rssi
                },
                "parsed_data": parsed,
                "payload": payload,
                "timestamp": datetime.now().isoformat(),
                "last_seen": datetime.now().strftime("%H:%M:%S")
            }
            
            # Save to file
            save_data = {
                device.address: {
                    "name": device.name or "PiRover",
                    "address": device.address,
                    "rssi": adv.rssi,
                    "payload": payload,
                    "last_seen": datetime.now().strftime("%H:%M:%S"),
                    "parsed_data": parsed
                }
            }
            
            with OUTPUT_FILE.open("w", encoding="utf-8") as f:
                json.dump(save_data, f, indent=2)
            
            # Update current data
            CURRENT_DATA["device_info"] = data_packet["device_info"]
            CURRENT_DATA["last_update"] = data_packet["timestamp"]
            CURRENT_DATA["parsed_data"] = parsed
            CURRENT_DATA["payload"] = payload
            CURRENT_DATA["status"] = "active"
            
            # Add to history (keep last 50 entries)
            CURRENT_DATA["history"].insert(0, data_packet)
            if len(CURRENT_DATA["history"]) > 50:
                CURRENT_DATA["history"] = CURRENT_DATA["history"][:50]
            
            # Put in queue for Flask to emit via WebSocket
            data_queue.put(data_packet)
            
            # Print to console with environment data
            gas_warning = " ⚠️GAS!" if parsed['gas_detected'] else ""
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] PiRover Update{gas_warning}")
            print(f"📡 RSSI: {adv.rssi} dBm")
            print(f"📏 Distance: {parsed['distance_cm']}cm | Speed: {parsed['speed_percent']}% | Mode: {parsed['mode']}")
            print(f"🌡️ Temp: {parsed['temperature']:.1f}°C | 💧 Humidity: {parsed['humidity']:.1f}%")
    
    async def run_scanner():
        scanner = BleakScanner(
            detection_callback=callback,
            scanning_mode="active",
        )
        
        await scanner.start()
        print("✓ BLE Scanner started. Looking for PiRover...")
        
        while scanning_active:
            await asyncio.sleep(1)
        
        await scanner.stop()
        print("✓ BLE Scanner stopped")
    
    # Run the async scanner
    asyncio.run(run_scanner())

@app.route('/')
def index():
    """Serve the dashboard"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/latest')
def get_latest():
    """Get the latest rover data"""
    if CURRENT_DATA.get("parsed_data"):
        return jsonify({
            "status": "success",
            "data": {
                "device_info": CURRENT_DATA["device_info"],
                "parsed_data": CURRENT_DATA["parsed_data"],
                "payload": CURRENT_DATA.get("payload", ""),
                "timestamp": CURRENT_DATA["last_update"],
                "rssi": CURRENT_DATA["device_info"].get("rssi", 0)
            }
        })
    else:
        return jsonify({
            "status": "waiting",
            "message": "No data received yet"
        })

@app.route('/api/history')
def get_history():
    """Get telemetry history"""
    return jsonify({
        "status": "success",
        "history": CURRENT_DATA["history"]
    })

@app.route('/api/clear_history', methods=['POST'])
def clear_history():
    """Clear telemetry history"""
    CURRENT_DATA["history"] = []
    return jsonify({"status": "success", "message": "History cleared"})

@app.route('/api/status')
def get_status():
    """Get scanner status"""
    return jsonify({
        "status": CURRENT_DATA["status"],
        "last_update": CURRENT_DATA["last_update"],
        "scanning_active": scanning_active
    })

@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection"""
    emit('status_update', {
        "status": CURRENT_DATA["status"],
        "message": "Connected to server"
    })
    
    # Send latest data if available
    if CURRENT_DATA.get("parsed_data"):
        emit('rover_data', {
            "device_info": CURRENT_DATA["device_info"],
            "parsed_data": CURRENT_DATA["parsed_data"],
            "payload": CURRENT_DATA.get("payload", ""),
            "timestamp": CURRENT_DATA["last_update"],
            "rssi": CURRENT_DATA["device_info"].get("rssi", 0)
        })

def emit_data_from_queue():
    """Background task to emit data from queue via WebSocket"""
    while True:
        try:
            data = data_queue.get(timeout=1)
            socketio.emit('rover_data', {
                "device_info": data["device_info"],
                "parsed_data": data["parsed_data"],
                "payload": data["payload"],
                "timestamp": data["timestamp"],
                "rssi": data["device_info"]["rssi"]
            })
            socketio.emit('status_update', {
                "status": "active",
                "message": "Receiving data"
            })
        except:
            pass

def main():
    global scanning_active
    
    print("="*60)
    print("🤖 PiRover Enhanced BLE Scanner with Flask Server")
    print("   Including DHT11 and MQ135 sensors")
    print("="*60)
    
    # Start BLE scanner in background thread
    scanning_active = True
    scanner_thread = threading.Thread(target=ble_scanner_thread, daemon=True)
    scanner_thread.start()
    
    # Start WebSocket emitter thread
    emitter_thread = threading.Thread(target=emit_data_from_queue, daemon=True)
    emitter_thread.start()
    
    # Start Flask server
    print("\n🌐 Starting Flask server...")
    print("📱 Access the dashboard at:")
    print("   - http://localhost:5000")
    print("   - http://<raspberry_pi_ip>:5000")
    print("\n📡 BLE scanner is running in background")
    print("🌡️ Monitoring DHT11 temperature/humidity")
    print("🌫️ Monitoring MQ135 gas sensor")
    print("⏹️  Press Ctrl+C to stop\n")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down...")
        scanning_active = False
        print("✅ Server stopped")

if __name__ == "__main__":
    main()