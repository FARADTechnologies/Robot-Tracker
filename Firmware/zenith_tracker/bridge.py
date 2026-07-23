import serial
import re
import time
import threading
import os
import random
from datetime import datetime
from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO

app = Flask(__name__, static_folder='.', static_url_path='')
app.config['SECRET_KEY'] = 'tactical_zenith_key'
CORS(app)
# Transition to WebSockets instead of REST API interval polling
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

@app.route('/')
def serve_dashboard():
    return app.send_static_file('index.html')

# Base coordinates for fallback
telemetry = {
    "lat": 41.0082, "lng": 28.9784, "sats": 0, "status": "OFFLINE",
    "hdop": 2.5, "heading": 0.0, "battery": 100.0, "rssi": -65
}

PATTERN = re.compile(r"FIX\s+\[(\d+)[^\]]*\]\s*->\s*Lat:\s*([-\d\.]+)\s*\|\s*Lon:\s*([-\d\.]+)", re.IGNORECASE)

def emit_log(direction, text):
    """Sends raw logs directly to the frontend Tactical Terminal"""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{timestamp}] {direction} {text}")
    socketio.emit('terminal_log', {"dir": direction, "time": timestamp, "text": text})

def simulate_tactical_data(lat, lng):
    """Simulates advanced C2 metrics pending firmware upgrades"""
    # Fluctuate heading organically
    telemetry["heading"] += random.uniform(-10.0, 10.0)
    telemetry["heading"] %= 360
    
    # Drain simulated battery
    telemetry["battery"] -= random.uniform(0.01, 0.08)
    if telemetry["battery"] <= 0: telemetry["battery"] = 100.0
        
    # Simulate HDOP precision logic. Lower = better accuracy.
    telemetry["hdop"] = random.uniform(0.8, 1.8) if telemetry["sats"] > 6 else random.uniform(2.5, 6.0)
    
    # Fluctuate RSSI organically
    telemetry["rssi"] = int(random.uniform(-85, -50))

def serial_worker():
    global telemetry
    com_port = os.environ.get('ZENITH_COM_PORT', 'COM10')
    emit_log("SYS", f"Initializing Serial Stream on {com_port}")
    
    while True:
        try:
            ser = serial.Serial(com_port, 115200, timeout=1)
            telemetry["status"] = "LISTENING"
            socketio.emit('telemetry_update', telemetry)
            emit_log("SYS", f"Connected to {com_port} at 115200 baud")
            
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        emit_log("IN", line)
                        
                    if "FIX" in line:
                        match = PATTERN.search(line)
                        if match:
                            try:
                                sats = int(match.group(1))
                                lat = float(match.group(2))
                                lng = float(match.group(3))
                                
                                telemetry["sats"] = sats
                                telemetry["lat"] = lat
                                telemetry["lng"] = lng
                                telemetry["status"] = "LIVE"
                                
                                # Process simulation variables
                                simulate_tactical_data(lat, lng)
                                
                                # Real-time Socket.IO Broadcast payload
                                socketio.emit('telemetry_update', telemetry)
                            except ValueError:
                                pass
                time.sleep(0.01)
                
        except serial.SerialException as e:
            telemetry["status"] = f"ERROR"
            socketio.emit('telemetry_update', telemetry)
            emit_log("ERR", f"COM Port {com_port} busy or unavailable. Retrying in 3s...")
            time.sleep(3)
        except Exception as e:
            telemetry["status"] = f"SYS_FAULT"
            emit_log("ERR", str(e))
            socketio.emit('telemetry_update', telemetry)
            time.sleep(3)

if __name__ == '__main__':
    t = threading.Thread(target=serial_worker, daemon=True)
    t.start()
    
    print("[*] Vanguard C2 Server online.")
    # Run the WebSocket application
    socketio.run(app, host='127.0.0.1', port=5000, debug=False, allow_unsafe_werkzeug=True)
