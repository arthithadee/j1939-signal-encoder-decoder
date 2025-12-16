"""
Main Flask Application for J1939 Signal Generator
"""

from flask import Flask, render_template, jsonify, request, send_file, Response
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import json
import time
import threading
import os
from io import BytesIO

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Get the absolute path to the frontend directory
current_dir = os.path.dirname(os.path.abspath(__file__))
frontend_dir = os.path.join(current_dir, '..', 'frontend')

# Configure template and static folders
app.template_folder = frontend_dir
app.static_folder = frontend_dir

# J1939 PGN/SPN Database
PGN_DATABASE = {
    61444: {  # 0xF004
        "name": "Engine Torque / Speed",
        "priority": 3,
        "spns": {
            190: {
                "name": "Engine Speed",
                "dlc": 8,
                "start_byte": 4,
                "start_bit": 1,
                "length_bits": 16,
                "transmission_rate": "Fixed 10–50 ms",
                "resolution": 0.125,
                "offset": 0,
                "unit": "rpm",
                "min_data": 0,
                "max_data": 8031.875,
                "min_oper": None,
                "max_oper": None
            },
            512: {
                "name": "Driver Demand Engine % Torque",
                "dlc": 8,
                "start_byte": 2,
                "start_bit": 1,
                "length_bits": 8,
                "transmission_rate": "Fixed 10–50 ms",
                "resolution": 1.0,
                "offset": -125,
                "unit": "%",
                "min_data": -125,
                "max_data": 125,
                "min_oper": -125,
                "max_oper": 125
            }
        }
    },
    65265: {  # 0xFEF1
        "name": "Vehicle Speed",
        "priority": 6,
        "spns": {
            84: {
                "name": "Wheel-Based Vehicle Speed",
                "dlc": 8,
                "start_byte": 2,
                "start_bit": 1,
                "length_bits": 16,
                "transmission_rate": 100,
                "resolution": 1/256,
                "offset": 0,
                "unit": "km/h",
                "min_data": 0,
                "max_data": 250.996,
                "min_oper": None,
                "max_oper": None
            }
        }
    },
    65269: {  # 0xFEF5
        "name": "Environmental Data",
        "priority": 6,
        "spns": {
            170: {
                "name": "Cab Interior Temperature",
                "dlc": 8,
                "start_byte": 2,
                "start_bit": 1,
                "length_bits": 16,
                "transmission_rate": 1000,
                "resolution": 0.03125,
                "offset": -273,
                "unit": "°C",
                "min_data": -273,
                "max_data": 1735,
                "min_oper": None,
                "max_oper": None
            }
        }
    }
}

# Live Signals Configuration
LIVE_SIGNALS = {
    "engine_speed": {
        "pgn": 61444,
        "spn": 190,
        "name": "Engine Speed",
        "resolution": 0.125,
        "offset": 0,
        "unit": "rpm",
        "transmission_rate": 50,
        "min_physical": 0,
        "max_physical": 8031.875,
        "start_byte": 4,
        "start_bit": 1,
        "length_bits": 16
    },
    "vehicle_speed": {
        "pgn": 65265,
        "spn": 84,
        "name": "Vehicle Speed",
        "resolution": 1/256,
        "offset": 0,
        "unit": "km/h",
        "transmission_rate": 100,
        "min_physical": 0,
        "max_physical": 250.996,
        "start_byte": 2,
        "start_bit": 1,
        "length_bits": 16
    },
    "cab_temperature": {
        "pgn": 65269,
        "spn": 170,
        "name": "Cab Temperature",
        "resolution": 0.03125,
        "offset": -273,
        "unit": "°C",
        "transmission_rate": 1000,
        "min_physical": -273,
        "max_physical": 1735,
        "start_byte": 2,
        "start_bit": 1,
        "length_bits": 16
    },
    "driver_torque": {
        "pgn": 61444,
        "spn": 512,
        "name": "Driver Torque",
        "resolution": 1.0,
        "offset": -125,
        "unit": "%",
        "transmission_rate": 50,
        "min_physical": -125,
        "max_physical": 125,
        "start_byte": 2,
        "start_bit": 1,
        "length_bits": 8
    }
}

class J1939Encoder:
    """J1939 CAN Frame Encoder"""
    
    @staticmethod
    def calculate_can_id(pgn: int, source_address: int = 0) -> int:
        """Calculate J1939 CAN ID from PGN and source address"""
        priority = 6
        
        if pgn < 0xF000:
            pdu_format = (pgn >> 8) & 0xFF
            pdu_specific = pgn & 0xFF
        else:
            pdu_format = (pgn >> 8) & 0xFF
            pdu_specific = 0xFF
        
        can_id = ((priority & 0x7) << 26)
        can_id |= ((0 & 0x1) << 25)
        can_id |= ((0 & 0x1) << 24)
        can_id |= ((pdu_format & 0xFF) << 16)
        can_id |= ((pdu_specific & 0xFF) << 8)
        can_id |= (source_address & 0xFF)
        
        return can_id
    
    @staticmethod
    def physical_to_raw(physical_value: float, resolution: float, offset: float) -> int:
        """Convert physical value to raw integer value"""
        if resolution == 0:
            raise ValueError("Resolution cannot be zero")
        
        raw = round((physical_value - offset) / resolution)
        return max(0, raw)
    
    @staticmethod
    def encode_value(raw_value: int, start_byte: int, start_bit: int, length_bits: int) -> bytearray:
        """Encode a raw value into a CAN data field"""
        data = bytearray(8)
        
        byte_idx = start_byte - 1
        bit_idx = start_bit - 1
        
        max_value = (1 << length_bits) - 1
        if raw_value > max_value:
            raw_value = max_value
        
        for i in range(length_bits):
            bit_value = (raw_value >> i) & 0x01
            
            current_byte = byte_idx + ((bit_idx + i) // 8)
            current_bit = (bit_idx + i) % 8
            
            if bit_value:
                data[current_byte] |= (1 << current_bit)
            else:
                data[current_byte] &= ~(1 << current_bit)
        
        return data
    
    @staticmethod
    def encode_spn(physical_value: float, spn_config: dict) -> dict:
        """Encode a physical value for a specific SPN"""
        resolution = spn_config.get("resolution", 1.0)
        offset = spn_config.get("offset", 0.0)
        start_byte = spn_config.get("start_byte", 1)
        start_bit = spn_config.get("start_bit", 1)
        length_bits = spn_config.get("length_bits", 8)
        
        raw_value = J1939Encoder.physical_to_raw(physical_value, resolution, offset)
        data = J1939Encoder.encode_value(raw_value, start_byte, start_bit, length_bits)
        
        pgn = spn_config.get("pgn", 0)
        can_id = J1939Encoder.calculate_can_id(pgn)
        
        return {
            "physical_value": physical_value,
            "raw_value": raw_value,
            "raw_hex": f"0x{raw_value:X}",
            "raw_binary": f"{raw_value:0{length_bits}b}",
            "data_bytes": data.hex().upper(),
            "data_array": list(data),
            "can_id": can_id,
            "can_id_hex": f"0x{can_id:08X}",
            "pgn": pgn,
            "pgn_hex": f"0x{pgn:04X}",
            "spn": spn_config.get("spn"),
            "resolution": resolution,
            "offset": offset,
            "unit": spn_config.get("unit", "")
        }

# Initialize encoder
encoder = J1939Encoder()

# Live simulation state
live_simulation_active = False
live_simulation_thread = None

@app.route('/')
def index():
    """Serve the main HTML page"""
    return render_template('index.html')

@app.route('/api/pgns', methods=['GET'])
def get_pgns():
    """Get list of all PGNs"""
    pgns = []
    for pgn, data in PGN_DATABASE.items():
        pgns.append({
            "pgn": pgn,
            "hex": f"0x{pgn:04X}",
            "name": data["name"]
        })
    return jsonify({"pgns": sorted(pgns, key=lambda x: x["pgn"])})

@app.route('/api/spns/<int:pgn>', methods=['GET'])
def get_spns(pgn):
    """Get SPNs for a specific PGN"""
    if pgn not in PGN_DATABASE:
        return jsonify({"spns": []})
    
    spns = []
    for spn_id, spn_data in PGN_DATABASE[pgn]["spns"].items():
        spns.append({
            "spn": spn_id,
            "name": spn_data["name"],
            "unit": spn_data["unit"],
            "resolution": spn_data["resolution"],
            "offset": spn_data["offset"],
            "length_bits": spn_data["length_bits"],
            "start_byte": spn_data["start_byte"],
            "start_bit": spn_data.get("start_bit", 1)
        })
    
    return jsonify({"spns": sorted(spns, key=lambda x: x["spn"])})

@app.route('/api/encode', methods=['POST'])
def encode_value():
    """Encode a physical value to J1939 CAN frame"""
    try:
        data = request.json
        signal_id = data.get('signal_id')
        physical_value = float(data.get('physical_value', 0))
        
        if signal_id not in LIVE_SIGNALS:
            return jsonify({"error": "Invalid signal ID"}), 400
        
        config = LIVE_SIGNALS[signal_id]
        
        # Encode the value
        result = encoder.encode_spn(physical_value, config)
        
        return jsonify(result)
    
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/live/signals', methods=['GET'])
def get_live_signals():
    """Get configuration for live signals"""
    signals = []
    
    for key, config in LIVE_SIGNALS.items():
        signals.append({
            "id": key,
            "name": config["name"],
            "pgn": config["pgn"],
            "pgn_hex": f"0x{config['pgn']:04X}",
            "spn": config["spn"],
            "resolution": config["resolution"],
            "offset": config["offset"],
            "unit": config["unit"],
            "transmission_rate": config["transmission_rate"],
            "min_physical": config["min_physical"],
            "max_physical": config["max_physical"],
            "start_byte": config["start_byte"],
            "start_bit": config["start_bit"],
            "length_bits": config["length_bits"]
        })
    
    return jsonify({"signals": signals})

def live_simulation_worker():
    """Background thread for live signal simulation"""
    global live_simulation_active
    
    while live_simulation_active:
        try:
            for signal_id, config in LIVE_SIGNALS.items():
                import random
                physical_value = random.uniform(
                    config["min_physical"],
                    config["max_physical"]
                )
                
                encoded = encoder.encode_spn(physical_value, config)
                
                socketio.emit('live_signal', {
                    "signal_id": signal_id,
                    "timestamp": time.time() * 1000,
                    "physical_value": physical_value,
                    "raw_value": encoded["raw_value"],
                    "data_bytes": encoded["data_bytes"],
                    "can_id_hex": encoded["can_id_hex"],
                    "unit": config["unit"]
                })
                
                time.sleep(config["transmission_rate"] / 1000.0)
        
        except Exception as e:
            print(f"Live simulation error: {e}")
            break

@socketio.on('start_live_simulation')
def start_live_simulation():
    """Start live signal simulation"""
    global live_simulation_active, live_simulation_thread
    
    if not live_simulation_active:
        live_simulation_active = True
        live_simulation_thread = threading.Thread(target=live_simulation_worker)
        live_simulation_thread.daemon = True
        live_simulation_thread.start()
        
        emit('simulation_status', {"active": True})

@socketio.on('stop_live_simulation')
def stop_live_simulation():
    """Stop live signal simulation"""
    global live_simulation_active
    
    live_simulation_active = False
    emit('simulation_status', {"active": False})

@app.route('/<path:path>')
def serve_static(path):
    """Serve static files"""
    return app.send_static_file(path)

if __name__ == '__main__':
    socketio.run(app, debug=True, port=5000, host='0.0.0.0')