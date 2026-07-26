"""Send an encrypted, authenticated downlink command to the tracker over MQTT.

The device rejects any command that is not AES-256-GCM encrypted with its key and
carries a strictly increasing `seq`, so commands cannot be forged or replayed.

    TRACKER_KEY         64-hex AES-256 key (same one the device uses)
    TRACKER_CMD_TOPIC   downlink topic, e.g. yourprefix/trk01/cmd
    TRACKER_BROKER      broker host (default broker.hivemq.com)

Examples:
    python tools/send_command.py '{"interval":5}'
    python tools/send_command.py '{"hdopmax":2.5,"kalman":1}'

Supported keys: interval, gnssms, hdopmax, agps, hdopgate, statlock,
jumprej, kalman, lbs, verbose, and {"cmd":"report"|"agpsnow"|"lbsnow"}.
"""
import base64
import json
import os
import sys
import time

import paho.mqtt.client as mqtt
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

KEY_HEX = os.environ.get("TRACKER_KEY", "")
TOPIC = os.environ.get("TRACKER_CMD_TOPIC", "")
BROKER = os.environ.get("TRACKER_BROKER", "broker.hivemq.com")
raw = sys.argv[1] if len(sys.argv) > 1 else '{"cmd":"report"}'

if len(KEY_HEX) != 64 or not TOPIC:
    sys.exit("Set TRACKER_KEY (64 hex) and TRACKER_CMD_TOPIC first.")

cmd = json.loads(raw)
cmd["seq"] = int(time.time() * 1000)             # monotonic, defeats replay
iv = os.urandom(12)
blob = iv + AESGCM(bytes.fromhex(KEY_HEX)).encrypt(iv, json.dumps(cmd).encode(), None)
payload = base64.b64encode(blob).decode()

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(BROKER, 1883, 60)
client.loop_start()
time.sleep(1)
client.publish(TOPIC, payload, qos=1)
print(f"published (encrypted) -> {TOPIC}  {cmd}", flush=True)
time.sleep(2)
client.loop_stop()
