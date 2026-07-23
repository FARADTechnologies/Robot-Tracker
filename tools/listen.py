"""Subscribe to a tracker's uplink topic and print decrypted positions.

Settings come from environment variables so no secrets live in the repo:

    TRACKER_KEY     64-hex AES-256 key (same key the device uses)
    TRACKER_TOPIC   uplink topic, e.g. yourprefix/trk01/pos
    TRACKER_BROKER  broker host (default broker.hivemq.com)

Example (PowerShell):
    $env:TRACKER_KEY="<64 hex>"; $env:TRACKER_TOPIC="yourprefix/trk01/pos"
    python tools/listen.py
"""
import base64
import os
import sys
import time

import paho.mqtt.client as mqtt
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

KEY_HEX = os.environ.get("TRACKER_KEY", "")
TOPIC = os.environ.get("TRACKER_TOPIC", "")
BROKER = os.environ.get("TRACKER_BROKER", "broker.hivemq.com")

if len(KEY_HEX) != 64 or not TOPIC:
    sys.exit("Set TRACKER_KEY (64 hex chars) and TRACKER_TOPIC first.")

KEY = bytes.fromhex(KEY_HEX)


def on_connect(client, _userdata, _flags, reason, _props=None):
    print(f"connected ({reason}) -> {TOPIC}", flush=True)
    client.subscribe(TOPIC)


def on_message(_client, _userdata, msg):
    blob = base64.b64decode(msg.payload)
    try:
        plain = AESGCM(KEY).decrypt(blob[:12], blob[12:], None)
        print(plain.decode(), flush=True)
    except Exception as exc:                      # wrong key or corrupt frame
        print(f"decrypt failed: {exc}", flush=True)


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.loop_start()
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    client.loop_stop()
