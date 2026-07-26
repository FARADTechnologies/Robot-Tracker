"""Publish an encrypted mock track so the web UI can be exercised without hardware.

Draws a slow loop with a stop in the middle, which should render as a trail plus
a red dwell marker in the app.

    TRACKER_KEY        64-hex AES-256 key (same one entered in the app's vault)
    TRACKER_TOPIC      uplink topic to publish on, e.g. yourprefix/trk02/pos
    TRACKER_BROKER     broker host (default broker.hivemq.com)
    TRACKER_ORIGIN     "lat,lon" centre of the loop (default 41.0082,28.9784)
"""
import base64
import json
import math
import os
import sys
import time

import paho.mqtt.client as mqtt
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

KEY_HEX = os.environ.get("TRACKER_KEY", "")
TOPIC = os.environ.get("TRACKER_TOPIC", "")
BROKER = os.environ.get("TRACKER_BROKER", "broker.hivemq.com")
LAT0, LON0 = (float(x) for x in os.environ.get("TRACKER_ORIGIN", "41.0082,28.9784").split(","))

if len(KEY_HEX) != 64 or not TOPIC:
    sys.exit("Set TRACKER_KEY (64 hex chars) and TRACKER_TOPIC first.")

KEY = bytes.fromhex(KEY_HEX)
DEVICE_ID = TOPIC.split("/")[-2] if "/" in TOPIC else "mock"


def encrypt(obj):
    iv = os.urandom(12)
    return base64.b64encode(iv + AESGCM(KEY).encrypt(iv, json.dumps(obj).encode(), None)).decode()


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(BROKER, 1883, 60)
client.loop_start()
time.sleep(1)
print(f"publishing mock track -> {TOPIC}", flush=True)

STEPS, RADIUS = 200, 0.0025          # roughly a 280 m radius loop
for i in range(STEPS):
    angle = i / STEPS * 2 * math.pi
    moving = 1
    if 70 <= i < 100:                # a stationary stretch -> dwell marker
        angle = 70 / STEPS * 2 * math.pi
        moving = 0
    msg = {
        "id": DEVICE_ID,
        "seq": i + 1,
        "lat": round(LAT0 + RADIUS * math.sin(angle), 6),
        "lon": round(LON0 + RADIUS * math.cos(angle) * 0.76, 6),
        "alt": round(60 + 8 * math.sin(angle), 1),
        "spd": 0.0 if not moving else round(4 + 2 * math.sin(i / 5), 1),
        "hdop": round(0.9 + 0.5 * abs(math.sin(i / 9)), 1),
        "sat": 30 + (i % 6),
        "mov": moving,
        "rej": 0,
        "utc": time.strftime("%H%M%S") + ".00",
        "date": time.strftime("%d%m%y"),
    }
    client.publish(TOPIC, encrypt(msg), qos=0)
    if i % 20 == 0:
        print(f"  #{i} {msg['lat']},{msg['lon']} mov={moving}", flush=True)
    time.sleep(1.0)

client.loop_stop()
print("done", flush=True)
