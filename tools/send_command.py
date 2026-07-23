"""Send a downlink command to the tracker over MQTT (no USB needed).

    TRACKER_CMD_TOPIC   downlink topic, e.g. yourprefix/trk01/cmd
    TRACKER_BROKER      broker host (default broker.hivemq.com)

Examples:
    python tools/send_command.py '{"interval":5}'
    python tools/send_command.py '{"hdopmax":2.5,"kalman":1}'
    python tools/send_command.py '{"verbose":1}'

Supported keys: interval, gnssms, hdopmax, agps, hdopgate, statlock,
jumprej, kalman, lbs, verbose, and {"cmd":"report"|"agpsnow"|"lbsnow"}.
"""
import os
import sys
import time

import paho.mqtt.client as mqtt

TOPIC = os.environ.get("TRACKER_CMD_TOPIC", "")
BROKER = os.environ.get("TRACKER_BROKER", "broker.hivemq.com")
payload = sys.argv[1] if len(sys.argv) > 1 else '{"cmd":"report"}'

if not TOPIC:
    sys.exit("Set TRACKER_CMD_TOPIC first.")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(BROKER, 1883, 60)
client.loop_start()
time.sleep(1)
client.publish(TOPIC, payload, qos=1)
print(f"published -> {TOPIC}  {payload}", flush=True)
time.sleep(2)
client.loop_stop()
