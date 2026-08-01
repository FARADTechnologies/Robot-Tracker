# Robot Tracker

An end-to-end GNSS vehicle tracker: an ESP32-S3 + A7608E-H device reads its
position, **encrypts it with AES-256-GCM**, and publishes it over the cellular
network via MQTT. A browser dashboard subscribes, decrypts the payload
client-side and shows the vehicle live on a map.

Because the payload is encrypted before it leaves the device, the broker only
ever relays ciphertext — it never sees a real position.

```
[vehicle]  GNSS fix -> JSON -> AES-256-GCM -> MQTT over LTE
                                  |
                            [ MQTT broker ]         relays ciphertext only
                                  |
[browser]  MQTT over WebSocket -> decrypt in page -> live map + history
```

## Features

**Device**
- Multi-constellation GNSS (GPS + GLONASS + BeiDou + Galileo)
- AES-256-GCM payload encryption using the ESP32-S3 hardware accelerator
- Assisted GNSS (`AT+CAGPS`) for a faster, more reliable first fix
- Quality filtering: HDOP gating, impossible-jump rejection, stationary smoothing,
  and an optional HDOP-adaptive Kalman filter
- Adaptive reporting — frequent while moving, sparse while parked
- Store-and-forward buffer so positions survive cellular coverage gaps
- Remote configuration over MQTT (no USB required)
- Every feature can be toggled at runtime for debugging

**Dashboard**
- Live map with heading cone, accuracy circle, trail and dwell markers
- History scrubber with playback, and 15 min / 1 h / 1 day ranges
- Telemetry rail: speed, accuracy (±m and HDOP), satellites, altitude, heading
- Key vault — AES keys are typed in per session and kept in tab memory only
- Serial console (Web Serial) for direct AT access, plus an MQTT command tab
- Light and dark themes, English and Turkish, responsive down to phone size

## Repository layout

```
Firmware/
  gps_tracker/       main tracker firmware (config.example.h -> config.h)
  a7608_at_test/     modem bring-up sketch: AT, SIM, network, GNSS, HTTP
  gps/               minimal GNSS read example
  zenith_tracker/    early dashboard prototype
web/
  zenith.html        the dashboard (single file, no build step)
  config.example.js  copy to config.js and set broker + topics
tools/
  listen.py          subscribe and print decrypted positions
  send_command.py    send a downlink command to the device
  mock_walk.py       publish a fake track to exercise the UI
Mechanical/CAD/      enclosure and mounting design
docs/                hardware notes
```

## Hardware

| Part | Notes |
| --- | --- |
| ESP32-S3-N16R8 | 16 MB flash, 8 MB OPI PSRAM, hardware AES |
| A7608E-H | LTE Cat-4 modem with built-in multi-constellation GNSS |
| GNSS antenna | Ceramic patch — **mount it on a ≥70 × 70 mm ground plane** |
| LTE antenna | Full band 698–960 / 1710–2690 MHz |
| Power | 4S Li-ion pack + BMS -> buck converter -> 5 V rail |

Wiring between the two boards:

| ESP32-S3 | A7608E-H |
| --- | --- |
| GPIO15 (TX) | RXD |
| GPIO16 (RX) | TXD |
| GND | GND |

See [docs/hardware.md](docs/hardware.md) for power notes and accuracy tips.

## Getting started

### 1. Firmware

```bash
cp Firmware/gps_tracker/config.example.h Firmware/gps_tracker/config.h
# edit config.h: APN, broker, client id, topics, and your own AES key
openssl rand -hex 32          # generate a key
```

Build and upload with [arduino-cli](https://arduino.github.io/arduino-cli/):

```bash
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=default,USBMode=hwcdc Firmware/gps_tracker
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=default,USBMode=hwcdc Firmware/gps_tracker
```

The board exposes two USB-C ports: use the one wired to the USB-serial bridge for
flashing, and keep `CDCOnBoot=default` so the serial monitor works there.

### 2. Dashboard

```bash
cp web/config.example.js web/config.js    # set broker + topics
```

Open `web/zenith.html` in Chrome, Edge or Brave (Web Serial and Web Crypto are
required; both need a Chromium browser). Press **Connect**, then open the key
vault (🗝) and paste the device's 64-hex key. Nothing is stored on disk.

### 3. Tools

```bash
export TRACKER_KEY=<64 hex>
export TRACKER_TOPIC=yourprefix/trk01/pos
python tools/listen.py
```

## Remote configuration

Publish JSON to the device's command topic — from the dashboard's MQTT console
or with `tools/send_command.py`:

| Command | Effect |
| --- | --- |
| `{"interval":5}` | reporting period in seconds |
| `{"gnssms":1000}` | GNSS fix rate in ms (200–10000) |
| `{"hdopmax":2.5}` | reject fixes worse than this HDOP |
| `{"hdopgate":0}` | disable HDOP filtering |
| `{"statlock":0}` | disable stationary smoothing |
| `{"jumprej":0}` | disable jump rejection |
| `{"kalman":1}` | enable the Kalman filter |
| `{"agps":1}` | enable assisted GNSS |
| `{"antbias":0}` | stop feeding DC to the antenna (passive antenna) |
| `{"antmv":1800}` | antenna bias voltage in mV — also sets its LNA gain |
| `{"verbose":1}` | echo raw AT traffic to USB |
| `{"cmd":"report"}` | report position immediately |

Commands are themselves AES-256-GCM encrypted and carry a strictly increasing
`seq`, so the device rejects any command that is forged or replayed. The device
echoes its live settings back in the telemetry payload, so a command is confirmed
by the next message.

## Security model

- **Confidentiality + authenticity both directions.** Positions (uplink) and
  commands (downlink) are AES-256-GCM. The broker only ever relays ciphertext, and
  the GCM tag means neither side accepts a forged or tampered message.
- **Replay protection.** Every frame carries a monotonic `seq`. The device rejects
  commands whose seq does not increase (counter persisted in NVS, so it survives
  reboots), and the dashboard drops stale or out-of-order positions.
- **Per-device keys.** Each device gets its own AES-256 key, so one leaked key
  cannot expose the rest.
- **Keys never committed.** `config.h` and `web/config.js` are untracked. The
  dashboard holds a key in tab memory only — never in localStorage, never shown
  back in plaintext. For the strongest setup keep keys offline (on paper) and type
  them into the vault per session.
- **Subresource Integrity.** The dashboard's CDN dependencies are pinned with
  SRI hashes, so a compromised CDN cannot inject code into a page that handles keys.

## Accuracy notes

Reported accuracy tracks HDOP closely (roughly ±HDOP × 5 m). With a clear view of
the sky the device typically sees 30+ satellites at HDOP below 1.5.

The single biggest cheap improvement is the antenna ground plane: a bare ceramic
patch on a small PCB loses several dB of gain and, more importantly, its circular
polarisation degrades — which is exactly what rejects reflected (multipath)
signals. Mount the patch on a metal plate of at least 70 × 70 mm, ceramic facing
the sky. An active antenna with a built-in LNA is the next step up.

## Roadmap

- IMU dead reckoning for tunnels and urban canyons
- Server-side map matching and persistent history (Supabase, then a dedicated backend)
- Authenticated broker with TLS, multi-device management
- Keys burned into the ESP32-S3 eFuse so firmware cannot read them back
- Geofencing and alerts, battery and ignition telemetry
