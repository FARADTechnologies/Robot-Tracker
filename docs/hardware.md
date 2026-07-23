# Hardware notes

## Power

```
4S Li-ion pack (12.8–16.8 V) -> BMS (4S 40A) -> LM2596 buck -> 5.15 V
                                                     |
                            star split at the buck output, separate wires:
                                     ├── ESP32-S3 board 5Vin
                                     └── A7608E-H breakout V
```

The modem's ~2 A LTE transmit bursts must **not** flow through the ESP32 board —
split the rail at the buck output and run separate pairs to each board. Measured
5.06 V at the common point under load.

A bulk capacitor (≈1000 µF) across the 5 V rail near the modem is recommended.
Without it, LTE bursts can dip the rail; if the ESP32 reboots unexpectedly the
firmware prints the reset reason at boot, so look for `BROWNOUT` there.

## Serial links

| ESP32-S3 | A7608E-H | Note |
| --- | --- | --- |
| GPIO15 | RXD | 3.3 V TTL, no level shifter needed |
| GPIO16 | TXD | |
| GND | GND | common ground required |

Modem default baud rate is 115200 and the AT set is SIM7600-compatible.

The dev board has two USB-C connectors:

- **UART port** (via the USB-serial bridge) — use this for flashing and the
  serial monitor, with `CDCOnBoot=default`.
- **Native USB port** (ESP32-S3 USB-Serial/JTAG) — works too, but then the build
  needs `CDCOnBoot=cdc` or the monitor stays blank.

## Antennas

Two separate connectors: one for LTE, one for GNSS. **Never transmit without the
LTE antenna attached.**

The GNSS ceramic patch needs a ground plane to perform. On a bare breakout the
plane is only the tiny PCB, which costs 2–3 dB of gain, shifts the centre
frequency, and degrades the antenna's right-hand circular polarisation towards
linear. That polarisation is what rejects reflected signals, so a small plane
directly worsens multipath error — the wandering you see in built-up areas.

Mount the patch on a metal plate of at least 70 × 70 mm with the ceramic facing
the sky, away from metal and from the vehicle body edge.

To tell an active antenna (with an internal LNA) from a passive one, measure
resistance between the coax centre and shield: a passive patch reads a short or
an open and is symmetric, while an active one reads a few hundred ohms to a few
kΩ and changes when the probes are reversed.

## Status LED (NETLIGHT)

| Pattern | Meaning |
| --- | --- |
| Steady on | searching for network |
| 200 ms on / 200 ms off | registered on LTE, data ready |
| 800 ms on / 800 ms off | registered on 2G/3G |
| Off | no power or asleep |

## Accuracy checklist

1. Ground plane under the GNSS patch, clear view of the sky.
2. Assisted GNSS enabled (`AT+CAGPS`) — costs only tens of kB, valid for days.
3. HDOP gating on; reported accuracy is roughly ±HDOP × 5 m.
4. Raise the fix rate (`{"gnssms":1000}`) — commercial trackers run at 1–10 Hz.
5. Add an IMU for dead reckoning where the sky is blocked.
