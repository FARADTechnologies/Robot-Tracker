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

### Protection board

The 4S board (marked `4S 40A`, `YS-Z1053`) carries two groups of pads that are
easy to confuse:

| Pads | Purpose |
| --- | --- |
| `0V` `4.2V` `8.4V` `12.6V` `16.8V` | balance and sense taps, one per cell junction |
| `J1` (+) `J2` (−) | the output port |

There is no separate charge terminal, so charge and discharge share `J1`/`J2`.
Both the buck converter and the charging socket hang off those two pads.

The taps double as the pack connection — `16.8V` is the pack positive and `0V`
the negative, so there is no extra `B+` wire. Verify them with a meter before
trusting the pack: each adjacent pair should read one cell, and the spread
between cells should stay under 0.1 V. Measured 2026-08-11 with the tracker
running: 3.66 / 3.66 / 3.64 / 3.72 V, pack 14.57 V, cells within 0.08 V.

That same measurement showed **14.14 V at `J1`/`J2` against 14.57 V at the pack**
— a 0.4 V loss through the board at roughly 100 mA, which implies a poor joint
rather than the MOSFETs. Worth reflowing when the 5 V wiring is soldered.

A protection board is not a charger. It cuts off on overcharge, over-discharge
and overcurrent, but it produces no charging profile of its own — the source
still has to supply a proper 16.8 V constant-current/constant-voltage curve.

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

### Powering an active antenna

The module feeds DC to the antenna over the coax from its `VDD_AUX` pin, and an
active antenna is the module's default assumption. Two AT commands control it:

| Command | Effect |
| --- | --- |
| `AT+CVAUXS=1` / `=0` | enable or disable the bias |
| `AT+CVAUXV=<mV>` | bias voltage: 1200, 1250, 1700, 1800, 1850, 1900, 2500–3300 |

Turn the bias **off** for a passive antenna. The firmware exposes both as
`{"antbias":0|1}` and `{"antmv":1800}`.

The voltage matters for more than power: on most active antennas it also sets the
LNA gain. The module wants **under 18 dB total gain** at its port (its reference
figure is a 20 dB LNA), so a high-gain antenna on a long coax can overshoot and
compress the front end. Subtract the cable loss (RG174 is roughly 1 dB/m at
1.5 GHz) from the antenna's LNA gain, and drop the bias voltage if the result is
still well above 18 dB.

### Fitting an antenna with the wrong connector

Antennas often ship with SMA while the module board uses u.FL (IPEX MHF-I). Chain
them as **coax → SMA male → SMA female-to-u.FL pigtail → board**. A u.FL connector
cannot go directly onto RG174: u.FL is made for 1.13 mm micro-coax, RG174 is 2.8 mm.

Never improvise a splice at 1.5 GHz. A twisted joint wrecks the 50 Ω impedance and
throws away more than the antenna gains. Fit a proper connector, then check for a
short between centre and shield before plugging it into the module — a shorted
antenna shorts the module's bias supply.

Cable length is a design knob, not just a nuisance: RG174 costs roughly 1 dB/m at
1.5 GHz, which usefully offsets a high-gain active antenna. Around 1 m of cable
with the bias at 1800 mV lands close to the module's recommended input level.

### Antenna passband and constellations

Check the antenna's passband against the constellations you want. The module
receives GPS/Galileo at 1575.42 MHz, GLONASS at 1597.5–1605.8 MHz, and BeiDou B1I
at 1561.098 MHz. A GPS/GLONASS antenna specified from 1574 MHz upwards covers the
first two but filters BeiDou out — so switching antennas can change which
constellations appear in `AT+CGNSSINFO`, not just the signal quality.

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
