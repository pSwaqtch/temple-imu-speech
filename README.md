# temple-imu-speech

Bone-conducted speech from an ICM-20948 IMU at the temple → ESP32 → WiFi (UDP) → PC.

## Wiring (ESP32 DevKit ↔ 7Semi ICM-20948), SPI (default)

| ESP32 pin | IMU board pin        | Function        |
|-----------|----------------------|-----------------|
| 3V3       | VCC / VIN            | power (3.3 V)   |
| GND       | GND                  | ground          |
| GPIO 18   | SCL / SCLK           | SPI clock       |
| GPIO 23   | SDA / SDI            | SPI MOSI        |
| GPIO 19   | AD0 / SDO            | SPI MISO        |
| GPIO 5    | CS / NCS             | chip select     |
| —         | INT, FSYNC, AUX_*    | leave unconnected |

I2C fallback (fewer wires, max ~1.1 kHz, no future high-rate mode): set `IMU_BUS IMU_BUS_I2C` in
`firmware/main/config.h`; wire SDA→GPIO 21, SCL→GPIO 22, CS→3V3 (keeps the chip in I2C mode).

Notes
- Power from **3V3, not 5V**. The ICM-20948 core is 1.8 V; the breakout must have its own regulator
  and level shifting (check the board) — never drive the bare chip at 3.3 V logic.
- Keep the cable ≤ 30 cm, thin and flexible, with strain relief so cable tugs don't couple into the
  sensor. If you see bit errors at 4 MHz, lower `IMU_SPI_HZ`.

## Mounting

Flat against bone just above/in front of the ear (temporal bone), held firmly with medical tape or a
headband: pressure matters more than exact spot. Keep the ESP32 off the head (collar/pocket).
Note which IMU axis points into the skull — usually the strongest vibration axis.

## Firmware (ESP-IDF v6)

```sh
cd firmware
. $IDF_PATH/export.sh
idf.py build flash monitor -p /dev/cu.usbserial-0001
```
WiFi credentials: `firmware/main/secrets.h` (git-ignored; template in `secrets.example.h`).
**ESP32 is 2.4 GHz only.**

Status line every 2 s on serial: sensor rate, FIFO overflows, TX queue drops, WiFi/RSSI, subscriber.

Design: core 1 drains the FIFO every 2 ms (hardware-timed samples, no jitter); core 0 does WiFi/UDP.
A ~2.7 s queue between them absorbs WiFi stalls. The sensor is re-initialized automatically on any
fault (WHO_AM_I check every 1 s, stall/garbage detection), and WiFi reconnects on its own.

## Host

One process, the **hub**, talks to the device; everything else connects to the hub.

```
ESP32 --UDP--> hub (python -m timu.hub) --> recordings/*.timu      (Record button or --record LABEL)
                                         --> http://localhost:8765  live web view
                                         --> ws://localhost:8765/ws binary = raw packets, text = JSON status/s
```

```sh
cd host
uv run python -m timu.hub                     # then open http://localhost:8765
uv run python -m timu.hub --record bench      # start recording immediately
uv run python -m timu.hub --bind 0.0.0.0      # let other machines on the LAN view (and control recording)
```

Web view: 50 Hz high-passed acceleration (x/y/z), live spectrogram, per-axis voice-band (80–500 Hz)
RMS meters, orientation, link health, record start/stop with a label, and live voice detection with
the pitch (f0) line drawn over the spectrogram.

Voice detection (`timu/voice.py`) runs inside the hub and is pushed to websocket clients as
`{"type":"voice","frames":[[sample, voiced, f0_hz|null, snr_db], ...]}`. It finds narrow pitch lines
(70–350 Hz) against an adaptive per-bin noise floor and rejects taps/cable knocks by their large
sample-to-sample jumps. Evaluate it offline on recordings:
```sh
uv run python -m timu.voice recordings/*.timu
```

Python processing clients (run alongside the hub, any number of them):
```python
from timu.client import stream
for pkt in stream():                  # timu.protocol.Packet, exactly as the device sent it
    process(pkt.first_sample, pkt.accel_g(), pkt.gyro_dps())
```
`async for pkt in timu.client.packets(): ...` is the asyncio equivalent.

Load a recording:
```python
from timu.recording import load
r = load("recordings/20261124-230000_aaa_vowel.timu")
r.accel_g, r.gyro_dps, r.sample_index, r.segment, r.rate_hz
```

## Protocol

`firmware/main/protocol.h` ⇔ `host/timu/protocol.py`. 48-byte header + int16 frames
(ax ay az gx gy gz). Lost samples are exactly countable from `first_sample`; `segment` marks
device-side gaps of unknown length; `boot_id` marks reboots. Recordings store raw packets verbatim,
so the format can evolve without losing old data.

## Roadmap

1. **Stream (now)** – verify rate ≈ 1125 Hz, zero loss, voice-band RMS responds to speech.
2. **Signal characterization** – spectrograms while speaking vs silent; pick best axis/placement;
   measure noise floor; decide accel-only vs accel+gyro.
3. **Synchronized dataset** – record mic audio alongside IMU (PC mic, aligned via host timestamps +
   clap) to get labels/targets; prompted-utterance recording tool.
4. **Higher bandwidth** – DLPF-bypass mode (accel ~4.5 kHz over SPI) if harmonics above 500 Hz
   prove useful.
5. **Models** – voice activity detection → keyword/command recognition → IMU-conditioned speech
   enhancement/recognition.
