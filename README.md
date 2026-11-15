# Dwarka SkyGate: Flight Computer

Dual-bus avionics for an experimental rocket, built on the Raspberry Pi Pico W (RP2040) and MicroPython. The board reads a barometer and an IMU on two isolated I2C buses, locks a pad-zero pressure reference, and streams checksummed telemetry over a LoRa link to a ground station.

**Status:** ground-demo stage. The breadboard rig runs standalone off a power bank and transmits live telemetry to a laptop receiver. Not yet flight-ready; see [Roadmap](#roadmap).

![Breadboard rig](docs/images/breadboard_rig.png)

## Architecture

![System architecture](docs/images/architecture.png)

| Block | Part | Interface |
| --- | --- | --- |
| MCU | Raspberry Pi Pico W (RP2040, dual-core Cortex-M0+, 133 MHz) | n/a |
| Barometer | BMP280 | I2C0, GP4 (SDA) / GP5 (SCL), address `0x77`, 400 kHz |
| IMU | LSM6DS3 / LSM6DSOX family | I2C1, GP2 (SDA) / GP3 (SCL), address `0x6B`, 400 kHz |
| Radio | Ebyte E32-900T20D (868 MHz LoRa, transparent mode) | UART0, GP0 (TX) / GP1 (RX), 9600 baud |
| Ground station | Second E32 module + CP2102 USB-TTL adapter + laptop | USB serial |

**Why two I2C buses.** The barometer and IMU each get their own hardware bus so that neither can stall or collide with the other as sample rates go up.

**GPIO-powered IMU.** IMU VCC (GP15) and chip-select (GP14) are driven from GPIO. Driving CS high locks the IMU into I2C mode, and removing the separate power and CS wires cuts connection points that could shake loose under high-G vibration.

**UART LoRa instead of SPI.** Keeping the radio on UART leaves the SPI bus free for SD card logging.

## Boot sequence

1. Fast LED blink on power-up; IMU powered and CS locked high; 500 ms for the IMU to stabilise.
2. Both I2C buses initialised at 400 kHz and scanned independently.
3. Sensor drivers initialised. The IMU driver checks the `WHO_AM_I` register; any failure halts the board in an SOS blink instead of flying with a dead sensor.
4. A 5 s settle window (slow blink) lets handling vibration die out.
5. Pad zero: 50 pressure samples, highest and lowest 10% trimmed, averaged into the ground reference `P0`.
6. Solid LED = GO. The telemetry loop starts.

Altitude relative to the pad is computed from pressure:

```
h = 44330 * (1 - (p / p0) ^ (1 / 5.255))
```

## Telemetry format

NMEA-style ASCII frames, so they survive a plain UART to LoRa to UART hop without binary alignment issues:

```
$DSKY,<pkt_id>,<t_s>,<alt_m>,<press_pa>,<temp_c>,<ax_g>,<ay_g>,<az_g>,<gx_dps>,<gy_dps>,<gz_dps>*<XOR>\r\n
```

The checksum is an 8-bit XOR of every byte between `$` and `*`. `pkt_id` wraps at 16 bits; the ground station uses it to count dropped packets. Current rate: one frame every 2 s.

## Repository layout

| Path | Contents |
| --- | --- |
| `firmware/` | Pico code: `main.py` (boot + telemetry loop), `bmp280.py` and `lsm6ds3.py` (sensor drivers with retry and error handling), `telemetry.py` (packet framing) |
| `ground_station/` | `lora_gcs_receiver.py`: pyserial receiver that validates checksums, prints a live table and reports packet loss |
| `bench/` | Small LoRa link test scripts used during bring-up |
| `docs/images/` | Architecture diagram and rig photo |

## Run it

Flight computer: copy everything in `firmware/` to the Pico (MicroPython), keeping `main.py` as the entry point.

Ground station:

```
pip install pyserial
python ground_station/lora_gcs_receiver.py COM5
```

Replace `COM5` with the port the CP2102 appears as.

## Roadmap

- [x] Dual-I2C bring-up with BMP280 and IMU
- [x] Pad-zero calibration and relative altitude
- [x] Checksummed telemetry over LoRa with ground-side loss counting
- [ ] Non-blocking SD card logging
- [ ] Digital low-pass / Kalman filtering for sensor noise
- [ ] Apogee detection and MOSFET-driven parachute deployment
- [ ] PCB sized for a 38 mm or 54 mm body tube

## About

Part of Dwarka SkyGate, a student-led experimental rocketry project. Avionics, firmware and ground station by [Yogiraj Tambule](https://github.com/YogirajTambule); airframe, propulsion and hardware prototyping by the rest of the team. Build log on X: [@YogirajTambule](https://x.com/YogirajTambule).

Released under the MIT License.
