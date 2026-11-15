"""Dwarka SkyGate -- ground control station receiver.

Listens on the E32 LoRa module attached via CP2102 (USB-TTL), decodes the
$DSKY,... fused telemetry frames transmitted by CODE/pico/main.py, validates
the XOR checksum, and prints a live table. Tracks dropped packets via the
16-bit pkt_id sequence so a flaky RF link is visible, not silent.

Usage:
    python lora_gcs_receiver.py [COM_PORT]

Defaults to COM5 (the port used in the 2026-08-14 bench RF test) if no
argument is given -- check Device Manager if your CP2102 enumerates
elsewhere.
"""

import sys
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 9600


def parse_packet(line):
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None

    body, _, chk_str = line[1:].partition("*")
    try:
        expected = int(chk_str, 16)
    except ValueError:
        return None

    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    if checksum != expected:
        return None

    fields = body.split(",")
    if len(fields) != 12 or fields[0] != "DSKY":
        return None

    try:
        return {
            "pkt_id": int(fields[1]),
            "t_s": float(fields[2]),
            "alt_m": float(fields[3]),
            "press_pa": float(fields[4]),
            "temp_c": float(fields[5]),
            "accel_g": tuple(float(x) for x in fields[6:9]),
            "gyro_dps": tuple(float(x) for x in fields[9:12]),
        }
    except ValueError:
        return None


def main():
    print("Dwarka SkyGate GCS -- listening on {} @ {} baud".format(PORT, BAUD))
    try:
        ser = serial.Serial(PORT, BAUD, timeout=2)
    except serial.SerialException as e:
        print("[FATAL] Could not open {}: {}".format(PORT, e))
        print("Check Device Manager for the CP2102's actual COM port, then run:")
        print("    python lora_gcs_receiver.py COM7")
        return

    last_pkt_id = None
    packets_seen = 0
    packets_lost = 0
    corrupt_lines = 0

    header = "{:>5} {:>8} {:>9} {:>11} {:>7} {:>22} {:>18}".format(
        "PKT", "T(s)", "ALT(m)", "P(Pa)", "T(C)", "ACCEL(g) x,y,z", "GYRO(dps) x,y,z"
    )
    print("-" * len(header))
    print(header)
    print("-" * len(header))

    try:
        while True:
            try:
                raw = ser.readline()
            except serial.SerialException as e:
                print("[FATAL] Serial link dropped:", e)
                break

            if not raw:
                continue

            try:
                line = raw.decode("ascii", errors="strict")
            except UnicodeDecodeError:
                corrupt_lines += 1
                continue

            pkt = parse_packet(line)
            if pkt is None:
                corrupt_lines += 1
                continue

            packets_seen += 1
            if last_pkt_id is not None:
                gap = (pkt["pkt_id"] - last_pkt_id) & 0xFFFF
                if gap > 1:
                    packets_lost += gap - 1
            last_pkt_id = pkt["pkt_id"]

            ax, ay, az = pkt["accel_g"]
            gx, gy, gz = pkt["gyro_dps"]
            print("{:>5} {:>8.1f} {:>9.2f} {:>11.1f} {:>7.1f} {:>6.2f},{:>6.2f},{:>6.2f} {:>5.1f},{:>5.1f},{:>5.1f}".format(
                pkt["pkt_id"], pkt["t_s"], pkt["alt_m"], pkt["press_pa"], pkt["temp_c"],
                ax, ay, az, gx, gy, gz,
            ))
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("-" * len(header))
        print("Session summary: {} packets received, {} lost, {} corrupt/unparseable lines".format(
            packets_seen, packets_lost, corrupt_lines
        ))


if __name__ == "__main__":
    main()
