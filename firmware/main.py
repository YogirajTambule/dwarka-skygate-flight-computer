# Dwarka SkyGate -- fused ground-demo avionics loop.
# Boots dual-I2C sensor bus, locks a ground-level pressure reference, then
# transmits a fused BMP280 + LSM6DS3 telemetry packet over the E32 LoRa link
# every TELEMETRY_PERIOD_S. Runs standalone off VBUS (powerbank or any 5V
# source) -- no REPL tether required once flashed.
#
# Pad-zero calibration is deliberately preceded by a SETTLE_DELAY_S window
# (distinct slow LED blink) so any handling vibration from connecting power
# has time to die out before the pressure baseline is sampled -- a board
# disturbed during the sampling window otherwise locks in a biased P0 and
# every altitude reading downstream inherits that fixed offset. The
# 50-sample average is also trimmed (discards the extreme high/low readings)
# as a second line of defense against a stray outlier sample.

import machine
import time
from bmp280 import BMP280, BMP280Error
from lsm6ds3 import LSM6DS3, LSM6DS3Error
from telemetry import build_packet

STATUS_LED = machine.Pin("LED", machine.Pin.OUT)
IMU_VCC = machine.Pin(15, machine.Pin.OUT)      # GPIO power hack: IMU VCC
IMU_CS_LOCK = machine.Pin(14, machine.Pin.OUT)  # GPIO power hack: IMU CS, HIGH = I2C mode

TELEMETRY_PERIOD_S = 2.0
CALIBRATION_SAMPLES = 50
SETTLE_DELAY_S = 5.0
PAD_ZERO_TRIM_FRACTION = 0.1  # discard the highest/lowest 10% of samples before averaging


def _blink(times, period_s):
    for _ in range(times):
        STATUS_LED.toggle()
        time.sleep(period_s)


def _fault_halt(message):
    print("[WARNING]", message)
    print("[WARNING] Check wiring -- halting in SOS blink.")
    while True:
        _blink(6, 0.15)
        time.sleep(0.6)


def _trimmed_mean(samples, trim_fraction):
    ordered = sorted(samples)
    trim = int(len(ordered) * trim_fraction)
    kept = ordered[trim: len(ordered) - trim] if trim > 0 else ordered
    return sum(kept) / len(kept), trim


def main():
    print("Dwarka SkyGate: Master Avionics Boot Sequence Initiated...")
    print("-" * 60)
    _blink(40, 0.05)
    STATUS_LED.value(0)

    IMU_VCC.value(1)
    IMU_CS_LOCK.value(1)
    time.sleep(0.5)  # IMU silicon stabilization

    i2c_bmp = machine.I2C(0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400000)
    i2c_imu = machine.I2C(1, sda=machine.Pin(2), scl=machine.Pin(3), freq=400000)

    print("--- Scanning Avionics Buses ---")
    bmp_addrs = i2c_bmp.scan()
    imu_addrs = i2c_imu.scan()
    print("I2C0 (BMP280):", [hex(a) for a in bmp_addrs])
    print("I2C1 (LSM6DSO):", [hex(a) for a in imu_addrs])

    try:
        bmp = BMP280(i2c_bmp, addr=0x77)
    except BMP280Error as e:
        _fault_halt("BMP280 init failed: {}".format(e))

    try:
        imu = LSM6DS3(i2c_imu, addr=0x6B)
    except LSM6DS3Error as e:
        _fault_halt("LSM6DSO init failed: {}".format(e))

    print("[SUCCESS] BMP280 + LSM6DSO online.")

    uart = machine.UART(0, baudrate=9600, tx=machine.Pin(0), rx=machine.Pin(1))

    print("Settling {:.0f}s before pad-zero -- place the rig down now, do NOT touch it.".format(SETTLE_DELAY_S))
    _blink(int(SETTLE_DELAY_S / 0.5), 0.5)  # slow blink = "settling", distinct from boot/fault patterns
    STATUS_LED.value(0)

    print("Executing pad zeroing... do NOT move the rig.")
    pressure_samples = []
    for _ in range(CALIBRATION_SAMPLES):
        try:
            _, press = bmp.read_retry()
        except BMP280Error as e:
            print("[WARNING] Calibration sample dropped:", e)
            time.sleep(0.05)
            continue
        pressure_samples.append(press)
        time.sleep(0.05)

    if len(pressure_samples) < CALIBRATION_SAMPLES // 2:
        _fault_halt("Pad zeroing failed -- too few good BMP280 samples ({}/{})".format(
            len(pressure_samples), CALIBRATION_SAMPLES))

    p0, trimmed = _trimmed_mean(pressure_samples, PAD_ZERO_TRIM_FRACTION)
    print("Pad zero locked. P0 = {:.2f} Pa ({} samples, {} trimmed each side)".format(
        p0, len(pressure_samples), trimmed))
    print("-" * 60)
    print("[GO] All avionics online -- telemetry link active.")
    STATUS_LED.value(1)
    print("--- DWARKA SKYGATE: FUSED TELEMETRY LINK ACTIVE ---")

    pkt_id = 0
    boot_ticks = time.ticks_ms()

    while True:
        loop_start = time.ticks_ms()
        t_s = time.ticks_diff(loop_start, boot_ticks) / 1000.0

        try:
            temp_c, press_pa = bmp.read_retry()
            altitude_m = 44330.0 * (1.0 - (press_pa / p0) ** (1.0 / 5.255))
        except BMP280Error as e:
            print("[FAULT] BMP280 read failed:", e)
            temp_c, press_pa, altitude_m = float("nan"), float("nan"), float("nan")

        try:
            accel_g, gyro_dps = imu.read_all_retry()
        except LSM6DS3Error as e:
            print("[FAULT] LSM6DSO read failed:", e)
            accel_g, gyro_dps = (float("nan"),) * 3, (float("nan"),) * 3

        pkt_id = (pkt_id + 1) & 0xFFFF
        packet = build_packet(pkt_id, t_s, altitude_m, press_pa, temp_c, accel_g, gyro_dps)
        uart.write(packet)

        print("TX #{} | ALT {:.2f} m | P {:.1f} Pa | T {:.1f} C | A ({:.2f},{:.2f},{:.2f}) g | G ({:.1f},{:.1f},{:.1f}) dps".format(
            pkt_id, altitude_m, press_pa, temp_c,
            accel_g[0], accel_g[1], accel_g[2],
            gyro_dps[0], gyro_dps[1], gyro_dps[2],
        ))

        elapsed_s = time.ticks_diff(time.ticks_ms(), loop_start) / 1000.0
        sleep_s = TELEMETRY_PERIOD_S - elapsed_s
        if sleep_s > 0:
            time.sleep(sleep_s)


main()
