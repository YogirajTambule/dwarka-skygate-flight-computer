# Driver targets the LSM6DS3 / LSM6DSOX register map (CTRL1_XL @ 0x10,
# CTRL2_G @ 0x11, gyro out @ 0x22, accel out @ 0x28, WHO_AM_I @ 0x0F).
# Archive_v1 used addr 0x6B and this same register layout for the accel-only
# read, so the IMU is assumed to be this family. Confirm exact part number
# (silkscreen or WHO_AM_I response) if readings look off.

import ustruct
import time

class LSM6DS3Error(Exception):
    pass

class LSM6DS3:
    WHO_AM_I = 0x0F
    CTRL1_XL = 0x10
    CTRL2_G = 0x11
    OUTX_L_G = 0x22
    OUTX_L_XL = 0x28

    ACCEL_FS_2G = 0.000061      # g/LSB
    GYRO_FS_245DPS = 0.00875    # dps/LSB

    def __init__(self, i2c, addr=0x6B):
        self.i2c = i2c
        self.addr = addr
        self._verify_id()
        self._configure()

    def _read_bytes(self, reg, n):
        try:
            return self.i2c.readfrom_mem(self.addr, reg, n)
        except OSError as e:
            raise LSM6DS3Error("I2C read failed at reg {:#x}: {}".format(reg, e))

    def _write_byte(self, reg, val):
        try:
            self.i2c.writeto_mem(self.addr, reg, bytes([val]))
        except OSError as e:
            raise LSM6DS3Error("I2C write failed at reg {:#x}: {}".format(reg, e))

    def _verify_id(self):
        who_am_i = self._read_bytes(self.WHO_AM_I, 1)[0]
        # 0x69 (LSM6DS3), 0x6C (LSM6DSOX) are the expected values for this family
        if who_am_i not in (0x69, 0x6C):
            raise LSM6DS3Error(
                "Unexpected WHO_AM_I {:#x} at addr {:#x} - confirm IMU part number".format(
                    who_am_i, self.addr))

    def _configure(self):
        self._write_byte(self.CTRL1_XL, 0x40)  # 104 Hz, +/-2g
        self._write_byte(self.CTRL2_G, 0x40)   # 104 Hz, 245 dps

    def read_accel_g(self):
        data = self._read_bytes(self.OUTX_L_XL, 6)
        ax, ay, az = ustruct.unpack('<hhh', data)
        return ax * self.ACCEL_FS_2G, ay * self.ACCEL_FS_2G, az * self.ACCEL_FS_2G

    def read_gyro_dps(self):
        data = self._read_bytes(self.OUTX_L_G, 6)
        gx, gy, gz = ustruct.unpack('<hhh', data)
        return gx * self.GYRO_FS_245DPS, gy * self.GYRO_FS_245DPS, gz * self.GYRO_FS_245DPS

    def read_all(self):
        """Returns ((ax, ay, az), (gx, gy, gz)) in (g, g, g), (dps, dps, dps)."""
        return self.read_accel_g(), self.read_gyro_dps()

    def read_all_retry(self, retries=2, delay_ms=5):
        last_err = None
        for _ in range(retries + 1):
            try:
                return self.read_all()
            except LSM6DS3Error as e:
                last_err = e
                time.sleep_ms(delay_ms)
        raise last_err
