import ustruct
import time

class BMP280Error(Exception):
    pass

class BMP280:
    CTRL_MEAS = 0xF4
    CONFIG = 0xF5
    PRESS_MSB = 0xF7
    CALIB_START = 0x88

    def __init__(self, i2c, addr=0x77):
        self.i2c = i2c
        self.addr = addr
        self.t_fine = 0
        self._load_calibration()
        self._configure()

    def _read_bytes(self, reg, n):
        try:
            return self.i2c.readfrom_mem(self.addr, reg, n)
        except OSError as e:
            raise BMP280Error("I2C read failed at reg {:#x}: {}".format(reg, e))

    def _write_byte(self, reg, val):
        try:
            self.i2c.writeto_mem(self.addr, reg, bytes([val]))
        except OSError as e:
            raise BMP280Error("I2C write failed at reg {:#x}: {}".format(reg, e))

    def _load_calibration(self):
        raw = self._read_bytes(self.CALIB_START, 24)
        (self.dig_T1, self.dig_T2, self.dig_T3,
         self.dig_P1, self.dig_P2, self.dig_P3,
         self.dig_P4, self.dig_P5, self.dig_P6,
         self.dig_P7, self.dig_P8, self.dig_P9) = ustruct.unpack('<HhhHhhhhhhhh', raw)

    def _configure(self):
        # osrs_t=1, osrs_p=4 (x16), mode=normal
        self._write_byte(self.CTRL_MEAS, 0x2F)
        # t_sb=0.5ms, IIR filter x8
        self._write_byte(self.CONFIG, 0x0C)

    def _read_raw(self):
        data = self._read_bytes(self.PRESS_MSB, 6)
        raw_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        raw_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        return raw_t, raw_p

    def read(self):
        """Returns (temperature_c, pressure_pa). Raises BMP280Error on I2C fault."""
        raw_t, raw_p = self._read_raw()

        var1 = (raw_t / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = (raw_t / 131072.0 - self.dig_T1 / 8192.0) ** 2 * self.dig_T3
        self.t_fine = int(var1 + var2)
        temp = (var1 + var2) / 5120.0

        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (self.dig_P6 / 32768.0)
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = (var2 / 4.0) + (self.dig_P4 * 65536.0)
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0:
            raise BMP280Error("Pressure compensation div-by-zero (var1==0)")

        p = 1048576.0 - raw_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = self.dig_P9 * p * p / 2147483648.0
        var2 = p * self.dig_P8 / 32768.0
        pressure = p + (var1 + var2 + self.dig_P7) / 16.0

        return temp, pressure

    def read_retry(self, retries=2, delay_ms=5):
        last_err = None
        for _ in range(retries + 1):
            try:
                return self.read()
            except BMP280Error as e:
                last_err = e
                time.sleep_ms(delay_ms)
        raise last_err
