# BME280 MicroPython Driver for I2C
# Based on Bosch BME280 datasheet

from machine import I2C
import time

BME280_I2C_ADDR = 0x76  # or 0x77

# Register addresses
BME280_REG_ID = 0xD0
BME280_REG_RESET = 0xE0
BME280_REG_CTRL_HUM = 0xF2
BME280_REG_STATUS = 0xF3
BME280_REG_CTRL_MEAS = 0xF4
BME280_REG_CONFIG = 0xF5
BME280_REG_DATA = 0xF7
BME280_REG_CALIB_00 = 0x88
BME280_REG_CALIB_26 = 0xE1


class BME280:
    def __init__(self, i2c: I2C, addr: int = BME280_I2C_ADDR):
        self.i2c = i2c
        self.addr = addr

        # Check chip ID
        chip_id = self._read_byte(BME280_REG_ID)
        if chip_id != 0x60:
            raise RuntimeError(f"BME280 not found. Chip ID: 0x{chip_id:02x}")
        print(f"BME280 found, chip ID: 0x{chip_id:02x}")

        # Read calibration data
        self._read_calibration()

        # Configure sensor
        # Humidity oversampling x1
        self._write_byte(BME280_REG_CTRL_HUM, 0x01)
        # Temperature oversampling x1, Pressure oversampling x1, Normal mode
        self._write_byte(BME280_REG_CTRL_MEAS, 0x27)
        # Standby 1000ms, filter off
        self._write_byte(BME280_REG_CONFIG, 0xA0)

        time.sleep_ms(100)

    def _read_byte(self, reg: int) -> int:
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _read_bytes(self, reg: int, length: int) -> bytes:
        return self.i2c.readfrom_mem(self.addr, reg, length)

    def _write_byte(self, reg: int, value: int) -> None:
        self.i2c.writeto_mem(  # pyright: ignore[reportUnknownMemberType]
            self.addr, reg, bytes([value])
        )

    def _read_calibration(self) -> None:
        # Read temperature and pressure calibration (0x88-0x9F)
        calib1 = self._read_bytes(BME280_REG_CALIB_00, 26)
        # Read humidity calibration (0xE1-0xE7)
        calib2 = self._read_bytes(BME280_REG_CALIB_26, 7)

        # Temperature calibration
        self.dig_T1 = calib1[0] | (calib1[1] << 8)
        self.dig_T2 = self._signed16(calib1[2] | (calib1[3] << 8))
        self.dig_T3 = self._signed16(calib1[4] | (calib1[5] << 8))

        # Pressure calibration
        self.dig_P1 = calib1[6] | (calib1[7] << 8)
        self.dig_P2 = self._signed16(calib1[8] | (calib1[9] << 8))
        self.dig_P3 = self._signed16(calib1[10] | (calib1[11] << 8))
        self.dig_P4 = self._signed16(calib1[12] | (calib1[13] << 8))
        self.dig_P5 = self._signed16(calib1[14] | (calib1[15] << 8))
        self.dig_P6 = self._signed16(calib1[16] | (calib1[17] << 8))
        self.dig_P7 = self._signed16(calib1[18] | (calib1[19] << 8))
        self.dig_P8 = self._signed16(calib1[20] | (calib1[21] << 8))
        self.dig_P9 = self._signed16(calib1[22] | (calib1[23] << 8))

        # Humidity calibration
        self.dig_H1 = calib1[25]
        self.dig_H2 = self._signed16(calib2[0] | (calib2[1] << 8))
        self.dig_H3 = calib2[2]
        self.dig_H4 = self._signed16((calib2[3] << 4) | (calib2[4] & 0x0F))
        self.dig_H5 = self._signed16((calib2[5] << 4) | (calib2[4] >> 4))
        self.dig_H6 = self._signed8(calib2[6])

    def _signed16(self, value: int) -> int:
        if value >= 0x8000:
            return value - 0x10000
        return value

    def _signed8(self, value: int) -> int:
        if value >= 0x80:
            return value - 0x100
        return value

    def read_raw(self) -> tuple[int, int, int]:
        """Read raw sensor data"""
        data = self._read_bytes(BME280_REG_DATA, 8)

        raw_press: int = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        raw_temp: int = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        raw_hum: int = (data[6] << 8) | data[7]

        return raw_temp, raw_press, raw_hum

    def read(self) -> tuple[float, float, float]:
        """Read compensated temperature (C), pressure (hPa), humidity (%)"""
        raw_temp, raw_press, raw_hum = self.read_raw()

        # Temperature compensation
        var1: int = ((raw_temp >> 3) - (self.dig_T1 << 1)) * self.dig_T2 >> 11
        var2: int = (((((raw_temp >> 4) - self.dig_T1) * ((raw_temp >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
        t_fine: int = var1 + var2
        temperature: float = ((t_fine * 5 + 128) >> 8) / 100.0

        # Pressure compensation
        var1 = t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = ((var1 * var1 * self.dig_P3) >> 8) + ((var1 * self.dig_P2) << 12)
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        pressure: float
        if var1 == 0:
            pressure = 0.0
        else:
            p: int = 1048576 - raw_press
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P8 * p) >> 19
            pressure = (((p + var1 + var2) >> 8) + (self.dig_P7 << 4)) / 256.0 / 100.0

        # Humidity compensation
        h: int = t_fine - 76800
        humidity: float
        if h == 0:
            humidity = 0.0
        else:
            h = (((((raw_hum << 14) - (self.dig_H4 << 20) - (self.dig_H5 * h)) + 16384) >> 15) *
                 (((((((h * self.dig_H6) >> 10) * (((h * self.dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
                   self.dig_H2 + 8192) >> 14))
            h = h - (((((h >> 15) * (h >> 15)) >> 7) * self.dig_H1) >> 4)
            h = 0 if h < 0 else h
            h = 419430400 if h > 419430400 else h
            humidity = (h >> 12) / 1024.0

        return temperature, pressure, humidity
