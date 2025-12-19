# SH1107 OLED Display Driver for MicroPython
# For 128x128 monochrome OLED displays like Adafruit 1.12" OLED

import framebuf
from machine import I2C

# SH1107 commands
SET_CONTRAST = 0x81
SET_NORM_INV = 0xA6
SET_DISP = 0xAE
SET_DISP_START_LINE = 0xDC
SET_PAGE_ADDR = 0xB0
SET_COL_ADDR_LOW = 0x00
SET_COL_ADDR_HIGH = 0x10
SET_SEG_REMAP = 0xA0
SET_MUX_RATIO = 0xA8
SET_COM_OUT_DIR = 0xC0
SET_DISP_OFFSET = 0xD3
SET_DISP_CLK_DIV = 0xD5
SET_PRECHARGE = 0xD9
SET_VCOM_DESEL = 0xDB
SET_ENTIRE_ON = 0xA4


class SH1107_I2C(framebuf.FrameBuffer):
    def __init__(self, width: int, height: int, i2c: I2C, addr: int = 0x3D, rotate: int = 90):
        self.i2c = i2c
        self.addr = addr
        self.width = width
        self.height = height
        self.rotate = rotate

        # Buffer for display data
        self.buffer = bytearray(width * height // 8)

        # For 90/270 rotation, swap width/height for framebuffer
        if rotate in (90, 270):
            super().__init__(  # pyright: ignore[reportUnknownMemberType]
                self.buffer, height, width, framebuf.MONO_VLSB
            )
        else:
            super().__init__(  # pyright: ignore[reportUnknownMemberType]
                self.buffer, width, height, framebuf.MONO_VLSB
            )

        self._init_display()

    def _write_cmd(self, cmd: int) -> None:
        self.i2c.writeto(  # pyright: ignore[reportUnknownMemberType]
            self.addr, bytes([0x00, cmd])
        )

    def _write_data(self, data: bytes | bytearray) -> None:
        self.i2c.writeto(  # pyright: ignore[reportUnknownMemberType]
            self.addr, bytes([0x40]) + data
        )

    def _init_display(self) -> None:
        # Turn display off
        self._write_cmd(SET_DISP | 0x00)

        # Set clock divider
        self._write_cmd(SET_DISP_CLK_DIV)
        self._write_cmd(0x51)

        # Set multiplex ratio
        self._write_cmd(SET_MUX_RATIO)
        self._write_cmd(0x7F)  # 128 MUX

        # Set display offset
        self._write_cmd(SET_DISP_OFFSET)
        self._write_cmd(0x00)

        # Set display start line
        self._write_cmd(SET_DISP_START_LINE)
        self._write_cmd(0x00)

        # Set segment remap based on rotation
        if self.rotate in (0, 90):
            self._write_cmd(SET_SEG_REMAP | 0x00)
        else:
            self._write_cmd(SET_SEG_REMAP | 0x01)

        # Set COM output scan direction based on rotation
        if self.rotate in (0, 270):
            self._write_cmd(SET_COM_OUT_DIR | 0x08)
        else:
            self._write_cmd(SET_COM_OUT_DIR | 0x00)

        # Set contrast
        self._write_cmd(SET_CONTRAST)
        self._write_cmd(0x80)

        # Set precharge period
        self._write_cmd(SET_PRECHARGE)
        self._write_cmd(0x22)

        # Set VCOM deselect level
        self._write_cmd(SET_VCOM_DESEL)
        self._write_cmd(0x35)

        # Entire display on (use RAM)
        self._write_cmd(SET_ENTIRE_ON)

        # Normal display (not inverted)
        self._write_cmd(SET_NORM_INV)

        # Turn display on
        self._write_cmd(SET_DISP | 0x01)

        print("SH1107 display initialized")

    def show(self) -> None:
        """Update the display with the buffer contents"""
        for page in range(self.height // 8):
            self._write_cmd(SET_PAGE_ADDR | page)
            self._write_cmd(SET_COL_ADDR_LOW | 0)
            self._write_cmd(SET_COL_ADDR_HIGH | 0)

            start = page * self.width
            end = start + self.width
            self._write_data(self.buffer[start:end])

    def poweroff(self) -> None:
        self._write_cmd(SET_DISP | 0x00)

    def poweron(self) -> None:
        self._write_cmd(SET_DISP | 0x01)

    def contrast(self, value: int) -> None:
        self._write_cmd(SET_CONTRAST)
        self._write_cmd(value & 0xFF)

    def invert(self, invert: bool) -> None:
        self._write_cmd(SET_NORM_INV | (1 if invert else 0))
