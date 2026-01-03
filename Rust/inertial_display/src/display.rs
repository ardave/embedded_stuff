use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
};
use esp_idf_hal::{
    delay::FreeRtos,
    i2c::I2cDriver,
};
use esp_idf_svc::sys::EspError;

// I2C address for SH1107 display
pub const SH1107_ADDR: u8 = 0x3D;

// SH1107 display dimensions
pub const DISPLAY_WIDTH: u32 = 128;
pub const DISPLAY_HEIGHT: u32 = 128;

/// SH1107 OLED display driver for Adafruit 128x128 display
pub struct Sh1107Display<'a> {
    i2c: I2cDriver<'a>,
    buffer: [u8; (DISPLAY_WIDTH * DISPLAY_HEIGHT / 8) as usize],
}

impl<'a> Sh1107Display<'a> {
    pub fn new(i2c: I2cDriver<'a>) -> Self {
        Self {
            i2c,
            buffer: [0; (DISPLAY_WIDTH * DISPLAY_HEIGHT / 8) as usize],
        }
    }

    /// Get mutable access to the I2C driver for use by other devices on the bus
    pub fn i2c_mut(&mut self) -> &mut I2cDriver<'a> {
        &mut self.i2c
    }

    fn send_command(&mut self, cmd: u8) -> Result<(), EspError> {
        self.i2c.write(SH1107_ADDR, &[0x00, cmd], 100)
    }

    fn send_commands(&mut self, cmds: &[u8]) -> Result<(), EspError> {
        for &cmd in cmds {
            self.send_command(cmd)?;
        }
        Ok(())
    }

    pub fn init(&mut self) -> Result<(), EspError> {
        // SH1107 initialization sequence for Adafruit 128x128 OLED
        self.send_commands(&[
            0xAE,       // Display off
            0xDC, 0x00, // Set display start line to 0
            0x81, 0x2F, // Set contrast
            0x20,       // Set memory addressing mode (page addressing)
            0xA0,       // Set segment re-map (column address 0 = SEG0)
            0xC0,       // Set COM output scan direction (normal)
            0xA8, 0x7F, // Set multiplex ratio (128)
            0xD3, 0x00, // Set display offset to 0
            0xD5, 0x51, // Set display clock divide ratio
            0xD9, 0x22, // Set pre-charge period
            0xDB, 0x35, // Set VCOMH deselect level
            0xB0,       // Set page address to 0
            0xA4,       // Entire display ON (resume from RAM)
            0xA6,       // Set normal display (not inverted)
            0xAF,       // Display on
        ])?;

        FreeRtos::delay_ms(100);
        Ok(())
    }

    pub fn clear(&mut self) {
        self.buffer.fill(0);
    }

    pub fn flush(&mut self) -> Result<(), EspError> {
        // SH1107 128x128: 16 pages (vertical strips), 128 rows each
        for page in 0..16u8 {
            // Set page address (0xB0-0xBF for pages 0-15)
            self.send_command(0xB0 | page)?;
            // Set row address to 0 (lower nibble)
            self.send_command(0x00)?;
            // Set row address to 0 (upper nibble)
            self.send_command(0x10)?;

            // Send page data (128 bytes for 128 rows)
            let start = (page as usize) * 128;
            let end = start + 128;

            // Send data with 0x40 prefix indicating data mode
            let mut data_cmd = [0u8; 129];
            data_cmd[0] = 0x40; // Data mode
            data_cmd[1..].copy_from_slice(&self.buffer[start..end]);
            self.i2c.write(SH1107_ADDR, &data_cmd, 100)?;
        }
        Ok(())
    }

    fn set_pixel(&mut self, x: u32, y: u32, on: bool) {
        if x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT {
            return;
        }

        // SH1107 128x128 memory layout for Adafruit display:
        // The display is rotated - pages are vertical columns
        // Flip X to correct mirror effect
        let x = (DISPLAY_WIDTH - 1) - x;
        let page = x / 8;
        let bit = x % 8;
        let index = (page * DISPLAY_HEIGHT + y) as usize;

        if on {
            self.buffer[index] |= 1 << bit;
        } else {
            self.buffer[index] &= !(1 << bit);
        }
    }
}

// Implement DrawTarget for embedded-graphics integration
impl<'a> DrawTarget for Sh1107Display<'a> {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            if coord.x >= 0 && coord.y >= 0 {
                self.set_pixel(coord.x as u32, coord.y as u32, color.is_on());
            }
        }
        Ok(())
    }
}

impl<'a> OriginDimensions for Sh1107Display<'a> {
    fn size(&self) -> Size {
        Size::new(DISPLAY_WIDTH, DISPLAY_HEIGHT)
    }
}
