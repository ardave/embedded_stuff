use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{OriginDimensions, Size};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::Pixel;
use embedded_hal::i2c::I2c;

const ADDR: u8 = 0x3D;
const WIDTH: u32 = 128;
const HEIGHT: u32 = 128;
const PAGES: usize = (HEIGHT as usize) / 8;
const BUF_SIZE: usize = (WIDTH as usize) * (HEIGHT as usize) / 8;

const CMD_PREFIX: u8 = 0x00;
const DATA_PREFIX: u8 = 0x40;

pub struct Sh1107<I> {
    i2c: I,
    buf: [u8; BUF_SIZE],
}

impl<I: I2c> Sh1107<I> {
    pub fn new(i2c: I) -> Self {
        Self {
            i2c,
            buf: [0u8; BUF_SIZE],
        }
    }

    pub fn init(&mut self) -> Result<(), I::Error> {
        // Adafruit SH1107 128x128 init sequence
        let cmds: &[u8] = &[
            0xAE, // display off
            0xD5, 0x51, // set display clock div
            0x20, // set memory mode (page addressing)
            0x81, 0x4F, // set contrast
            0xAD, 0x8A, // internal IREF
            0xA0, // segment remap = 0
            0xC0, // COM scan normal
            0xDC, 0x00, // display start line = 0
            0xD3, 0x00, // display offset = 0
            0xD9, 0x22, // pre-charge period
            0xDB, 0x35, // VCOM deselect level
            0xA8, 0x7F, // multiplex ratio = 127
            0xA4, // entire display on (follow RAM)
            0xA6, // normal display (not inverted)
            0xAF, // display on
        ];
        self.send_commands(cmds)
    }

    pub fn clear_buffer(&mut self) {
        self.buf.fill(0);
    }

    pub fn flush(&mut self) -> Result<(), I::Error> {
        for page in 0..PAGES {
            self.send_commands(&[
                0xB0 | (page as u8), // set page address
                0x00,                // lower column address = 0
                0x10,                // upper column address = 0
            ])?;

            let start = page * (WIDTH as usize);
            let end = start + (WIDTH as usize);

            // Send data in a single I2C write: prefix byte + 128 bytes of page data
            let mut data_buf = [0u8; 1 + WIDTH as usize];
            data_buf[0] = DATA_PREFIX;
            data_buf[1..].copy_from_slice(&self.buf[start..end]);
            self.i2c.write(ADDR, &data_buf)?;
        }
        Ok(())
    }

    fn send_commands(&mut self, cmds: &[u8]) -> Result<(), I::Error> {
        for &cmd in cmds {
            self.i2c.write(ADDR, &[CMD_PREFIX, cmd])?;
        }
        Ok(())
    }
}

impl<I: I2c> OriginDimensions for Sh1107<I> {
    fn size(&self) -> Size {
        Size::new(WIDTH, HEIGHT)
    }
}

impl<I: I2c> DrawTarget for Sh1107<I> {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<P>(&mut self, pixels: P) -> Result<(), Self::Error>
    where
        P: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            let x = point.x;
            let y = point.y;
            if x < 0 || x >= WIDTH as i32 || y < 0 || y >= HEIGHT as i32 {
                continue;
            }
            let x = x as usize;
            let y = y as usize;
            let page = y / 8;
            let bit = y % 8;
            let idx = page * (WIDTH as usize) + x;
            match color {
                BinaryColor::On => self.buf[idx] |= 1 << bit,
                BinaryColor::Off => self.buf[idx] &= !(1 << bit),
            }
        }
        Ok(())
    }
}
