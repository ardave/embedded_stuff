#![allow(clippy::large_stack_frames)]

use core::fmt::Write;

use domain::display_content::{DisplayContent, Mph, NumSatellites};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use esp_hal::i2c::master::I2c;
use esp_println::println;

use crate::sh1107::Sh1107;

#[allow(clippy::large_stack_frames)]
#[embassy_executor::task]
pub async fn display_task(
    i2c: I2c<'static, esp_hal::Async>,
    signal: &'static Signal<CriticalSectionRawMutex, DisplayContent>,
) {
    let mut display = Sh1107::new(i2c);

    if let Err(_e) = display.init().await {
        println!("[Display] init error");
        return;
    }
    println!("[Display] SH1107 initialized");

    display.clear_buffer();
    let _ = display.flush().await;

    let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    let mut line_buf = [0u8; 13];

    loop {
        let content = signal.wait().await;

        display.clear_buffer();

        match content {
            DisplayContent::Reading(mph, num_satellites) => {
                if let Some(Mph(speed)) = mph {
                    let s = format_to_buf(&mut line_buf, |w| write!(w, "{:.1} mph", speed));
                    let _ = Text::new(s, Point::new(0, 30), style).draw(&mut display);
                } else {
                    let _ = Text::new("-- mph", Point::new(0, 30), style).draw(&mut display);
                }
                if let Some(NumSatellites(n)) = num_satellites {
                    let s = format_to_buf(&mut line_buf, |w| write!(w, "{} sats", n));
                    let _ = Text::new(s, Point::new(0, 64), style).draw(&mut display);
                } else {
                    let _ = Text::new("-- sats", Point::new(0, 64), style).draw(&mut display);
                }
            }
            DisplayContent::GPSNoFix => {
                let _ = Text::new("No fix", Point::new(0, 30), style).draw(&mut display);
            }
            DisplayContent::GPSError => {
                let _ = Text::new("GPS Err", Point::new(0, 30), style).draw(&mut display);
            }
            DisplayContent::Initialized => {
                let _ = Text::new("Ready!", Point::new(0, 30), style).draw(&mut display);
            }
            DisplayContent::DbgUartErr => {
                let _ = Text::new("UART Err", Point::new(0, 30), style).draw(&mut display);
            }
        }

        if let Err(_e) = display.flush().await {
            println!("[Display] flush error");
        }
    }
}

struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BufWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
}

impl core::fmt::Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.pos;
        let len = bytes.len().min(remaining);
        self.buf[self.pos..self.pos + len].copy_from_slice(&bytes[..len]);
        self.pos += len;
        Ok(())
    }
}

fn format_to_buf(buf: &mut [u8], f: impl FnOnce(&mut BufWriter) -> core::fmt::Result) -> &str {
    let len = {
        let mut writer = BufWriter::new(buf);
        let _ = f(&mut writer);
        writer.pos
    };
    core::str::from_utf8(&buf[..len]).unwrap_or("")
}
