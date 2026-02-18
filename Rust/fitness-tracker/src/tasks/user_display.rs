use crate::tasks::sh1107::Sh1107;
use crate::QueueReceiver;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use std::fmt::Write;
use std::thread;

pub fn start<I: I2c + Send + 'static>(
    i2c: I,
    queue: QueueReceiver<DisplayContent>,
) -> thread::JoinHandle<()> {
    ThreadSpawnConfiguration {
        name: Some(b"Display\0"),
        stack_size: 8192,
        priority: 9,
        inherit: false,
        pin_to_core: None,
        ..Default::default()
    }
    .set()
    .expect("Failed to set Display thread config");

    thread::Builder::new()
        .name("Display".to_string())
        .stack_size(8192)
        .spawn(move || {
            let mut display = Sh1107::new(i2c);

            if let Err(e) = display.init() {
                info!("[Display] init error: {:?}", e);
                return;
            }
            info!("[Display] SH1107 initialized");

            // Clear and flush to start with a blank screen
            display.clear_buffer();
            let _ = display.flush();

            let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

            let mut line_buf = [0u8; 13]; // MAX_LINE_LEN + 1 for safety

            loop {
                let (display_content, _) = queue.recv_front(u32::MAX).unwrap();

                display.clear_buffer();

                let DisplayContent {
                    mph,
                    num_satellites,
                } = display_content;

                if let Some(MPH(speed)) = mph {
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

                if let Err(e) = display.flush() {
                    info!("[Display] flush error: {:?}", e);
                }
            }
        })
        .expect("Failed to spawn Display task")
}

#[derive(Clone, Copy, Debug)]
pub struct DisplayContent {
    pub mph: Option<MPH>,
    pub num_satellites: Option<NumSatellites>,
}

#[derive(Clone, Copy, Debug)]
pub struct MPH(pub f32);

#[derive(Clone, Copy, Debug)]
pub struct NumSatellites(pub usize);

struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BufWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
}

impl std::fmt::Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> std::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.pos;
        let len = bytes.len().min(remaining);
        self.buf[self.pos..self.pos + len].copy_from_slice(&bytes[..len]);
        self.pos += len;
        Ok(())
    }
}

fn format_to_buf<'a>(
    buf: &'a mut [u8],
    f: impl FnOnce(&mut BufWriter) -> std::fmt::Result,
) -> &'a str {
    let mut writer = BufWriter::new(buf);
    let _ = f(&mut writer);
    let len = writer.pos;
    // SAFETY: writer is dropped here, releasing the mutable borrow on buf
    drop(writer);
    core::str::from_utf8(&buf[..len]).unwrap_or("")
}
