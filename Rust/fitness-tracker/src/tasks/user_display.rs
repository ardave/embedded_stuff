use crate::tasks::sh1107::Sh1107;
use esp_idf_svc::hal::task::queue::Queue;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use std::thread;

pub const MAX_LINE_LEN: usize = 12; // 128px / 10px per char (FONT_10X20)

#[derive(Clone, Copy)]
pub struct DisplayLine {
    buf: [u8; MAX_LINE_LEN],
    len: u8,
}

impl DisplayLine {
    pub fn new(s: &str) -> Self {
        let mut buf = [0u8; MAX_LINE_LEN];
        let len = s.len().min(MAX_LINE_LEN);
        buf[..len].copy_from_slice(&s.as_bytes()[..len]);
        Self {
            buf,
            len: len as u8,
        }
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len as usize]).unwrap_or("")
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum DisplayMessage {
    Line1(DisplayLine),
    Line2(DisplayLine),
}

pub fn start<I: I2c + Send + 'static>(
    i2c: I,
    queue: &'static Queue<DisplayMessage>,
) -> thread::JoinHandle<()> {
    ThreadSpawnConfiguration {
        name: Some(b"Display\0"),
        stack_size: 4096,
        priority: 4,
        inherit: false,
        pin_to_core: None,
        ..Default::default()
    }
    .set()
    .expect("Failed to set Display thread config");

    thread::Builder::new()
        .name("Display".to_string())
        .stack_size(4096)
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

            let mut line1 = DisplayLine::new("");
            let mut line2 = DisplayLine::new("");

            let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

            loop {
                let (msg, _) = queue.recv_front(u32::MAX).unwrap();
                match msg {
                    DisplayMessage::Line1(l) => line1 = l,
                    DisplayMessage::Line2(l) => line2 = l,
                }

                display.clear_buffer();
                let _ = Text::new(line1.as_str(), Point::new(0, 30), style).draw(&mut display);
                let _ = Text::new(line2.as_str(), Point::new(0, 64), style).draw(&mut display);

                if let Err(e) = display.flush() {
                    info!("[Display] flush error: {:?}", e);
                }
            }
        })
        .expect("Failed to spawn Display task")
}
