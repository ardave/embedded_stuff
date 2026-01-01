use anyhow::{bail, Result};
use bme280::i2c::BME280;
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::PinDriver,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    rmt::{config::TransmitConfig, FixedLengthSignal, PinState, Pulse, PulseTicks, TxRmtDriver},
    units::Hertz,
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    http::client::{Configuration as HttpConfig, EspHttpConnection},
    nvs::EspDefaultNvsPartition,
    wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi},
};
use log::{error, info, warn};
use prost::Message;
use std::time::Duration;

// Config from .env file (read at build time)
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
const API_ENDPOINT: &str = env!("API_ENDPOINT");
const DEVICE_ID: &str = env!("DEVICE_ID");

// Hardware configuration for Adafruit QT Py ESP32-C3
const I2C_FREQ_HZ: u32 = 100_000;

const SLEEP_DURATION_MINUTES: u64 = 15;
const WIFI_MAX_RETRIES: u32 = 5;
const HTTP_MAX_RETRIES: u32 = 3;
const HTTP_INITIAL_BACKOFF_MS: u64 = 1000;

// Protobuf message definition
pub mod sensor {
    include!(concat!(env!("OUT_DIR"), "/sensor.rs"));
}

fn set_ws2812_color(tx: &mut TxRmtDriver<'_>, r: u8, g: u8, b: u8) -> Result<()> {
    // WS2812 timing at 10MHz (100ns per tick):
    // T0H = 400ns = 4 ticks, T0L = 850ns = 8.5 ticks
    // T1H = 800ns = 8 ticks, T1L = 450ns = 4.5 ticks
    let t0h = Pulse::new(PinState::High, PulseTicks::new(4)?);
    let t0l = Pulse::new(PinState::Low, PulseTicks::new(8)?);
    let t1h = Pulse::new(PinState::High, PulseTicks::new(8)?);
    let t1l = Pulse::new(PinState::Low, PulseTicks::new(4)?);

    // WS2812 expects GRB order
    let colors = [g, r, b];
    let mut signal = FixedLengthSignal::<24>::new();

    for (i, &color) in colors.iter().enumerate() {
        for bit in (0..8).rev() {
            let index = i * 8 + (7 - bit);
            if (color >> bit) & 1 == 1 {
                signal.set(index, &(t1h, t1l))?;
            } else {
                signal.set(index, &(t0h, t0l))?;
            }
        }
    }

    tx.start_blocking(&signal)?;
    Ok(())
}

fn show_post_result(tx: &mut TxRmtDriver<'_>, success: bool) {
    let (r, g) = if success { (0, 255) } else { (255, 0) };
    info!("POST {}", if success { "SUCCESS" } else { "FAILED" });

    for _ in 0..3 {
        let _ = set_ws2812_color(tx, r, g, 0);
        FreeRtos::delay_ms(80);
        let _ = set_ws2812_color(tx, 0, 0, 0);
        FreeRtos::delay_ms(80);
    }
}

struct SensorReading {
    temperature_c: f32,
    pressure_pa: f32,
    humidity: f32,
}

impl SensorReading {
    fn temperature_f(&self) -> f32 {
        self.temperature_c * 9.0 / 5.0 + 32.0
    }

    fn pressure_inhg(&self) -> f32 {
        self.pressure_pa / 100.0 * 0.02953 // Convert Pa to hPa, then to inHg
    }
}

fn init_bme280<'a>(
    i2c: I2cDriver<'a>,
    delay: &mut FreeRtos,
) -> Result<BME280<I2cDriver<'a>>> {
    let mut bme280 = BME280::new_secondary(i2c);
    bme280.init(delay).map_err(|e| anyhow::anyhow!("BME280 init failed: {:?}", e))?;
    info!("BME280 sensor initialized");
    Ok(bme280)
}

fn read_sensor(bme280: &mut BME280<I2cDriver>, delay: &mut FreeRtos) -> Result<SensorReading> {
    let measurements = bme280
        .measure(delay)
        .map_err(|e| anyhow::anyhow!("Failed to read BME280: {:?}", e))?;

    Ok(SensorReading {
        temperature_c: measurements.temperature,
        pressure_pa: measurements.pressure,
        humidity: measurements.humidity,
    })
}

fn connect_wifi(
    wifi: &mut BlockingWifi<EspWifi<'static>>,
) -> Result<()> {
    let wifi_config = Configuration::Client(ClientConfiguration {
        ssid: WIFI_SSID.try_into().unwrap(),
        password: WIFI_PASSWORD.try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        ..Default::default()
    });

    wifi.set_configuration(&wifi_config)?;
    wifi.start()?;
    info!("WiFi started, connecting to SSID: {}", WIFI_SSID);

    // Connect with retry logic
    for attempt in 1..=WIFI_MAX_RETRIES {
        match wifi.connect() {
            Ok(_) => {
                info!("WiFi connected on attempt {}", attempt);
                break;
            }
            Err(e) => {
                warn!("WiFi connect attempt {}/{} failed: {}", attempt, WIFI_MAX_RETRIES, e);
                if attempt == WIFI_MAX_RETRIES {
                    bail!("Failed to connect to WiFi after {} attempts", WIFI_MAX_RETRIES);
                }
                FreeRtos::delay_ms(1000);
            }
        }
    }

    wifi.wait_netif_up()?;

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;
    info!("Got IP address: {:?}", ip_info.ip);

    Ok(())
}

fn post_sensor_reading(reading: &SensorReading) -> Result<bool> {
    let proto_reading = sensor::SensorReading {
        temperature_f: reading.temperature_f(),
        pressure_inhg: reading.pressure_inhg(),
        humidity_percent: reading.humidity,
        device_id: DEVICE_ID.to_string(),
    };

    info!(
        "Posting reading: T={:.1}F, P={:.2}inHg, H={:.1}%, id={}",
        proto_reading.temperature_f,
        proto_reading.pressure_inhg,
        proto_reading.humidity_percent,
        proto_reading.device_id
    );

    // Encode protobuf
    let mut buf = Vec::new();
    proto_reading.encode(&mut buf)?;
    info!("Encoded protobuf message: {} bytes", buf.len());

    let mut backoff_ms = HTTP_INITIAL_BACKOFF_MS;

    for attempt in 1..=HTTP_MAX_RETRIES {
        match do_http_post(&buf) {
            Ok(status) if (200..300).contains(&status) => {
                info!("POST successful, status code: {}", status);
                return Ok(true);
            }
            Ok(status) => {
                warn!(
                    "POST returned status code: {} (attempt {}/{})",
                    status, attempt, HTTP_MAX_RETRIES
                );
            }
            Err(e) => {
                warn!(
                    "POST failed: {} (attempt {}/{})",
                    e, attempt, HTTP_MAX_RETRIES
                );
            }
        }

        if attempt < HTTP_MAX_RETRIES {
            info!("Retrying in {} ms...", backoff_ms);
            FreeRtos::delay_ms(backoff_ms as u32);
            backoff_ms *= 2;
        }
    }

    error!("All {} retry attempts failed", HTTP_MAX_RETRIES);
    Ok(false)
}

fn do_http_post(body: &[u8]) -> Result<u16> {
    let config = HttpConfig {
        timeout: Some(Duration::from_secs(10)),
        crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
        ..Default::default()
    };

    let mut client = EspHttpConnection::new(&config)?;

    let headers = [
        ("Content-Type", "application/x-protobuf"),
        ("Content-Length", &body.len().to_string()),
    ];

    client.initiate_request(
        esp_idf_svc::http::Method::Post,
        API_ENDPOINT,
        &headers,
    )?;

    client.write_all(body)?;
    client.initiate_response()?;

    let status = client.status();
    Ok(status)
}

fn enter_deep_sleep() -> ! {
    info!("Entering deep sleep for {} minutes...", SLEEP_DURATION_MINUTES);

    unsafe {
        let sleep_duration_us = SLEEP_DURATION_MINUTES * 60 * 1_000_000;
        esp_idf_svc::sys::esp_sleep_enable_timer_wakeup(sleep_duration_us);
        esp_idf_svc::sys::esp_deep_sleep_start();
    }
}

fn main() -> Result<()> {
    // Link ESP-IDF patches and initialize logging
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Weather Server (Rust) starting...");

    // Initialize peripherals
    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // Initialize NeoPixel power (GPIO8) and turn it on
    let mut neopixel_power = PinDriver::output(peripherals.pins.gpio8)?;
    neopixel_power.set_high()?;
    FreeRtos::delay_ms(10);

    // Configure RMT for WS2812 on GPIO2
    let rmt_config = TransmitConfig::new().clock_divider(8); // 80MHz / 8 = 10MHz
    let mut rmt_tx = TxRmtDriver::new(
        peripherals.rmt.channel0,
        peripherals.pins.gpio2,
        &rmt_config,
    )?;

    // Show yellow while connecting
    set_ws2812_color(&mut rmt_tx, 255, 180, 0)?;
    info!("LED initialized");

    // Initialize I2C (SDA=GPIO5, SCL=GPIO6) and BME280
    let i2c_config = I2cConfig::new().baudrate(Hertz(I2C_FREQ_HZ));
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio5,
        peripherals.pins.gpio6,
        &i2c_config,
    )?;

    let mut delay = FreeRtos;
    let mut bme280 = init_bme280(i2c, &mut delay)?;

    // Initialize WiFi
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?,
        sysloop,
    )?;

    if let Err(e) = connect_wifi(&mut wifi) {
        error!("WiFi connection failed: {}", e);
        show_post_result(&mut rmt_tx, false);
        enter_deep_sleep();
    }

    // Read sensor and POST data
    let success = match read_sensor(&mut bme280, &mut delay) {
        Ok(reading) => post_sensor_reading(&reading).unwrap_or(false),
        Err(e) => {
            error!("Failed to read sensor: {}", e);
            false
        }
    };

    // Show result on LED
    show_post_result(&mut rmt_tx, success);

    // Enter deep sleep
    enter_deep_sleep();
}
