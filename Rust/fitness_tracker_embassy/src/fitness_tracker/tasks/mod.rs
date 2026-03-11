pub mod display;
#[cfg(feature = "fake-gps")]
pub mod fake_gps;
#[cfg(not(feature = "fake-gps"))]
pub mod gps_acquisition;
pub mod gps_aggregator;
pub mod usb_otg_logger;