#[derive(Clone, Copy, Debug)]
pub enum DisplayContent {
    Reading(Option<Mph>, Option<NumSatellites>),
    GPSError,
    GPSNoFix,
    Initialized,
    /// Debug: UART read error
    DbgUartErr,
}

#[derive(Clone, Copy, Debug)]
pub struct Mph(pub f32);

#[derive(Clone, Copy, Debug)]
pub struct NumSatellites(pub usize);
