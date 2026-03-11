#[derive(Clone, Copy, Debug)]
pub enum DisplayContent {
    Reading(Option<Mph>, Option<NumSatellites>),
    GPSError,
    GPSNoFix,
    Initialized,
    /// Debug: UART read returned N bytes
    DbgUartRead(usize),
    /// Debug: UART read error
    DbgUartErr,
    /// Debug: parser produced a result
    DbgParsed,
}

#[derive(Clone, Copy, Debug)]
pub struct Mph(pub f32);

#[derive(Clone, Copy, Debug)]
pub struct NumSatellites(pub usize);
