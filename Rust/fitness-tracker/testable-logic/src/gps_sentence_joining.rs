#[derive(Default)]
pub struct GPSSentenceState {
    pub maybe_fix_data: Option<FixData>,
    pub maybe_min_nav_data: Option<MinNavData>,
}

#[derive(Clone, Copy)]
pub enum FitnessTrackerSentence {
    FixData(FixData),
    MinNav(MinNavData),
}

#[derive(Clone, Copy)]
pub struct FixData {
    // 123519.00 -> 12:35:19.00 UTC
    utc_time: chrono::NaiveTime,
    lat: f64,
    lon: f64,
    ns: NS,
    ew: EW,
    fix_method: FixMethod,
    horizontal_geometry_quality: usize,
    num_satellites: usize,
}

impl From<nmea0183::GGA> for FixData {
    fn from(value: nmea0183::GGA) -> Self {
        let secs = value.time.seconds.trunc() as u32;
        let nanos = (value.time.seconds.fract() * 1_000_000_000.0) as u32;
        Self {
            utc_time: chrono::NaiveTime::from_hms_nano_opt(
                value.time.hours as u32,
                value.time.minutes as u32,
                secs,
                nanos,
            )
            .expect("invalid GPS time"),
            lat: value.latitude.as_f64(),
            lon: value.longitude.as_f64(),
            ns: match value.latitude.hemisphere {
                nmea0183::coords::Hemisphere::North => NS::N,
                nmea0183::coords::Hemisphere::South => NS::S,
                _ => unreachable!(),
            },
            ew: match value.longitude.hemisphere {
                nmea0183::coords::Hemisphere::East => EW::E,
                nmea0183::coords::Hemisphere::West => EW::W,
                _ => unreachable!(),
            },
            fix_method: match value.gps_quality {
                nmea0183::GPSQuality::NoFix => FixMethod::Invalid,
                nmea0183::GPSQuality::GPS => FixMethod::GPS,
                nmea0183::GPSQuality::DGPS => FixMethod::DGPS,
                nmea0183::GPSQuality::PPS => FixMethod::PPS,
                nmea0183::GPSQuality::RTK => FixMethod::RTKFixed,
                nmea0183::GPSQuality::FRTK => FixMethod::RTKFloat,
                nmea0183::GPSQuality::Estimated => FixMethod::Estimated,
                nmea0183::GPSQuality::Manual => FixMethod::Manual,
                nmea0183::GPSQuality::Simulated => FixMethod::Simulation,
            },
            horizontal_geometry_quality: value.hdop as usize,
            num_satellites: value.sat_in_use as usize,
        }
    }
}

#[derive(Clone, Copy)]
pub struct MinNavData {}

impl From<nmea0183::RMC> for MinNavData {
    fn from(value: nmea0183::RMC) -> Self {
        Self {}
    }
}

#[derive(Clone, Copy)]
pub enum NS {
    N,
    S,
}

#[derive(Clone, Copy)]
pub enum EW {
    E,
    W,
}

#[derive(Clone, Copy)]
pub enum FixMethod {
    Invalid,
    GPS,
    DGPS,
    PPS,
    RTKFixed,
    RTKFloat,
    Estimated,
    Manual,
    Simulation,
}

impl TryFrom<usize> for FixMethod {
    type Error = &'static str;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Invalid),
            1 => Ok(Self::GPS),
            2 => Ok(Self::DGPS),
            3 => Ok(Self::PPS),
            4 => Ok(Self::RTKFixed),
            5 => Ok(Self::RTKFloat),
            6 => Ok(Self::Estimated),
            7 => Ok(Self::Manual),
            8 => Ok(Self::Simulation),
            _ => Err("unknown fix quality"),
        }
    }
}
