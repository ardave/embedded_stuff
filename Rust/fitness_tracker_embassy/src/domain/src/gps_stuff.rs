use embassy_time::Instant;

#[derive(Clone, Copy, Debug)]
pub enum FitnessTrackerSentence {
    FixData(FixQualityData),
    MinNav(RequiredNavData),
}

#[derive(Clone, Copy, Debug)]
pub struct FixQualityData {
    pub utc_time: chrono::NaiveTime,
    pub lat: f64,
    pub lon: f64,
    pub fix_method: FixMethod,
    pub hdop: f32,
    pub num_satellites: usize,
    pub altitude_meters: Option<f32>,
    pub received_at: Instant,
}

impl From<nmea0183::GGA> for FixQualityData {
    fn from(value: nmea0183::GGA) -> Self {
        let secs = value.time.seconds as u32;
        let nanos = ((value.time.seconds - secs as f32) * 1_000_000_000.0) as u32;
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
            hdop: value.hdop,
            num_satellites: value.sat_in_use as usize,
            altitude_meters: value.altitude.map(|a| a.meters),
            received_at: Instant::now(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RequiredNavData {
    pub timestamp: chrono::NaiveDateTime,
    pub source: NavSystem,
    pub lat: f64,
    pub lon: f64,
    pub speed_mph: f32,
    pub course: Option<f32>,
    pub mode: NavMode,
    pub received_at: Instant,
}

impl From<nmea0183::RMC> for RequiredNavData {
    fn from(value: nmea0183::RMC) -> Self {
        let d = value.datetime.date;
        let t = value.datetime.time;
        let secs = t.seconds as u32;
        let nanos = ((t.seconds - secs as f32) * 1_000_000_000.0) as u32;
        let naive = chrono::NaiveDate::from_ymd_opt(d.year as i32, d.month as u32, d.day as u32)
            .expect("invalid GPS date")
            .and_hms_nano_opt(t.hours as u32, t.minutes as u32, secs, nanos)
            .expect("invalid GPS time");
        Self {
            timestamp: naive,
            source: match value.source {
                nmea0183::Source::GPS => NavSystem::GPS,
                nmea0183::Source::GLONASS => NavSystem::GLONASS,
                nmea0183::Source::Gallileo => NavSystem::Galileo,
                nmea0183::Source::Beidou => NavSystem::Beidou,
                nmea0183::Source::GNSS => NavSystem::GNSS,
            },
            lat: value.latitude.as_f64(),
            lon: value.longitude.as_f64(),
            speed_mph: value.speed.as_mph(),
            course: value.course.map(|c| c.degrees),
            mode: match value.mode {
                nmea0183::Mode::Autonomous => NavMode::Autonomous,
                nmea0183::Mode::Differential => NavMode::Differential,
                nmea0183::Mode::Estimated => NavMode::Estimated,
                nmea0183::Mode::Manual => NavMode::Manual,
                nmea0183::Mode::Simulator => NavMode::Simulator,
                nmea0183::Mode::NotValid => NavMode::NotValid,
            },
            received_at: Instant::now(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum NavSystem {
    GPS,
    GLONASS,
    Galileo,
    Beidou,
    GNSS,
}

#[derive(Clone, Copy, Debug)]
pub enum NavMode {
    Autonomous,
    Differential,
    Estimated,
    Manual,
    Simulator,
    NotValid,
}

#[derive(Clone, Copy, Debug)]
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

#[derive(Clone, Copy, Debug)]
pub enum GPSAcquisitionError {
    UartReadError,
    NMEAParseError,
    NoFix,
}
