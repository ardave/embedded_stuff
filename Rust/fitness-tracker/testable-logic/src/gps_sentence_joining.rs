use chrono::{DateTime, Utc};

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
    pub utc_time: chrono::NaiveTime,
    pub lat: f64,
    pub lon: f64,
    pub ns: NS,
    pub ew: EW,
    pub fix_method: FixMethod,
    pub horizontal_geometry_quality: usize,
    pub num_satellites: usize,
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
pub struct MinNavData {
    pub timestamp: DateTime<Utc>,
    pub source: NavSystem,
    pub lat: f64,
    pub lon: f64,
    pub ns: NS,
    pub ew: EW,
    pub speed_mph: f32,
    pub course: Option<f32>,
    pub mode: NavMode,
}

impl From<nmea0183::RMC> for MinNavData {
    fn from(value: nmea0183::RMC) -> Self {
        let d = value.datetime.date;
        let t = value.datetime.time;
        let secs = t.seconds.trunc() as u32;
        let nanos = (t.seconds.fract() * 1_000_000_000.0) as u32;
        let naive = chrono::NaiveDate::from_ymd_opt(d.year as i32, d.month as u32, d.day as u32)
            .expect("invalid GPS date")
            .and_hms_nano_opt(t.hours as u32, t.minutes as u32, secs, nanos)
            .expect("invalid GPS time");
        Self {
            timestamp: DateTime::from_naive_utc_and_offset(naive, Utc),
            source: match value.source {
                nmea0183::Source::GPS => NavSystem::GPS,
                nmea0183::Source::GLONASS => NavSystem::GLONASS,
                nmea0183::Source::Gallileo => NavSystem::Galileo,
                nmea0183::Source::Beidou => NavSystem::Beidou,
                nmea0183::Source::GNSS => NavSystem::GNSS,
            },
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
        }
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
pub enum NavSystem {
    GPS,
    GLONASS,
    Galileo,
    Beidou,
    GNSS,
}

#[derive(Clone, Copy)]
pub enum NavMode {
    Autonomous,
    Differential,
    Estimated,
    Manual,
    Simulator,
    NotValid,
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
