use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GgaData {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f32,
    pub satellite_count: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RmcData {
    pub speed_knots: f32,
    pub course_degrees: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GpsSentence {
    Gga(GgaData),
    Rmc(RmcData),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsReading {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f32,
    pub speed_knots: f32,
    pub course_degrees: f32,
    pub satellite_count: u8,
}

pub struct ProcessResult {
    pub sentence: GpsSentence,
    pub reading: Option<GpsReading>,
}

pub struct GpsJoiner {
    last_gga: Option<(GgaData, Instant)>,
    last_rmc: Option<(RmcData, Instant)>,
    max_age: Duration,
}

impl GpsJoiner {
    pub fn new(max_age: Duration) -> Self {
        Self {
            last_gga: None,
            last_rmc: None,
            max_age,
        }
    }

    pub fn process_gga(&mut self, gga: GgaData, now: Instant) -> ProcessResult {
        self.last_gga = Some((gga, now));
        let reading = match &self.last_rmc {
            Some((rmc, rmc_time)) if now.duration_since(*rmc_time) < self.max_age => {
                Some(assemble_reading(&gga, rmc))
            }
            _ => None,
        };
        ProcessResult {
            sentence: GpsSentence::Gga(gga),
            reading,
        }
    }

    pub fn process_rmc(&mut self, rmc: RmcData, now: Instant) -> ProcessResult {
        self.last_rmc = Some((rmc, now));
        ProcessResult {
            sentence: GpsSentence::Rmc(rmc),
            reading: None,
        }
    }
}

fn assemble_reading(gga: &GgaData, rmc: &RmcData) -> GpsReading {
    GpsReading {
        latitude: gga.latitude,
        longitude: gga.longitude,
        altitude_m: gga.altitude_m,
        speed_knots: rmc.speed_knots,
        course_degrees: rmc.course_degrees,
        satellite_count: gga.satellite_count,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_gga() -> GgaData {
        GgaData {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude_m: 10.0,
            satellite_count: 8,
        }
    }

    fn sample_rmc() -> RmcData {
        RmcData {
            speed_knots: 5.5,
            course_degrees: 180.0,
        }
    }

    #[test]
    fn gga_without_prior_rmc() {
        let mut joiner = GpsJoiner::new(Duration::from_secs(2));
        let result = joiner.process_gga(sample_gga(), Instant::now());
        assert_eq!(result.sentence, GpsSentence::Gga(sample_gga()));
        assert!(result.reading.is_none());
    }

    #[test]
    fn rmc_never_produces_reading() {
        let mut joiner = GpsJoiner::new(Duration::from_secs(2));
        let result = joiner.process_rmc(sample_rmc(), Instant::now());
        assert_eq!(result.sentence, GpsSentence::Rmc(sample_rmc()));
        assert!(result.reading.is_none());
    }

    #[test]
    fn fresh_rmc_then_gga_produces_reading() {
        let mut joiner = GpsJoiner::new(Duration::from_secs(2));
        let t0 = Instant::now();
        joiner.process_rmc(sample_rmc(), t0);
        let result = joiner.process_gga(sample_gga(), t0);
        assert!(result.reading.is_some());
        let reading = result.reading.unwrap();
        assert_eq!(reading.latitude, sample_gga().latitude);
        assert_eq!(reading.longitude, sample_gga().longitude);
        assert_eq!(reading.altitude_m, sample_gga().altitude_m);
        assert_eq!(reading.speed_knots, sample_rmc().speed_knots);
        assert_eq!(reading.course_degrees, sample_rmc().course_degrees);
        assert_eq!(reading.satellite_count, sample_gga().satellite_count);
    }

    #[test]
    fn stale_rmc_then_gga_no_reading() {
        let mut joiner = GpsJoiner::new(Duration::from_secs(2));
        let t0 = Instant::now();
        joiner.process_rmc(sample_rmc(), t0);
        let result = joiner.process_gga(sample_gga(), t0 + Duration::from_secs(3));
        assert!(result.reading.is_none());
    }

    #[test]
    fn rmc_exactly_at_threshold_no_reading() {
        let mut joiner = GpsJoiner::new(Duration::from_secs(2));
        let t0 = Instant::now();
        joiner.process_rmc(sample_rmc(), t0);
        // 2s is not < 2s, so no reading
        let result = joiner.process_gga(sample_gga(), t0 + Duration::from_secs(2));
        assert!(result.reading.is_none());
    }

    #[test]
    fn assemble_reading_maps_fields() {
        let gga = sample_gga();
        let rmc = sample_rmc();
        let reading = assemble_reading(&gga, &rmc);
        assert_eq!(reading.latitude, gga.latitude);
        assert_eq!(reading.longitude, gga.longitude);
        assert_eq!(reading.altitude_m, gga.altitude_m);
        assert_eq!(reading.speed_knots, rmc.speed_knots);
        assert_eq!(reading.course_degrees, rmc.course_degrees);
        assert_eq!(reading.satellite_count, gga.satellite_count);
    }
}
