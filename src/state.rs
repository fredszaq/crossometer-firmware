use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embedded_sdmmc::asynchronous::{TimeSource, Timestamp};
use nmea_parser::gnss::GgaQualityIndicator;
use std::sync::atomic::{AtomicI32, AtomicU8, Ordering};

pub struct State {
    pub current_day0: AtomicU8,
    pub current_month0: AtomicU8,
    pub current_year1970: AtomicI32,

    pub current_hours: AtomicU8,
    pub current_minutes: AtomicU8,
    pub current_seconds: AtomicU8,

    pub current_lon_x10_000_000: AtomicI32,
    pub current_lat_x10_000_000: AtomicI32,

    pub current_altitude_gps_m: AtomicI32,
    pub current_altitude_baro_calibrated_mm: AtomicI32,
    pub current_altitude_baro_uncalibrated_mm: AtomicI32,
    pub current_speed_kmh: AtomicI32,

    pub current_altitude_change_mms: AtomicI32,
    pub current_glide_ratio_x10: AtomicI32,

    pub current_satellite_count: AtomicU8,

    pub gps_quality_channel: Channel<CriticalSectionRawMutex, GgaQualityIndicator, 1>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            current_day0: AtomicU8::new(0),
            current_month0: AtomicU8::new(0),
            current_year1970: AtomicI32::new(0),
            current_hours: AtomicU8::new(0),
            current_minutes: AtomicU8::new(0),
            current_seconds: AtomicU8::new(0),
            current_lon_x10_000_000: AtomicI32::new(0),
            current_lat_x10_000_000: AtomicI32::new(0),
            current_altitude_gps_m: AtomicI32::new(-999),
            current_altitude_baro_calibrated_mm: AtomicI32::new(-999999),
            current_altitude_baro_uncalibrated_mm: AtomicI32::new(-999999),
            current_speed_kmh: AtomicI32::new(-1),
            current_altitude_change_mms: AtomicI32::new(0),
            current_glide_ratio_x10: AtomicI32::new(0),
            current_satellite_count: AtomicU8::new(0),
            gps_quality_channel: Channel::new(),
        }
    }
}

impl TimeSource for &State {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: self.current_year1970.load(Ordering::Relaxed) as u8,
            zero_indexed_month: self.current_month0.load(Ordering::Relaxed),
            zero_indexed_day: self.current_day0.load(Ordering::Relaxed),
            hours: self.current_hours.load(Ordering::Relaxed),
            minutes: self.current_minutes.load(Ordering::Relaxed),
            seconds: self.current_seconds.load(Ordering::Relaxed),
        }
    }
}
