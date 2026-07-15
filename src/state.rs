use crate::barometer::BarometerMeasurement;
use crate::gps::GpsMeasurement;
use crate::sdcard::SdCardStatus;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;
use embedded_sdmmc::asynchronous::{TimeSource, Timestamp};
use nmea_parser::chrono::{DateTime, Datelike, Timelike, Utc};
use nmea_parser::gnss::GgaQualityIndicator;
use std::ops::Deref;
use std::sync::atomic::{AtomicI32, AtomicU8};

pub struct State {
    pub current_hours: AtomicU8,
    pub current_minutes: AtomicU8,

    pub current_altitude_gps_m: AtomicI32,
    pub current_altitude_baro_calibrated_mm: AtomicI32,
    pub current_speed_kmh: AtomicI32,

    pub current_altitude_change_mms: AtomicI32,
    pub current_glide_ratio_x10: AtomicI32,

    pub current_satellite_count: AtomicU8,

    pub barometer_measurements:
        PubSubChannel<CriticalSectionRawMutex, BarometerMeasurement, 4, 3, 1>,
    pub gps_measurements: PubSubChannel<CriticalSectionRawMutex, GpsMeasurement, 4, 3, 1>,

    pub time_source: Mutex<CriticalSectionRawMutex, GpsTimeSource>,

    pub gps_quality_channel: Channel<CriticalSectionRawMutex, GgaQualityIndicator, 1>,

    pub sdcard_status: Watch<CriticalSectionRawMutex, SdCardStatus, 2>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            current_hours: AtomicU8::new(0),
            current_minutes: AtomicU8::new(0),
            current_altitude_gps_m: AtomicI32::new(-999),
            current_altitude_baro_calibrated_mm: AtomicI32::new(-999999),
            current_speed_kmh: AtomicI32::new(-1),
            current_altitude_change_mms: AtomicI32::new(0),
            current_glide_ratio_x10: AtomicI32::new(0),
            current_satellite_count: AtomicU8::new(0),
            barometer_measurements: PubSubChannel::new(),
            gps_measurements: PubSubChannel::new(),
            time_source: Mutex::new(GpsTimeSource::default()),
            gps_quality_channel: Channel::new(),
            sdcard_status: Watch::new(),
        }
    }
}

impl TimeSource for &State {
    fn get_timestamp(&self) -> Timestamp {
        embassy_futures::block_on(async { self.time_source.lock().await.deref().get_timestamp() })
    }
}

pub struct GpsTimeSource {
    last_gps_fix_date: DateTime<Utc>,
    last_gps_fix_instant: std::time::Instant,
}

impl GpsTimeSource {
    pub fn update_fix(&mut self, fix: DateTime<Utc>) {
        self.last_gps_fix_date = fix;
        self.last_gps_fix_instant = std::time::Instant::now();
    }

    pub fn now(&self) -> DateTime<Utc> {
        let elapsed = self.last_gps_fix_instant.elapsed();
        self.last_gps_fix_date + elapsed
    }
}

impl Default for GpsTimeSource {
    fn default() -> Self {
        Self {
            last_gps_fix_date: DateTime::default(),
            last_gps_fix_instant: std::time::Instant::now(),
        }
    }
}

impl TimeSource for &GpsTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        let now = self.now();
        Timestamp {
            year_since_1970: (now.year() - 1970) as u8,
            zero_indexed_month: now.month0() as u8,
            zero_indexed_day: now.day0() as u8,
            hours: now.hour() as u8,
            minutes: now.minute() as u8,
            seconds: now.second() as u8,
        }
    }
}
