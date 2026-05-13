use crate::state::State;

use embassy_time::{Duration, Timer};
use esp_idf_hal::uart::{AsyncUartDriver, UartDriver};
#[cfg(not(feature = "fake-gps"))]
use nmea_parser::chrono::Timelike;
use nmea_parser::chrono::{DateTime, Utc};
#[cfg(not(feature = "fake-gps"))]
use nmea_parser::gnss::{GgaData, GgaQualityIndicator};
#[cfg(not(feature = "fake-gps"))]
use nmea_parser::ParsedMessage;
#[cfg(not(feature = "fake-gps"))]
use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::sync::Arc;

#[derive(Copy, Clone, Debug)]
pub struct GpsMeasurement {
    pub timestamp: DateTime<Utc>,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f64,
    pub satellite_count: u8,
}

#[cfg(feature = "fake-gps")]
pub async fn gps_loop<'a>(_: AsyncUartDriver<'a, UartDriver<'a>>, state: Arc<State>) {
    let publisher = state.gps_measurements.publisher().unwrap();
    state.current_satellite_count.store(1, Ordering::Release);

    let center_lat = 48.858370_f64;
    let center_lon = 2.294481_f64;
    let radius_m = 500.0_f64;
    let speed_kmh = 30.0_f64;
    let speed_ms = speed_kmh * 1000.0 / 3600.0;
    let angular_speed_rad_s = speed_ms / radius_m;

    // meters per degree at this latitude (spherical-Earth approximation)
    let m_per_deg_lat = 111_320.0_f64;
    let m_per_deg_lon = m_per_deg_lat * center_lat.to_radians().cos();

    state
        .current_speed_kmh
        .store(speed_kmh as i32, Ordering::Relaxed);

    let start = embassy_time::Instant::now();
    let mut tick: u64 = 0;

    loop {
        let elapsed_s = (embassy_time::Instant::now() - start).as_micros() as f64 / 1_000_000.0;
        let theta = angular_speed_rad_s * elapsed_s;
        let latitude = center_lat + (radius_m / m_per_deg_lat) * theta.sin();
        let longitude = center_lon + (radius_m / m_per_deg_lon) * theta.cos();

        let now = { state.time_source.lock().await.now() };
        publisher.publish_immediate(GpsMeasurement {
            timestamp: now,
            latitude,
            longitude,
            altitude_m: 0.0,
            satellite_count: 1,
        });

        // ~once a minute, hold for 10s instead of 1s to simulate a GPS dropout
        let gap_s = if tick % 60 == 59 { 10 } else { 1 };
        tick += 1;
        Timer::after(Duration::from_secs(gap_s)).await;
    }
}

#[cfg(not(feature = "fake-gps"))]
pub async fn gps_loop<'a>(gps_uart: AsyncUartDriver<'a, UartDriver<'a>>, state: Arc<State>) {
    let mut buffer = [0u8; 83]; // NMEA sentence is max 79 + 3 bytes in length
    let mut buffer_len;
    let mut sentence_end;
    let mut parser = nmea_parser::NmeaParser::new();

    let mut glide_ratio_lat = None;
    let mut glide_ratio_lon = None;
    let mut glide_ratio_alt_m = None;

    let mut glide_ratio_buffer = VecDeque::with_capacity(10);
    struct GlideRatioElem {
        d: f64,
        h: f64,
    }

    impl GlideRatioElem {
        fn ratio(&self) -> f64 {
            self.d / self.h
        }
    }

    let publisher = state.gps_measurements.publisher().unwrap();

    loop {
        buffer_len = 0;
        sentence_end = false;

        while !sentence_end {
            let mut read_buffer = [0x0];
            match gps_uart.read(&mut read_buffer).await {
                Ok(0) => Timer::after(Duration::from_millis(1)).await,
                Ok(_) => {
                    buffer[buffer_len] = read_buffer[0];
                    sentence_end = buffer[buffer_len] as char == '\n';
                    buffer_len += 1;
                }
                Err(e) => panic!("{e:?}"),
            }
        }

        let sentence = String::from_utf8_lossy(&buffer[0..buffer_len]);
        let sentence = sentence.trim();
        let nmea = parser.parse_sentence(sentence);

        // println!("{:?}  => {}", nmea.is_ok(), sentence,);
        if let Ok(ParsedMessage::Rmc(data)) = nmea {
            // println!(
            //     "{:?}  {:?}",
            //     data.latitude.map(|it| it as f32),
            //     data.longitude.map(|it| it as f32)
            // );
            state.current_speed_kmh.store(
                data.sog_knots
                    .map(|speed_knots| (speed_knots * 1.852) as i32)
                    .unwrap_or(-1),
                Ordering::Relaxed,
            );
        } else if let Ok(ParsedMessage::Gga(GgaData {
            timestamp: Some(timestamp),
            latitude: Some(latitude),
            longitude: Some(longitude),
            quality,
            satellite_count: Some(satellite_count),
            altitude: Some(altitude_m),
            ..
        })) = nmea
        {
            if quality != GgaQualityIndicator::Invalid {
                {
                    state.time_source.lock().await.update_fix(timestamp);
                }
                publisher.publish_immediate(GpsMeasurement {
                    timestamp,
                    latitude,
                    longitude,
                    altitude_m,
                    satellite_count,
                });
            }

            state.gps_quality_channel.send(quality).await;
            state
                .current_altitude_gps_m
                .store(altitude_m as i32, Ordering::Relaxed);
            // hardcoding UTC+2 for now, should be in the config wifi interface when we have one
            state
                .current_hours
                .store(((timestamp.hour() + 2) % 24) as u8, Ordering::Relaxed);
            state
                .current_minutes
                .store(timestamp.minute() as u8, Ordering::Relaxed);

            state
                .current_satellite_count
                .store(satellite_count, Ordering::Release);

            let altitude_baro_m = (state
                .current_altitude_baro_calibrated_mm
                .load(Ordering::Relaxed) as f64)
                / 1000.0;
            if let &(Some(old_lat), Some(old_lon), Some(old_alt)) =
                &(glide_ratio_lat, glide_ratio_lon, glide_ratio_alt_m)
            {
                if glide_ratio_buffer.len() >= 10 {
                    glide_ratio_buffer.pop_back();
                }
                glide_ratio_buffer.push_front(GlideRatioElem {
                    d: haversine_distance_m(latitude, longitude, old_lat, old_lon),
                    h: old_alt - altitude_baro_m,
                })
            }
            glide_ratio_lat = Some(latitude);
            glide_ratio_lon = Some(longitude);
            glide_ratio_alt_m = Some(altitude_baro_m);

            state.current_glide_ratio_x10.store(
                (glide_ratio_buffer
                    .iter()
                    .fold(GlideRatioElem { d: 0.0, h: 0.0 }, |mut acc, e| {
                        acc.d += e.d;
                        acc.h += e.h;
                        acc
                    })
                    .ratio()
                    * 10.0) as i32,
                Ordering::Relaxed,
            )
        }
    }
}

#[cfg(not(feature = "fake-gps"))]
fn haversine_distance_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let r_earth = 6371000.0;

    2.0 * r_earth
        * ((((lat2 - lat1) / 2.0).to_radians()).sin().powi(2)
            + lat1.to_radians().cos()
                * lat2.to_radians().cos()
                * (((lon2 - lon1) / 2.0).to_radians()).sin().powi(2))
        .sqrt()
        .asin()
}
