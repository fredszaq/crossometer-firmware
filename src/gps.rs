use crate::state::State;
use embassy_time::{Duration, Timer};
use esp_idf_hal::uart::{AsyncUartDriver, UartDriver};
use nmea_parser::chrono::{Datelike, Timelike};
use nmea_parser::gnss::GgaData;
use nmea_parser::ParsedMessage;
use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::sync::Arc;

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
            altitude: Some(altitude),
            ..
        })) = nmea
        {
            state.gps_quality_channel.send(quality).await;
            state
                .current_altitude_gps_m
                .store(altitude as i32, Ordering::Relaxed);
            // hardcoding UTC+2 for now, should be in the config wifi interface when we have one
            state
                .current_hours
                .store(((timestamp.hour() + 2) % 24) as u8, Ordering::Relaxed);
            state
                .current_minutes
                .store(timestamp.minute() as u8, Ordering::Relaxed);
            state
                .current_seconds
                .store(timestamp.second() as u8, Ordering::Relaxed);

            state
                .current_day0
                .store(timestamp.day0() as u8, Ordering::Relaxed);
            state
                .current_month0
                .store(timestamp.month0() as u8, Ordering::Relaxed);
            state
                .current_year1970
                .store(timestamp.year() - 1970, Ordering::Relaxed);

            state
                .current_lat_x10_000_000
                .store((latitude * 10_000_000.0) as i32, Ordering::Relaxed);
            state
                .current_lon_x10_000_000
                .store((longitude * 10_000_000.0) as i32, Ordering::Relaxed);

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
