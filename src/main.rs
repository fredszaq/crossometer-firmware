use bme280::spi::BME280;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver};
use esp_idf_hal::prelude::*;
use esp_idf_svc::hal::peripherals::Peripherals;
use nmea_parser::chrono::Timelike;
use nmea_parser::gnss::GgaData;
use nmea_parser::ParsedMessage;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicI32, AtomicU8, Ordering};
use std::sync::Arc;
use std::time::Duration;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let uart_gps = peripherals.uart2;
    let uart_gps_tx_pin = pins.gpio16;
    let uart_gps_rx_pin = pins.gpio17;

    let spi2 = peripherals.spi2;
    let spi2_sclk = pins.gpio14;
    let spi2_mosi = pins.gpio13;
    let spi2_miso = pins.gpio26;
    let spi2_cs = pins.gpio27;

    let i2c_sda0 = pins.gpio21;
    let i2c_scl0 = pins.gpio22;

    let i2c0 = esp_idf_hal::i2c::I2cDriver::new(
        peripherals.i2c0,
        i2c_sda0,
        i2c_scl0,
        &esp_idf_hal::i2c::config::Config::new().baudrate(100000.Hz()),
    )
    .unwrap();

    use esp_idf_hal::spi;
    let config = <spi::config::Config as Default>::default().baudrate(9600.Hz());

    let spi2 = spi::SpiDriver::new(
        spi2,
        spi2_sclk,
        spi2_mosi,
        Some(spi2_miso),
        &spi::SpiDriverConfig::new(),
    )
    .unwrap();

    let spi2device = spi::SpiDeviceDriver::new(spi2, Some(spi2_cs), &config).unwrap();

    let mut bmp280 = BME280::new(spi2device).unwrap();

    bmp280.init(&mut esp_idf_hal::delay::FreeRtos).unwrap();

    let current_hours = Arc::new(AtomicU8::new(0));
    let current_minutes = Arc::new(AtomicU8::new(0));

    let current_altitude_gps_m = Arc::new(AtomicI32::new(-999));
    let current_altitude_baro_mm = Arc::new(AtomicI32::new(-999999));
    let current_speed_kmh = Arc::new(AtomicI32::new(-1));

    let current_altitude_change_mms = Arc::new(AtomicI32::new(0));
    let current_glide_ratio_x10 = Arc::new(AtomicI32::new(0));

    let current_altitude_change_mms_ = Arc::clone(&current_altitude_change_mms);
    let current_altitude_gps_m_ = Arc::clone(&current_altitude_gps_m);
    let current_altitude_baro_mm_ = Arc::clone(&current_altitude_baro_mm);

    let current_satellite_count = Arc::new(AtomicU8::new(0));
    let (gps_quality_tx, gps_quality_rx) = std::sync::mpsc::sync_channel(1);

    std::thread::spawn(move || {
        // init altitude to a fairly negative value so that we get a nice welcome sound as the
        // current altitude moves to the actual measured value
        let mut old_altitude_m = -100.0;
        let mut old_altitude_change_ms = 0.0;
        let mut last_measure_time = std::time::Instant::now();
        let mut calibrated = false;
        let mut sea_level_p = 101325.0;

        loop {
            let measurements = bmp280.measure(&mut esp_idf_hal::delay::FreeRtos).unwrap();
            let measure_time = std::time::Instant::now();

            if !calibrated {
                let current_altitude_gps_m = current_altitude_gps_m_.load(Ordering::SeqCst);
                if current_altitude_gps_m > 0 {
                    // https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf page 17
                    sea_level_p = measurements.pressure as f64
                        / (1.0 - current_altitude_gps_m as f64 / 44330.0).powf(5.255);
                    calibrated = true;
                }
            }

            // https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf page 16
            let altitude_m =
                44330.0 * (1.0 - (measurements.pressure as f64 / sea_level_p).powf(0.190294957));

            let elapsed = (measure_time - last_measure_time).as_secs_f64();
            // apply a bit of smoothing on the data
            let altitude_change_ms =
                old_altitude_change_ms * 0.8 + 0.2 * ((altitude_m - old_altitude_m) / elapsed);

            // println!(
            //     "measure_loop:{},{},{},{}",
            //     altitude_change_ms as f32,
            //     altitude_m as f32,
            //     measurements.pressure as f32,
            //     elapsed as f32
            // );

            old_altitude_m = altitude_m;
            old_altitude_change_ms = altitude_change_ms;
            last_measure_time = measure_time;
            current_altitude_baro_mm_.store((altitude_m * 1000.0) as i32, Ordering::SeqCst);
            current_altitude_change_mms_
                .store((altitude_change_ms * 1000.0) as i32, Ordering::SeqCst);
        }
    });

    let current_hours_ = Arc::clone(&current_hours);
    let current_minutes_ = Arc::clone(&current_minutes);
    let current_altitude_change_mms_ = Arc::clone(&current_altitude_change_mms);
    let current_altitude_gps_m_ = Arc::clone(&current_altitude_gps_m);
    let current_altitude_baro_mm_ = Arc::clone(&current_altitude_baro_mm);
    let current_speed_kmh_ = Arc::clone(&current_speed_kmh);
    let current_glide_ratio_x10_ = Arc::clone(&current_glide_ratio_x10);
    let current_satellite_count_ = Arc::clone(&current_satellite_count);

    std::thread::Builder::new()
        .stack_size(131072)
        .spawn(move || {
            use ssd1306::mode::DisplayConfig;
            let interface = ssd1306::I2CDisplayInterface::new(i2c0);

            log::info!("got interface");
            let mut display = ssd1306::Ssd1306::new(
                interface,
                ssd1306::size::DisplaySize128x64,
                ssd1306::rotation::DisplayRotation::Rotate0,
            )
            .into_buffered_graphics_mode();
            log::info!("got display");
            display.init().unwrap();
            log::info!("display init ok");

            use embedded_graphics::Drawable;

            let text_style_data = embedded_graphics::mono_font::MonoTextStyleBuilder::new()
                .font(&embedded_graphics::mono_font::ascii::FONT_10X20)
                .text_color(embedded_graphics::pixelcolor::BinaryColor::On)
                .build();

            let text_style_units = embedded_graphics::mono_font::MonoTextStyleBuilder::new()
                .font(&embedded_graphics::mono_font::ascii::FONT_5X8)
                .text_color(embedded_graphics::pixelcolor::BinaryColor::On)
                .build();

            let mut gps_qual = None;

            loop {
                display.clear_buffer();

                if let Ok(qual) = gps_quality_rx.try_recv() {
                    gps_qual = Some(qual)
                }

                let altitude_gps_m = current_altitude_gps_m_.load(Ordering::SeqCst);
                embedded_graphics::text::Text::with_baseline(
                    &format!("{:>4}", altitude_gps_m),
                    embedded_graphics::geometry::Point::zero(),
                    text_style_data,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                embedded_graphics::text::Text::with_baseline(
                    "m",
                    embedded_graphics::geometry::Point::new(40, 10),
                    text_style_units,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                embedded_graphics::text::Text::with_baseline(
                    "GPS",
                    embedded_graphics::geometry::Point::new(40, 0),
                    text_style_units,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                let altitude_baro_m = current_altitude_baro_mm_.load(Ordering::SeqCst) / 1000;
                embedded_graphics::text::Text::with_baseline(
                    &format!("{:>4}", altitude_baro_m),
                    embedded_graphics::geometry::Point::new(67, 22),
                    text_style_data,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                embedded_graphics::text::Text::with_baseline(
                    "m",
                    embedded_graphics::geometry::Point::new(107, 32),
                    text_style_units,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                embedded_graphics::text::Text::with_baseline(
                    "BARO",
                    embedded_graphics::geometry::Point::new(107, 22),
                    text_style_units,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                let speed_kmh = current_speed_kmh_.load(Ordering::SeqCst);

                embedded_graphics::text::Text::with_baseline(
                    &format!("{:>3}", speed_kmh),
                    embedded_graphics::geometry::Point::new(77, 0),
                    text_style_data,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                embedded_graphics::text::Text::with_baseline(
                    "km/h",
                    embedded_graphics::geometry::Point::new(107, 10),
                    text_style_units,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                let altitude_change_mms = current_altitude_change_mms_.load(Ordering::SeqCst);
                // TODO this is positively ugly and we should use rust formating of floats but it sometimes crashes on esp32...
                embedded_graphics::text::Text::with_baseline(
                    &format!(
                        "{}{}.{}",
                        if altitude_change_mms.is_negative() {
                            "-"
                        } else {
                            "+"
                        },
                        altitude_change_mms.abs() / 1000,
                        (altitude_change_mms.abs() % 1000) / 100
                    ),
                    embedded_graphics::geometry::Point::new(0, 22),
                    text_style_data,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();
                embedded_graphics::text::Text::with_baseline(
                    "m/s",
                    embedded_graphics::geometry::Point::new(40, 32),
                    text_style_units,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                let current_glide_ratio_x10_ = current_glide_ratio_x10_.load(Ordering::SeqCst);
                // TODO this is positively ugly and we should use rust formating of floats but it sometimes crashes on esp32...
                embedded_graphics::text::Text::with_baseline(
                    &format!(
                        "{}{}.{}",
                        if current_glide_ratio_x10_.is_negative() {
                            "-"
                        } else {
                            " "
                        },
                        current_glide_ratio_x10_.abs() / 10,
                        current_glide_ratio_x10_.abs() % 10
                    ),
                    embedded_graphics::geometry::Point::new(0, 44),
                    text_style_data,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                embedded_graphics::text::Text::with_baseline(
                    &if let Some(qual) = gps_qual {
                        format!("{}", qual)
                    } else {
                        "None".to_string()
                    },
                    embedded_graphics::geometry::Point::new(60, 44),
                    text_style_units,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                embedded_graphics::text::Text::with_baseline(
                    &format!("{}sat", current_satellite_count_.load(Ordering::SeqCst),),
                    embedded_graphics::geometry::Point::new(103, 44),
                    text_style_units,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                embedded_graphics::text::Text::with_baseline(
                    &format!(
                        "{:02}:{:02}",
                        current_hours_.load(Ordering::SeqCst),
                        current_minutes_.load(Ordering::SeqCst)
                    ),
                    embedded_graphics::geometry::Point::new(103, 56),
                    text_style_units,
                    embedded_graphics::text::Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                display.flush().unwrap();
                std::thread::sleep(Duration::from_millis(200));
            }
        })
        .unwrap();

    let current_altitude_baro_mm_ = Arc::clone(&current_altitude_baro_mm);
    let current_glide_ratio_x10_ = Arc::clone(&current_glide_ratio_x10);

    std::thread::Builder::new()
        .spawn(move || {
            let config = esp_idf_hal::uart::UartConfig::new().baudrate(9600.Hz());
            let serial = esp_idf_hal::uart::UartDriver::new(
                uart_gps,
                uart_gps_tx_pin,
                uart_gps_rx_pin,
                Option::<esp_idf_hal::gpio::Gpio0>::None,
                Option::<esp_idf_hal::gpio::Gpio0>::None,
                &config,
            )
            .unwrap();

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
                    match serial.read(&mut read_buffer, esp_idf_hal::delay::BLOCK) {
                        Ok(0) => {
                            // can this happen in real life ? we can probably remove this
                            // leftover from esp_idf_hal 0.39 migration
                            std::thread::sleep(Duration::from_millis(10));
                        }
                        Ok(_) => {
                            buffer[buffer_len] = read_buffer[0];
                            sentence_end = buffer[buffer_len] as char == '\n';
                            buffer_len += 1;
                        }
                        Err(e) => panic!("{:?}", e),
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
                    current_speed_kmh.store(
                        data.sog_knots
                            .map(|speed_knots| (speed_knots * 1.852) as i32)
                            .unwrap_or(-1),
                        Ordering::SeqCst,
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
                    gps_quality_tx.send(quality).unwrap();
                    current_altitude_gps_m.store(altitude as i32, Ordering::SeqCst);
                    // hardcoding UTC+2 for now, should be in the config wifi interface when we have one
                    current_hours.store(((timestamp.hour() + 2) % 24) as u8, Ordering::SeqCst);
                    current_minutes.store(timestamp.minute() as u8, Ordering::SeqCst);
                    current_satellite_count.store(satellite_count, Ordering::SeqCst);

                    let altitude_baro_m =
                        (current_altitude_baro_mm_.load(Ordering::SeqCst) as f64) / 1000.0;
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

                    current_glide_ratio_x10_.store(
                        (glide_ratio_buffer
                            .iter()
                            .fold(GlideRatioElem { d: 0.0, h: 0.0 }, |mut acc, e| {
                                acc.d += e.d;
                                acc.h += e.h;
                                acc
                            })
                            .ratio()
                            * 10.0) as i32,
                        Ordering::SeqCst,
                    )
                }
            }
        })
        .unwrap();

    let mut timer_hw = peripherals.ledc.timer0;
    let mut channel_hw = peripherals.ledc.channel0;
    let mut pin_hw = pins.gpio25;

    loop {
        let altitude_change_mms = current_altitude_change_mms.load(Ordering::SeqCst);
        let freq = altitude_change_to_freq(altitude_change_mms);
        let duration = altitude_change_to_beep_duration(altitude_change_mms);
        // println!(
        //     "buzzer_loop:{},{}",
        //     freq.unwrap_or(0.Hz()).0,
        //     duration.as_millis()
        // );

        if let Some(freq) = freq {
            let config = TimerConfig::default().frequency(freq);

            let mut timer = LedcDriver::new(
                &mut channel_hw,
                LedcTimerDriver::new(&mut timer_hw, &config).unwrap(),
                &mut pin_hw,
            )
            .unwrap();

            let max_duty = timer.get_max_duty();
            timer.set_duty(max_duty / 2).unwrap();
            std::thread::sleep(duration);
        } else {
            std::thread::sleep(duration);
        }

        let altitude_change_mms = current_altitude_change_mms.load(Ordering::SeqCst);
        let duration = altitude_change_to_no_beep_duration(altitude_change_mms);
        std::thread::sleep(duration);
    }
}

fn altitude_change_to_freq(altitude_change_mms: i32) -> Option<Hertz> {
    if altitude_change_mms > -3000 && altitude_change_mms < 555 {
        None
    } else {
        Some(
            ((2000 + altitude_change_mms / 3).clamp(90 /* -5.7 m/s */, 5000 /* ~ +9m/s */) as u32)
                .Hz(),
        )
    }
}

fn altitude_change_to_beep_duration(altitude_change_mms: i32) -> Duration {
    Duration::from_millis((150 - altitude_change_mms / 33).clamp(60, 400) as u64)
}

fn altitude_change_to_no_beep_duration(altitude_change_mms: i32) -> Duration {
    Duration::from_millis((150 - altitude_change_mms / 33).clamp(60, 400) as u64)
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
