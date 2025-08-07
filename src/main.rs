use bme280::spi::AsyncBME280;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embedded_sdmmc::asynchronous::{
    BlockDevice, File, Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};
use esp_idf_hal::gpio::Gpio25;
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver, CHANNEL0, TIMER0};
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::{SpiDeviceDriver, SpiDriver};
use esp_idf_hal::task::block_on;
use esp_idf_hal::uart::{AsyncUartDriver, UartConfig, UartDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use igc_parser::records::file_header::FileHeader;
use igc_parser::records::fix::Fix;
use igc_parser::records::flight_recorder_id::FlightRecorderID;
use igc_parser::records::Record;
use nmea_parser::chrono::{Datelike, Timelike};
use nmea_parser::gnss::{GgaData, GgaQualityIndicator};
use nmea_parser::ParsedMessage;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::I2CInterface;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::Ssd1306;
use std::collections::VecDeque;
use std::fmt::{Display, Formatter};
use std::rc::Rc;
use std::sync::atomic::{AtomicI32, AtomicU8, Ordering};
use std::sync::Arc;

struct State {
    current_day0: AtomicU8,
    current_month0: AtomicU8,
    current_year1970: AtomicI32,

    current_hours: AtomicU8,
    current_minutes: AtomicU8,
    current_seconds: AtomicU8,

    current_lon_x10_000_000: AtomicI32,
    current_lat_x10_000_000: AtomicI32,

    current_altitude_gps_m: AtomicI32,
    current_altitude_baro_calibrated_mm: AtomicI32,
    current_altitude_baro_uncalibrated_mm: AtomicI32,
    current_speed_kmh: AtomicI32,

    current_altitude_change_mms: AtomicI32,
    current_glide_ratio_x10: AtomicI32,

    current_satellite_count: AtomicU8,

    gps_quality_channel: Channel<CriticalSectionRawMutex, GgaQualityIndicator, 1>,
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
    let spi2_config = <spi::config::Config as Default>::default().baudrate(9600.Hz());

    let spi2 = spi::SpiDriver::new(
        spi2,
        spi2_sclk,
        spi2_mosi,
        Some(spi2_miso),
        &spi::SpiDriverConfig::new(),
    )
    .unwrap();

    let spi2device = spi::SpiDeviceDriver::new(spi2, Some(spi2_cs), &spi2_config).unwrap();

    let bmp280 = AsyncBME280::new(spi2device).unwrap();

    let state = Arc::new(State::default());

    let state_ = Arc::clone(&state);

    std::thread::Builder::new()
        .stack_size(64635)
        .spawn(move || {
            block_on(baro_loop(bmp280, state_));
        })
        .unwrap();

    let state_ = Arc::clone(&state);
    let interface = ssd1306::I2CDisplayInterface::new(i2c0);

    log::info!("got interface");
    let display = ssd1306::Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    log::info!("got display");

    std::thread::Builder::new()
        .stack_size(64635)
        .name("display_thread".to_string())
        .spawn(move || {
            block_on(display_loop(display, state_));
        })
        .unwrap();

    let state_ = Arc::clone(&state);

    let config = UartConfig::new().baudrate(9600.Hz());
    let serial = AsyncUartDriver::wrap(
        UartDriver::new(
            uart_gps,
            uart_gps_tx_pin,
            uart_gps_rx_pin,
            Option::<esp_idf_hal::gpio::Gpio0>::None,
            Option::<esp_idf_hal::gpio::Gpio0>::None,
            &config,
        )
        .unwrap(),
    )
    .unwrap();

    std::thread::Builder::new()
        .name("gps_thread".to_string())
        .stack_size(64635)
        .spawn(move || {
            block_on(gps_loop(serial, state_));
        })
        .unwrap();

    let spi3 = peripherals.spi3;
    let spi3_sclk = pins.gpio18;
    let spi3_mosi = pins.gpio23;
    let spi3_miso = pins.gpio19;
    let spi3_cs = pins.gpio5;

    let spi3 = spi::SpiDriver::new(
        spi3,
        spi3_sclk,
        spi3_mosi,
        Some(spi3_miso),
        &spi::SpiDriverConfig::new(),
    )
    .unwrap();

    let spi3_config = <spi::config::Config as Default>::default().baudrate(400_000.Hz());

    let spi3device = spi::SpiDeviceDriver::new(spi3, Some(spi3_cs), &spi3_config).unwrap();

    block_on(embassy_futures::join::join(
        beep_loop(
            Arc::clone(&state),
            peripherals.ledc.timer0,
            peripherals.ledc.channel0,
            pins.gpio25,
        ),
        sdcard_loop(
            embedded_sdmmc::asynchronous::SdCard::new(spi3device, embassy_time::Delay),
            state,
        ),
    ));
}

async fn beep_loop(
    state: Arc<State>,
    mut timer_hw: TIMER0,
    mut channel_hw: CHANNEL0,
    mut pin_hw: Gpio25,
) {
    loop {
        let altitude_change_mms = state.current_altitude_change_mms.load(Ordering::Acquire);
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
            Timer::after(duration).await
        } else {
            Timer::after(duration).await
        }
        let altitude_change_mms = state.current_altitude_change_mms.load(Ordering::Acquire);
        let duration = altitude_change_to_no_beep_duration(altitude_change_mms);
        Timer::after(duration).await
    }
}

async fn display_loop<'a>(
    mut display: Ssd1306<
        I2CInterface<I2cDriver<'a>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
    state_: Arc<State>,
) {
    use ssd1306::mode::DisplayConfig;
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

        if let Ok(qual) = state_.gps_quality_channel.try_receive() {
            gps_qual = Some(qual)
        }

        let altitude_gps_m = state_.current_altitude_gps_m.load(Ordering::Acquire);
        embedded_graphics::text::Text::with_baseline(
            &format!("{altitude_gps_m:>4}"),
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

        let altitude_baro_m = state_
            .current_altitude_baro_calibrated_mm
            .load(Ordering::Relaxed)
            / 1000;
        embedded_graphics::text::Text::with_baseline(
            &format!("{altitude_baro_m:>4}"),
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

        let speed_kmh = state_.current_speed_kmh.load(Ordering::Relaxed);

        embedded_graphics::text::Text::with_baseline(
            &format!("{speed_kmh:>3}"),
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

        let altitude_change_mms = state_.current_altitude_change_mms.load(Ordering::Relaxed);
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

        let current_glide_ratio_x10_ = state_.current_glide_ratio_x10.load(Ordering::Relaxed);
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
                format!("{qual}")
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
            &format!(
                "{}sat",
                state_.current_satellite_count.load(Ordering::Relaxed),
            ),
            embedded_graphics::geometry::Point::new(103, 44),
            text_style_units,
            embedded_graphics::text::Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        embedded_graphics::text::Text::with_baseline(
            &format!(
                "{:02}:{:02}",
                state_.current_hours.load(Ordering::Relaxed),
                state_.current_minutes.load(Ordering::Relaxed)
            ),
            embedded_graphics::geometry::Point::new(103, 56),
            text_style_units,
            embedded_graphics::text::Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        display.flush().unwrap();
        Timer::after(Duration::from_millis(200)).await
    }
}
async fn gps_loop<'a>(serial: AsyncUartDriver<'a, UartDriver<'a>>, state_: Arc<State>) {
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
            match serial.read(&mut read_buffer).await {
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
            state_.current_speed_kmh.store(
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
            state_.gps_quality_channel.send(quality).await;
            state_
                .current_altitude_gps_m
                .store(altitude as i32, Ordering::Relaxed);
            // hardcoding UTC+2 for now, should be in the config wifi interface when we have one
            state_
                .current_hours
                .store(((timestamp.hour() + 2) % 24) as u8, Ordering::Relaxed);
            state_
                .current_minutes
                .store(timestamp.minute() as u8, Ordering::Relaxed);
            state_
                .current_seconds
                .store(timestamp.second() as u8, Ordering::Relaxed);

            state_
                .current_day0
                .store(timestamp.day0() as u8, Ordering::Relaxed);
            state_
                .current_month0
                .store(timestamp.month0() as u8, Ordering::Relaxed);
            state_
                .current_year1970
                .store(timestamp.year() - 1970, Ordering::Relaxed);

            state_
                .current_lat_x10_000_000
                .store((latitude * 10_000_000.0) as i32, Ordering::Relaxed);
            state_
                .current_lon_x10_000_000
                .store((longitude * 10_000_000.0) as i32, Ordering::Relaxed);

            state_
                .current_satellite_count
                .store(satellite_count, Ordering::Release);

            let altitude_baro_m = (state_
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

            state_.current_glide_ratio_x10.store(
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

#[path = "../config.rs"]
mod config;

async fn baro_loop<'a>(
    mut bmp280: AsyncBME280<SpiDeviceDriver<'a, SpiDriver<'a>>>,
    state_: Arc<State>,
) {
    // init altitude to a fairly negative value so that we get a nice welcome sound as the
    // current altitude moves to the actual measured value
    let mut old_altitude_m = -100.0;
    let mut old_altitude_change_ms = 0.0;
    let mut last_measure_time = std::time::Instant::now();
    let mut calibrated = false;
    let sea_level_uncalibrated_p = 101325.0;
    let mut sea_level_p = sea_level_uncalibrated_p;
    bmp280.init(&mut embassy_time::Delay).await.unwrap();

    loop {
        let measurements = bmp280.measure(&mut embassy_time::Delay).await.unwrap();
        let measure_time = std::time::Instant::now();

        if !calibrated {
            let current_altitude_gps_m = state_.current_altitude_gps_m.load(Ordering::Acquire);
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

        let altitude_uncalibrated_m = 44330.0
            * (1.0 - (measurements.pressure as f64 / sea_level_uncalibrated_p).powf(0.190294957));

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
        state_
            .current_altitude_baro_calibrated_mm
            .store((altitude_m * 1000.0) as i32, Ordering::Relaxed);
        state_
            .current_altitude_baro_uncalibrated_mm
            .store((altitude_uncalibrated_m * 1000.0) as i32, Ordering::Relaxed);
        state_
            .current_altitude_change_mms
            .store((altitude_change_ms * 1000.0) as i32, Ordering::Release);
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

async fn sdcard_loop<'a>(
    card: embedded_sdmmc::asynchronous::SdCard<
        SpiDeviceDriver<'a, SpiDriver<'a>>,
        embassy_time::Delay,
    >,
    state: Arc<State>,
) {
    while state.current_satellite_count.load(Ordering::Relaxed) == 0 {
        log::info!("waiting for GPS to get a fix before opening sdcard");
        Timer::after(Duration::from_secs(1)).await;
    }

    let volume_manager = VolumeManager::new(card, &*state);
    let volume0 = volume_manager
        .open_volume(VolumeIdx(0))
        .await
        .expect("could not open sdcard volume 0, is the sdcard properly formatted?");
    let root_dir = volume0.open_root_dir().unwrap();

    // embedded-sdmmc only supports 8.3 filenames, and date is not reliable when gps is starting
    // so filenames are just XCM00001.igc, XCM00002.igc, etc. Let's find the first one that does
    // not exist

    let mut file_id = 1;

    fn filename(id: i32) -> String {
        format!("XCM{id:05}.igc")
    }

    while root_dir
        .find_directory_entry(filename(file_id).as_str())
        .await
        .is_ok()
    {
        file_id += 1;
    }

    let filename = filename(file_id);
    log::info!("saving data to file {filename}");
    let file = root_dir
        .open_file_in_dir(filename.as_str(), Mode::ReadWriteCreateOrAppend)
        .await
        .expect("could not open file");

    // see https://www.fai.org/sites/default/files/igc_specification_dec_2024_with_al9.pdf for
    // the igc file format specification
    let mut mac: [u8; 6] = [0; 6];
    if unsafe {
        esp_idf_svc::sys::esp_read_mac(
            mac.as_mut_ptr(),
            esp_idf_svc::sys::esp_mac_type_t_ESP_MAC_WIFI_STA,
        )
    } == esp_idf_svc::sys::ESP_OK
    {
        log::info!("mac address: {mac:02X?}");
    } else {
        log::error!("could not read mac address");
    }

    let serial = format!("{:02X}{:02X}{:02X}", mac[3], mac[4], mac[5]);
    log::info!("serial: {serial}");

    write(
        &file,
        Record::A(FlightRecorderID {
            manufacturer: rcstr("XCM"),
            id: rcstr(&serial),
            extension: rcstr(""),
        }),
    )
    .await;

    write(
        &file,
        Record::H(FileHeader::PilotInCharge(rcstr(config::PILOT_IN_CHARGE))),
    )
    .await;

    write(&file, Record::H(FileHeader::SecondPilot(rcstr("Nil")))).await;

    write(
        &file,
        Record::H(FileHeader::GliderType(rcstr(config::GLIDER_TYPE))),
    )
    .await;

    write(
        &file,
        Record::H(FileHeader::GliderID(rcstr(config::GLIDER_ID))),
    )
    .await;

    write(&file, Record::H(FileHeader::GPSDatum(rcstr("WGS84")))).await;

    write(
        &file,
        Record::H(FileHeader::Firmware(rcstr(env!("CARGO_PKG_VERSION")))),
    )
    .await;

    write(&file, Record::H(FileHeader::Hardware(rcstr("0.1")))).await;

    write(
        &file,
        Record::H(FileHeader::LoggerType(rcstr("Crossometer Mini"))),
    )
    .await;

    write(
        &file,
        Record::H(FileHeader::GPSManufacturer(rcstr("UBLOX,UBX-M8030-KT"))),
    )
    .await;
    write(
        &file,
        Record::H(FileHeader::PressureSensor(rcstr("BOSCH,BMP280"))),
    )
    .await;

    loop {
        let altitude_gps_m = state.current_altitude_gps_m.load(Ordering::Acquire);
        let altitude_baro_uncalibrated_mm = state
            .current_altitude_baro_uncalibrated_mm
            .load(Ordering::Relaxed);

        let latitude = state.current_lat_x10_000_000.load(Ordering::Relaxed);
        let longitude = state.current_lon_x10_000_000.load(Ordering::Relaxed);

        fn to_dms<T: Into<f64>>(value_x10_000_000: T) -> (u8, f32, bool) {
            let value = value_x10_000_000.into() / 10_000_000.0;
            let is_positive = value >= 0.0;
            let abs_value = value.abs();
            let degrees = abs_value.trunc() as u8;
            let minutes = ((abs_value - degrees as f64) * 60.0) as f32;
            (degrees, minutes, is_positive)
        }

        let (latitude_degrees, latitude_minutes, is_north) = to_dms(latitude);
        let (longitude_degrees, longitude_minutes, is_east) = to_dms(longitude);

        write(
            &file,
            Record::B(Fix {
                timestamp: igc_parser::records::util::Time {
                    h: state.current_hours.load(Ordering::Relaxed),
                    m: state.current_minutes.load(Ordering::Relaxed),
                    s: state.current_seconds.load(Ordering::Relaxed),
                },
                coordinates: igc_parser::records::util::Coordinate {
                    latitude: igc_parser::records::util::Latitude {
                        degrees: latitude_degrees,
                        minutes: latitude_minutes,
                        is_north,
                    },
                    longitude: igc_parser::records::util::Longitude {
                        degrees: longitude_degrees,
                        minutes: longitude_minutes,
                        is_east,
                    },
                },
                pressure_alt: (altitude_baro_uncalibrated_mm / 1000) as i16,
                gps_alt: Some(altitude_gps_m as i16),
                extension: Rc::from(String::new().into_boxed_str()),
            }),
        )
        .await;

        file.flush().await.unwrap();
        Timer::after(Duration::from_secs(1)).await;
    }
}

fn rcstr(s: &str) -> Rc<str> {
    Rc::from(s.to_string().into_boxed_str())
}

async fn write<
    'a,
    D: BlockDevice,
    T: TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    file: &File<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    record: Record,
) {
    struct RecordWrapper(Record);

    impl Display for RecordWrapper {
        fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
            self.0.print(f)
        }
    }

    let to_write = format!("{}", RecordWrapper(record));

    print!("{to_write}"); //output igc to console for debugging

    file.write(to_write.as_bytes())
        .await
        .expect("cannot write to file");
}

trait IgcPrint {
    fn print(&self, formatter: &mut Formatter) -> std::fmt::Result;
}

impl IgcPrint for Record {
    fn print(&self, formatter: &mut Formatter) -> std::fmt::Result {
        match self {
            Record::A(frid) => {
                write!(formatter, "A")?;
                frid.print(formatter)?
            }
            Record::B(fix) => {
                write!(formatter, "B")?;
                fix.print(formatter)?
            }
            Record::H(header) => {
                write!(formatter, "H")?;
                header.print(formatter)?
            }
            _ => {
                todo!()
            }
        }
        write!(formatter, "\r\n")
    }
}

impl IgcPrint for FlightRecorderID {
    fn print(&self, formatter: &mut Formatter) -> std::fmt::Result {
        write!(
            formatter,
            "{}{}{}",
            self.manufacturer, self.id, self.extension
        )
    }
}

impl IgcPrint for Fix {
    fn print(&self, formatter: &mut Formatter) -> std::fmt::Result {
        self.timestamp.print(formatter)?;
        self.coordinates.print(formatter)?;
        write!(formatter, "A")?; // hardcode the fact that we have a gps altitude fix for now
        write!(formatter, "{:05}", self.pressure_alt)?;
        write!(formatter, "{:05}", self.gps_alt.unwrap_or(0))
    }
}

impl IgcPrint for igc_parser::records::util::Time {
    fn print(&self, formatter: &mut Formatter) -> std::fmt::Result {
        write!(
            formatter,
            "{:02}{:02}{:02}",
            self.h % 100,
            self.m % 100,
            self.s % 100
        )
    }
}

impl IgcPrint for igc_parser::records::util::Latitude {
    fn print(&self, formatter: &mut Formatter) -> std::fmt::Result {
        write!(
            formatter,
            "{:02}{:02}{:03}{}",
            self.degrees % 100,
            self.minutes.trunc() as u8 % 100,
            ((self.minutes.fract() * 1000.0).round() as u16) % 1000,
            if self.is_north { 'N' } else { 'S' }
        )
    }
}

impl IgcPrint for igc_parser::records::util::Longitude {
    fn print(&self, formatter: &mut Formatter) -> std::fmt::Result {
        write!(
            formatter,
            "{:03}{:02}{:03}{}",
            self.degrees,
            self.minutes.trunc() as u8 % 100,
            ((self.minutes.fract() * 1000.0).round() as u16) % 1000,
            if self.is_east { 'E' } else { 'W' }
        )
    }
}

impl IgcPrint for igc_parser::records::util::Coordinate {
    fn print(&self, formatter: &mut Formatter) -> std::fmt::Result {
        self.latitude.print(formatter)?;
        self.longitude.print(formatter)
    }
}

impl IgcPrint for FileHeader {
    fn print(&self, f: &mut Formatter) -> std::fmt::Result {
        match self {
            FileHeader::Date(date) => {
                write!(
                    f,
                    "FDTEDATE:{:02}{:02}{:02}",
                    date.d % 100,
                    date.m % 100,
                    date.y % 100
                )
            }
            FileHeader::FixAccuracy(acc) => {
                write!(f, "FFXA:{acc:03}")
            }
            FileHeader::PilotInCharge(str) => {
                write!(f, "FPLTPILOTINCHARGE:{str}")
            }
            FileHeader::SecondPilot(str) => {
                write!(f, "FCM2CREW2:{str}")
            }
            FileHeader::GliderType(str) => {
                write!(f, "FGTYGLIDERTYPE:{str}")
            }
            FileHeader::GliderID(str) => {
                write!(f, "FGIDGLIDERID:{str}")
            }
            FileHeader::GPSDatum(str) => {
                write!(f, "FDTMNNNGPSDATUM:{str}")
            }
            FileHeader::Firmware(str) => {
                write!(f, "FRFWFIRMWAREVERSION:{str}")
            }
            FileHeader::Hardware(str) => {
                write!(f, "FRFWFIRMWAREVERSION:{str}")
            }
            FileHeader::LoggerType(str) => {
                write!(f, "FFTYFRTYPE:{str}")
            }
            FileHeader::GPSManufacturer(str) => {
                write!(f, "FGPS:{str}")
            }
            FileHeader::PressureSensor(str) => {
                write!(f, "FPRSPRESSALTSENSOR:{str}")
            }
            FileHeader::CompetitionID(str) => {
                write!(f, "FCIDCOMPETITIONID:{str}")
            }
            FileHeader::CompetitionClass(str) => {
                write!(f, "FCCLCOMPETITIONCLASS:{str}")
            }
        }
    }
}
