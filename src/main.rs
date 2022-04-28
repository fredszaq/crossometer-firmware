use bme280::spi::BME280;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::{Channel, Timer};
use esp_idf_hal::prelude::*;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::Arc;
use std::time::Duration;

fn main() -> ! {
    esp_idf_sys::link_patches();
    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;
    use esp_idf_hal::spi;

    let config = <spi::config::Config as Default>::default().baudrate(9600.Hz().into());

    let spi3 = spi::Master::<spi::SPI3, _, _, _, _>::new(
        peripherals.spi3,
        spi::Pins {
            sclk: pins.gpio18,
            sdo: pins.gpio23,
            sdi: Some(pins.gpio19),
            cs: Option::<esp_idf_hal::gpio::Gpio21<esp_idf_hal::gpio::Unknown>>::None,
        },
        config,
    )
    .unwrap();

    let cs = pins.gpio5.into_output().unwrap();

    let mut bmp280 = BME280::new(spi3, cs, esp_idf_hal::delay::FreeRtos).unwrap();
    bmp280.init().unwrap();

    let current_altitude_change_mms = Arc::new(AtomicI32::new(0));
    let current_altitude_change_mms_ = Arc::clone(&current_altitude_change_mms);

    std::thread::spawn(move || {
        // init altitude to a fairly negative value so that we get a nice welcome sound as the
        // current altitude moves to the actual measured value
        let mut old_altitude_m = -100.0;
        let mut old_altitude_change_ms = 0.0;
        let mut last_measure_time = std::time::Instant::now();

        loop {
            let measurements = bmp280.measure().unwrap();
            let measure_time = std::time::Instant::now();

            let sea_level_p = 101325.0;
            // https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf page 16
            let altitude_m =
                44330.0 * (1.0 - (measurements.pressure as f64 / sea_level_p).powf(0.190294957));

            let elapsed = (measure_time - last_measure_time).as_secs_f64();
            // apply a bit of smoothing on the data
            let altitude_change_ms =
                old_altitude_change_ms * 0.7 + 0.3 * ((altitude_m - old_altitude_m) / elapsed);

            println!(
                "measure_loop:{},{},{},{}",
                altitude_change_ms as f32,
                altitude_m as f32,
                measurements.pressure as f32,
                elapsed as f32
            );

            old_altitude_m = altitude_m;
            old_altitude_change_ms = altitude_change_ms;
            last_measure_time = measure_time;
            current_altitude_change_mms_
                .store((altitude_change_ms * 1000.0) as i32, Ordering::SeqCst);
        }
    });

    let mut timer_hw = peripherals.ledc.timer0;
    let mut channel_hw = peripherals.ledc.channel0;
    let mut pin_hw = pins.gpio25;

    loop {
        let altitude_change_mms = current_altitude_change_mms.load(Ordering::SeqCst);
        let freq = altitude_change_to_freq(altitude_change_mms);
        let duration = altitude_change_to_beep_duration(altitude_change_mms);
        println!(
            "buzzer_loop:{},{}",
            freq.unwrap_or(0.Hz()).0,
            duration.as_millis()
        );

        if let Some(freq) = freq {
            let config = TimerConfig::default().frequency(freq.into());
            let timer = Timer::new(timer_hw, &config).unwrap();
            let mut channel = Channel::new(channel_hw, &timer, pin_hw).unwrap();
            let max_duty = channel.get_max_duty();
            channel.set_duty(max_duty / 2).unwrap();
            std::thread::sleep(duration);
            (channel_hw, pin_hw) = channel.release().unwrap();
            timer_hw = timer.release().unwrap();
        } else {
            std::thread::sleep(duration);
        }

        std::thread::sleep(std::time::Duration::from_millis(50));
    }
}

fn altitude_change_to_freq(altitude_change_mms: i32) -> Option<Hertz> {
    if altitude_change_mms > -1800 && altitude_change_mms < 500 {
        None
    } else {
        Some(
            ((1200 + altitude_change_mms / 3)
                .max(100) // ~ -3.3 m/s
                .min(4000)// ~ +8.4m/s
                as u32)
                .Hz(),
        )
    }
}

fn altitude_change_to_beep_duration(altitude_change_mms: i32) -> Duration {
    Duration::from_millis((150 - altitude_change_mms / 33).max(60).min(250) as u64)
}
