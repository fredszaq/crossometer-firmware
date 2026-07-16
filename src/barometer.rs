use crate::state::State;
use bme280::spi::AsyncBME280;
use esp_idf_hal::spi::{SpiDeviceDriver, SpiDriver};
use std::sync::atomic::Ordering;
use std::sync::Arc;

#[derive(Copy, Clone, Debug)]
pub struct BarometerMeasurement {
    pub temperature_c: f32,
    pub pressure_pa: f32,
    pub altitude_uncalibrated_m: f32,
}

pub async fn baro_loop<'a>(
    mut bmp280: AsyncBME280<SpiDeviceDriver<'a, SpiDriver<'a>>>,
    state: Arc<State>,
) {
    // init altitude to a fairly negative value so that we get a nice welcome sound as the
    // current altitude moves to the actual measured value
    let mut old_altitude_m = -100.0;
    let mut old_altitude_change_ms = 0.0;
    let mut last_measure_time = std::time::Instant::now();
    let mut calibrated = false;
    let sea_level_uncalibrated_p = 101325.0;
    let mut sea_level_p = sea_level_uncalibrated_p;
    // a dead or miswired sensor (or a lost SPI completion) should not silently freeze the
    // vario: put timeouts around everything, complain loudly and keep retrying
    loop {
        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(2),
            bmp280.init(&mut embassy_time::Delay),
        )
        .await
        {
            Ok(Ok(())) => {
                log::info!("bme280 init ok");
                break;
            }
            Ok(Err(e)) => log::error!("bme280 init failed: {e:?}"),
            Err(_) => log::error!("bme280 init timed out, is the sensor properly connected?"),
        }
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
    let publisher = state.barometer_measurements.publisher().unwrap();

    #[cfg(feature = "print-free-stack")]
    let mut iteration: u32 = 0;
    #[cfg(feature = "print-free-stack")]
    let mut last_stack_report = embassy_time::Instant::now();
    let mut consecutive_failures: u32 = 0;

    loop {
        // every 5s: heartbeat + how close we ever got to overflowing this thread's stack
        #[cfg(feature = "print-free-stack")]
        {
            iteration += 1;
            if last_stack_report.elapsed() >= embassy_time::Duration::from_secs(5) {
                last_stack_report = embassy_time::Instant::now();
                let min_free_stack_bytes =
                    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(std::ptr::null_mut()) };
                log::info!(
                    "baro thread: iteration {iteration}, min free stack ever: {min_free_stack_bytes} bytes"
                );
            }
        }

        let measurements = match embassy_time::with_timeout(
            embassy_time::Duration::from_millis(500),
            bmp280.measure(&mut embassy_time::Delay),
        )
        .await
        {
            Ok(Ok(measurements)) => {
                consecutive_failures = 0;
                measurements
            }
            result => {
                match result {
                    Ok(Err(e)) => log::error!("bme280 measure failed: {e:?}"),
                    _ => log::error!(
                        "bme280 measure timed out, is the sensor properly connected?"
                    ),
                }
                // isolated glitches recover on the next try, only back off when the sensor
                // looks properly dead
                consecutive_failures += 1;
                if consecutive_failures >= 3 {
                    embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
                }
                continue;
            }
        };
        let measure_time = std::time::Instant::now();

        // https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf page 16
        let altitude_m =
            44330.0 * (1.0 - (measurements.pressure as f64 / sea_level_p).powf(0.190294957));

        let altitude_uncalibrated_m = 44330.0
            * (1.0 - (measurements.pressure as f64 / sea_level_uncalibrated_p).powf(0.190294957));

        publisher.publish_immediate(BarometerMeasurement {
            temperature_c: measurements.temperature,
            pressure_pa: measurements.pressure,
            altitude_uncalibrated_m: altitude_uncalibrated_m as f32,
        });

        if !calibrated {
            let current_altitude_gps_m = state.current_altitude_gps_m.load(Ordering::Acquire);
            if current_altitude_gps_m > 0 {
                // https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf page 17
                sea_level_p = measurements.pressure as f64
                    / (1.0 - current_altitude_gps_m as f64 / 44330.0).powf(5.255);
                calibrated = true;
            }
        }

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
        state
            .current_altitude_baro_calibrated_mm
            .store((altitude_m * 1000.0) as i32, Ordering::Relaxed);
        state
            .current_altitude_change_mms
            .store((altitude_change_ms * 1000.0) as i32, Ordering::Release);

        embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
    }
}
