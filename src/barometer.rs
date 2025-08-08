use crate::state::State;
use bme280::spi::AsyncBME280;
use esp_idf_hal::spi::{SpiDeviceDriver, SpiDriver};
use std::sync::atomic::Ordering;
use std::sync::Arc;

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
    bmp280.init(&mut embassy_time::Delay).await.unwrap();

    loop {
        let measurements = bmp280.measure(&mut embassy_time::Delay).await.unwrap();
        let measure_time = std::time::Instant::now();

        if !calibrated {
            let current_altitude_gps_m = state.current_altitude_gps_m.load(Ordering::Acquire);
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
        state
            .current_altitude_baro_calibrated_mm
            .store((altitude_m * 1000.0) as i32, Ordering::Relaxed);
        state
            .current_altitude_baro_uncalibrated_mm
            .store((altitude_uncalibrated_m * 1000.0) as i32, Ordering::Relaxed);
        state
            .current_altitude_change_mms
            .store((altitude_change_ms * 1000.0) as i32, Ordering::Release);
    }
}
