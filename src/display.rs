use crate::state::State;
use embassy_time::{Duration, Timer};
use embedded_graphics::Drawable;
use esp_idf_hal::i2c::I2cDriver;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::{DisplaySize128x64, I2CInterface};
use ssd1306::Ssd1306;
use std::sync::atomic::Ordering;
use std::sync::Arc;

pub async fn display_loop<'a>(
    mut display: Ssd1306<
        I2CInterface<I2cDriver<'a>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
    state: Arc<State>,
) {
    display.init().unwrap();
    log::info!("display init ok");

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

        if let Ok(qual) = state.gps_quality_channel.try_receive() {
            gps_qual = Some(qual)
        }

        let altitude_gps_m = state.current_altitude_gps_m.load(Ordering::Acquire);
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

        let altitude_baro_m = state
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

        let speed_kmh = state.current_speed_kmh.load(Ordering::Relaxed);

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

        let altitude_change_mms = state.current_altitude_change_mms.load(Ordering::Relaxed);
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

        let current_glide_ratio_x10_ = state.current_glide_ratio_x10.load(Ordering::Relaxed);
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
                state.current_satellite_count.load(Ordering::Relaxed),
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
                state.current_hours.load(Ordering::Relaxed),
                state.current_minutes.load(Ordering::Relaxed)
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
