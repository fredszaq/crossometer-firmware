use crate::state::State;
#[cfg(not(feature = "silent"))]
use embassy_time::Duration;
#[cfg(not(feature = "silent"))]
use embassy_time::Timer;
use esp_idf_hal::gpio::OutputPin;
#[cfg(not(feature = "silent"))]
use esp_idf_hal::ledc::config::TimerConfig;
#[cfg(not(feature = "silent"))]
use esp_idf_hal::ledc::Resolution;
use esp_idf_hal::ledc::{LedcChannel, LedcTimer, SpeedMode};
#[cfg(not(feature = "silent"))]
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver};
#[cfg(not(feature = "silent"))]
use esp_idf_hal::units::Hertz;
#[cfg(feature = "silent")]
use std::marker::PhantomData;
#[cfg(not(feature = "silent"))]
use std::sync::atomic::Ordering;
use std::sync::Arc;

pub struct BuzzerDriver<'d, S: SpeedMode> {
    #[cfg(not(feature = "silent"))]
    timer_driver: LedcTimerDriver<'d, S>,
    #[cfg(not(feature = "silent"))]
    ledc_driver: LedcDriver<'d>,
    #[cfg(feature = "silent")]
    _phantom: PhantomData<&'d S>,
}

impl<'d, S: SpeedMode> BuzzerDriver<'d, S> {
    #[cfg(not(feature = "silent"))]
    pub fn new<T, C>(timer: T, channel: C, pin: impl OutputPin + 'd) -> Self
    where
        T: LedcTimer<SpeedMode = S> + 'd,
        C: LedcChannel<SpeedMode = S> + 'd,
    {
        // 13 bits of resolution so that a single clock choice covers the whole frequency
        // range we later sweep with set_frequency (with the 80MHz APB clock and 13 bits the
        // ledc divider stays valid from ~10Hz to ~9.7kHz, we use 90-5000Hz)
        let timer_driver = LedcTimerDriver::new(
            timer,
            &TimerConfig::default()
                .frequency(Hertz(1000))
                .resolution(Resolution::Bits13),
        )
        .unwrap();
        // the ledc driver only copies the timer info, it does not hold the borrow: we can
        // keep the timer driver next to it and change the frequency for each beep
        let mut ledc_driver = LedcDriver::new(channel, &timer_driver, pin).unwrap();
        ledc_driver.set_duty(0).unwrap();

        BuzzerDriver {
            timer_driver,
            ledc_driver,
        }
    }

    #[cfg(feature = "silent")]
    pub fn new<T, C>(_timer: T, _channel: C, _pin: impl OutputPin + 'd) -> Self
    where
        T: LedcTimer<SpeedMode = S> + 'd,
        C: LedcChannel<SpeedMode = S> + 'd,
    {
        BuzzerDriver {
            _phantom: PhantomData,
        }
    }
}

#[cfg(feature = "silent")]
pub async fn beep_loop<S: SpeedMode>(_state: Arc<State>, mut _buzzer: BuzzerDriver<'_, S>) {}
#[cfg(not(feature = "silent"))]
pub async fn beep_loop<S: SpeedMode>(state: Arc<State>, mut buzzer: BuzzerDriver<'_, S>) {
    let max_duty = buzzer.ledc_driver.get_max_duty();

    loop {
        let altitude_change_mms = state.current_altitude_change_mms.load(Ordering::Acquire);
        let freq = altitude_change_to_freq(altitude_change_mms);
        println!(
            "buzzer_loop: change={} freq={:?}",
            altitude_change_mms,
            freq.map(|f| f.0)
        );
        let duration = altitude_change_to_beep_duration(altitude_change_mms);

        if let Some(freq) = freq {
            // a failed beep should not take down the whole firmware
            match buzzer.timer_driver.set_frequency(freq) {
                Ok(()) => {
                    if let Err(e) = buzzer.ledc_driver.set_duty(max_duty / 2) {
                        log::error!("buzzer set_duty({}) failed: {e}", max_duty / 2);
                    }
                }
                Err(e) => log::error!("buzzer set_frequency({freq}) failed: {e}"),
            }
            Timer::after(duration).await;
            if let Err(e) = buzzer.ledc_driver.set_duty(0) {
                log::error!("buzzer set_duty(0) failed: {e}");
            }
        } else {
            Timer::after(duration).await
        }
        let altitude_change_mms = state.current_altitude_change_mms.load(Ordering::Acquire);
        let duration = altitude_change_to_no_beep_duration(altitude_change_mms);
        Timer::after(duration).await
    }
}

#[cfg(not(feature = "silent"))]
fn altitude_change_to_freq(altitude_change_mms: i32) -> Option<Hertz> {
    if altitude_change_mms > -3000 && altitude_change_mms < 555 {
        None
    } else {
        Some(Hertz(
            (2000 + altitude_change_mms / 3).clamp(90 /* -5.7 m/s */, 5000 /* ~ +9m/s */) as u32,
        ))
    }
}

#[cfg(not(feature = "silent"))]
fn altitude_change_to_beep_duration(altitude_change_mms: i32) -> Duration {
    Duration::from_millis((150 - altitude_change_mms / 33).clamp(60, 400) as u64)
}

#[cfg(not(feature = "silent"))]
fn altitude_change_to_no_beep_duration(altitude_change_mms: i32) -> Duration {
    Duration::from_millis((150 - altitude_change_mms / 33).clamp(60, 400) as u64)
}
