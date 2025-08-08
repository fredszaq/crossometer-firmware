use crate::state::State;
#[cfg(not(feature = "silent"))]
use embassy_time::Duration;
#[cfg(not(feature = "silent"))]
use embassy_time::Timer;
use esp_idf_hal::gpio::AnyOutputPin;
#[cfg(not(feature = "silent"))]
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::{LedcChannel, LedcTimer};
#[cfg(not(feature = "silent"))]
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver};
#[cfg(not(feature = "silent"))]
use esp_idf_hal::peripheral::Peripheral;
#[cfg(not(feature = "silent"))]
use esp_idf_hal::prelude::{FromValueType, Hertz};
#[cfg(feature = "silent")]
use std::marker::PhantomData;
#[cfg(not(feature = "silent"))]
use std::sync::atomic::Ordering;
use std::sync::Arc;

pub struct BuzzerDriver<TIMER, CHANNEL> {
    #[cfg(not(feature = "silent"))]
    timer: TIMER,
    #[cfg(not(feature = "silent"))]
    channel: CHANNEL,
    #[cfg(not(feature = "silent"))]
    pin: AnyOutputPin,
    #[cfg(feature = "silent")]
    _phantom_timer: PhantomData<TIMER>,
    #[cfg(feature = "silent")]
    _phantom_channel: PhantomData<CHANNEL>,
}

impl<TIMER, CHANNEL> BuzzerDriver<TIMER, CHANNEL>
where
    CHANNEL: LedcChannel<SpeedMode = <TIMER as LedcTimer>::SpeedMode>,
    TIMER: LedcTimer,
{
    #[cfg(not(feature = "silent"))]
    pub fn new(timer: TIMER, channel: CHANNEL, pin: AnyOutputPin) -> Self {
        BuzzerDriver {
            timer,
            channel,
            pin,
        }
    }

    #[cfg(feature = "silent")]
    pub fn new(_timer: TIMER, _channel: CHANNEL, _pin: AnyOutputPin) -> Self {
        BuzzerDriver {
            _phantom_timer: PhantomData,
            _phantom_channel: PhantomData,
        }
    }
}

#[cfg(feature = "silent")]
pub async fn beep_loop<TH, CH>(_state: Arc<State>, mut _buzzer: BuzzerDriver<TH, CH>) {}
#[cfg(not(feature = "silent"))]
pub async fn beep_loop<C, T, TH, CH>(state: Arc<State>, mut buzzer: BuzzerDriver<TH, CH>)
where
    C: LedcChannel<SpeedMode = <T as LedcTimer>::SpeedMode>,
    T: LedcTimer,
    TH: Peripheral<P = T>,
    CH: Peripheral<P = C>,
{
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
                &mut buzzer.channel,
                LedcTimerDriver::new(&mut buzzer.timer, &config).unwrap(),
                &mut buzzer.pin,
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

#[cfg(not(feature = "silent"))]
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

#[cfg(not(feature = "silent"))]
fn altitude_change_to_beep_duration(altitude_change_mms: i32) -> Duration {
    Duration::from_millis((150 - altitude_change_mms / 33).clamp(60, 400) as u64)
}

#[cfg(not(feature = "silent"))]
fn altitude_change_to_no_beep_duration(altitude_change_mms: i32) -> Duration {
    Duration::from_millis((150 - altitude_change_mms / 33).clamp(60, 400) as u64)
}
