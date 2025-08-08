use crate::state::State;
use board::Board;
use esp_idf_hal::task::block_on;
use std::sync::Arc;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let board = Board::take().unwrap();

    let state = Arc::new(State::default());

    let state_ = Arc::clone(&state);

    std::thread::Builder::new()
        .stack_size(64635)
        .spawn(move || {
            block_on(barometer::baro_loop(board.barometer, state_));
        })
        .unwrap();

    let state_ = Arc::clone(&state);

    std::thread::Builder::new()
        .stack_size(64635)
        .name("display_thread".to_string())
        .spawn(move || {
            block_on(display::display_loop(board.display, state_));
        })
        .unwrap();

    let state_ = Arc::clone(&state);

    std::thread::Builder::new()
        .name("gps_thread".to_string())
        .stack_size(64635)
        .spawn(move || {
            block_on(gps::gps_loop(board.gps, state_));
        })
        .unwrap();

    block_on(embassy_futures::join::join(
        buzzer::beep_loop(Arc::clone(&state), board.buzzer),
        sdcard::sdcard_loop(board.sdcard, state),
    ));
}

mod barometer;

#[cfg(feature = "crossometer-mini-v0-1")]
#[path = "boards/crossometer-mini_v0.1.rs"]
mod board;
mod buzzer;
#[path = "../config.rs"]
mod config;
mod display;
mod gps;
mod sdcard;
mod state;
