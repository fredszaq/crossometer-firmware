use crate::barometer::BarometerMeasurement;
use crate::gps::GpsMeasurement;
use crate::state::State;
use embassy_futures::select::{select, Either};
use embassy_sync::pubsub::WaitResult;
use esp_idf_hal::modem::Modem;
use esp_idf_hal::task::block_on;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{AccessPointConfiguration, AuthMethod, Configuration, EspWifi};
use log::{info, warn};
use nmea_parser::chrono::{Datelike, Timelike};
use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::Ordering;
use std::sync::Arc;

const NMEA_TCP_PORT: u16 = 10110;

const LK8EX1_PERIOD: embassy_time::Duration = embassy_time::Duration::from_millis(200);

pub fn wifi_nmea_server(modem: Modem<'static>, state: Arc<State>) {
    info!("wifi thread started");
    // a failing wifi should not take down the rest of the firmware, log and give up
    if let Err(e) = run_wifi_nmea_server(modem, state) {
        log::error!("wifi nmea server stopped: {e:#}");
    }
}

fn run_wifi_nmea_server(modem: Modem<'static>, state: Arc<State>) -> anyhow::Result<()> {
    use anyhow::{anyhow, Context};

    info!("taking the system event loop");
    let sys_loop = EspSystemEventLoop::take().context("could not take the system event loop")?;
    info!("initializing nvs");
    let nvs = EspDefaultNvsPartition::take().context("could not take the nvs partition")?;

    info!("creating the wifi driver");
    let mut wifi =
        EspWifi::new(modem, sys_loop, Some(nvs)).context("could not create the wifi driver")?;

    let auth_method = if crate::config::WIFI_PASSWORD.is_empty() {
        AuthMethod::None
    } else {
        AuthMethod::WPA2Personal
    };

    info!("configuring the access point");
    wifi.set_configuration(&Configuration::AccessPoint(AccessPointConfiguration {
        ssid: crate::config::WIFI_SSID
            .try_into()
            .map_err(|_| anyhow!("SSID too long: {}", crate::config::WIFI_SSID))?,
        password: crate::config::WIFI_PASSWORD
            .try_into()
            .map_err(|_| anyhow!("wifi password too long"))?,
        auth_method,
        max_connections: 2,
        ..Default::default()
    }))
    .context("could not configure the access point")?;

    info!("starting the wifi");
    wifi.start().context("could not start the wifi")?;

    // cap the TX power (units are 0.25dBm, 34 ≈ 8.5dBm): the client sits half a meter away
    // in the cockpit, and the full power TX current spikes are hard on the 5V rail
    if let Err(e) =
        esp_idf_svc::sys::esp!(unsafe { esp_idf_svc::sys::esp_wifi_set_max_tx_power(34) })
    {
        warn!("could not cap wifi TX power: {e}");
    }

    info!("wifi access point '{}' started", crate::config::WIFI_SSID);

    let listener = TcpListener::bind(("0.0.0.0", NMEA_TCP_PORT))
        .with_context(|| format!("could not bind the NMEA server on port {NMEA_TCP_PORT}"))?;
    info!("NMEA server listening on port {NMEA_TCP_PORT}");

    loop {
        match listener.accept() {
            Ok((mut stream, addr)) => {
                info!("NMEA client connected from {addr}");
                match block_on(stream_nmea(&mut stream, &state)) {
                    Ok(()) => info!("NMEA client {addr} disconnected"),
                    Err(e) => info!("NMEA client {addr} disconnected: {e}"),
                }
            }
            Err(e) => warn!("could not accept NMEA client: {e}"),
        }
    }
}

async fn stream_nmea(stream: &mut TcpStream, state: &State) -> std::io::Result<()> {
    // don't let a dead client (out of range kobo) block us forever on a full socket buffer
    stream.set_write_timeout(Some(std::time::Duration::from_secs(5)))?;
    stream.set_nodelay(true).ok();

    let mut gps_measurements = state.gps_measurements.subscriber().unwrap();
    let mut barometer_measurements = state.barometer_measurements.subscriber().unwrap();

    let mut last_lk8ex1 = embassy_time::Instant::MIN;

    loop {
        match select(
            gps_measurements.next_message(),
            barometer_measurements.next_message(),
        )
        .await
        {
            Either::First(WaitResult::Message(gps)) => {
                stream.write_all(gga_sentence(&gps).as_bytes())?;
                stream.write_all(rmc_sentence(&gps).as_bytes())?;
            }
            Either::Second(WaitResult::Message(baro)) => {
                if last_lk8ex1.elapsed() >= LK8EX1_PERIOD {
                    last_lk8ex1 = embassy_time::Instant::now();
                    stream.write_all(lk8ex1_sentence(&baro, state).as_bytes())?;
                }
            }
            Either::First(WaitResult::Lagged(_)) | Either::Second(WaitResult::Lagged(_)) => {}
        }
    }
}

fn gga_sentence(gps: &GpsMeasurement) -> String {
    let t = gps.timestamp;
    with_checksum(&format!(
        "GPGGA,{:02}{:02}{:02},{},{},1,{:02},1.0,{:.1},M,0.0,M,,",
        t.hour(),
        t.minute(),
        t.second(),
        format_latitude(gps.latitude),
        format_longitude(gps.longitude),
        gps.satellite_count,
        gps.altitude_m,
    ))
}

fn rmc_sentence(gps: &GpsMeasurement) -> String {
    let t = gps.timestamp;
    let speed_knots = gps
        .speed_kmh
        .map(|speed_kmh| format!("{:.1}", speed_kmh / 1.852))
        .unwrap_or_default();
    let course = gps
        .course_deg
        .map(|course_deg| format!("{course_deg:.1}"))
        .unwrap_or_default();
    with_checksum(&format!(
        "GPRMC,{:02}{:02}{:02},A,{},{},{},{},{:02}{:02}{:02},,,A",
        t.hour(),
        t.minute(),
        t.second(),
        format_latitude(gps.latitude),
        format_longitude(gps.longitude),
        speed_knots,
        course,
        t.day(),
        t.month(),
        t.year() % 100,
    ))
}

/// LK8000's own sentence carrying the raw pressure, this lets it compute a proper baro
/// altitude and vario instead of relying on the GPS altitude
/// format is $LK8EX1,pressure_pa,altitude_m,vario_cms,temperature_c,battery,*checksum
/// (99999 altitude means "use the pressure", 999 battery means "no battery data")
fn lk8ex1_sentence(baro: &BarometerMeasurement, state: &State) -> String {
    let vario_cms = state.current_altitude_change_mms.load(Ordering::Relaxed) / 10;
    with_checksum(&format!(
        "LK8EX1,{:.0},99999,{},{:.0},999,",
        baro.pressure_pa, vario_cms, baro.temperature_c,
    ))
}

fn format_latitude(latitude: f64) -> String {
    let hemisphere = if latitude >= 0.0 { 'N' } else { 'S' };
    let abs = latitude.abs();
    let degrees = abs as u32;
    let minutes = (abs - degrees as f64) * 60.0;
    format!("{degrees:02}{minutes:07.4},{hemisphere}")
}

fn format_longitude(longitude: f64) -> String {
    let hemisphere = if longitude >= 0.0 { 'E' } else { 'W' };
    let abs = longitude.abs();
    let degrees = abs as u32;
    let minutes = (abs - degrees as f64) * 60.0;
    format!("{degrees:03}{minutes:07.4},{hemisphere}")
}

fn with_checksum(body: &str) -> String {
    let checksum = body.bytes().fold(0u8, |acc, b| acc ^ b);
    format!("${body}*{checksum:02X}\r\n")
}
