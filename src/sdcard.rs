use crate::config;
use crate::state::State;
use embassy_time::{Duration, Timer};
use embedded_sdmmc::asynchronous::{
    BlockDevice, File, Mode, SdCard, TimeSource, VolumeIdx, VolumeManager,
};
use esp_idf_hal::spi::{SpiDeviceDriver, SpiDriver};
use igc_parser::records::file_header::FileHeader;
use igc_parser::records::fix::Fix;
use igc_parser::records::flight_recorder_id::FlightRecorderID;
use igc_parser::records::Record;
use std::fmt::{Display, Formatter};
use std::rc::Rc;
use std::sync::atomic::Ordering;
use std::sync::Arc;

pub async fn sdcard_loop<'a>(
    sdcard: SdCard<SpiDeviceDriver<'a, SpiDriver<'a>>, embassy_time::Delay>,
    state: Arc<State>,
) {
    while state.current_satellite_count.load(Ordering::Relaxed) == 0 {
        log::info!("waiting for GPS to get a fix before opening sdcard");
        Timer::after(Duration::from_secs(1)).await;
    }

    let volume_manager = VolumeManager::new(sdcard, &*state);
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

fn rcstr(s: &str) -> Rc<str> {
    Rc::from(s.to_string().into_boxed_str())
}
