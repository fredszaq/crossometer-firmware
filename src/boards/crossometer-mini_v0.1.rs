use crate::buzzer::BuzzerDriver;
#[cfg(not(feature = "no-display"))]
use crate::i2c_async::AsyncI2cDriver;
use bme280::spi::AsyncBME280;
use embedded_sdmmc::asynchronous::SdCard;
use esp_idf_hal::ledc::LowSpeed;
use esp_idf_hal::spi::{SpiDeviceDriver, SpiDriver};
use esp_idf_hal::uart::{AsyncUartDriver, UartDriver};
use esp_idf_hal::units::Hertz;
#[cfg(not(feature = "no-display"))]
use ssd1306::mode::BufferedGraphicsModeAsync;
#[cfg(not(feature = "no-display"))]
use ssd1306::prelude::{DisplaySize128x64, I2CInterface};
#[cfg(not(feature = "no-display"))]
use ssd1306::Ssd1306Async;

pub struct Board<'a> {
    pub gps: AsyncUartDriver<'a, UartDriver<'a>>,
    #[cfg(feature = "wifi")]
    pub modem: esp_idf_hal::modem::Modem<'a>,
    pub barometer: AsyncBME280<SpiDeviceDriver<'a, SpiDriver<'a>>>,
    #[cfg(not(feature = "no-display"))]
    pub display: Ssd1306Async<
        I2CInterface<AsyncI2cDriver<'a>>,
        DisplaySize128x64,
        BufferedGraphicsModeAsync<DisplaySize128x64>,
    >,
    pub sdcard: SdCard<SpiDeviceDriver<'a, SpiDriver<'a>>, embassy_time::Delay>,
    pub buzzer: BuzzerDriver<'a, LowSpeed>,
}

impl Board<'_> {
    pub fn take() -> anyhow::Result<Self> {
        let peripherals = Peripherals::take().unwrap();
        let pins = peripherals.pins;

        let uart_gps = peripherals.uart2;
        let uart_gps_tx_pin = pins.gpio16;
        let uart_gps_rx_pin = pins.gpio17;

        let spi_bmp280 = peripherals.spi2;
        let spi_bmp280_sclk = pins.gpio14;
        let spi_bmp280_mosi = pins.gpio13;
        let spi_bmp280_miso = pins.gpio26;
        let spi_bmp280_cs = pins.gpio27;

        #[cfg(not(feature = "no-display"))]
        let i2c_ssd1306 = peripherals.i2c0;
        #[cfg(not(feature = "no-display"))]
        let i2c_ssd1306_sda = pins.gpio21;
        #[cfg(not(feature = "no-display"))]
        let i2c_ssd1306_scl = pins.gpio22;

        let spi_sdcard = peripherals.spi3;
        let spi_sdcard_sclk_pin = pins.gpio18;
        let spi_sdcard_mosi_pin = pins.gpio23;
        let spi_sdcard_miso_pin = pins.gpio19;
        let spi_sdcard_cs_pin = pins.gpio5;

        let buzzer_timer = peripherals.ledc.timer0;
        let buzzer_channel = peripherals.ledc.channel0;
        let buzzer_pin = pins.gpio25;

        let buzzer = BuzzerDriver::new(buzzer_timer, buzzer_channel, buzzer_pin);

        // 0x3C is the address ssd1306::I2CDisplayInterface::new below binds to
        #[cfg(not(feature = "no-display"))]
        let i2c_ssd1306 = AsyncI2cDriver::new(
            i2c_ssd1306,
            i2c_ssd1306_sda,
            i2c_ssd1306_scl,
            0x3C,
            Hertz(100_000),
        )
        .unwrap();

        use esp_idf_hal::peripherals::Peripherals;
        use esp_idf_hal::spi;
        use esp_idf_hal::uart::UartConfig;
        #[cfg(not(feature = "no-display"))]
        use ssd1306::prelude::DisplayRotation;
        // polling(false): blocking transfers wait on the transaction interrupt instead of
        // spinning in a critical section, the spinning once tripped the interrupt watchdog
        // when it collided with the display i2c ISR during the boot init pile-up
        let spi_bmp280_config = <spi::config::Config as Default>::default()
            .baudrate(Hertz(1_000_000))
            .polling(false);

        let spi_bmp280_driver = spi::SpiDriver::new(
            spi_bmp280,
            spi_bmp280_sclk,
            spi_bmp280_mosi,
            Some(spi_bmp280_miso),
            &spi::SpiDriverConfig::new(),
        )
        .unwrap();

        let spi_device_bmp280 =
            spi::SpiDeviceDriver::new(spi_bmp280_driver, Some(spi_bmp280_cs), &spi_bmp280_config)
                .unwrap();

        let bmp280 = AsyncBME280::new(spi_device_bmp280).unwrap();

        #[cfg(not(feature = "no-display"))]
        let ssd1306 = {
            let ssd1306_display_interface = ssd1306::I2CDisplayInterface::new(i2c_ssd1306);

            Ssd1306Async::new(
                ssd1306_display_interface,
                DisplaySize128x64,
                DisplayRotation::Rotate0,
            )
            .into_buffered_graphics_mode()
        };

        let uart_gps_config = UartConfig::new().baudrate(Hertz(9600));
        let uart_gps_driver = AsyncUartDriver::wrap(
            UartDriver::new(
                uart_gps,
                uart_gps_tx_pin,
                uart_gps_rx_pin,
                Option::<esp_idf_hal::gpio::Gpio0>::None,
                Option::<esp_idf_hal::gpio::Gpio0>::None,
                &uart_gps_config,
            )
            .unwrap(),
        )
        .unwrap();

        let spi_sdcard = spi::SpiDriver::new(
            spi_sdcard,
            spi_sdcard_sclk_pin,
            spi_sdcard_mosi_pin,
            Some(spi_sdcard_miso_pin),
            &spi::SpiDriverConfig::new(),
        )
        .unwrap();
        let spi_sdcard_config =
            <spi::config::Config as Default>::default().baudrate(Hertz(400_000));

        let spi_sdcard_device =
            spi::SpiDeviceDriver::new(spi_sdcard, Some(spi_sdcard_cs_pin), &spi_sdcard_config)
                .unwrap();

        let sdcard =
            embedded_sdmmc::asynchronous::SdCard::new(spi_sdcard_device, embassy_time::Delay);

        Ok(Board {
            gps: uart_gps_driver,
            #[cfg(feature = "wifi")]
            modem: peripherals.modem,
            barometer: bmp280,
            #[cfg(not(feature = "no-display"))]
            display: ssd1306,
            sdcard,
            buzzer,
        })
    }
}
