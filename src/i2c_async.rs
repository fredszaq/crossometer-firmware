//! Async I2C driver for the display, built on ESP-IDF's *new* `i2c_master` driver
//! (`driver/i2c_master.h`), called directly through the esp-idf-sys bindings.
//!
//! esp-idf-hal only wraps the legacy `driver/i2c.h` driver, whose ISR can enter an
//! unclearable interrupt storm: when a stray status bit (bus glitch during wifi TX)
//! latches while the driver state is neither READ nor WRITE, the ISR returns without
//! clearing it and re-fires forever until the interrupt watchdog reboots the board
//! (i2c.c:543, observed in flight). The new driver doesn't have that hole, and its
//! completion callback lets us await transactions instead of blocking the thread.
//! To be replaced by esp-idf-hal's own driver once esp-rs/esp-idf-hal#388 lands.

use std::marker::PhantomData;
use std::num::NonZeroU32;

use embassy_time::{with_timeout, Duration};
use esp_idf_hal::gpio::{InputPin, OutputPin};
use esp_idf_hal::i2c::I2c;
use esp_idf_hal::interrupt::asynch::HalIsrNotification;
use esp_idf_hal::units::Hertz;
use esp_idf_svc::sys::*;

// a non-zero depth is what switches the driver into asynchronous mode; we only ever
// have one transaction in flight (each write awaits completion before returning)
const TRANS_QUEUE_DEPTH: usize = 2;
// backstop for a wedged bus where the completion callback never fires
const TRANSACTION_TIMEOUT: Duration = Duration::from_millis(1000);

pub struct AsyncI2cDriver<'d> {
    bus: i2c_master_bus_handle_t,
    dev: i2c_master_dev_handle_t,
    address: u8,
    // boxed so the completion ISR sees a stable address for the driver's lifetime
    notification: Box<HalIsrNotification>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> AsyncI2cDriver<'d> {
    pub fn new<I2C: I2c + 'd>(
        _i2c: I2C,
        sda: impl InputPin + OutputPin + 'd,
        scl: impl InputPin + OutputPin + 'd,
        address: u8,
        baudrate: Hertz,
    ) -> Result<Self, EspError> {
        let mut flags = i2c_master_bus_config_t__bindgen_ty_1::default();
        flags.set_enable_internal_pullup(1);

        let bus_config = i2c_master_bus_config_t {
            i2c_port: I2C::port() as i2c_port_num_t,
            sda_io_num: sda.pin() as gpio_num_t,
            scl_io_num: scl.pin() as gpio_num_t,
            clk_source: soc_periph_i2c_clk_src_t_I2C_CLK_SRC_DEFAULT,
            glitch_ignore_cnt: 7,
            intr_priority: 0,
            trans_queue_depth: TRANS_QUEUE_DEPTH,
            flags,
        };

        let mut bus: i2c_master_bus_handle_t = std::ptr::null_mut();
        esp!(unsafe { i2c_new_master_bus(&bus_config, &mut bus) })?;

        let dev_config = i2c_device_config_t {
            dev_addr_length: i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7,
            device_address: address as u16,
            scl_speed_hz: baudrate.into(),
            scl_wait_us: 0,
            flags: Default::default(),
        };

        let mut dev: i2c_master_dev_handle_t = std::ptr::null_mut();
        if let Err(e) = esp!(unsafe { i2c_master_bus_add_device(bus, &dev_config, &mut dev) }) {
            unsafe { i2c_del_master_bus(bus) };
            return Err(e);
        }

        let notification = Box::new(HalIsrNotification::new());
        let callbacks = i2c_master_event_callbacks_t {
            on_trans_done: Some(on_trans_done),
        };
        if let Err(e) = esp!(unsafe {
            i2c_master_register_event_callbacks(
                dev,
                &callbacks,
                &*notification as *const HalIsrNotification as *mut core::ffi::c_void,
            )
        }) {
            unsafe {
                i2c_master_bus_rm_device(dev);
                i2c_del_master_bus(bus);
            }
            return Err(e);
        }

        Ok(Self {
            bus,
            dev,
            address,
            notification,
            _p: PhantomData,
        })
    }

    async fn write_async(&mut self, bytes: &[u8]) -> Result<(), EspError> {
        self.notification.reset();

        // callbacks are registered, so this queues the transaction and returns
        // immediately; the ISR reads from `bytes` until the transaction completes
        esp!(unsafe { i2c_master_transmit(self.dev, bytes.as_ptr(), bytes.len(), -1) })?;

        // if we are cancelled (or time out) below, the hardware may still be reading
        // `bytes`: block until the queue is drained before letting the borrow end
        let mut in_flight = InFlightGuard {
            bus: self.bus,
            armed: true,
        };

        match with_timeout(TRANSACTION_TIMEOUT, self.notification.wait()).await {
            Ok(events) => {
                in_flight.armed = false;
                let events = events.get();
                if events & (1 << i2c_master_event_t_I2C_EVENT_DONE) != 0 {
                    Ok(())
                } else if events & (1 << i2c_master_event_t_I2C_EVENT_NACK) != 0 {
                    Err(EspError::from_infallible::<ESP_ERR_INVALID_RESPONSE>())
                } else {
                    Err(EspError::from_infallible::<ESP_ERR_TIMEOUT>())
                }
            }
            Err(_) => {
                drop(in_flight);
                // the completion may have fired between the timeout and the drain
                // just above, don't let it leak into the next transaction
                self.notification.reset();
                Err(EspError::from_infallible::<ESP_ERR_TIMEOUT>())
            }
        }
    }
}

struct InFlightGuard {
    bus: i2c_master_bus_handle_t,
    armed: bool,
}

impl Drop for InFlightGuard {
    fn drop(&mut self) {
        if self.armed {
            unsafe { i2c_master_bus_wait_all_done(self.bus, -1) };
        }
    }
}

impl Drop for AsyncI2cDriver<'_> {
    fn drop(&mut self) {
        unsafe {
            i2c_master_bus_wait_all_done(self.bus, -1);
            i2c_master_bus_rm_device(self.dev);
            i2c_del_master_bus(self.bus);
        }
    }
}

unsafe impl Send for AsyncI2cDriver<'_> {}

unsafe extern "C" fn on_trans_done(
    _dev: i2c_master_dev_handle_t,
    evt_data: *const i2c_master_event_data_t,
    user_ctx: *mut core::ffi::c_void,
) -> bool {
    let notification = &*(user_ctx as *const HalIsrNotification);
    let event = if evt_data.is_null() {
        i2c_master_event_t_I2C_EVENT_DONE
    } else {
        (*evt_data).event
    };
    match NonZeroU32::new(1 << event) {
        Some(bits) => notification.notify(bits),
        None => false,
    }
}

#[derive(Debug)]
pub struct I2cError(pub EspError);

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        if self.0.code() == ESP_ERR_INVALID_RESPONSE {
            embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Unknown,
            )
        } else {
            embedded_hal::i2c::ErrorKind::Other
        }
    }
}

impl embedded_hal::i2c::ErrorType for AsyncI2cDriver<'_> {
    type Error = I2cError;
}

impl embedded_hal_async::i2c::I2c for AsyncI2cDriver<'_> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // contrary to the trait contract each operation gets its own start/stop, which
        // is fine for the write-only ssd1306 that only ever sends single operations
        for operation in operations {
            match operation {
                embedded_hal::i2c::Operation::Write(bytes) => {
                    embedded_hal_async::i2c::I2c::write(self, address, bytes).await?
                }
                embedded_hal::i2c::Operation::Read(_) => {
                    return Err(I2cError(EspError::from_infallible::<ESP_ERR_NOT_SUPPORTED>()))
                }
            }
        }
        Ok(())
    }

    async fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        // the device address is bound at bus setup, this driver can't reach others
        if address != self.address {
            return Err(I2cError(EspError::from_infallible::<ESP_ERR_INVALID_ARG>()));
        }
        self.write_async(bytes).await.map_err(I2cError)
    }
}
