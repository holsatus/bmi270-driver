#![no_std]

mod firmware;

pub mod compat_embedded_hal;

use device_driver::AsyncRegisterInterface;
use embedded_hal_async::delay::DelayNs;
use futures_util::TryFutureExt as _;

device_driver::compile!(
    options: "",
    manifest: "manifest_bmi270.ddsl"
);

/// The expected chip ID for the BMI270.
const BMI270_CHIP_ID: u8 = 0x24;

#[derive(Debug, Clone, PartialEq)]
pub enum Error<E> {
    Transport(E),
    ChipId(u8),
    BadInit,
    CmdTimeout,
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Error::Transport(value)
    }
}

pub struct Bmi270<I> {
    inner: InnerBmi270<I>,
    acc_range: AccRangeVariant,
    gyr_range: GyrRangeVariant,
}

impl AccRangeVariant {
    pub const fn scalar(&self) -> f32 {
        const FSDIV: f32 = i16::MAX as f32 + 1.0;
        match self {
            AccRangeVariant::Gs2 => 2.0 / FSDIV,
            AccRangeVariant::Gs4 => 4.0 / FSDIV,
            AccRangeVariant::Gs8 => 8.0 / FSDIV,
            AccRangeVariant::Gs16 => 16.0 / FSDIV,
        }
    }
}

impl GyrRangeVariant {
    pub const fn scalar(&self) -> f32 {
        const FSDIV: f32 = i16::MAX as f32 + 1.0;
        match self {
            GyrRangeVariant::Dps2000 => 2000.0 / FSDIV,
            GyrRangeVariant::Dps1000 => 1000.0 / FSDIV,
            GyrRangeVariant::Dps500 => 500.0 / FSDIV,
            GyrRangeVariant::Dps250 => 250.0 / FSDIV,
            GyrRangeVariant::Dps125 => 125.0 / FSDIV,
        }
    }
}

impl<I: AsyncRegisterInterface<AddressType = u8>> Bmi270<I> {
    /// Initialize the BMI270: soft reset, disable APS, load features, verify chip ID.
    ///
    /// The firmware blob is loaded automatically from a bundled constant.
    pub async fn initialize(
        interface: I,
        mut delay: impl DelayNs,
    ) -> Result<Self, Error<I::Error>> {
        let mut bmi = Bmi270 {
            inner: InnerBmi270::new(interface),
            acc_range: AccRangeVariant::default(),
            gyr_range: GyrRangeVariant::default(),
        };

        // Dummy read to ensure SPI can work correctly
        let _ = bmi.read_chip_id().await?;
        delay.delay_ms(2).await;

        // Soft reset and wait for ready
        bmi.soft_reset().await?;
        bmi.wait_cmd_ready(&mut delay).await?;

        // Disable advanced power save
        bmi.set_adv_power_save(false).await?;

        // Wait at least 450 us for APS disable to take effect
        delay.delay_ms(1).await;

        // Load feature configuration
        bmi.load_init_data(delay).await?;

        // Verify chip ID
        let chip_id = bmi.read_chip_id().await?;
        if chip_id != BMI270_CHIP_ID {
            return Err(Error::ChipId(chip_id));
        }

        // Set default ranges
        bmi.set_acc_range(bmi.acc_range).await?;
        bmi.set_gyr_range(bmi.gyr_range).await?;

        Ok(bmi)
    }

    /// Load feature initialization data into the device from the bundled firmware.
    async fn load_init_data(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I::Error>> {
        let data = firmware::BMI270_FIRMWARE;

        // Prepare config load: write 0x00 to arm the firmware loader
        self.inner
            .init_ctrl()
            .write_async(|reg| reg.set_init_ctrl(InitCtrlVariant::Prepare))
            .await?;

        // Write init data in 32-byte chunks
        for (chunk_idx, chunk) in data.chunks(32).enumerate() {
            // Address unit is 2 bytes, so chunk_idx * 16 gives the byte offset / 2
            let addr = (chunk_idx * 16) as u16;
            self.inner
                .init_addr_0()
                .write_async(|reg| reg.set_base_0_3((addr & 0x0F) as u8))
                .await?;
            self.inner
                .init_addr_1()
                .write_async(|reg| reg.set_base_11_4((addr >> 4) as u8))
                .await?;

            // BMI270 auto-increments the internal address pointer after each INIT_DATA write.
            // Writing bytes individually is equivalent to a burst write.
            for &byte in chunk {
                self.inner
                    .init_data()
                    .write_async(|reg| reg.set_data(byte))
                    .await?;
            }
        }

        // Datasheet: wait 450µs after firmware upload before triggering init
        delay.delay_us(450).await;

        // Trigger initialization
        self.inner
            .init_ctrl()
            .write_async(|reg| reg.set_init_ctrl(InitCtrlVariant::Trigger))
            .await?;

        // Wait for initialization to complete
        self.wait_init_done(delay).await?;
        Ok(())
    }

    /// Poll INTERNAL_STATUS until init_status == InitOk, or error.
    async fn wait_init_done(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I::Error>> {
        // Datasheet: init completes within at most 20 ms
        for _ in 0..20 {
            delay.delay_ms(1).await;
            let status = self.inner.internal_status().read_async().await?;
            match status.init_status() {
                InitStatusVariant::NotInit => continue,
                InitStatusVariant::InitOk => return Ok(()),
                _ => return Err(Error::BadInit),
            }
        }
        Err(Error::BadInit)
    }

    /// Poll STATUS register until cmd_rdy is set.
    async fn wait_cmd_ready(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I::Error>> {
        // Datasheet: cmd completes within at most 20 ms
        for _ in 0..20 {
            delay.delay_ms(1).await;
            let status = self.inner.status().read_async().await?;
            if status.cmd_rdy() != 0 {
                return Ok(());
            }
        }
        Err(Error::CmdTimeout)
    }

    // --- Chip ID ---

    /// Read the chip ID (should be 0x24).
    pub fn read_chip_id(&mut self) -> impl Future<Output = Result<u8, I::Error>> {
        self.inner
            .chip_id()
            .read_async()
            .map_ok(|data| data.chip_id())
    }

    // --- Power Control ---

    /// Enable or disable the accelerometer.
    pub fn set_acc_enabled(&mut self, enabled: bool) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .pwr_ctrl()
            .modify_async(move |reg| reg.set_acc_en(enabled as u8))
    }

    /// Enable or disable the gyroscope.
    pub fn set_gyr_enabled(&mut self, enabled: bool) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .pwr_ctrl()
            .modify_async(move |reg| reg.set_gyr_en(enabled as u8))
    }

    /// Enable or disable the temperature sensor.
    pub fn set_temp_enabled(
        &mut self,
        enabled: bool,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .pwr_ctrl()
            .modify_async(move |reg| reg.set_temp_en(enabled as u8))
    }

    /// Set advanced power save mode.
    pub fn set_adv_power_save(&mut self, on: bool) -> impl Future<Output = Result<(), I::Error>> {
        self.inner.pwr_conf().modify_async(move |reg| {
            reg.set_adv_power_save(if on {
                AdvPowerSave::On
            } else {
                AdvPowerSave::Off
            })
        })
    }

    // --- Accelerometer Configuration ---

    /// Set the accelerometer output data rate.
    pub fn set_acc_odr(
        &mut self,
        odr: AccOdrVariant,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .acc_conf()
            .modify_async(move |reg| reg.set_acc_odr(odr))
    }

    /// Set the accelerometer bandwidth / average filter.
    pub fn set_acc_bwp(
        &mut self,
        bwp: AccBwpVariant,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .acc_conf()
            .modify_async(move |reg| reg.set_acc_bwp(bwp))
    }

    /// Set the accelerometer filter performance mode.
    pub fn set_acc_filter_perf(
        &mut self,
        perf: AccFilterPerf,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .acc_conf()
            .modify_async(move |reg| reg.set_acc_filter_perf(perf))
    }

    /// Set the accelerometer measurement range.
    pub async fn set_acc_range(&mut self, range: AccRangeVariant) -> Result<(), I::Error> {
        self.inner
            .acc_range()
            .modify_async(move |reg| reg.set_acc_range(range))
            .await?;
        self.acc_range = range;
        Ok(())
    }

    // --- Gyroscope Configuration ---

    /// Set the gyroscope output data rate.
    pub fn set_gyr_odr(
        &mut self,
        odr: GyrOdrVariant,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .gyr_conf()
            .modify_async(move |reg| reg.set_gyr_odr(odr))
    }

    /// Set the gyroscope bandwidth.
    pub fn set_gyr_bwp(
        &mut self,
        bwp: GyrBwpVariant,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .gyr_conf()
            .modify_async(move |reg| reg.set_gyr_bwp(bwp))
    }

    /// Set the gyroscope noise performance mode.
    pub fn set_gyr_noise_perf(
        &mut self,
        perf: GyrNoisePerf,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .gyr_conf()
            .modify_async(move |reg| reg.set_gyr_noise_perf(perf))
    }

    /// Set the gyroscope filter performance mode.
    pub fn set_gyr_filter_perf(
        &mut self,
        perf: GyrFilterPerf,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .gyr_conf()
            .modify_async(move |reg| reg.set_gyr_filter_perf(perf))
    }

    /// Set the gyroscope full-scale range.
    pub async fn set_gyr_range(&mut self, range: GyrRangeVariant) -> Result<(), I::Error> {
        self.inner
            .gyr_range()
            .modify_async(move |reg| reg.set_gyr_range(range))
            .await?;
        self.gyr_range = range;
        Ok(())
    }

    // --- Data Readout ---

    /// Read raw accelerometer data in LSB (X, Y, Z).
    pub fn read_acc_raw(&mut self) -> impl Future<Output = Result<[i16; 3], I::Error>> {
        self.inner
            .acc_data()
            .read_async()
            .map_ok(|data| [data.acc_x(), data.acc_y(), data.acc_z()])
    }

    /// Read accelerometer data scaled to g according to the configured range.
    pub fn read_acc_scaled(&mut self) -> impl Future<Output = Result<[f32; 3], I::Error>> {
        let scalar = self.acc_range.scalar();
        self.inner.acc_data().read_async().map_ok(move |acc| {
            [
                acc.acc_x() as f32 * scalar,
                acc.acc_y() as f32 * scalar,
                acc.acc_z() as f32 * scalar,
            ]
        })
    }

    /// Read raw gyroscope data in LSB (X, Y, Z).
    pub fn read_gyr_raw(&mut self) -> impl Future<Output = Result<[i16; 3], I::Error>> {
        self.inner
            .gyr_data()
            .read_async()
            .map_ok(|data| [data.gyr_x(), data.gyr_y(), data.gyr_z()])
    }

    /// Read gyroscope data scaled to deg/s according to the configured range.
    pub fn read_gyr_scaled(&mut self) -> impl Future<Output = Result<[f32; 3], I::Error>> {
        let scalar = self.gyr_range.scalar();
        self.inner.gyr_data().read_async().map_ok(move |gyr| {
            [
                gyr.gyr_x() as f32 * scalar,
                gyr.gyr_y() as f32 * scalar,
                gyr.gyr_z() as f32 * scalar,
            ]
        })
    }

    /// Read the temperature sensor data and convert to degrees Celsius.
    pub fn read_temp(&mut self) -> impl Future<Output = Result<f32, I::Error>> {
        self.inner.temperature().read_async().map_ok(|t| {
            let raw = t.temp_data();
            // 0x0000 = 23C, resolution 1/512 K/LSB
            raw as f32 * (1.0 / 512.0) + 23.0
        })
    }

    // --- Status and Data Ready ---

    /// Check if accelerometer data is ready.
    pub fn data_ready_acc(&mut self) -> impl Future<Output = Result<bool, I::Error>> {
        self.inner
            .status()
            .read_async()
            .map_ok(|s| s.drdy_acc() != 0)
    }

    /// Check if gyroscope data is ready.
    pub fn data_ready_gyr(&mut self) -> impl Future<Output = Result<bool, I::Error>> {
        self.inner
            .status()
            .read_async()
            .map_ok(|s| s.drdy_gyr() != 0)
    }

    /// Check data-ready from interrupt status register.
    pub fn data_ready_int_acc(&mut self) -> impl Future<Output = Result<bool, I::Error>> {
        self.inner
            .int_status_1()
            .read_async()
            .map_ok(|s| s.acc_drdy_int() != 0)
    }

    /// Check data-ready from interrupt status register (gyro).
    pub fn data_ready_int_gyr(&mut self) -> impl Future<Output = Result<bool, I::Error>> {
        self.inner
            .int_status_1()
            .read_async()
            .map_ok(|s| s.gyr_drdy_int() != 0)
    }

    /// Read the event register for POR detection and error codes.
    pub fn read_event(
        &mut self,
    ) -> impl Future<Output = Result<(bool, ErrorCodeVariant), I::Error>> {
        self.inner
            .event()
            .read_async()
            .map_ok(|e| (e.por_detected() != 0, e.error_code()))
    }

    /// Read the error register.
    pub fn read_err_reg(
        &mut self,
    ) -> impl Future<Output = Result<(bool, u8, bool, bool), I::Error>> {
        self.inner.err_reg().read_async().map_ok(|e| {
            (
                e.fatal_err() != 0,
                e.internal_err(),
                e.fifo_err() != 0,
                e.aux_err() != 0,
            )
        })
    }

    // --- Sensortime ---

    /// Read the 24-bit sensortime counter (39.0625 us per tick).
    pub fn read_sensortime(&mut self) -> impl Future<Output = Result<u32, I::Error>> {
        self.inner
            .sensortime()
            .read_async()
            .map_ok(|t| t.sensor_time())
    }

    // --- Soft Reset ---

    /// Perform a soft-reset (write 0xB6 to CMD register).
    pub fn soft_reset(&mut self) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .cmd()
            .write_async(|reg| reg.set_cmd(CmdVariant::Softreset))
    }

    /// Flush the FIFO (write 0xB0 to CMD register).
    pub fn fifo_flush(&mut self) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .cmd()
            .write_async(|reg| reg.set_cmd(CmdVariant::FifoFlush))
    }

    // --- Interrupt Configuration ---

    /// Configure INT1 pin electrical behavior.
    pub fn configure_int1(
        &mut self,
        active_high: bool,
        open_drain: bool,
        output_en: bool,
        input_en: bool,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner.int_1_io_ctrl().modify_async(move |reg| {
            reg.set_int_1_lvl(active_high as u8);
            reg.set_int_1_od(open_drain as u8);
            reg.set_int_1_output_en(output_en as u8);
            reg.set_int_1_input_en(input_en as u8);
        })
    }

    /// Configure INT2 pin electrical behavior.
    pub fn configure_int2(
        &mut self,
        active_high: bool,
        open_drain: bool,
        output_en: bool,
        input_en: bool,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner.int_2_io_ctrl().modify_async(move |reg| {
            reg.set_int_2_lvl(active_high as u8);
            reg.set_int_2_od(open_drain as u8);
            reg.set_int_2_output_en(output_en as u8);
            reg.set_int_2_input_en(input_en as u8);
        })
    }

    /// Map data-ready, FIFO, and error interrupts to the INT1 pin.
    pub fn map_int1(
        &mut self,
        fifo_watermark: bool,
        fifo_full: bool,
        data_ready: bool,
        error: bool,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner.int_map_data().modify_async(move |reg| {
            reg.set_fwm_int_1(fifo_watermark as u8);
            reg.set_ffull_int_1(fifo_full as u8);
            reg.set_drdy_int_1(data_ready as u8);
            reg.set_err_int_1(error as u8);
        })
    }

    /// Map data-ready, FIFO, and error interrupts to the INT2 pin.
    pub fn map_int2(
        &mut self,
        fifo_watermark: bool,
        fifo_full: bool,
        data_ready: bool,
        error: bool,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner.int_map_data().modify_async(move |reg| {
            reg.set_fwm_int_2(fifo_watermark as u8);
            reg.set_ffull_int_2(fifo_full as u8);
            reg.set_drdy_int_2(data_ready as u8);
            reg.set_err_int_2(error as u8);
        })
    }

    // --- FIFO Configuration ---

    /// Set the FIFO watermark level (in bytes, 13-bit).
    pub fn set_fifo_watermark(&mut self, level: u16) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_wtm()
            .modify_async(move |reg| reg.set_fifo_water_mark(level & 0x1FFF))
    }

    /// Enable or disable FIFO stop-on-full behavior.
    pub fn set_fifo_stop_on_full(
        &mut self,
        stop: bool,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_config_0()
            .modify_async(move |reg| reg.set_fifo_stop_on_full(stop as u8))
    }

    /// Enable or disable sensortime frame in FIFO.
    pub fn set_fifo_time_en(&mut self, enable: bool) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_config_0()
            .modify_async(move |reg| reg.set_fifo_time_en(enable as u8))
    }

    /// Enable or disable FIFO header mode.
    pub fn set_fifo_header_en(
        &mut self,
        enable: bool,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_config_1()
            .modify_async(move |reg| reg.set_fifo_header_en(enable as u8))
    }

    /// Enable or disable storing accelerometer data in FIFO.
    pub fn set_fifo_acc_en(&mut self, enable: bool) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_config_1()
            .modify_async(move |reg| reg.set_fifo_acc_en(enable as u8))
    }

    /// Enable or disable storing gyroscope data in FIFO.
    pub fn set_fifo_gyr_en(&mut self, enable: bool) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_config_1()
            .modify_async(move |reg| reg.set_fifo_gyr_en(enable as u8))
    }

    /// Read the FIFO fill level in bytes (14-bit).
    pub fn read_fifo_length(&mut self) -> impl Future<Output = Result<u16, I::Error>> {
        self.inner
            .fifo_length()
            .read_async()
            .map_ok(|f| f.byte_counter())
    }

    // FIFO data buffer read is available via the FifoData buffer register (0x26).
    // Use the generated inner.fifo_data() methods directly with a BufferInterfaceBase bound.

    /// Set accelerometer FIFO downsampling factor (2^n).
    pub fn set_acc_fifo_downs(&mut self, downs: u8) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_downs()
            .modify_async(move |reg| reg.set_acc_fifo_downs(downs & 0x07))
    }

    /// Set gyroscope FIFO downsampling factor (2^n).
    pub fn set_gyr_fifo_downs(&mut self, downs: u8) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_downs()
            .modify_async(move |reg| reg.set_gyr_fifo_downs(downs & 0x07))
    }

    /// Select whether accelerometer FIFO uses filtered or unfiltered data.
    pub fn set_acc_fifo_filt_data(
        &mut self,
        filtered: bool,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_downs()
            .modify_async(move |reg| reg.set_acc_fifo_filt_data(filtered as u8))
    }

    /// Select whether gyroscope FIFO uses filtered or unfiltered data.
    pub fn set_gyr_fifo_filt_data(
        &mut self,
        filtered: bool,
    ) -> impl Future<Output = Result<(), I::Error>> {
        self.inner
            .fifo_downs()
            .modify_async(move |reg| reg.set_gyr_fifo_filt_data(filtered as u8))
    }
}
