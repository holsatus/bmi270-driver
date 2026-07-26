use device_driver::{
    AsyncBufferInterface, AsyncRegisterInterface, BufferInterfaceBase, RegisterInterfaceBase,
};
use embedded_hal_async::{
    i2c::{self, I2c},
    spi::{self, SpiDevice},
};

pub struct SpiWrap<I> {
    pub(crate) inner: I,
}

impl<I: SpiDevice> RegisterInterfaceBase for SpiWrap<I> {
    type Error = <I as spi::ErrorType>::Error;
    type AddressType = u8;
}

impl<I: SpiDevice> AsyncRegisterInterface for SpiWrap<I> {
    async fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &device_driver::FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.inner
            .transaction(&mut [
                spi::Operation::Write(&[address]),
                spi::Operation::Write(data),
            ])
            .await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &device_driver::FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        const READ_DIR: u8 = 0x80;
        // BMI270 SPI requires a dummy byte before actual register data.
        self.inner
            .transaction(&mut [
                spi::Operation::Write(&[address | READ_DIR]),
                spi::Operation::Read(&mut [0u8]),
                spi::Operation::Read(data),
            ])
            .await
    }
}

impl<I: SpiDevice> BufferInterfaceBase for SpiWrap<I> {
    type Error = <I as spi::ErrorType>::Error;
    type AddressType = u8;
}

impl<I: SpiDevice> AsyncBufferInterface for SpiWrap<I> {
    async fn write(&mut self, address: u8, buf: &[u8]) -> Result<usize, Self::Error> {
        self.inner
            .transaction(&mut [
                spi::Operation::Write(&[address]),
                spi::Operation::Write(buf),
            ])
            .await
            .map(|_| buf.len())
    }

    async fn flush(&mut self, _address: u8) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn read(&mut self, address: u8, buf: &mut [u8]) -> Result<usize, Self::Error> {
        const READ_DIR: u8 = 0x80;
        self.inner
            .transaction(&mut [
                spi::Operation::Write(&[address | READ_DIR]),
                spi::Operation::Read(&mut [0u8]),
                spi::Operation::Read(buf),
            ])
            .await
            .map(|_| buf.len())
    }
}

pub struct I2cWrap<I> {
    pub(crate) inner: I,
    pub(crate) device_address: u8,
}

impl<I: I2c> RegisterInterfaceBase for I2cWrap<I> {
    type Error = <I as i2c::ErrorType>::Error;
    type AddressType = u8;
}

impl<I: I2c> AsyncRegisterInterface for I2cWrap<I> {
    async fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &device_driver::FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.inner
            .transaction(
                self.device_address,
                &mut [
                    i2c::Operation::Write(&[address]),
                    i2c::Operation::Write(data),
                ],
            )
            .await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &device_driver::FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.inner
            .transaction(
                self.device_address,
                &mut [
                    i2c::Operation::Write(&[address]),
                    i2c::Operation::Read(data),
                ],
            )
            .await
    }
}

impl<I: I2c> BufferInterfaceBase for I2cWrap<I> {
    type Error = <I as i2c::ErrorType>::Error;
    type AddressType = u8;
}

impl<I: I2c> AsyncBufferInterface for I2cWrap<I> {
    async fn write(&mut self, address: u8, buf: &[u8]) -> Result<usize, Self::Error> {
        self.inner
            .transaction(
                self.device_address,
                &mut [
                    i2c::Operation::Write(&[address]),
                    i2c::Operation::Write(buf),
                ],
            )
            .await
            .map(|_| buf.len())
    }

    async fn flush(&mut self, _address: u8) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn read(&mut self, address: u8, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.inner
            .transaction(
                self.device_address,
                &mut [i2c::Operation::Write(&[address]), i2c::Operation::Read(buf)],
            )
            .await
            .map(|_| buf.len())
    }
}
