//! A platform agnostic driver to interface with the L3GD20 (gyroscope)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/0.2
//!
//! # Examples
//!
//! You should find at least one example in the [f3] crate.
//!
//! [f3]: https://docs.rs/f3/0.6

#![deny(missing_docs)]
#![deny(warnings)]
#![allow(non_camel_case_types)]
#![no_std]



use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::{Mode};

/// SPI mode
pub const MODE: Mode = embedded_hal::spi::MODE_3;


/// L3GD20 driver
pub struct L3gd20<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS, E> L3gd20<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    /// Creates a new driver from a SPI peripheral and a NCS pin
    pub fn new(spi: SPI, cs: CS) -> Result<Self, E> {
        let mut l3gd20 = L3gd20 { spi, cs };

        // Reset memory content
        l3gd20.write_register(Register::FIFO_CTRL_REG, 0b1000_0000)?;
        l3gd20.write_register(Register::FIFO_CTRL_REG, 0b0000_0000)?;

        // power up and enable all the axes
        l3gd20.write_register(Register::CTRL_REG1, 0b00_00_1_111)?;

        Ok(l3gd20)
    }

    /// Temperature measurement + gyroscope measurements
    pub fn all(&mut self, ) -> Result<Measurements, E> {
        let mut bytes = [0u8; 9];
        self.read_many(Register::OUT_TEMP, &mut bytes)?;

        Ok(Measurements {
            gyro: I16x3 {
                x: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
                y: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
                z: (bytes[7] as u16 + ((bytes[8] as u16) << 8)) as i16,
            },
            temp: bytes[1] as i8,
        })
    }

    /// Gyroscope measurements
    pub fn gyro(&mut self) -> Result<I16x3, E> {
        let mut bytes = [0u8; 7];
        self.read_many(Register::OUT_X_L, &mut bytes)?;

        Ok(I16x3 {
            x: (bytes[1] as u16 + ((bytes[2] as u16) << 8)) as i16,
            y: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
            z: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
        })
    }

    /// Gyroscope FIFO reading
    /// note; constant generics cannot be used in fuction
    /// must supply N = (# rows to read) * 6 + 1
    pub fn gyro_fifo<const N: usize, const M: usize>(&mut self) -> Result<I16x3Buf<M>, E> {

        let mut bytes = [0u8; N];
        self.read_many(Register::OUT_X_L, &mut bytes)?;

        let mut results: I16x3Buf::<M> = I16x3Buf { i16x3buf: [I16x3 {x: 0, y: 0, z: 0 }; M]};

        for i in 0..M {
            let k = i*6;
            results.i16x3buf[i] = I16x3 {
                                x: (bytes[k+1] as u16 + ((bytes[k+2] as u16) << 8)) as i16,
                                y: (bytes[k+3] as u16 + ((bytes[k+4] as u16) << 8)) as i16,
                                z: (bytes[k+5] as u16 + ((bytes[k+6] as u16) << 8)) as i16,
                                };
        }

        Ok(results)
    }


    /// Temperature sensor measurement
    pub fn temp(&mut self) -> Result<i8, E> {
        Ok(self.read_register(Register::OUT_TEMP)? as i8)
    }

    /// Reads the WHO_AM_I register; should return `0xD4`
    pub fn who_am_i(&mut self) -> Result<u8, E> {
        self.read_register(Register::WHO_AM_I)
    }

    /// Read `STATUS_REG` of sensor
    pub fn status(&mut self) -> Result<Status, E> {
        let sts = self.read_register(Register::STATUS_REG)?;
        Ok(Status::from_u8(sts))
    }

    /// Get the current Output Data Rate
    pub fn odr(&mut self) -> Result<Odr, E> {
        // Read control register
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        Ok(Odr::from_u8(reg1))
    }
    /// Get the current INT2/DRDY mode
    pub fn int2_mode(&mut self) -> Result<I2Mode, E> {
        let reg3 = self.read_register(Register::CTRL_REG3)?;
        Ok(I2Mode::from_u8(reg3))
    }
    
    /// Get the current FIFO enable/disable state
    pub fn fifo_toggle(&mut self) -> Result<FIFOToggle, E> {
        let reg5 = self.read_register(Register::CTRL_REG5)?;
        Ok(FIFOToggle::from_u8(reg5))
    }

    /// Get the current FIFO mode setting
    pub fn fifo_mode(&mut self) -> Result<FIFOMode, E> {
        let fcr = self.read_register(Register::FIFO_CTRL_REG)?;
        Ok(FIFOMode::from_u8(fcr))
    }


    /// Set the Output Data Rate
    pub fn set_odr(&mut self, odr: Odr) -> Result<&mut Self, E> {
        self.change_config(Register::CTRL_REG1, odr)
    }

    /// Get current Bandwidth
    pub fn bandwidth(&mut self) -> Result<Bandwidth, E> {
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        Ok(Bandwidth::from_u8(reg1))
    }

    /// Set low-pass cut-off frequency (i.e. bandwidth)
    ///
    /// See `Bandwidth` for further explanation
    pub fn set_bandwidth(&mut self, bw: Bandwidth) -> Result<&mut Self, E> {
        self.change_config(Register::CTRL_REG1, bw)
    }

    /// Get the current Full Scale Selection
    ///
    /// This is the sensitivity of the sensor, see `Scale` for more information
    pub fn scale(&mut self) -> Result<Scale, E> {
        let scl = self.read_register(Register::CTRL_REG4)?;
        Ok(Scale::from_u8(scl))
    }

    /// Set the Full Scale Selection
    ///
    /// This sets the sensitivity of the sensor, see `Scale` for more
    /// information
    pub fn set_scale(&mut self, scale: Scale) -> Result<&mut Self, E> {
        self.change_config(Register::CTRL_REG4, scale)
    }

    /// Change INT2/DRDY settings
    ///
    /// This allows changing modes for  INT2/DRDY

    pub fn set_int2_mode(&mut self, i2mode: I2Mode) -> Result<&mut Self, E> {
        self.change_config(Register::CTRL_REG3, i2mode)
    }
    
    /// Enable or disable the FIFO
    ///
    /// This allows changing modes for  INT2/DRDY
    pub fn set_fifo_toggle(&mut self, fifotoggle: FIFOToggle) -> Result <&mut Self, E> {
        self.change_config(Register::CTRL_REG5, fifotoggle)
    }

    /// Change FIFO mode
    /// There are 5 different FIFO mode: Bypass, FIFO, Stream, Stream-to-FIFO, Bypass-to-Stream

    pub fn set_fifo_mode(&mut self, fifomode: FIFOMode) -> Result <&mut Self, E> {

        self.change_config(Register::FIFO_CTRL_REG, fifomode)
    }

    /// Change watermark level
    /// Watermark can be set between 0 and 32 (6bit)

    pub fn set_wtm_tr(&mut self, threshold: usize) -> Result <&mut Self, E> {

        let bitvalue_threshold: BitValueu8 = BitValueu8 {value: threshold as u8};
        self.change_config(Register::FIFO_CTRL_REG, bitvalue_threshold)
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, E> {
        let _ = self.cs.set_low();

        let mut buffer = [reg.addr() | SINGLE | READ, 0];
        self.spi.transfer(&mut buffer)?;

        let _ = self.cs.set_high();

        Ok(buffer[1])
    }


    /// Read multiple bytes starting from the `start_reg` register.
    /// This function will attempt to fill the provided buffer.
    fn read_many(&mut self, start_reg: Register, buffer: &mut [u8])-> Result<(), E> {

        let _ = self.cs.set_low();
        buffer[0] = start_reg.addr() | MULTI | READ ;
        self.spi.transfer(buffer)?;
        let _ = self.cs.set_high();

        Ok(())
    }


    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        let _ = self.cs.set_low();

        let buffer = [reg.addr() | SINGLE | WRITE, byte];
        self.spi.write(&buffer)?;

        let _ = self.cs.set_high();

        Ok(())
    }

    /// Change configuration in register
    ///
    /// Helper function to update a particular part of a register without
    /// affecting other parts of the register that might contain desired
    /// configuration. This allows the `L3gd20` struct to be used like
    /// a builder interface when configuring specific parameters.
    fn change_config<B: BitValue>(&mut self, reg: Register, bits: B) -> Result<&mut Self, E> {
        // Create bit mask from width and shift of value
        let mask = B::mask() << B::shift();
        // Extract the value as u8
        let bits = (bits.value() << B::shift()) & mask;
        // Read current value of register
        let current = self.read_register(reg)?;
        // Use supplied mask so we don't affect more than necessary
        let masked = current & !mask;
        // Use `or` to apply the new value without affecting other parts
        let new_reg = masked | bits;
        self.write_register(reg, new_reg)?;
        Ok(self)
    }
}

/// Trait to represent a value that can be sent to sensor
trait BitValue {
    /// The width of the bitfield in bits
    fn width() -> u8;
    /// The bit 'mask' of the value
    fn mask() -> u8 {
        (1 << Self::width()) - 1
    }
    /// The number of bits to shift the mask by
    fn shift() -> u8;
    /// Convert the type to a byte value to be sent to sensor
    ///
    /// # Note
    /// This value should not be bit shifted.
    fn value(&self) -> u8;
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
enum Register {
    WHO_AM_I = 0x0F,
    CTRL_REG1 = 0x20,
    CTRL_REG2 = 0x21,
    CTRL_REG3 = 0x22,
    CTRL_REG4 = 0x23,
    CTRL_REG5 = 0x24,
    REFERENCE = 0x25,
    OUT_TEMP = 0x26,
    STATUS_REG = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    FIFO_CTRL_REG = 0x2E,
    FIFO_SRC_REG = 0x2F,
    INT1_CFG = 0x30,
    INT1_SRC = 0x31,
    INT1_TSH_XH = 0x32,
    INT1_TSH_XL = 0x33,
    INT1_TSH_YH = 0x34,
    INT1_TSH_YL = 0x35,
    INT1_TSH_ZH = 0x36,
    INT1_TSH_ZL = 0x37,
    INT1_DURATION = 0x38,
}



/// Output Data Rate
#[derive(Debug, Clone, Copy)]
pub enum Odr {
    /// 95 Hz data rate
    Hz95 = 0x00,
    /// 190 Hz data rate
    Hz190 = 0x01,
    /// 380 Hz data rate
    Hz380 = 0x02,
    /// 760 Hz data rate
    Hz760 = 0x03,
}

impl BitValue for Odr {
    fn width() -> u8 {
        2
    }
    fn shift() -> u8 {
        6
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Odr {
    fn from_u8(from: u8) -> Self {
        // Extract ODR value, converting to enum (ROI: 0b1100_0000)
        match (from >> Odr::shift()) & Odr::mask() {
            x if x == Odr::Hz95 as u8 => Odr::Hz95,
            x if x == Odr::Hz190 as u8 => Odr::Hz190,
            x if x == Odr::Hz380 as u8 => Odr::Hz380,
            x if x == Odr::Hz760 as u8 => Odr::Hz760,
            _ => unreachable!(),
        }
    }
}

/// Full scale selection
#[derive(Debug, Clone, Copy)]
pub enum Scale {
    /// 250 Degrees Per Second
    Dps250 = 0x00,
    /// 500 Degrees Per Second
    Dps500 = 0x01,
    /// 2000 Degrees Per Second
    Dps2000 = 0x03,
}

impl BitValue for Scale {
    fn width() -> u8 {
        2
    }
    fn shift() -> u8 {
        4
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Scale {
    fn from_u8(from: u8) -> Self {
        // Extract scale value from register, ensure that we mask with
        // `0b0000_0011` to extract `FS1-FS2` part of register
        match (from >> Scale::shift()) & Scale::mask() {
            x if x == Scale::Dps250 as u8 => Scale::Dps250,
            x if x == Scale::Dps500 as u8 => Scale::Dps500,
            x if x == Scale::Dps2000 as u8 => Scale::Dps2000,
            // Special case for Dps2000
            0x02 => Scale::Dps2000,
            _ => unreachable!(),
        }
    }
}

/// Bandwidth of sensor
///
/// The bandwidth of the sensor is equal to the cut-off for the low-pass
/// filter. The cut-off depends on the `Odr` of the sensor, for specific
/// information consult the data sheet.
#[derive(Debug, Clone, Copy)]
pub enum Bandwidth {
    /// Lowest possible cut-off for any `Odr` configuration
    Low = 0x00,
    /// Medium cut-off, can be the same as `High` for some `Odr` configurations
    Medium = 0x01,
    /// High cut-off
    High = 0x02,
    /// Maximum cut-off for any `Odr` configuration
    Maximum = 0x03,
}

impl BitValue for Bandwidth {
    fn width() -> u8 {
        2
    }
    fn shift() -> u8 {
        4
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Bandwidth {
    fn from_u8(from: u8) -> Self {
        // Shift and mask bandwidth of register, (ROI: 0b0011_0000)
        match (from >> Bandwidth::shift()) & Bandwidth::mask() {
            x if x == Bandwidth::Low as u8 => Bandwidth::Low,
            x if x == Bandwidth::Medium as u8 => Bandwidth::Medium,
            x if x == Bandwidth::High as u8 => Bandwidth::High,
            x if x == Bandwidth::Maximum as u8 => Bandwidth::Maximum,
            _ => unreachable!(),
        }
    }
}

/// Interrupt mode on INT2/DRDY
///
/// Interrupt mode on INT2/DRDY, 5 options are available
/// Disabled (default), Interrupt on data ready, Interrupt on watermark level,
/// Interrupt on FIFO overflow (32 capacity), interrupt on empty FIFO
#[derive(Debug, Clone, Copy)]
pub enum I2Mode {
    /// Disable interrupts on INT2/DRDY
    I2_Disable = 0x00,
    /// Interrupt when date-ready on DRDY/INT2
    I2_DRDY = 0x08, 
    /// Interrupt when watermark level is reached
    I2_WTM = 0x04,
    /// Interrupt occurs when there is an overflow on fifo
    I2_ORun = 0x02,
    /// Interrupt occurs when the FIFO is empty
    I2_Empty = 0x01,
}

impl BitValue for I2Mode {
    fn width() -> u8 {
        4
    }
    fn shift() -> u8 {
        0
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl I2Mode {
    fn from_u8(from: u8) -> Self {
        // Shift and mask I2Mode of register, (ROI: 0b0000_1111)
        match (from >> I2Mode::shift()) & I2Mode::mask() {
            x if x == I2Mode::I2_Disable as u8 =>  I2Mode::I2_Disable,
            x if x == I2Mode::I2_DRDY as u8 =>   I2Mode::I2_DRDY,
            x if x == I2Mode::I2_WTM as u8 =>  I2Mode::I2_WTM,
            x if x == I2Mode::I2_ORun as u8 =>  I2Mode::I2_ORun,
            x if x == I2Mode::I2_Empty as u8 =>  I2Mode::I2_Empty,
            _ => unreachable!(),
        }
    }
}

/// Enable FIFO
///
/// Enable the FIFO, can be used to store values
/// Burst values out when necessary

#[derive(Debug, Clone, Copy)]
pub enum FIFOToggle {
    /// Disables the FIFO (default)
    FIFO_DI = 0x00,
    /// Enabls the FIFO
    FIFO_EN = 0x01,
}

impl BitValue for FIFOToggle {
    fn width() -> u8 {
        1
    }
    fn shift() -> u8 {
        6
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl FIFOToggle {
    fn from_u8(from: u8) -> Self {
        // Shift and mask FIFOToggle of register, (ROI: 0b0100_0000)
        match (from >> FIFOToggle::shift()) & FIFOToggle::mask() {
            x if x == FIFOToggle::FIFO_DI as u8 =>  FIFOToggle::FIFO_DI,
            x if x == FIFOToggle::FIFO_EN as u8 =>   FIFOToggle::FIFO_EN,
            _ => unreachable!(),
        }
    }
}

/// Change FIFO Operating mode
///
/// Change funcionality of the FIFO, review L3gd20 for more detail

#[derive(Debug, Clone, Copy)]
pub enum FIFOMode {
    /// Bypass mode (default)
    Bypass = 0x00,
    /// FIFO mode, data is stored til FIFO overflows then stops
    Fifo = 0x01,
    /// Stream mode, data is stored in FIFO, old data is replaced with new data
    Stream = 0x02,
    /// Stream-to-FIFO, can set value to change from stream to FIFO mode
    StrFif = 0x03,
    /// Bypass-to-Stream, can set value to change from Bypass to stream mode
    BypStr = 0x04,
}

impl BitValue for FIFOMode {
    fn width() -> u8 {
        3
    }
    fn shift() -> u8 {
        5
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl FIFOMode {
    fn from_u8(from: u8) -> Self {
        // Shift and mask FIFOMode of register, (ROI: 0b1110_0000)
        match (from >> FIFOMode::shift()) & FIFOMode::mask() {
            x if x ==  FIFOMode::Bypass as u8 =>  FIFOMode::Bypass,
            x if x ==  FIFOMode::Fifo as u8 =>   FIFOMode::Fifo,
            x if x ==  FIFOMode::Stream as u8 =>   FIFOMode::Stream,
            x if x ==  FIFOMode::StrFif as u8 =>   FIFOMode::StrFif,
            x if x ==  FIFOMode::BypStr as u8 =>   FIFOMode::BypStr,
            _ => unreachable!(),
        }
    }
}


/// Create a u8 struct that implements BitValue for the watermark u8
///
/// Makes it possible to use the config fn

#[derive(Debug, Clone, Copy)]
pub struct BitValueu8 {
    /// threshold value used to edit the register
    pub value: u8,

}

impl BitValue for BitValueu8 {
    fn width() -> u8 {
        5
    }
    fn shift() -> u8 {
        0
    }
    fn value(&self) -> u8 {
        self.value
    }
}


const READ: u8 = 1 << 7;
const WRITE: u8 = 0 << 7;
const MULTI: u8 = 1 << 6;
const SINGLE: u8 = 0 << 6;

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

impl Scale {
    /// Convert a measurement to degrees
    pub fn degrees(&self, val: i16) -> f32 {
        match *self {
            Scale::Dps250 => val as f32 * 0.00875,
            Scale::Dps500 => val as f32 * 0.0175,
            Scale::Dps2000 => val as f32 * 0.07,
        }
    }

    /// Convert a measurement to radians
    pub fn radians(&self, val: i16) -> f32 {
        // TODO: Use `to_radians` or other built in method
        // NOTE: `to_radians` is only exported in `std` (07.02.18)
        self.degrees(val) * (core::f32::consts::PI / 180.0)
    }
}

/// XYZ triple
#[derive(Debug, Clone, Copy)]
pub struct I16x3 {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// XYZ triple Buffer
#[derive(Debug, Clone, Copy)]
pub struct I16x3Buf <const N: usize> {

    /// buffer for multiple gyro values read from the fifo
    pub i16x3buf: [I16x3; N],
    
}

/// Several measurements
#[derive(Debug)]
pub struct Measurements {
    /// Gyroscope measurements
    pub gyro: I16x3,
    /// Temperature sensor measurement
    pub temp: i8,
}

/// Sensor status
#[derive(Debug, Clone, Copy)]
pub struct Status {
    /// Overrun (data has overwritten previously unread data)
    /// has occurred on at least one axis
    pub overrun: bool,
    /// Overrun occurred on Z-axis
    pub z_overrun: bool,
    /// Overrun occurred on Y-axis
    pub y_overrun: bool,
    /// Overrun occurred on X-axis
    pub x_overrun: bool,
    /// New data is available for either X, Y, Z - axis
    pub new_data: bool,
    /// New data is available on Z-axis
    pub z_new: bool,
    /// New data is available on Y-axis
    pub y_new: bool,
    /// New data is available on X-axis
    pub x_new: bool,
}

impl Status {
    fn from_u8(from: u8) -> Self {
        Status {
            overrun: (from & 1 << 7) != 0,
            z_overrun: (from & 1 << 6) != 0,
            y_overrun: (from & 1 << 5) != 0,
            x_overrun: (from & 1 << 4) != 0,
            new_data: (from & 1 << 3) != 0,
            z_new: (from & 1 << 2) != 0,
            y_new: (from & 1 << 1) != 0,
            x_new: (from & 1 << 0) != 0,
        }
    }
}
