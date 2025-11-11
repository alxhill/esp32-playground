#![no_std]

use embedded_hal::i2c::{Error, ErrorKind, I2c};

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[allow(non_camel_case_types)]
pub enum Register {
    STATUS_MMA8452Q = 0x00,
    OUT_X_MSB = 0x01,
    OUT_X_LSB = 0x02,
    OUT_Y_MSB = 0x03,
    OUT_Y_LSB = 0x04,
    OUT_Z_MSB = 0x05,
    OUT_Z_LSB = 0x06,
    SYSMOD = 0x0B,
    INT_SOURCE = 0x0C,
    WHO_AM_I = 0x0D,
    XYZ_DATA_CFG = 0x0E,
    HP_FILTER_CUTOFF = 0x0F,
    PL_STATUS = 0x10,
    PL_CFG = 0x11,
    PL_COUNT = 0x12,
    PL_BF_ZCOMP = 0x13,
    P_L_THS_REG = 0x14,
    FF_MT_CFG = 0x15,
    FF_MT_SRC = 0x16,
    FF_MT_THS = 0x17,
    FF_MT_COUNT = 0x18,
    TRANSIENT_CFG = 0x1D,
    TRANSIENT_SRC = 0x1E,
    TRANSIENT_THS = 0x1F,
    TRANSIENT_COUNT = 0x20,
    PULSE_CFG = 0x21,
    PULSE_SRC = 0x22,
    PULSE_THSX = 0x23,
    PULSE_THSY = 0x24,
    PULSE_THSZ = 0x25,
    PULSE_TMLT = 0x26,
    PULSE_LTCY = 0x27,
    PULSE_WIND = 0x28,
    ASLP_COUNT = 0x29,
    CTRL_REG1 = 0x2A,
    CTRL_REG2 = 0x2B,
    CTRL_REG3 = 0x2C,
    CTRL_REG4 = 0x2D,
    CTRL_REG5 = 0x2E,
    OFF_X = 0x2F,
    OFF_Y = 0x30,
    OFF_Z = 0x31,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Scale {
    Scale2G = 0b00,
    Scale4G = 0b01,
    Scale8G = 0b10,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OutputDataRate {
    Odr800,
    Odr400,
    Odr200,
    Odr100,
    Odr50,
    Odr12,
    Odr6,
    Odr1,
}

pub struct Accel<I2C: embedded_hal::i2c::I2c> {
    addr: u8,
    i2c: I2C,
}

impl<I2C: I2c> Accel<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Accel { i2c, addr }
    }

    /*
    byte MMA8452Q::init(MMA8452Q_Scale fsr, MMA8452Q_ODR odr)
    {
    scale = fsr; // Haul fsr into our class variable, scale

    if (_i2cPort == NULL)
    {
        _i2cPort = &Wire;
    }

    _i2cPort->begin(); // Initialize I2C

    byte c = readRegister(WHO_AM_I); // Read WHO_AM_I register

    if (c != 0x2A) // WHO_AM_I should always be 0x2A
    {
        return 0;
    }

    standby(); // Must be in standby to change registers

    setScale(scale);  // Set up accelerometer scale
    setDataRate(odr); // Set up output data rate
    setupPL();		  // Set up portrait/landscape detection
    // Multiply parameter by 0.0625g to calculate threshold.
    setupTap(0x80, 0x80, 0x08); // Disable x, y, set z to 0.5g

    active(); // Set to active to start reading

    return 1;
    }
     *
     */

    pub fn init(&mut self, scale: Scale, odr: OutputDataRate) -> Result<(), I2C::Error> {
        if self.read_register(Register::WHO_AM_I)? != 0x2A {
            panic!("incorrect device id");
        }

        self.standby()?;

        self.set_scale(scale)?;
        self.set_data_rate(odr)?;

        // todo: set pl + tap

        Ok(())
    }

    pub fn get_x(&mut self) -> Result<i16, I2C::Error> {
        let mut raw = [0u8; 2];
        // reads both OUT_X_MSB and OUT_X_LSB registers
        self.read_registers(Register::OUT_X_MSB, &mut raw)?;
        Ok((raw[0] as i16) << 8 | (raw[1] as i16 >> 4))
    }

    pub fn set_scale(&mut self, scale: Scale) -> Result<(), I2C::Error> {
        let mut cfg = self.read_register(Register::XYZ_DATA_CFG)?;
        cfg &= 0xFC; // mask out scale bits
        cfg |= scale as u8;
        self.write_register(Register::XYZ_DATA_CFG, cfg)
    }

    pub fn set_data_rate(&mut self, odr: OutputDataRate) -> Result<(), I2C::Error> {
        let mut ctrl = self.read_register(Register::CTRL_REG1)?;
        ctrl &= 0xF0; // mask out data rate bits
        ctrl |= odr as u8;
        self.write_register(Register::CTRL_REG1, ctrl)
    }

    pub fn standby(&mut self) -> Result<(), I2C::Error> {
        let ctrl_standby = self.read_register(Register::CTRL_REG1)? & !0x01;
        self.write_register(Register::CTRL_REG1, ctrl_standby)
    }

    pub fn active(&mut self) -> Result<(), I2C::Error> {
        let ctrl_active = self.read_register(Register::CTRL_REG1)? | 0x01;
        self.write_register(Register::CTRL_REG1, ctrl_active)
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut res = [0u8; 1];
        self.read_registers(reg, &mut res).map(|r| r[0])
    }

    fn read_registers<'a>(
        &mut self,
        reg: Register,
        res: &'a mut [u8],
    ) -> Result<&'a mut [u8], I2C::Error> {
        self.i2c
            .write_read(self.addr, &[reg as u8], res)
            .map(|_| res)
    }

    fn write_register(&mut self, reg: Register, val: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg as u8, val])
    }
}
