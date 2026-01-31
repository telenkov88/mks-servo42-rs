#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MotorType {
    Deg09 = 0x00,
    Deg18 = 0x01,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum WorkMode {
    Open = 0x00,
    Vfoc = 0x01,
    Uart = 0x02,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EnLogic {
    Low = 0x00,
    High = 0x01,
    AlwaysOn = 0x02,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BaudRate {
    Baud9600 = 0x01,
    Baud19200 = 0x02,
    Baud25000 = 0x03,
    Baud38400 = 0x04,
    Baud57600 = 0x05,
    Baud115200 = 0x06,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ZeroMode {
    Disable = 0x00,
    DirMode = 0x01,
    NearMode = 0x02,
}
