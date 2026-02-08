/// Motor step angle configuration.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MotorType {
    /// 0.9° per step motor.
    Deg09 = 0x00,
    /// 1.8° per step motor.
    Deg18 = 0x01,
}

/// Motor operating mode.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum WorkMode {
    /// Open-loop mode.
    Open = 0x00,
    /// Vector Field Oriented Control (FOC) mode.
    Vfoc = 0x01,
    /// UART control mode.
    Uart = 0x02,
}

/// Enable (EN) pin logic configuration.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EnLogic {
    /// Active low.
    Low = 0x00,
    /// Active high.
    High = 0x01,
    /// Motor is always enabled.
    AlwaysOn = 0x02,
}

/// UART baud rate settings.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BaudRate {
    /// 9600 bps.
    Baud9600 = 0x01,
    /// 19200 bps.
    Baud19200 = 0x02,
    /// 25000 bps.
    Baud25000 = 0x03,
    /// 38400 bps.
    Baud38400 = 0x04,
    /// 57600 bps.
    Baud57600 = 0x05,
    /// 115200 bps.
    Baud115200 = 0x06,
}

/// Return-to-zero mode settings.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ZeroMode {
    /// Return to zero disabled.
    Disable = 0x00,
    /// Direction-based return to zero.
    DirMode = 0x01,
    /// Near-point-based return to zero.
    NearMode = 0x02,
}

/// Save/Clear status operation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SaveClearStatus {
    /// Save the current status.
    Save = 0xC8,
    /// Clear the saved status.
    Clear = 0xCA,
}

/// Motor shaft status.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ShaftStatus {
    /// Motor is blocked (resistance detected).
    Blocked = 0x01,
    /// Motor is unblocked (running freely).
    Unblocked = 0x02,
    /// Error reading status.
    Error = 0x00,
}

/// Rotation direction configuration.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum RotationDirection {
    /// Clockwise rotation (CW).
    Clockwise = 0x00,
    /// Counter-clockwise rotation (CCW).
    CounterClockwise = 0x01,
}
