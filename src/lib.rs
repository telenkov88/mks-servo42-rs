//! A generic, `no_std` Rust driver for **MKS SERVO42** closed-loop stepper motors.
//!
//! This library provides a type-safe interface for generating the serial protocol commands
//! used by the MKS SERVO42C firmware (V1.0+). It is transport-agnostic, meaning it generates
//! byte buffers that you can send over any serial interface (UART, USB-Serial, etc.).

#![no_std]

pub mod enums;
mod errors;
pub mod helpers;
pub mod response;

pub use enums::{
    BaudRate, EnLogic, MotorType, RotationDirection, SaveClearStatus, ShaftStatus, WorkMode,
    ZeroMode,
};
pub use errors::Error;
pub use helpers::{
    angle_to_steps, encoder_val_to_degrees, parse_en_pin_status_response, parse_encoder_response,
    parse_motor_shaft_angle_error, parse_motor_shaft_angle_response, parse_shaft_status_response,
    parse_success_response, strip_leading_garbage, EnPinStatus, EncoderValue, MotorShaftAngle,
    ShaftErrValue,
};
pub use response::{InvalidResponse, Response};

/// Default hardware address for MKS SERVO42 targets.
pub const DEFAULT_ADDRESS: u8 = 0xE0;
/// Minimum allowed slave address.
pub const MIN_ADDRESS: u8 = 0xE0;
/// Maximum allowed slave address.
pub const MAX_ADDRESS: u8 = 0xE9;

/// Maximum speed value for move commands.
pub const MAX_SPEED: u8 = 0x7F;
/// Maximum index for current limit settings.
pub const MAX_CURRENT_INDEX: u8 = 0x0F;
/// Maximum index for subdivision (microstepping).
pub const MAX_SUBDIVISION_INDEX: u8 = 0x08;
/// Maximum speed index for return-to-zero.
pub const MAX_ZERO_SPEED: u8 = 0x04;

/// Milliamps per unit of current limit index.
pub const CURRENT_STEP_MA: u16 = 200;

/// Maximum torque limit (0x4B0).
pub const MAX_TORQUE_LIMIT: u16 = 0x4B0;

const CMD_BUFFER_SIZE: usize = 10;

mod cmd {
    pub const READ_ENCODER_VALUE: u8 = 0x30;
    pub const READ_PULSE_COUNT: u8 = 0x33;
    pub const READ_MOTOR_SHAFT_ANGLE: u8 = 0x36;
    pub const READ_MOTOR_SHAFT_ANGLE_ERROR: u8 = 0x39;
    pub const READ_EN_PIN_STATUS: u8 = 0x3A;
    pub const READ_RELEASE_STATUS: u8 = 0x3D;
    pub const READ_SHAFT_STATUS: u8 = 0x3E;
    pub const SAVE_CLEAR_STATUS: u8 = 0xFF;

    pub const CALIBRATE_ENCODER: u8 = 0x80;
    pub const SET_CURRENT_LIMIT: u8 = 0x83;
    pub const SET_SUBDIVISION: u8 = 0x84;
    pub const SET_EN_LOGIC: u8 = 0x85;
    pub const SET_DIRECTION: u8 = 0x86;
    pub const SET_AUTO_SCREEN_OFF: u8 = 0x87;
    pub const SET_PROTECTION: u8 = 0x88;
    pub const SET_INTERPOLATION: u8 = 0x89;

    pub const SET_ZERO_MODE: u8 = 0x90;
    pub const SET_CURRENT_AS_ZERO: u8 = 0x91;
    pub const SET_ZERO_SPEED: u8 = 0x92;
    pub const SET_ZERO_DIRECTION: u8 = 0x93;
    pub const GO_TO_ZERO: u8 = 0x94;

    pub const SET_POSITION_KP: u8 = 0xA1;
    pub const SET_POSITION_KI: u8 = 0xA2;
    pub const SET_POSITION_KD: u8 = 0xA3;
    pub const SET_ACCELERATION: u8 = 0xA4;
    pub const SET_MAX_TORQUE: u8 = 0xA5;

    pub const ENABLE_MOTOR: u8 = 0xF3;
    pub const RUN_WITH_CONSTANT_SPEED: u8 = 0xF6;
    pub const STOP: u8 = 0xF7;
    pub const RUN_MOTOR: u8 = 0xFD;
}

/// Main driver for communicating with an MKS SERVO42 motor.
///
/// This struct manages the slave address and an internal buffer used to
/// construct serial commands.
#[derive(Debug, Copy, Clone)]
pub struct Driver {
    address: u8,
    buffer: [u8; CMD_BUFFER_SIZE],
}

type Result<T> = core::result::Result<T, Error>;

impl Default for Driver {
    /// Creates a new driver with the default address (0xE0).
    fn default() -> Self {
        Self {
            address: DEFAULT_ADDRESS,
            buffer: [0; CMD_BUFFER_SIZE],
        }
    }
}

impl Driver {
    /// Creates a new driver instance with a specific target address.
    #[must_use]
    pub fn with_address(address: u8) -> Self {
        Self {
            address,
            ..Default::default()
        }
    }

    /// Generates a command to enable or disable the motor.
    pub fn enable_motor(&mut self, enable: bool) -> &[u8] {
        self.build_command(&[self.address, cmd::ENABLE_MOTOR, u8::from(enable)])
    }

    /// Generates a command to run the motor at a constant speed.
    ///
    /// # Errors
    /// Returns `Error::InvalidValue` if speed exceeds `MAX_SPEED`.
    pub fn run_with_constant_speed(
        &mut self,
        direction: RotationDirection,
        speed: u8,
    ) -> Result<&[u8]> {
        if speed > MAX_SPEED {
            return Err(Error::InvalidValue);
        }
        let dir_mask = match direction {
            RotationDirection::Clockwise => 0x00,
            RotationDirection::CounterClockwise => 0x80,
        };
        Ok(self.build_command(&[self.address, cmd::RUN_WITH_CONSTANT_SPEED, speed | dir_mask]))
    }

    /// Generates a command to stop the motor immediately.
    pub fn stop(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::STOP])
    }

    /// Generates a command to save or clear the current status.
    ///
    /// This command is used to save or clear the status set by the `set_work_mode` command.
    /// After saving successfully, the driver board will be disabled and needs to be re-enabled.
    pub fn save_clear_status(&mut self, operation: SaveClearStatus) -> &[u8] {
        self.build_command(&[self.address, cmd::SAVE_CLEAR_STATUS, operation as u8])
    }

    /// Generates a command to move the motor to a specific position (relative pulses).
    ///
    /// # Errors
    /// Returns `Error::InvalidValue` if speed exceeds `MAX_SPEED`.
    pub fn run_motor(
        &mut self,
        direction: RotationDirection,
        speed: u8,
        pulses: u32,
    ) -> Result<&[u8]> {
        if speed > MAX_SPEED {
            return Err(Error::InvalidValue);
        }
        let dir_mask = match direction {
            RotationDirection::Clockwise => 0x00,
            RotationDirection::CounterClockwise => 0x80,
        };
        let pulse_bytes = pulses.to_be_bytes();
        Ok(self.build_command(&[
            self.address,
            cmd::RUN_MOTOR,
            speed | dir_mask,
            pulse_bytes[0],
            pulse_bytes[1],
            pulse_bytes[2],
            pulse_bytes[3],
        ]))
    }

    /// Generates a command to trigger encoder calibration.
    pub fn calibrate_encoder(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::CALIBRATE_ENCODER, 0x00])
    }

    /// Generates a command to set the current limit index.
    ///
    /// # Errors
    /// Returns `Error::InvalidValue` if index exceeds `MAX_CURRENT_INDEX`.
    pub fn set_current_limit(&mut self, index: u8) -> Result<&[u8]> {
        if index > MAX_CURRENT_INDEX {
            return Err(Error::InvalidValue);
        }
        Ok(self.build_command(&[self.address, cmd::SET_CURRENT_LIMIT, index]))
    }

    /// Generates a command to set the subdivision (microstepping) level.
    ///
    /// # Errors
    /// Returns `Error::InvalidValue` if index exceeds `MAX_SUBDIVISION_INDEX`.
    pub fn set_subdivision(&mut self, step_index: u8) -> Result<&[u8]> {
        if step_index > MAX_SUBDIVISION_INDEX {
            return Err(Error::InvalidValue);
        }
        Ok(self.build_command(&[self.address, cmd::SET_SUBDIVISION, step_index]))
    }

    /// Generates a command to set the enable logic.
    pub fn set_enable_logic(&mut self, logic: EnLogic) -> &[u8] {
        self.build_command(&[self.address, cmd::SET_EN_LOGIC, logic as u8])
    }

    /// Generates a command to set the motor direction polarity.
    pub fn set_direction(&mut self, direction: RotationDirection) -> &[u8] {
        self.build_command(&[self.address, cmd::SET_DIRECTION, direction as u8])
    }

    /// Generates a command to enable or disable automatic screen off.
    pub fn set_auto_screen_off(&mut self, enable: bool) -> &[u8] {
        self.build_command(&[self.address, cmd::SET_AUTO_SCREEN_OFF, u8::from(!enable)])
    }

    /// Generates a command to enable or disable stall protection.
    pub fn set_stall_protection(&mut self, enable: bool) -> &[u8] {
        self.build_command(&[self.address, cmd::SET_PROTECTION, u8::from(!enable)])
    }

    /// Generates a command to enable or disable step interpolation.
    pub fn set_interpolation(&mut self, enable: bool) -> &[u8] {
        self.build_command(&[self.address, cmd::SET_INTERPOLATION, u8::from(!enable)])
    }

    /// Generates a command to set the return-to-zero mode.
    pub fn set_zero_mode(&mut self, mode: ZeroMode) -> &[u8] {
        self.build_command(&[self.address, cmd::SET_ZERO_MODE, mode as u8])
    }

    /// Generates a command to set the current position as zero.
    pub fn set_current_as_zero(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::SET_CURRENT_AS_ZERO, 0x00])
    }

    /// Generates a command to set the return-to-zero speed.
    ///
    /// # Errors
    /// Returns `Error::InvalidValue` if speed index exceeds `MAX_ZERO_SPEED`.
    pub fn set_zero_speed(&mut self, speed: u8) -> Result<&[u8]> {
        if speed > MAX_ZERO_SPEED {
            return Err(Error::InvalidValue);
        }
        Ok(self.build_command(&[self.address, cmd::SET_ZERO_SPEED, speed]))
    }

    /// Generates a command to initiate return-to-zero sequence.
    pub fn go_to_zero(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::GO_TO_ZERO, 0x00])
    }

    /// Generates a command to set the return-to-zero direction.
    pub fn set_zero_direction(&mut self, direction: RotationDirection) -> &[u8] {
        self.build_command(&[self.address, cmd::SET_ZERO_DIRECTION, direction as u8])
    }

    /// Generates a command to set the position loop Proportional (Kp) coefficient.
    pub fn set_position_kp(&mut self, value: u16) -> &[u8] {
        let bytes = value.to_be_bytes();
        self.build_command(&[self.address, cmd::SET_POSITION_KP, bytes[0], bytes[1]])
    }

    /// Generates a command to set the position loop Integral (Ki) coefficient.
    pub fn set_position_ki(&mut self, value: u16) -> &[u8] {
        let bytes = value.to_be_bytes();
        self.build_command(&[self.address, cmd::SET_POSITION_KI, bytes[0], bytes[1]])
    }

    /// Generates a command to set the position loop Derivative (Kd) coefficient.
    pub fn set_position_kd(&mut self, value: u16) -> &[u8] {
        let bytes = value.to_be_bytes();
        self.build_command(&[self.address, cmd::SET_POSITION_KD, bytes[0], bytes[1]])
    }

    /// Generates a command to set the motor acceleration.
    pub fn set_acceleration(&mut self, value: u16) -> &[u8] {
        let bytes = value.to_be_bytes();
        self.build_command(&[self.address, cmd::SET_ACCELERATION, bytes[0], bytes[1]])
    }

    /// Generates a command to set the maximum torque limit.
    ///
    /// # Errors
    /// Returns `Error::InvalidValue` if value exceeds `MAX_TORQUE_LIMIT`.
    pub fn set_max_torque(&mut self, value: u16) -> Result<&[u8]> {
        if value > MAX_TORQUE_LIMIT {
            return Err(Error::InvalidValue);
        }
        let bytes = value.to_be_bytes();
        Ok(self.build_command(&[self.address, cmd::SET_MAX_TORQUE, bytes[0], bytes[1]]))
    }

    /// Generates a command to read the motor shaft status (Blocked/Unblocked/Error).
    pub fn read_shaft_status(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::READ_SHAFT_STATUS])
    }

    /// Generates a command to read the current encoder value.
    pub fn read_encoder_value(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::READ_ENCODER_VALUE])
    }

    /// Generates a command to read the total pulse count.
    pub fn read_pulse_count(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::READ_PULSE_COUNT])
    }

    /// Generates a command to read the motor shaft angle.
    ///
    /// Returns a 4-byte signed integer representing the angle in encoder units.
    /// One full rotation corresponds to 0-65535.
    pub fn read_motor_shaft_angle(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::READ_MOTOR_SHAFT_ANGLE])
    }

    /// Generates a command to read the EN pin status.
    ///
    /// Returns:
    /// - 0x01: Enable
    /// - 0x02: Disable  
    /// - 0x00: Error
    pub fn read_en_pin_status(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::READ_EN_PIN_STATUS])
    }

    /// Generates a command to read the motor shaft angle error.
    pub fn read_motor_shaft_angle_error(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::READ_MOTOR_SHAFT_ANGLE_ERROR])
    }

    /// Generates a command to read the release status of the motor.
    pub fn read_release_status(&mut self) -> &[u8] {
        self.build_command(&[self.address, cmd::READ_RELEASE_STATUS])
    }

    fn build_command(&mut self, cmd: &[u8]) -> &[u8] {
        let len = cmd.len();
        self.buffer[..len].copy_from_slice(cmd);
        self.buffer[len] = calculate_checksum(cmd);
        &self.buffer[..=len]
    }
}

fn calculate_checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_checksum() {
        assert_eq!(0xD7, calculate_checksum(&[0xE0, 0xF6, 0x01]));
    }

    #[test]
    fn test_default_address() {
        let driver = Driver::default();
        assert_eq!(driver.address, DEFAULT_ADDRESS);
    }

    #[test]
    fn test_with_address() {
        let driver = Driver::with_address(0xE5);
        assert_eq!(driver.address, 0xE5);

        // Test edge addresses
        let driver_min = Driver::with_address(MIN_ADDRESS);
        assert_eq!(driver_min.address, MIN_ADDRESS);

        let driver_max = Driver::with_address(MAX_ADDRESS);
        assert_eq!(driver_max.address, MAX_ADDRESS);
    }

    #[test]
    fn test_set_subdivision_invalid_value() {
        let mut driver = Driver::default();
        // MAX_SUBDIVISION_INDEX is 0x08, so 0x09 should fail
        let result = driver.set_subdivision(MAX_SUBDIVISION_INDEX + 1);
        assert!(matches!(result, Err(Error::InvalidValue)));

        // Valid value should succeed
        let result = driver.set_subdivision(MAX_SUBDIVISION_INDEX);
        assert!(result.is_ok());
    }

    #[test]
    fn test_set_current_limit_invalid_value() {
        let mut driver = Driver::default();
        // MAX_CURRENT_INDEX is 0x0F, so 0x10 should fail
        let result = driver.set_current_limit(MAX_CURRENT_INDEX + 1);
        assert!(matches!(result, Err(Error::InvalidValue)));

        // Valid value should succeed
        let result = driver.set_current_limit(MAX_CURRENT_INDEX);
        assert!(result.is_ok());
    }

    #[test]
    fn test_run_motor_invalid_speed() {
        let mut driver = Driver::default();
        // MAX_SPEED is 0x7F, so 0x80 should fail
        let result = driver.run_motor(RotationDirection::Clockwise, MAX_SPEED + 1, 100);
        assert!(matches!(result, Err(Error::InvalidValue)));

        // Valid speed should succeed
        let result = driver.run_motor(RotationDirection::Clockwise, MAX_SPEED, 100);
        assert!(result.is_ok());
    }

    #[test]
    fn test_run_with_constant_speed_invalid_speed() {
        let mut driver = Driver::default();
        // MAX_SPEED is 0x7F, so 0x80 should fail
        let result = driver.run_with_constant_speed(RotationDirection::Clockwise, MAX_SPEED + 1);
        assert!(matches!(result, Err(Error::InvalidValue)));

        // Valid speed should succeed
        let result = driver.run_with_constant_speed(RotationDirection::Clockwise, MAX_SPEED);
        assert!(result.is_ok());
    }

    #[test]
    fn test_calibrate_encoder() {
        // This command is too slow (40-60s) and dangerous to test on real hardware
        let mut driver = Driver::default();
        let cmd = driver.calibrate_encoder();

        // Verify command format: [address, cmd::CALIBRATE_ENCODER, 0x00, checksum]
        assert_eq!(cmd.len(), 4);
        assert_eq!(cmd[0], DEFAULT_ADDRESS);
        assert_eq!(cmd[1], 0x80); // cmd::CALIBRATE_ENCODER
        assert_eq!(cmd[2], 0x00);
        // Checksum: 0xE0 + 0x80 + 0x00 = 0x160 -> low byte 0x60
        assert_eq!(cmd[3], 0x60);
    }
}
