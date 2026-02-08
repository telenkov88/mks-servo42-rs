use crate::Error;

/// Standard steps per revolution for a 1.8° motor.
pub const STEPS_PER_REV: f32 = 200.0;
/// Total resolution of the 16-bit encoder.
pub const ENCODER_RESOLUTION: f32 = 65536.0;

/// Represents an absolute encoder value including multi-turn carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderValue {
    /// Number of full rotations (positive or negative).
    pub carry: i32,
    /// 16-bit absolute position within the current turn.
    pub value: u16,
}

impl EncoderValue {
    /// Converts the full multi-turn encoder value to total degrees.
    #[must_use]
    pub fn to_degrees(self) -> f32 {
        let degrees = (f32::from(self.value) / ENCODER_RESOLUTION) * 360.0;
        (self.carry as f32 * 360.0) + degrees
    }
}

/// Utility to calculate required pulses for a given angle and microstepping level.
#[must_use]
pub fn angle_to_steps(angle: f32, microsteps: f32) -> u32 {
    let steps = (angle / 360.0) * STEPS_PER_REV * microsteps;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    {
        (steps + 0.5) as u32
    }
}

/// Converts a 16-bit encoder value to degrees (0-360).
#[must_use]
pub fn encoder_val_to_degrees(val: u16) -> f32 {
    (f32::from(val) / ENCODER_RESOLUTION) * 360.0
}

/// Parses raw serial feedback into an `EncoderValue`.
///
/// This function scans the provided buffer for a valid packet matching the
/// MKS SERVO42 protocol.
pub fn parse_encoder_response(data: &[u8]) -> Result<EncoderValue, Error> {
    let mut idx = 0;
    while idx < data.len() {
        if data[idx] >= crate::MIN_ADDRESS
            && data[idx] <= crate::MAX_ADDRESS
            && idx + 5 < data.len()
        {
            let sum: u32 = data[idx..idx + 7].iter().map(|&b| u32::from(b)).sum();
            if (sum as u8) == data[idx + 7] {
                let carry_bytes = &data[idx + 1..idx + 5];
                let carry = i32::from_be_bytes([
                    carry_bytes[0],
                    carry_bytes[1],
                    carry_bytes[2],
                    carry_bytes[3],
                ]);

                let val_bytes = &data[idx + 5..idx + 7];
                let value = u16::from_be_bytes([val_bytes[0], val_bytes[1]]);

                return Ok(EncoderValue { carry, value });
            }
        }
        idx += 1;
    }

    Err(Error::InvalidPacket)
}

/// Represents an encoder shaft error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShaftErrValue {
    /// 16-bit shaft error in encoder units.
    pub value: i16,
}

impl ShaftErrValue {
    /// Converts the full multi-turn encoder value to total degrees.
    #[must_use]
    pub fn to_degrees(self) -> f32 {
        f32::from(self.value) / 360.0
    }
}

/// Parses the motor shaft angle error response.
///
/// This function parses responses from the `READ_MOTOR_SHAFT_ANGLE_ERROR` command (0x39).
/// The response format is: `[slave_address, error_low_byte, error_high_byte, crc, trailing 0x00]`
/// where the error is a signed 16-bit integer representing the angle error in encoder units.
/// return value in error in encoder units
///
/// According to the MKS SERVO42 protocol:
/// - 0x0000-0xFFFF corresponds to 0-360°
/// - 1° error ≈ 182 encoder units (65536/360)
pub fn parse_motor_shaft_angle_error(data: &[u8]) -> Result<ShaftErrValue, Error> {
    let mut idx = 0;
    while idx < data.len() {
        if data[idx] >= crate::MIN_ADDRESS
            && data[idx] <= crate::MAX_ADDRESS
            && idx + 4 < data.len()
        {
            // Check for the trailing 0x00 byte (undocumented unexpected byte)
            if data[idx + 4] != 0x00 {
                idx += 1;
                continue;
            }

            let sum: u32 = data[idx..idx + 3].iter().map(|&b| u32::from(b)).sum();
            if (sum as u8) != data[idx + 3] {
                idx += 1;
                continue;
            }

            let error_bytes = &data[idx + 1..idx + 3];
            let value = i16::from_be_bytes([error_bytes[0], error_bytes[1]]);
            return Ok(ShaftErrValue { value });
        }
        idx += 1;
    }

    Err(Error::InvalidPacket)
}

/// Represents a motor shaft angle value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorShaftAngle {
    /// 32-bit signed angle in encoder units.
    pub value: i32,
}

impl MotorShaftAngle {
    /// Converts the angle to degrees.
    #[must_use]
    pub fn to_degrees(self) -> f32 {
        (self.value as f32 / 65536.0) * 360.0
    }
}

/// Parses the motor shaft angle response.
///
/// This function parses responses from the `READ_MOTOR_SHAFT_ANGLE` command (0x36).
/// The response format is: `[slave_address, angle_byte1, angle_byte2, angle_byte3, angle_byte4, crc]`
/// where the angle is a signed 32-bit integer representing the angle in encoder units.
///
/// According to the MKS SERVO42 protocol:
/// - One full rotation (360°) corresponds to 0-65535 encoder units
/// - Example: 90° = 16384 encoder units (0x4000)
pub fn parse_motor_shaft_angle_response(data: &[u8]) -> Result<MotorShaftAngle, Error> {
    let mut idx = 0;
    while idx < data.len() {
        if data[idx] >= 0xE0 && data[idx] <= 0xE9 && idx + 5 < data.len() {
            let sum: u32 = data[idx..idx + 5].iter().map(|&b| u32::from(b)).sum();
            if (sum as u8) == data[idx + 5] {
                let angle_bytes = &data[idx + 1..idx + 5];
                let value = i32::from_be_bytes([
                    angle_bytes[0],
                    angle_bytes[1],
                    angle_bytes[2],
                    angle_bytes[3],
                ]);
                return Ok(MotorShaftAngle { value });
            }
        }
        idx += 1;
    }

    Err(Error::InvalidPacket)
}

/// Represents EN pin status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnPinStatus {
    /// Motor is enabled.
    Enabled = 0x01,
    /// Motor is disabled.
    Disabled = 0x02,
    /// Error state.
    Error = 0x00,
}

/// Parses the EN pin status response.
///
/// This function parses responses from the `READ_EN_PIN_STATUS` command (0x3A).
/// The response format is: `[slave_address, status_byte, crc]`
/// where status is:
/// - 0x01: Enable
/// - 0x02: Disable
/// - 0x00: Error
pub fn parse_en_pin_status_response(data: &[u8]) -> Result<EnPinStatus, Error> {
    let mut idx = 0;
    while idx < data.len() {
        if data[idx] >= crate::MIN_ADDRESS
            && data[idx] <= crate::MAX_ADDRESS
            && idx + 2 < data.len()
        {
            let sum: u32 = data[idx..idx + 2].iter().map(|&b| u32::from(b)).sum();
            if (sum as u8) == data[idx + 2] {
                let status_byte = data[idx + 1];
                return match status_byte {
                    0x01 => Ok(EnPinStatus::Enabled),
                    0x02 => Ok(EnPinStatus::Disabled),
                    0x00 => Ok(EnPinStatus::Error),
                    _ => Err(Error::InvalidPacket),
                };
            }
        }
        idx += 1;
    }

    Err(Error::InvalidPacket)
}

/// Parses the motor shaft status response.
///
/// This function parses responses from the `READ_SHAFT_STATUS` command (0x3E).
/// The response format is: `[slave_address, status_byte, crc]`
/// where status is:
/// - 0x01: Blocked
/// - 0x02: Unblocked
/// - 0x00: Error
pub fn parse_shaft_status_response(data: &[u8]) -> Result<crate::enums::ShaftStatus, Error> {
    if data.len() < 3 {
        return Err(Error::InvalidPacket);
    }
    for window in data.windows(3) {
        let addr = window[0];
        if !(crate::MIN_ADDRESS..=crate::MAX_ADDRESS).contains(&addr) {
            continue;
        }
        let status_byte = window[1];
        let checksum = window[2];
        let expected_checksum = addr.wrapping_add(status_byte);
        if checksum != expected_checksum {
            continue;
        }
        return match status_byte {
            0x01 => Ok(crate::enums::ShaftStatus::Blocked),
            0x02 => Ok(crate::enums::ShaftStatus::Unblocked),
            0x00 => Ok(crate::enums::ShaftStatus::Error),
            _ => Err(Error::InvalidPacket),
        };
    }
    Err(Error::InvalidPacket)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_angle_to_steps() {
        assert_eq!(angle_to_steps(360.0, 1.0), 200);
        assert_eq!(angle_to_steps(360.0, 4.0), 800);
        assert_eq!(angle_to_steps(180.0, 4.0), 400);
    }

    #[test]
    fn test_encoder_val_to_degrees() {
        assert_eq!(encoder_val_to_degrees(0), 0.0);
        assert_eq!(encoder_val_to_degrees(32768), 180.0);
        assert_eq!(encoder_val_to_degrees(65535), (65535.0 / 65536.0) * 360.0);
    }

    #[test]
    fn test_encoder_value_to_degrees() {
        let ev = EncoderValue { carry: 1, value: 0 }; // 1 full rotation
        assert_eq!(ev.to_degrees(), 360.0);

        let ev = EncoderValue {
            carry: -1,
            value: 0,
        }; // -1 full rotation
        assert_eq!(ev.to_degrees(), -360.0);

        let ev = EncoderValue {
            carry: 0,
            value: 32768,
        }; // 180 degrees
        assert_eq!(ev.to_degrees(), 180.0);
    }

    #[test]
    fn test_parse_encoder_response() {
        let data = [0xE0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x20];
        let res = parse_encoder_response(&data).unwrap();
        assert_eq!(res.carry, 0);
        assert_eq!(res.value, 0x4000);
        assert_eq!(res.to_degrees(), 90.0);
    }

    #[test]
    fn test_parse_encoder_response_with_prefix() {
        let data = [0xFF, 0xFE, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x20];
        let res = parse_encoder_response(&data).unwrap();
        assert_eq!(res.carry, 0);
        assert_eq!(res.value, 0x4000);
    }

    #[test]
    fn test_parse_encoder_response_invalid_checksum() {
        let data = [0xE0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x21];
        let res = parse_encoder_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_motor_shaft_angle_error() {
        // Example from documentation: e0 00 B7 97 00 (error 1°)
        let data = [0xE0, 0x00, 0xB7, 0x97, 0x00];
        let error = parse_motor_shaft_angle_error(&data).unwrap();
        let shaft_error = ShaftErrValue { value: 183 };
        assert_eq!(error, shaft_error); // 183 encoder units ≈ 1°

        // Test with negative error (two's complement)
        // Example: e0 FF 4A 29 00 (error -182 ≈ -1°)
        // Checksum: 0xE0 + 0xFF + 0x4A = 0x229 → low byte 0x29
        let data = [0xE0, 0xFF, 0x4A, 0x29, 0x00];
        let error = parse_motor_shaft_angle_error(&data).unwrap();
        let shaft_error = ShaftErrValue { value: -182 };
        assert_eq!(error, shaft_error); // 0xFF4A = -182 encoder units ≈ -1°

        // Test zero error
        // Checksum: 0xE0 + 0x00 + 0x00 = 0xE0
        let data = [0xE0, 0x00, 0x00, 0xE0, 0x00];
        let error = parse_motor_shaft_angle_error(&data).unwrap();
        let shaft_error = ShaftErrValue { value: 0 };
        assert_eq!(error, shaft_error);

        // Test maximum positive error (0x7FFF = 32767)
        // Checksum: 0xE0 + 0x7F + 0xFF = 0x25E → low byte 0x5E
        let data = [0xE0, 0x7F, 0xFF, 0x5E, 0x00];
        let error = parse_motor_shaft_angle_error(&data).unwrap();
        let shaft_error = ShaftErrValue { value: 32767 };
        assert_eq!(error, shaft_error);
    }

    #[test]
    fn test_parse_motor_shaft_angle_error_with_prefix() {
        // Test with garbage bytes before valid packet
        let data = [0xFF, 0xFE, 0xE0, 0x00, 0xB7, 0x97, 0x00];
        let error = parse_motor_shaft_angle_error(&data).unwrap();
        let shaft_error = ShaftErrValue { value: 183 };
        assert_eq!(error, shaft_error);
    }

    #[test]
    fn test_parse_motor_shaft_angle_error_invalid_checksum() {
        // Wrong checksum
        let data = [0xE0, 0x00, 0xB7, 0x98, 0x00];
        let res = parse_motor_shaft_angle_error(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_motor_shaft_angle_error_missing_trailing_zero() {
        // Missing trailing 0x00 byte
        let data = [0xE0, 0x00, 0xB7, 0x97, 0x01];
        let res = parse_motor_shaft_angle_error(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_motor_shaft_angle_error_too_short() {
        // Packet too short
        let data = [0xE0, 0x00, 0xB7, 0x97];
        let res = parse_motor_shaft_angle_error(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_motor_shaft_angle_error_invalid_address() {
        // Invalid address (outside E0-E9 range)
        let data = [0xDF, 0x00, 0xB7, 0x97, 0x00];
        let res = parse_motor_shaft_angle_error(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_motor_shaft_angle_response() {
        // Example from documentation: e0 00 00 40 00 20 (angle 90°)
        let data = [0xE0, 0x00, 0x00, 0x40, 0x00, 0x20];
        let angle = parse_motor_shaft_angle_response(&data).unwrap();
        assert_eq!(angle.value, 0x4000); // 16384 encoder units
        assert_eq!(angle.to_degrees(), 90.0);

        // Test zero angle
        // Checksum: 0xE0 + 0x00 + 0x00 + 0x00 + 0x00 = 0xE0
        let data = [0xE0, 0x00, 0x00, 0x00, 0x00, 0xE0];
        let angle = parse_motor_shaft_angle_response(&data).unwrap();
        assert_eq!(angle.value, 0);
        assert_eq!(angle.to_degrees(), 0.0);

        // Test 180° angle (32768 encoder units = 0x8000)
        // Checksum: 0xE0 + 0x00 + 0x00 + 0x80 + 0x00 = 0x160 → low byte 0x60
        let data = [0xE0, 0x00, 0x00, 0x80, 0x00, 0x60];
        let angle = parse_motor_shaft_angle_response(&data).unwrap();
        assert_eq!(angle.value, 0x8000);
        assert_eq!(angle.to_degrees(), 180.0);

        // Test negative angle (two's complement)
        // -90° = 65536 - 16384 = 49152 = 0xC000
        // Checksum: 0xE0 + 0xFF + 0xFF + 0xC0 + 0x00 = 0x39E → low byte 0x9E
        let data = [0xE0, 0xFF, 0xFF, 0xC0, 0x00, 0x9E];
        let angle = parse_motor_shaft_angle_response(&data).unwrap();
        assert_eq!(angle.value, -16384); // -90° in encoder units
        assert_eq!(angle.to_degrees(), -90.0);
    }

    #[test]
    fn test_parse_motor_shaft_angle_response_with_prefix() {
        // Test with garbage bytes before valid packet
        let data = [0xFF, 0xFE, 0xE0, 0x00, 0x00, 0x40, 0x00, 0x20];
        let angle = parse_motor_shaft_angle_response(&data).unwrap();
        assert_eq!(angle.value, 0x4000);
        assert_eq!(angle.to_degrees(), 90.0);
    }

    #[test]
    fn test_parse_motor_shaft_angle_response_invalid_checksum() {
        // Wrong checksum
        let data = [0xE0, 0x00, 0x00, 0x40, 0x00, 0x21];
        let res = parse_motor_shaft_angle_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_motor_shaft_angle_response_too_short() {
        // Packet too short
        let data = [0xE0, 0x00, 0x00, 0x40, 0x00];
        let res = parse_motor_shaft_angle_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_motor_shaft_angle_response_invalid_address() {
        // Invalid address (outside E0-E9 range)
        let data = [0xDF, 0x00, 0x00, 0x40, 0x00, 0x20];
        let res = parse_motor_shaft_angle_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_en_pin_status_response() {
        // Example from documentation: e0 01 e1 (enable)
        let data = [0xE0, 0x01, 0xE1];
        let status = parse_en_pin_status_response(&data).unwrap();
        assert_eq!(status, EnPinStatus::Enabled);

        // Test disabled status
        // Checksum: 0xE0 + 0x02 = 0xE2
        let data = [0xE0, 0x02, 0xE2];
        let status = parse_en_pin_status_response(&data).unwrap();
        assert_eq!(status, EnPinStatus::Disabled);

        // Test error status
        // Checksum: 0xE0 + 0x00 = 0xE0
        let data = [0xE0, 0x00, 0xE0];
        let status = parse_en_pin_status_response(&data).unwrap();
        assert_eq!(status, EnPinStatus::Error);
    }

    #[test]
    fn test_parse_en_pin_status_response_with_prefix() {
        // Test with garbage bytes before valid packet
        let data = [0xFF, 0xFE, 0xE0, 0x01, 0xE1];
        let status = parse_en_pin_status_response(&data).unwrap();
        assert_eq!(status, EnPinStatus::Enabled);
    }

    #[test]
    fn test_parse_en_pin_status_response_invalid_checksum() {
        // Wrong checksum
        let data = [0xE0, 0x01, 0xE2];
        let res = parse_en_pin_status_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_en_pin_status_response_invalid_status() {
        // Invalid status byte
        let data = [0xE0, 0x03, 0xE3];
        let res = parse_en_pin_status_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_en_pin_status_response_too_short() {
        // Packet too short
        let data = [0xE0, 0x01];
        let res = parse_en_pin_status_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_en_pin_status_response_invalid_address() {
        // Invalid address (outside E0-E9 range)
        let data = [0xDF, 0x01, 0xE0];
        let res = parse_en_pin_status_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_shaft_status_response() {
        // Test Blocked
        // Checksum: 0xE0 + 0x01 = 0xE1
        let data = [0xE0, 0x01, 0xE1];
        let status = parse_shaft_status_response(&data).unwrap();
        assert_eq!(status, crate::enums::ShaftStatus::Blocked);

        // Test Unblocked
        // Checksum: 0xE0 + 0x02 = 0xE2
        let data = [0xE0, 0x02, 0xE2];
        let status = parse_shaft_status_response(&data).unwrap();
        assert_eq!(status, crate::enums::ShaftStatus::Unblocked);

        // Test Error
        // Checksum: 0xE0 + 0x00 = 0xE0
        let data = [0xE0, 0x00, 0xE0];
        let status = parse_shaft_status_response(&data).unwrap();
        assert_eq!(status, crate::enums::ShaftStatus::Error);
    }

    #[test]
    fn test_parse_shaft_status_response_too_short() {
        // Packet too short (less than 3 bytes)
        let data = [0xE0, 0x01];
        let res = parse_shaft_status_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));

        // Empty packet
        let data: [u8; 0] = [];
        let res = parse_shaft_status_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_shaft_status_response_invalid_checksum() {
        // Wrong checksum
        let data = [0xE0, 0x01, 0xE2];
        let res = parse_shaft_status_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_shaft_status_response_invalid_address() {
        // Invalid address (outside E0-E9 range)
        let data = [0xDF, 0x01, 0xE0];
        let res = parse_shaft_status_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_shaft_status_response_invalid_status() {
        // Invalid status byte (0x03 is not valid)
        // Checksum: 0xE0 + 0x03 = 0xE3
        let data = [0xE0, 0x03, 0xE3];
        let res = parse_shaft_status_response(&data);
        assert!(matches!(res, Err(Error::InvalidPacket)));
    }

    #[test]
    fn test_parse_shaft_status_response_with_prefix() {
        // Test with garbage bytes before valid packet
        let data = [0xFF, 0xFE, 0xE0, 0x01, 0xE1];
        let status = parse_shaft_status_response(&data).unwrap();
        assert_eq!(status, crate::enums::ShaftStatus::Blocked);
    }
}
