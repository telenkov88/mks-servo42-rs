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
            && idx + 7 < data.len()
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
}
