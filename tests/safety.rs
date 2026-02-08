//! Safety framework for MKS SERVO42 E2E tests
//!
//! This module provides safety controls and constants for testing hardware
//! without risking damage or connection loss.

/// Maximum safe speed for movement tests (gear 1 = minimal speed)
pub const MAX_SAFE_SPEED: u8 = 1;

/// Maximum safe angle for position movement tests (10 degrees)
pub const MAX_SAFE_ANGLE_DEGREES: f32 = 10.0;

/// Safe microstepping level for tests
pub const SAFE_MICROSTEPS: f32 = 4.0;

/// Commands that should NEVER be tested on real hardware
/// These commands can cause connection loss or irreversible changes
pub const DANGEROUS_COMMANDS: &[(&str, &str)] = &[
    (
        "set_work_mode",
        "Changing control mode can break UART communication",
    ),
    ("set_baud_rate", "Changing baud rate will lose connection"),
    (
        "set_slave_address",
        "Could make driver unresponsive if address is lost",
    ),
];

/// Check if a command should be skipped during testing
pub fn should_skip_command(command_name: &str) -> bool {
    DANGEROUS_COMMANDS
        .iter()
        .any(|(name, _)| *name == command_name)
}

/// Validate speed parameter is within safe limits
pub fn validate_safe_speed(speed: u8) -> Result<(), String> {
    if speed > MAX_SAFE_SPEED {
        Err(format!(
            "Speed {} exceeds safe limit of {}",
            speed, MAX_SAFE_SPEED
        ))
    } else {
        Ok(())
    }
}

/// Validate angle parameter is within safe limits
pub fn validate_safe_angle(angle_degrees: f32) -> Result<(), String> {
    if angle_degrees.abs() > MAX_SAFE_ANGLE_DEGREES {
        Err(format!(
            "Angle {} degrees exceeds safe limit of {} degrees",
            angle_degrees, MAX_SAFE_ANGLE_DEGREES
        ))
    } else {
        Ok(())
     }
}
