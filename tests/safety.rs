//! Safety framework for MKS SERVO42 E2E tests
//!
//! This module provides safety controls and constants for testing hardware
//! without risking damage or connection loss.

use mks_servo42_rs::{BaudRate, WorkMode};

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

/// Commands that should be tested with extreme caution
/// These commands could affect motor performance or require special conditions
pub const CAUTION_COMMANDS: &[(&str, &str)] = &[
    ("calibrate_encoder", "Requires motor to be unloaded"),
    ("set_current_limit", "Could affect motor performance"),
    ("set_position_kp", "PID changes could cause vibration"),
    ("set_position_ki", "PID changes could cause vibration"),
    ("set_position_kd", "PID changes could cause vibration"),
    (
        "set_acceleration",
        "ACC parameter changes could damage driver if too large",
    ),
    ("set_max_torque", "Torque limit changes"),
    (
        "restore_defaults",
        "Restores all defaults including baud rate!",
    ),
];

/// Check if a command should be skipped during testing
pub fn should_skip_command(command_name: &str) -> bool {
    DANGEROUS_COMMANDS
        .iter()
        .any(|(name, _)| *name == command_name)
}

/// Get warning message for a caution command
pub fn get_caution_warning(command_name: &str) -> Option<&'static str> {
    CAUTION_COMMANDS
        .iter()
        .find(|(name, _)| *name == command_name)
        .map(|(_, warning)| *warning)
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
            "Angle {}° exceeds safe limit of {}°",
            angle_degrees, MAX_SAFE_ANGLE_DEGREES
        ))
    } else {
        Ok(())
    }
}

/// Check if work mode is safe for testing
pub fn is_safe_work_mode(mode: WorkMode) -> bool {
    // Only UART mode should be tested (assuming we're already in UART mode)
    // Changing from UART to other modes could break communication
    matches!(mode, WorkMode::Uart)
}

/// Check if baud rate is safe for testing  
pub fn is_safe_baud_rate(rate: BaudRate) -> bool {
    // Only test with current baud rate (38400 based on examples/base.rs)
    // Changing baud rate will lose connection
    matches!(rate, BaudRate::Baud38400)
}
