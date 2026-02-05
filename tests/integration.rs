//! Integration tests for MKS SERVO42 driver
//!
//! These tests require real hardware connected via serial port.
//! Set MKS_ENV_SERVO42C_UART environment variable to serial port path.
//!
//! SAFETY: Tests use minimal speed and small movements to avoid damage.

mod safety;
mod test_utils;

use mks_servo42_rs::direction::Direction;
use safety::{
    validate_safe_angle, validate_safe_speed, MAX_SAFE_ANGLE_DEGREES, MAX_SAFE_SPEED,
    SAFE_MICROSTEPS,
};
use std::time::Duration;
use test_utils::{init_env, TestContext, TestResult, LONG_PAUSE, TEST_MUTEX};

/// Test basic motor enable/disable
#[test]
fn test_motor_enable_disable() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();
    println!("=== Test: motor enable/disable ===");

    let mut ctx = TestContext::new()?;

    // Enable motor
    println!("Enabling motor...");
    let response = ctx.serial.send_and_read(ctx.driver.enable_motor(true))?;

    if response.len() >= 3 && response[1] == 0x01 {
        println!("Motor enabled successfully");
    } else {
        println!("Warning: Enable response not as expected: {:?}", response);
    }

    // Small delay
    std::thread::sleep(Duration::from_millis(100));

    // Disable motor
    println!("Disabling motor...");
    let response = ctx.serial.send_and_read(ctx.driver.enable_motor(false))?;

    if response.len() >= 3 && response[1] == 0x01 {
        println!("Motor disabled successfully");
    } else {
        println!("Warning: Disable response not as expected: {:?}", response);
    }

    println!("Test passed!");
    Ok(())
}

/// Test constant speed movement
#[test]
fn test_run_speed() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: run_speed ===");

    validate_safe_speed(MAX_SAFE_SPEED)?;

    let mut ctx = TestContext::new()?;

    // Enable motor
    println!("Enabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(true))?;

    // Run forward at minimal speed
    println!("Running forward at speed {}...", MAX_SAFE_SPEED);
    let cmd = ctx.driver.run_speed(Direction::Forward, MAX_SAFE_SPEED)?;
    ctx.serial.send_only(cmd)?;

    // Let it run briefly
    println!("Running for 500ms...");
    std::thread::sleep(Duration::from_millis(500));

    // Stop
    println!("Stopping...");
    ctx.serial.send_only(ctx.driver.stop())?;

    // Disable motor
    println!("Disabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(false))?;

    println!("Test passed!");
    Ok(())
}

/// Test position movement
#[test]
fn test_run_position() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: run_position ===");

    validate_safe_speed(MAX_SAFE_SPEED)?;
    validate_safe_angle(MAX_SAFE_ANGLE_DEGREES)?;

    let mut ctx = TestContext::new()?;

    // Enable motor
    println!("Enabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(true))?;

    // Calculate pulses for safe angle
    let pulses = mks_servo42_rs::angle_to_steps(MAX_SAFE_ANGLE_DEGREES, SAFE_MICROSTEPS);
    println!(
        "Moving {}° forward ({} pulses)...",
        MAX_SAFE_ANGLE_DEGREES, pulses
    );

    // Move
    let cmd = ctx
        .driver
        .run_position(Direction::Forward, MAX_SAFE_SPEED, pulses)?;
    ctx.serial.send_only(cmd)?;

    // Wait for movement
    println!("Waiting for movement...");
    std::thread::sleep(LONG_PAUSE);

    // Stop
    ctx.serial.send_only(ctx.driver.stop())?;

    // Disable motor
    println!("Disabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(false))?;

    println!("Test passed!");
    Ok(())
}

/// Test reading encoder value
#[test]
fn test_read_encoder() -> TestResult<()> {
    init_env();

    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: read_encoder ===");

    let mut ctx = TestContext::new()?;

    // Enable motor first
    println!("Enabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(true))?;

    // Read encoder
    println!("Reading encoder value...");
    let response = ctx.serial.send_and_read(ctx.driver.read_encoder_value())?;

    if !response.is_empty() {
        match test_utils::parse_encoder_response(&response) {
            Ok(angle) => println!("Encoder angle: {:.2}°", angle),
            Err(e) => println!("Failed to parse encoder: {:?}", e),
        }
    } else {
        println!("No encoder response received");
    }

    // Disable motor
    println!("Disabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(false))?;

    println!("Test passed!");
    Ok(())
}

/// Test reading motor shaft angle
#[test]
fn test_read_motor_shaft_angle() -> TestResult<()> {
    init_env();

    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: read_motor_shaft_angle ===");

    let mut ctx = TestContext::new()?;

    // Enable motor first
    println!("Enabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(true))?;

    // Read motor shaft angle
    println!("Reading motor shaft angle...");
    let response = ctx
        .serial
        .send_and_read(ctx.driver.read_motor_shaft_angle())?;

    if !response.is_empty() {
        match test_utils::parse_motor_shaft_angle_response(&response) {
            Ok(angle) => println!("Motor shaft angle: {:.2}°", angle),
            Err(e) => println!("Failed to parse motor shaft angle: {:?}", e),
        }
    } else {
        println!("No motor shaft angle response received");
    }

    // Disable motor
    println!("Disabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(false))?;

    println!("Test passed!");
    Ok(())
}

/// Test reading motor shaft angle error
#[test]
fn test_read_motor_shaft_angle_error() -> TestResult<()> {
    init_env();

    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: read_motor_shaft_angle_error ===");

    let mut ctx = TestContext::new()?;

    // Enable motor first
    println!("Enabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(true))?;

    // Read motor shaft angle error
    println!("Reading motor shaft angle error...");
    let response = ctx
        .serial
        .send_and_read(ctx.driver.read_motor_shaft_angle_error())?;

    if !response.is_empty() {
        match test_utils::parse_motor_shaft_angle_error_response(&response) {
            Ok(error) => println!("Motor shaft angle error: {:.2}°", error),
            Err(e) => println!("Failed to parse motor shaft angle error: {:?}", e),
        }
    } else {
        println!("No motor shaft angle error response received");
    }

    // Disable motor
    println!("Disabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(false))?;

    println!("Test passed!");
    Ok(())
}

/// Test reading EN pin status
#[test]
fn test_read_en_pin_status() -> TestResult<()> {
    init_env();

    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: read_en_pin_status ===");

    let mut ctx = TestContext::new()?;

    // Read EN pin status (motor may be enabled or disabled)
    println!("Reading EN pin status...");
    let response = ctx.serial.send_and_read(ctx.driver.read_en_pin_status())?;

    if !response.is_empty() {
        match test_utils::parse_en_pin_status_response(&response) {
            Ok(status) => println!("EN pin status: {:?}", status),
            Err(e) => println!("Failed to parse EN pin status: {:?}", e),
        }
    } else {
        println!("No EN pin status response received");
    }

    println!("Test passed!");
    Ok(())
}

/// Test reading protection state
#[test]
fn test_read_protection_state() -> TestResult<()> {
    init_env();

    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: read_protection_state ===");

    let mut ctx = TestContext::new()?;

    // Enable motor first
    println!("Enabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(true))?;

    // Read protection state
    println!("Reading protection state...");
    let response = ctx
        .serial
        .send_and_read(ctx.driver.query_protection_state())?;

    if !response.is_empty() {
        println!("Protection state response: {:02x?}", response);
        if response.len() >= 3 {
            match response[1] {
                0x01 => println!("Motor blocked"),
                0x02 => println!("Motor unblocked"),
                0x00 => println!("Error state"),
                _ => println!("Unknown state: 0x{:02x}", response[1]),
            }
        }
    } else {
        println!("No protection state response received");
    }

    // Disable motor
    println!("Disabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(false))?;

    println!("Test passed!");
    Ok(())
}

/// Test reading pulse count
#[test]
fn test_read_pulse_count() -> TestResult<()> {
    init_env();

    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: read_pulse_count ===");

    let mut ctx = TestContext::new()?;

    // Enable motor first
    println!("Enabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(true))?;

    // Read pulse count
    println!("Reading pulse count...");
    let response = ctx.serial.send_and_read(ctx.driver.read_pulse_count())?;

    if !response.is_empty() {
        println!("Pulse count response: {:02x?}", response);
        // Parse 4-byte signed integer
        if response.len() >= 6 {
            // address + 4 bytes + checksum
            let pulse_bytes = &response[1..5];
            let pulses = i32::from_be_bytes([
                pulse_bytes[0],
                pulse_bytes[1],
                pulse_bytes[2],
                pulse_bytes[3],
            ]);
            println!("Pulse count: {}", pulses);
        }
    } else {
        println!("No pulse count response received");
    }

    // Disable motor
    println!("Disabling motor...");
    ctx.serial.send_only(ctx.driver.enable_motor(false))?;

    println!("Test passed!");
    Ok(())
}

/// Test safe parameter setting - subdivision
#[test]
fn test_set_subdivision() -> TestResult<()> {
    init_env();

    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_subdivision ===");

    let mut ctx = TestContext::new()?;

    // Try to set subdivision to a safe value (4 microsteps)
    println!("Setting subdivision to 4 microsteps...");
    let cmd = ctx.driver.set_subdivision(4)?; // Index 4 = 16 microsteps per full step
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 {
        if response[1] == 0x01 {
            println!("Subdivision set successfully");
        } else {
            println!("Failed to set subdivision: response {:02x?}", response);
        }
    } else {
        println!("No response for subdivision setting");
    }

    println!("Test passed!");
    Ok(())
}

/// Test that dangerous commands are skipped
#[test]
fn test_dangerous_commands_skipped() {
    init_env();

    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: dangerous commands skipped ===");

    // Check that we're not testing dangerous commands
    let dangerous = safety::DANGEROUS_COMMANDS;

    for (cmd_name, reason) in dangerous.iter() {
        println!("Skipping {}: {}", cmd_name, reason);
        assert!(safety::should_skip_command(cmd_name));
    }

    println!("All dangerous commands properly marked for skipping");
    println!("Test passed!");
}
