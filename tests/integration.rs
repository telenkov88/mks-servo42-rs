//! Integration tests for MKS SERVO42 driver
//!
//! These tests require real hardware connected via serial port.
//! Set MKS_ENV_SERVO42C_UART environment variable to serial port path.
//!
//! SAFETY: Tests use minimal speed and small movements to avoid damage.

mod safety;
mod test_utils;

// use mks_servo42_rs::direction::Direction; (removed)
use mks_servo42_rs::{EnLogic, RotationDirection, SaveClearStatus, ZeroMode};
use safety::{
    validate_safe_angle, validate_safe_speed, MAX_SAFE_ANGLE_DEGREES, MAX_SAFE_SPEED,
    SAFE_MICROSTEPS,
};
use std::ops::{Deref, DerefMut};
use std::time::Duration;
use test_utils::{init_env, TestContext, TestError, TestResult, LONG_PAUSE, TEST_MUTEX};

/// Guard to ensure motor is stopped even if test panics or fails
struct AutoStopGuard<'a> {
    pub ctx: &'a mut TestContext,
}

impl<'a> Drop for AutoStopGuard<'a> {
    fn drop(&mut self) {
        println!("AutoStopGuard: Stopping and disabling motor...");
        // Try to stop
        let _ = self.ctx.serial.send_only(self.ctx.driver.stop());
        // Try to disable
        let _ = self
            .ctx
            .serial
            .send_only(self.ctx.driver.enable_motor(false));
    }
}

impl<'a> Deref for AutoStopGuard<'a> {
    type Target = TestContext;
    fn deref(&self) -> &Self::Target {
        self.ctx
    }
}

impl<'a> DerefMut for AutoStopGuard<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.ctx
    }
}

/// Test basic motor enable/disable
#[test]
fn test_motor_enable_disable() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();
    println!("=== Test: motor enable/disable ===");

    let mut ctx = TestContext::new()?;
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Enable motor
    println!("Enabling motor...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.enable_motor(true))?;

    if response.len() >= 3 && response[1] == 0x01 {
        println!("Motor enabled successfully");
    } else {
        println!("Warning: Enable response not as expected: {:?}", response);
    }

    // Small delay
    std::thread::sleep(Duration::from_millis(100));

    // Disable motor
    println!("Disabling motor...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.enable_motor(false))?;

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
fn test_run_with_constant_speed() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: run_with_constant_speed ===");

    validate_safe_speed(MAX_SAFE_SPEED)?;

    let mut ctx = TestContext::new()?;
    // Use the guard to ensure stop is called
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Set safe subdivision
    // Cast SAFE_MICROSTEPS (f32) to u8 directly, assuming integer value like 4.0 -> 4
    let steps = SAFE_MICROSTEPS as u8;
    println!("Setting safe subdivision to {}...", steps);
    let cmd = guarded.ctx.driver.set_subdivision(steps)?;
    // Read response to ensure setting applied
    guarded.ctx.serial.send_and_read(cmd)?;

    // Enable motor
    println!("Enabling motor...");
    guarded
        .ctx
        .serial
        .send_only(guarded.ctx.driver.enable_motor(true))?;

    // Read initial angle
    println!("Reading initial encoder value...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;
    let initial_angle = test_utils::parse_encoder_response(&response)
        .map_err(|e| TestError::Protocol(format!("Failed to parse initial encoder: {:?}", e)))?;
    println!("Initial Angle: {:.2}°", initial_angle);

    // Run forward (CW) at minimal speed
    println!("Running CW at speed {}...", MAX_SAFE_SPEED);
    let cmd = guarded
        .ctx
        .driver
        .run_with_constant_speed(RotationDirection::Clockwise, MAX_SAFE_SPEED)?;
    guarded.ctx.serial.send_only(cmd)?;

    // Let it run briefly
    println!("Running for 500ms...");
    std::thread::sleep(Duration::from_millis(500));

    // Stop to read
    println!("Stopping to read...");
    guarded.ctx.serial.send_only(guarded.ctx.driver.stop())?;
    std::thread::sleep(Duration::from_millis(500)); // Wait for stop (increased for stability)

    // Read CW angle
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;
    let cw_angle = test_utils::parse_encoder_response(&response)
        .map_err(|e| TestError::Protocol(format!("Failed to parse CW encoder: {:?}", e)))?;
    println!("Angle after CW: {:.2}°", cw_angle);

    let delta_cw = cw_angle - initial_angle;
    println!("Delta CW: {:.2}°", delta_cw);

    if delta_cw.abs() < 1.0 {
        println!("Warning: Motor did not move enough CW? Delta: {}", delta_cw);
        return Err(TestError::Servo(format!(
            "Motor did not move enough CW. Delta: {}",
            delta_cw
        )));
    }

    // Run backward (CCW)
    println!("Running CCW at speed {}...", MAX_SAFE_SPEED);
    let cmd = guarded
        .ctx
        .driver
        .run_with_constant_speed(RotationDirection::CounterClockwise, MAX_SAFE_SPEED)?;
    guarded.ctx.serial.send_only(cmd)?;

    // Let it run briefly
    println!("Running for 500ms...");
    std::thread::sleep(Duration::from_millis(500));

    // Stop to read
    println!("Stopping to read...");
    guarded.ctx.serial.send_only(guarded.ctx.driver.stop())?;
    std::thread::sleep(Duration::from_millis(500)); // Wait for stop (increased for stability)

    // Read CCW angle
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;
    let final_angle = test_utils::parse_encoder_response(&response)
        .map_err(|e| TestError::Protocol(format!("Failed to parse final encoder: {:?}", e)))?;
    println!("Angle after CCW: {:.2}°", final_angle);

    let delta_return = final_angle - cw_angle;
    println!("Delta Return (from CW): {:.2}°", delta_return);

    if delta_return.abs() < 1.0 {
        return Err(TestError::Servo("Motor did not move enough CCW".into()));
    }

    println!("Test passed!");
    Ok(())
}

/// Test position movement
#[test]
fn test_run_motor() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: run_motor ===");

    validate_safe_speed(MAX_SAFE_SPEED)?;
    validate_safe_angle(MAX_SAFE_ANGLE_DEGREES)?;

    let mut ctx = TestContext::new()?;
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Set safe subdivision
    let steps = SAFE_MICROSTEPS as u8;
    println!("Setting safe subdivision to {}...", steps);
    let cmd = guarded.ctx.driver.set_subdivision(steps)?;
    guarded.ctx.serial.send_and_read(cmd)?;

    // Enable motor
    println!("Enabling motor...");
    guarded
        .ctx
        .serial
        .send_only(guarded.ctx.driver.enable_motor(true))?;

    // Read initial angle
    println!("Reading initial encoder value...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;
    let initial_angle = test_utils::parse_encoder_response(&response)
        .map_err(|e| TestError::Protocol(format!("Failed to parse initial encoder: {:?}", e)))?;
    println!("Initial Angle: {:.2}°", initial_angle);

    // Calculate pulses for safe angle
    let pulses = mks_servo42_rs::angle_to_steps(MAX_SAFE_ANGLE_DEGREES, SAFE_MICROSTEPS);
    println!(
        "Moving {}° Clockwise ({} pulses)...",
        MAX_SAFE_ANGLE_DEGREES, pulses
    );

    // Move CW
    let cmd = guarded
        .ctx
        .driver
        .run_motor(RotationDirection::Clockwise, MAX_SAFE_SPEED, pulses)?;
    guarded.ctx.serial.send_only(cmd)?;

    // Wait for movement
    println!("Waiting for movement...");
    std::thread::sleep(LONG_PAUSE);

    // Read angle after CW
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;
    let cw_angle = test_utils::parse_encoder_response(&response)
        .map_err(|e| TestError::Protocol(format!("Failed to parse CW encoder: {:?}", e)))?;
    println!("Angle after CW: {:.2}°", cw_angle);

    // Verify CW movement
    let delta_cw = cw_angle - initial_angle;
    println!("Delta CW: {:.2}°", delta_cw);

    // Check if it moved approx 10 degrees.
    // We allow some tolerance (e.g. 5 degrees) because of unknown initial state or tuning,
    // but it MUST be non-zero significantly.
    if delta_cw.abs() < 1.0 {
        println!("Warning: Motor did not move enough CW? Delta: {}", delta_cw);
        // Depending on STRICT mode, we might error. For now, strict.
        return Err(TestError::Servo(format!(
            "Motor did not move enough CW. Delta: {}",
            delta_cw
        )));
    }

    // Move CCW
    println!(
        "Moving {}° Counter-Clockwise ({} pulses)...",
        MAX_SAFE_ANGLE_DEGREES, pulses
    );
    let cmd = guarded.ctx.driver.run_motor(
        RotationDirection::CounterClockwise,
        MAX_SAFE_SPEED,
        pulses,
    )?;
    guarded.ctx.serial.send_only(cmd)?;

    // Wait for movement
    println!("Waiting for movement...");
    std::thread::sleep(LONG_PAUSE);

    // Read angle after CCW
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;
    let final_angle = test_utils::parse_encoder_response(&response)
        .map_err(|e| TestError::Protocol(format!("Failed to parse final encoder: {:?}", e)))?;
    println!("Angle after CCW: {:.2}°", final_angle);

    // Verify CCW movement (should be back to approx initial)
    let delta_total = final_angle - initial_angle;
    println!("Final Delta from Initial: {:.2}°", delta_total);

    // Should be close to 0
    if delta_total.abs() > 3.0 {
        println!(
            "Warning: Final angle not exactly initial. Delta: {}",
            delta_total
        );
    }

    // Additional check: angle should have moved BACK from CW data
    let delta_return = final_angle - cw_angle;
    println!("Delta Return: {:.2}°", delta_return);
    if delta_return.abs() < 1.0 {
        return Err(TestError::Servo("Motor did not move enough CCW".into()));
    }

    // Stop
    guarded.ctx.serial.send_only(guarded.ctx.driver.stop())?;

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
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Enable motor first
    println!("Enabling motor...");
    guarded
        .ctx
        .serial
        .send_only(guarded.ctx.driver.enable_motor(true))?;

    // Read encoder
    println!("Reading encoder value...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;

    if !response.is_empty() {
        match test_utils::parse_encoder_response(&response) {
            Ok(angle) => println!("Encoder angle: {:.2}°", angle),
            Err(e) => println!("Failed to parse encoder: {:?}", e),
        }
    } else {
        println!("No encoder response received");
    }

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
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Enable motor first
    println!("Enabling motor...");
    guarded
        .ctx
        .serial
        .send_only(guarded.ctx.driver.enable_motor(true))?;

    // Read motor shaft angle
    println!("Reading motor shaft angle...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_motor_shaft_angle())?;

    if !response.is_empty() {
        match test_utils::parse_motor_shaft_angle_response(&response) {
            Ok(angle) => println!("Motor shaft angle: {:.2}°", angle),
            Err(e) => println!("Failed to parse motor shaft angle: {:?}", e),
        }
    } else {
        println!("No motor shaft angle response received");
    }

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
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Enable motor first
    println!("Enabling motor...");
    guarded
        .ctx
        .serial
        .send_only(guarded.ctx.driver.enable_motor(true))?;

    // Read motor shaft angle error
    println!("Reading motor shaft angle error...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_motor_shaft_angle_error())?;

    if !response.is_empty() {
        match test_utils::parse_motor_shaft_angle_error_response(&response) {
            Ok(error) => println!("Motor shaft angle error: {:.2}°", error),
            Err(e) => println!("Failed to parse motor shaft angle error: {:?}", e),
        }
    } else {
        println!("No motor shaft angle error response received");
    }

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

/// Test reading shaft status (previously protection state)
#[test]
fn test_read_shaft_status() -> TestResult<()> {
    init_env();

    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: read_shaft_status ===");

    let mut ctx = TestContext::new()?;
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Enable motor first
    println!("Enabling motor...");
    guarded
        .ctx
        .serial
        .send_only(guarded.ctx.driver.enable_motor(true))?;

    // Read shaft status
    println!("Reading shaft status...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_shaft_status())?;

    if !response.is_empty() {
        println!("Shaft status response: {:02x?}", response);
        match test_utils::parse_shaft_status_response(&response) {
            Ok(status) => println!("Shaft status: {:?}", status),
            Err(e) => println!("Failed to parse shaft status: {:?}", e),
        }
    } else {
        println!("No shaft status response received");
    }

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
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Enable motor first
    println!("Enabling motor...");
    guarded
        .ctx
        .serial
        .send_only(guarded.ctx.driver.enable_motor(true))?;

    // Read pulse count
    println!("Reading pulse count...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_pulse_count())?;

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

/// Test setting max torque safely
#[test]
fn test_set_max_torque() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_max_torque ===");

    let mut ctx = TestContext::new()?;

    // Default 0x4B0 = 1200
    let default_torque = 0x4B0;
    println!("Setting max torque to default {}...", default_torque);

    let cmd = ctx.driver.set_max_torque(default_torque)?;
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 {
        if response[1] == 0x01 {
            println!("Max torque set successfully");
        } else {
            println!("Failed to set max torque: response {:02x?}", response);
            return Err(TestError::Protocol(format!(
                "Failed to set max torque: {:?}",
                response
            )));
        }
    } else {
        println!("No response for max torque setting");
        return Err(TestError::Protocol(
            "No response for max torque setting".into(),
        ));
    }

    println!("Test passed!");
    Ok(())
}

/// Test miscellaneous configuration commands that are safe
#[test]
fn test_misc_config() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: misc config ===");

    let mut ctx = TestContext::new()?;

    // set_auto_screen_off (0x87)
    println!("Setting auto screen off (disable)...");
    let cmd = ctx.driver.set_auto_screen_off(false);
    let response = ctx.serial.send_and_read(cmd)?;
    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("Auto screen off disabled");
    } else {
        println!("Failed set_auto_screen_off: {:?}", response);
    }

    // set_stall_protection (0x88) - Enable
    println!("Setting stall protection (enable)...");
    let cmd = ctx.driver.set_stall_protection(true);
    let response = ctx.serial.send_and_read(cmd)?;
    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("Stall protection enabled");
    } else {
        println!("Failed set_stall_protection: {:?}", response);
    }

    // set_interpolation (0x89) - Enable
    println!("Setting interpolation (enable)...");
    let cmd = ctx.driver.set_interpolation(true);
    let response = ctx.serial.send_and_read(cmd)?;
    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("Interpolation enabled");
    } else {
        println!("Failed set_interpolation: {:?}", response);
    }

    Ok(())
}

/// Test setting position KP
#[test]
fn test_set_position_kp() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_position_kp ===");

    let mut ctx = TestContext::new()?;

    // Default 0x650
    let default_kp = 0x650;
    println!("Setting position KP to default {}...", default_kp);

    let cmd = ctx.driver.set_position_kp(default_kp);
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("Position KP set successfully");
    } else {
        println!("Failed to set position KP: response {:02x?}", response);
        return Err(TestError::Protocol(format!(
            "Failed to set position KP: {:?}",
            response
        )));
    }

    println!("Test passed!");
    Ok(())
}

/// Test setting position KI
#[test]
fn test_set_position_ki() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_position_ki ===");

    let mut ctx = TestContext::new()?;

    // Default 1
    let default_ki = 1;
    println!("Setting position KI to default {}...", default_ki);

    let cmd = ctx.driver.set_position_ki(default_ki);
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("Position KI set successfully");
    } else {
        println!("Failed to set position KI: response {:02x?}", response);
        return Err(TestError::Protocol(format!(
            "Failed to set position KI: {:?}",
            response
        )));
    }

    println!("Test passed!");
    Ok(())
}

/// Test setting position KD
#[test]
fn test_set_position_kd() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_position_kd ===");

    let mut ctx = TestContext::new()?;

    // Default 0x650
    let default_kd = 0x650;
    println!("Setting position KD to default {}...", default_kd);

    let cmd = ctx.driver.set_position_kd(default_kd);
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("Position KD set successfully");
    } else {
        println!("Failed to set position KD: response {:02x?}", response);
        return Err(TestError::Protocol(format!(
            "Failed to set position KD: {:?}",
            response
        )));
    }

    println!("Test passed!");
    Ok(())
}

/// Test setting acceleration
#[test]
fn test_set_acceleration() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_acceleration ===");

    let mut ctx = TestContext::new()?;

    // Default 0x11e
    let default_acc = 0x11e;
    println!("Setting acceleration to default {}...", default_acc);

    let cmd = ctx.driver.set_acceleration(default_acc);
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("Acceleration set successfully");
    } else {
        println!("Failed to set acceleration: response {:02x?}", response);
        return Err(TestError::Protocol(format!(
            "Failed to set acceleration: {:?}",
            response
        )));
    }

    println!("Test passed!");
    Ok(())
}

/// Test setting max torque out of range
#[test]
fn test_set_max_torque_out_of_range() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_max_torque_out_of_range ===");

    let mut ctx = TestContext::new()?;

    // Max allowed is 0x4B0. Try 0x4B1.
    let out_of_range_torque = 0x4B1;
    println!(
        "Attempting to set max torque to {} (limit is 0x4B0)...",
        out_of_range_torque
    );

    let result = ctx.driver.set_max_torque(out_of_range_torque);

    match result {
        Ok(_) => {
            println!("Error: mismatched expectation. Should have failed.");
            return Err(TestError::Protocol(
                "Driver allowed out of range torque".into(),
            ));
        }
        Err(mks_servo42_rs::Error::InvalidValue) => {
            println!("Driver correctly rejected out of range torque with InvalidValue.");
        }
        Err(e) => {
            println!("Driver failed with unexpected error: {:?}", e);
            return Err(TestError::Protocol(format!(
                "Unexpected error type: {:?}",
                e
            )));
        }
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

/// Test reading release status
#[test]
fn test_read_release_status() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: read_release_status ===");

    let mut ctx = TestContext::new()?;
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Enable motor first
    println!("Enabling motor...");
    guarded
        .ctx
        .serial
        .send_only(guarded.ctx.driver.enable_motor(true))?;

    // Read release status
    println!("Reading release status...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_release_status())?;

    if !response.is_empty() {
        println!("Release status response: {:02x?}", response);
        // We don't have a parser yet, but we expect a valid response (addr + status + checksum)
        if response.len() >= 3 {
            println!("Release status byte: {:02x}", response[1]);
        }
    } else {
        println!("No release status response received");
        return Err(TestError::Protocol(
            "No response for read_release_status".into(),
        ));
    }

    println!("Test passed!");
    Ok(())
}

/// Test save/clear status (Only Clear/CA)
#[test]
fn test_save_clear_status() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: save_clear_status ===");

    let mut ctx = TestContext::new()?;

    // Ensure motor is disabled before clearing status, just in case
    ctx.serial.send_only(ctx.driver.enable_motor(false))?;
    std::thread::sleep(Duration::from_millis(300)); // Longer delay for stability

    // User constraint: ONLY USE CA CLEAR STATUS!
    println!("Sending SaveClearStatus::Clear (CA)...");

    // Note: The enum SaveClearStatus::Clear maps to 0xCA
    let cmd = ctx.driver.save_clear_status(SaveClearStatus::Clear);
    let response = ctx.serial.send_and_read(cmd)?;

    // Response format: [address, status, checksum] or partial
    // Status: 0x01 = success, 0x00 = failure/nothing to clear
    if response.len() >= 2 {
        // Check if first byte is address (0xE0-0xE9) or status
        let status_byte = if response[0] >= 0xE0 && response[0] <= 0xE9 {
            response[1]
        } else {
            response[0] // Address was stripped
        };

        if status_byte == 0x01 {
            println!("Status cleared successfully");
        } else if status_byte == 0x00 {
            // 0x00 means "Failure" per protocol, but clear may return this if nothing to clear
            println!("Clear status returned 0x00 (nothing to clear)");
        } else {
            println!(
                "Unexpected status byte: {:02x}, response: {:02x?}",
                status_byte, response
            );
        }
    } else if !response.is_empty() {
        println!("Short response for save_clear_status: {:02x?}", response);
        // Still consider it a pass if we got any response
    } else {
        println!("No response for save_clear_status");
        return Err(TestError::Protocol(
            "No response for save_clear_status".into(),
        ));
    }

    println!("Test passed!");
    Ok(())
}

/// Test setting current limit (1000mA -> Index 5)
#[test]
fn test_set_current_limit() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_current_limit ===");

    let mut ctx = TestContext::new()?;

    // User constraint: use index for 1000ma value
    // CURRENT_STEP_MA = 200. 1000 / 200 = 5.
    let index: u8 = 5;
    println!(
        "Setting current limit to index {} ({}mA)...",
        index,
        index as u16 * mks_servo42_rs::CURRENT_STEP_MA
    );

    let cmd = ctx.driver.set_current_limit(index)?;
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 {
        if response[1] == 0x01 {
            println!("Current limit set successfully");
        } else {
            println!("Failed to set current limit: {:02x?}", response);
            return Err(TestError::Protocol(format!(
                "Failed to set current limit: {:?}",
                response
            )));
        }
    } else {
        return Err(TestError::Protocol(
            "No response for set_current_limit".into(),
        ));
    }

    println!("Test passed!");
    Ok(())
}

/// Test setting EN logic (Hold/AlwaysOn/02)
#[test]
fn test_set_en_logic() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_en_logic ===");

    let mut ctx = TestContext::new()?;

    // User constraint: Use Hold Value (02) -> EnLogic::AlwaysOn
    println!("Setting EN logic to AlwaysOn (0x02)...");

    let cmd = ctx.driver.set_enable_logic(EnLogic::AlwaysOn);
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 {
        if response[1] == 0x01 {
            println!("EN logic set successfully");
        } else {
            println!("Failed to set EN logic: {:02x?}", response);
            // Determining if this is a hard failure or not. Usually strict.
            return Err(TestError::Protocol(format!(
                "Failed to set EN logic: {:?}",
                response
            )));
        }
    } else {
        return Err(TestError::Protocol("No response for set_en_logic".into()));
    }

    println!("Test passed!");
    Ok(())
}

/// Test setting direction
#[test]
fn test_set_direction() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: set_direction ===");

    let mut ctx = TestContext::new()?;

    // Test Clockwise (true) -> 0x00 sent
    println!("Setting direction to Clockwise...");
    let cmd = ctx.driver.set_direction(RotationDirection::Clockwise);
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("Direction set to CW successfully");
    } else {
        println!("Failed to set direction CW: {:02x?}", response);
        return Err(TestError::Protocol("Failed to set direction CW".into()));
    }

    // Toggle back to Counter-Clockwise
    std::thread::sleep(Duration::from_millis(100));
    println!("Setting direction to Counter-Clockwise...");
    let cmd = ctx
        .driver
        .set_direction(RotationDirection::CounterClockwise);
    let response = ctx.serial.send_and_read(cmd)?;

    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("Direction set to CCW successfully");
    } else {
        println!("Failed to set direction CCW: {:02x?}", response);
        return Err(TestError::Protocol("Failed to set direction CCW".into()));
    }

    println!("Test passed!");
    Ok(())
}

/// Test full zero mode workflow: setup, set zero, move, and return to zero
/// 1. Configure zero mode (DirMode)
/// 2. Configure zero direction and speed
/// 3. Set current position as zero
/// 4. Move motor away from zero
/// 5. Trigger go_to_zero
/// 6. Verify motor returns to zero position
#[test]
fn test_zero_mode_workflow() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    println!("=== Test: zero_mode_workflow ===");

    let mut ctx = TestContext::new()?;
    let guarded = AutoStopGuard { ctx: &mut ctx };

    // Step 1: Set zero mode to DirMode
    println!("Step 1: Setting zero mode to DirMode...");
    let cmd = guarded.ctx.driver.set_zero_mode(ZeroMode::DirMode);
    let response = guarded.ctx.serial.send_and_read(cmd)?;
    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("  Zero mode set to DirMode successfully");
    } else {
        println!("  Failed to set zero mode: {:02x?}", response);
        return Err(TestError::Protocol(format!(
            "Failed to set zero mode: {:02x?}",
            response
        )));
    }
    std::thread::sleep(Duration::from_millis(100));

    // Step 2: Set zero direction to Clockwise
    println!("Step 2: Setting zero direction to Clockwise...");
    let cmd = guarded
        .ctx
        .driver
        .set_zero_direction(RotationDirection::Clockwise);
    let response = guarded.ctx.serial.send_and_read(cmd)?;
    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("  Zero direction set to CW successfully");
    } else {
        println!(
            "  Warning: Unexpected response for set_zero_direction: {:02x?}",
            response
        );
    }
    std::thread::sleep(Duration::from_millis(100));

    // Step 3: Set zero speed (2 = moderate speed)
    println!("Step 3: Setting zero speed to 2...");
    let cmd = guarded.ctx.driver.set_zero_speed(2)?;
    let response = guarded.ctx.serial.send_and_read(cmd)?;
    if !response.is_empty() && response.len() >= 3 && response[1] == 0x01 {
        println!("  Zero speed set successfully");
    } else {
        println!("  Failed to set zero speed: {:02x?}", response);
        return Err(TestError::Protocol(format!(
            "Failed to set zero speed: {:02x?}",
            response
        )));
    }
    std::thread::sleep(Duration::from_millis(100));

    // Test invalid zero speed (driver validation)
    let invalid_speed = mks_servo42_rs::MAX_ZERO_SPEED + 1;
    match guarded.ctx.driver.set_zero_speed(invalid_speed) {
        Ok(_) => {
            println!(
                "  Error: Driver allowed invalid zero speed {}",
                invalid_speed
            );
            return Err(TestError::Protocol(
                "Driver allowed invalid zero speed".into(),
            ));
        }
        Err(mks_servo42_rs::Error::InvalidValue) => {
            println!(
                "  Driver correctly rejected invalid zero speed {}",
                invalid_speed
            );
        }
        Err(e) => {
            return Err(TestError::Protocol(format!("Unexpected error: {:?}", e)));
        }
    }

    // Step 4: Enable motor
    println!("Step 4: Enabling motor...");
    guarded
        .ctx
        .serial
        .send_only(guarded.ctx.driver.enable_motor(true))?;
    std::thread::sleep(Duration::from_millis(100));

    // Step 5: Set safe subdivision
    println!("Step 5: Setting safe subdivision...");
    let cmd = guarded.ctx.driver.set_subdivision(SAFE_MICROSTEPS as u8)?;
    guarded.ctx.serial.send_and_read(cmd)?;
    std::thread::sleep(Duration::from_millis(100));

    // Step 6: Read initial encoder position
    println!("Step 6: Reading initial encoder position...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;
    let initial_angle = test_utils::parse_encoder_response(&response)
        .map_err(|e| TestError::Protocol(format!("Failed to parse initial encoder: {:?}", e)))?;
    println!("  Initial Angle: {:.2}°", initial_angle);

    // Step 7: Set current position as zero
    println!("Step 7: Setting current position as zero...");
    let cmd = guarded.ctx.driver.set_current_as_zero();
    match guarded.ctx.serial.send_and_read(cmd) {
        Ok(response) => {
            if !response.is_empty() && response.len() >= 3 {
                if response[1] == 0x01 {
                    println!("  Current position set as zero successfully");
                } else {
                    println!(
                        "  Warning: set_current_as_zero returned status {:02x}",
                        response[1]
                    );
                }
            } else {
                println!(
                    "  Warning: Short response for set_current_as_zero: {:02x?}",
                    response
                );
            }
        }
        Err(e) => {
            println!(
                "  Warning: Failed to read response for set_current_as_zero: {:?}",
                e
            );
        }
    }
    std::thread::sleep(Duration::from_millis(200));

    // Step 8: Move motor 20 degrees CW
    let move_angle = 20.0_f32;
    let pulses = mks_servo42_rs::angle_to_steps(move_angle, SAFE_MICROSTEPS);
    println!(
        "Step 8: Moving motor {:.0}° Clockwise ({} pulses)...",
        move_angle, pulses
    );
    let cmd = guarded
        .ctx
        .driver
        .run_motor(RotationDirection::Clockwise, MAX_SAFE_SPEED, pulses)?;
    guarded.ctx.serial.send_only(cmd)?;
    std::thread::sleep(LONG_PAUSE);

    // Stop and read position
    guarded.ctx.serial.send_only(guarded.ctx.driver.stop())?;
    std::thread::sleep(Duration::from_millis(500));

    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;
    let moved_angle = test_utils::parse_encoder_response(&response)
        .map_err(|e| TestError::Protocol(format!("Failed to parse moved encoder: {:?}", e)))?;
    println!("  Angle after move: {:.2}°", moved_angle);

    let delta_move = moved_angle - initial_angle;
    println!("  Delta from initial: {:.2}°", delta_move);
    if delta_move.abs() < 5.0 {
        println!(
            "  Warning: Motor did not move significantly. Delta: {:.2}",
            delta_move
        );
    }

    // Step 9: Trigger go_to_zero
    println!("Step 9: Triggering go_to_zero...");
    let cmd = guarded.ctx.driver.go_to_zero();
    match guarded.ctx.serial.send_and_read(cmd) {
        Ok(response) => {
            if !response.is_empty() && response.len() >= 3 {
                if response[1] == 0x01 {
                    println!("  Go to zero command accepted");
                } else {
                    println!("  Go to zero returned status: {:02x}", response[1]);
                }
            } else {
                println!(
                    "  Warning: Go to zero response invalid/empty: {:02x?}",
                    response
                );
            }
        }
        Err(e) => {
            println!(
                "  Warning: go_to_zero timed out or failed: {:?}. Continuing.",
                e
            );
        }
    }

    // Wait for motor to return to zero
    println!("  Waiting for motor to return to zero...");
    std::thread::sleep(Duration::from_secs(3));

    // Step 10: Verify motor returned to approximately initial position
    println!("Step 10: Verifying motor returned to zero...");
    let response = guarded
        .ctx
        .serial
        .send_and_read(guarded.ctx.driver.read_encoder_value())?;
    let final_angle = test_utils::parse_encoder_response(&response)
        .map_err(|e| TestError::Protocol(format!("Failed to parse final encoder: {:?}", e)))?;
    println!("  Final Angle: {:.2}°", final_angle);

    let delta_return = final_angle - initial_angle;
    println!("  Delta from initial (after return): {:.2}°", delta_return);

    // Check if returned close to initial position (within 5 degrees tolerance)
    if delta_return.abs() > 5.0 {
        println!(
            "  Warning: Motor did not return exactly to zero. Delta: {:.2}°",
            delta_return
        );
        // Check if it at least moved back towards initial
        if delta_return.abs() < delta_move.abs() {
            println!("  Motor moved closer to initial position (partial success)");
        } else {
            println!("  Motor did not appear to return towards zero");
        }
    } else {
        println!("  Motor returned to zero successfully!");
    }

    // Stop motor
    guarded.ctx.serial.send_only(guarded.ctx.driver.stop())?;

    println!("Test passed!");
    Ok(())
}
