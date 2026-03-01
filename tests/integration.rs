//! Integration tests for MKS SERVO42 driver
//!
//! Requires real hardware connected via serial port.
//! Set MKS_ENV_SERVO42C_UART and PICO_ENCODER_UART in .env

mod safety;
mod test_utils;

use mks_servo42_rs::{EnLogic, RotationDirection, SaveClearStatus, ZeroMode};
use safety::{
    DANGEROUS_COMMANDS, MAX_SAFE_ANGLE_DEGREES, MAX_SAFE_SPEED, SAFE_MICROSTEPS,
    should_skip_command, validate_safe_angle, validate_safe_speed,
};
use std::ops::{Deref, DerefMut};
use std::time::Duration;
use test_utils::{LONG_PAUSE, TEST_MUTEX, TestContext, TestError, TestResult, init_env};

// ── Test constants ─────────────────────────────────────────────────────────

const TEST_MOVE_DEGREES: f32 = 10.0;
const TEST_ROTATIONS_DEGREES: f32 = 3600.0; // 10 full rotations
const MOVE_DURATION_TOLERANCE_SECS: f64 = 0.5;
const MOTOR_STOP_TIMEOUT: Duration = Duration::from_secs(60);
const PULSE_TOLERANCE: i32 = 4;
const ZERO_RETURN_TOLERANCE_DEGREES: f64 = 5.0;
const SUBDIVISON_0_WAIT_MS: u64 = 5000;
const SUBDIVISON_OTHER_WAIT_MS: u64 = 1500;
const MOTOR_SETTLE_MS: u64 = 500;
const CMD_SETTLE_MS: u64 = 100;
const ZERO_RETURN_WAIT_SECS: u64 = 3;

// ── Test infrastructure ─────────────────────────────────────────────────────

struct AutoStopGuard<'a> {
    pub ctx: &'a mut TestContext,
}

impl<'a> Drop for AutoStopGuard<'a> {
    fn drop(&mut self) {
        println!("AutoStopGuard: Stopping and disabling motor...");
        let _ = self.ctx.stop_motor();
        let _ = self.ctx.disable_motor();
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

macro_rules! test_setup {
    ($ctx:ident) => {
        init_env();
        let _guard = TEST_MUTEX.lock().unwrap();
        let mut $ctx = TestContext::new()?;
    };
}

macro_rules! test_setup_guarded {
    ($ctx:ident, $guarded:ident) => {
        init_env();
        let _guard = TEST_MUTEX.lock().unwrap();
        let mut $ctx = TestContext::new()?;
        let $guarded = AutoStopGuard { ctx: &mut $ctx };
    };
}

#[test]
fn test_motor_enable_disable() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    guarded.ctx.enable_motor()?;
    std::thread::sleep(Duration::from_millis(CMD_SETTLE_MS));
    guarded.ctx.disable_motor()?;

    Ok(())
}

#[test]
fn test_run_with_constant_speed() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    validate_safe_speed(MAX_SAFE_SPEED)?;

    guarded.ctx.set_subdivision(SAFE_MICROSTEPS as u8)?;
    guarded.ctx.enable_motor()?;

    guarded
        .ctx
        .run_constant_speed(RotationDirection::Clockwise, MAX_SAFE_SPEED)?;
    std::thread::sleep(Duration::from_millis(MOTOR_SETTLE_MS));
    guarded.ctx.stop_motor()?;
    std::thread::sleep(Duration::from_millis(MOTOR_SETTLE_MS));

    let after_cw = guarded.ctx.read_encoder_angle()?;

    guarded
        .ctx
        .run_constant_speed(RotationDirection::CounterClockwise, MAX_SAFE_SPEED)?;
    std::thread::sleep(Duration::from_millis(MOTOR_SETTLE_MS));
    guarded.ctx.stop_motor()?;
    std::thread::sleep(Duration::from_millis(MOTOR_SETTLE_MS));

    let after_ccw = guarded.ctx.read_encoder_angle()?;

    let min_movement = 1.0_f64;
    if (after_ccw - after_cw).abs() < min_movement {
        return Err(TestError::Servo("Motor did not move CCW".into()));
    }

    Ok(())
}

#[test]
fn test_run_motor() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    validate_safe_speed(MAX_SAFE_SPEED)?;
    validate_safe_angle(MAX_SAFE_ANGLE_DEGREES)?;

    guarded.ctx.set_subdivision(SAFE_MICROSTEPS as u8)?;
    guarded.ctx.enable_motor()?;

    let pulses = mks_servo42_rs::angle_to_steps(MAX_SAFE_ANGLE_DEGREES, SAFE_MICROSTEPS);

    let pulses_before = guarded.ctx.read_pulse_count()?;
    guarded
        .ctx
        .run_motor(RotationDirection::Clockwise, MAX_SAFE_SPEED, pulses)?;
    std::thread::sleep(LONG_PAUSE);

    let after_cw = guarded.ctx.read_encoder_angle()?;
    let cw_pulses = guarded.ctx.read_pulse_count()?;
    let cw_pulse_diff = (cw_pulses - pulses_before).abs();

    if (cw_pulse_diff - pulses as i32).abs() > PULSE_TOLERANCE {
        return Err(TestError::Servo(format!(
            "CW pulse count off: got {cw_pulse_diff}, expected {pulses}"
        )));
    }

    guarded
        .ctx
        .run_motor(RotationDirection::CounterClockwise, MAX_SAFE_SPEED, pulses)?;
    std::thread::sleep(LONG_PAUSE);

    let after_ccw = guarded.ctx.read_encoder_angle()?;

    let min_movement = 1.0_f64;
    if (after_ccw - after_cw).abs() < min_movement {
        return Err(TestError::Servo("Motor did not move CCW".into()));
    }

    guarded.ctx.stop_motor()?;
    Ok(())
}

#[test]
fn test_run_motor_subdivisions() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    guarded.ctx.setup_motor()?;
    let angle_to_move = TEST_MOVE_DEGREES;

    for subdivision in [0u8, 100u8] {
        guarded.ctx.set_subdivision(subdivision)?;

        let usteps = if subdivision == 0 {
            256.0
        } else {
            subdivision as f32
        };
        let pulses = mks_servo42_rs::angle_to_steps(angle_to_move, usteps);
        let wait = if subdivision == 0 {
            SUBDIVISON_0_WAIT_MS
        } else {
            SUBDIVISON_OTHER_WAIT_MS
        };

        guarded
            .ctx
            .run_motor(RotationDirection::Clockwise, 1, pulses)?;
        std::thread::sleep(Duration::from_millis(wait));

        let angle = guarded.ctx.read_encoder_angle()?;
        println!("Subdivision {subdivision}: encoder angle = {angle:.2}°");
    }

    Ok(())
}

#[test]
fn test_read_encoder() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    guarded.ctx.enable_motor()?;
    let angle = guarded.ctx.read_encoder_angle()?;
    println!("Encoder angle: {angle:.2}°");

    Ok(())
}

#[test]
fn test_read_motor_shaft_angle() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    guarded.ctx.enable_motor()?;
    let angle = guarded.ctx.read_shaft_angle()?;
    println!("Shaft angle: {angle:.2}°");

    Ok(())
}

#[test]
fn test_read_motor_shaft_angle_error() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    guarded.ctx.enable_motor()?;
    let error = guarded.ctx.read_shaft_angle_error()?;
    println!("Shaft angle error: {error:.2}°");

    Ok(())
}

#[test]
fn test_read_en_pin_status() -> TestResult<()> {
    test_setup!(ctx);

    let status = ctx.read_en_pin_status()?;
    println!("EN pin status: {status:?}");

    Ok(())
}

#[test]
fn test_read_shaft_status() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    guarded.ctx.enable_motor()?;
    let status = guarded.ctx.read_shaft_status()?;
    println!("Shaft status: {status:?}");

    Ok(())
}

#[test]
fn test_read_pulse_count() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    guarded.ctx.enable_motor()?;
    let pulses = guarded.ctx.read_pulse_count()?;
    println!("Pulse count: {pulses}");

    Ok(())
}

#[test]
fn test_set_subdivision() -> TestResult<()> {
    test_setup!(ctx);

    ctx.set_subdivision(4)?;

    Ok(())
}

#[test]
fn test_set_max_torque() -> TestResult<()> {
    test_setup!(ctx);

    ctx.set_max_torque(0x4B0)?;

    Ok(())
}

#[test]
fn test_misc_config() -> TestResult<()> {
    test_setup!(ctx);

    ctx.set_auto_screen_off(false)?;
    ctx.set_stall_protection(true)?;
    ctx.set_interpolation(true)?;

    Ok(())
}

#[test]
fn test_set_position_kp() -> TestResult<()> {
    test_setup!(ctx);

    ctx.set_position_kp(0x650)?;

    Ok(())
}

#[test]
fn test_set_position_ki() -> TestResult<()> {
    test_setup!(ctx);

    ctx.set_position_ki(1)?;

    Ok(())
}

#[test]
fn test_set_position_kd() -> TestResult<()> {
    test_setup!(ctx);

    ctx.set_position_kd(0x650)?;

    Ok(())
}

#[test]
fn test_set_acceleration() -> TestResult<()> {
    test_setup!(ctx);

    ctx.set_acceleration(0x11e)?;

    Ok(())
}

#[test]
fn test_set_max_torque_out_of_range() -> TestResult<()> {
    test_setup!(ctx);

    match ctx.driver.set_max_torque(0x4B1) {
        Err(mks_servo42_rs::Error::InvalidValue) => Ok(()),
        Ok(_) => Err(TestError::Protocol(
            "Driver allowed out-of-range torque".into(),
        )),
        Err(e) => Err(TestError::Protocol(format!("Unexpected error: {e:?}"))),
    }
}

#[test]
fn test_dangerous_commands_skipped() {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    for (cmd_name, reason) in DANGEROUS_COMMANDS.iter() {
        println!("Skipping {cmd_name}: {reason}");
        assert!(should_skip_command(cmd_name));
    }
}

#[test]
fn test_read_release_status() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    guarded.ctx.enable_motor()?;
    let status = guarded.ctx.read_release_status()?;
    println!("Release status: {status:#04x}");

    Ok(())
}

#[test]
fn test_save_clear_status() -> TestResult<()> {
    test_setup!(ctx);

    ctx.disable_motor()?;
    std::thread::sleep(Duration::from_millis(CMD_SETTLE_MS * 3));
    ctx.save_clear_status(SaveClearStatus::Clear)?;

    Ok(())
}

#[test]
fn test_set_current_limit() -> TestResult<()> {
    test_setup!(ctx);

    let index = 5u8;
    ctx.set_current_limit(index)?;

    Ok(())
}

#[test]
fn test_set_en_logic() -> TestResult<()> {
    test_setup!(ctx);

    ctx.set_enable_logic(EnLogic::AlwaysOn)?;

    Ok(())
}

#[test]
fn test_set_direction() -> TestResult<()> {
    test_setup!(ctx);

    ctx.set_direction(RotationDirection::Clockwise)?;
    std::thread::sleep(Duration::from_millis(CMD_SETTLE_MS));
    ctx.set_direction(RotationDirection::CounterClockwise)?;

    Ok(())
}

#[test]
fn test_zero_mode_workflow() -> TestResult<()> {
    test_setup_guarded!(ctx, guarded);

    guarded.ctx.set_zero_mode(ZeroMode::DirMode)?;
    guarded
        .ctx
        .set_zero_direction(RotationDirection::Clockwise)?;

    let zero_speed = 2u8;
    guarded.ctx.set_zero_speed(zero_speed)?;

    let invalid_speed = mks_servo42_rs::MAX_ZERO_SPEED + 1;
    match guarded.ctx.driver.set_zero_speed(invalid_speed) {
        Err(mks_servo42_rs::Error::InvalidValue) => {}
        Ok(_) => {
            return Err(TestError::Protocol(
                "Driver allowed invalid zero speed".into(),
            ));
        }
        Err(e) => return Err(TestError::Protocol(format!("Unexpected error: {e:?}"))),
    }

    guarded.ctx.enable_motor()?;
    guarded.ctx.set_subdivision(SAFE_MICROSTEPS as u8)?;

    let angle_before = guarded.ctx.read_encoder_angle()?;
    guarded.ctx.set_current_as_zero()?;
    std::thread::sleep(Duration::from_millis(CMD_SETTLE_MS * 2));

    let move_degrees = TEST_MOVE_DEGREES * 2.0;
    let pulses = mks_servo42_rs::angle_to_steps(move_degrees, SAFE_MICROSTEPS);
    guarded
        .ctx
        .run_motor(RotationDirection::Clockwise, MAX_SAFE_SPEED, pulses)?;
    std::thread::sleep(LONG_PAUSE);
    guarded.ctx.stop_motor()?;
    std::thread::sleep(Duration::from_millis(MOTOR_SETTLE_MS));

    let angle_moved = guarded.ctx.read_encoder_angle()?;
    if (angle_moved - angle_before).abs() < ZERO_RETURN_TOLERANCE_DEGREES {
        return Err(TestError::Servo("Motor did not move far enough".into()));
    }

    guarded.ctx.go_to_zero()?;
    std::thread::sleep(Duration::from_secs(ZERO_RETURN_WAIT_SECS));

    let angle_final = guarded.ctx.read_encoder_angle()?;
    let delta_return = (angle_final - angle_before).abs();
    println!("Zero return delta: {delta_return:.2}°");
    if delta_return > ZERO_RETURN_TOLERANCE_DEGREES {
        println!("Warning: did not return exactly to zero (delta {delta_return:.2}°)");
    }

    guarded.ctx.stop_motor()?;
    Ok(())
}

/// Test estimate_move_duration hardware validation
#[test]
fn test_estimate_move_duration() -> TestResult<()> {
    init_env();
    let _guard = TEST_MUTEX.lock().unwrap();

    let mut ctx = TestContext::new()?;
    let guarded = AutoStopGuard { ctx: &mut ctx };

    guarded.ctx.setup_motor()?;

    let speed = 8u8;
    let usteps = 4u8;
    let pulses = mks_servo42_rs::angle_to_steps(TEST_ROTATIONS_DEGREES, usteps as f32);
    println!("Testing speed={speed}, pulses={pulses}, μsteps={usteps}");

    guarded.ctx.set_subdivision(usteps)?;

    let est_secs =
        mks_servo42_rs::estimate_move_duration(speed, pulses, u16::from(usteps)).as_secs_f64();
    println!("Estimated: {est_secs:.3}s");

    let counts_before = guarded.ctx.encoder_count();
    let pulses_before = guarded.ctx.read_pulse_count()?;

    let start = std::time::Instant::now();
    guarded
        .ctx
        .run_motor(RotationDirection::Clockwise, speed, pulses)?;

    let actual_secs =
        guarded
            .ctx
            .wait_for_move_complete(counts_before, start, MOTOR_STOP_TIMEOUT)?;

    let angle = guarded.ctx.encoder_delta_degrees(counts_before);
    let mks_pulses = (guarded.ctx.read_pulse_count()? - pulses_before).abs();
    let delta_secs = (actual_secs - est_secs).abs();

    println!(
        "Actual: {actual_secs:.3}s  |  Ext encoder: {angle:.1}°  |  MKS pulses: {mks_pulses}  |  Delta: {delta_secs:.3}s"
    );

    if delta_secs > MOVE_DURATION_TOLERANCE_SECS {
        return Err(TestError::Servo(format!(
            "Motor took {actual_secs:.3}s, estimated {est_secs:.3}s (delta {delta_secs:.3}s)"
        )));
    }

    println!("Estimate move duration OK");
    Ok(())
}
