//! Basic example demonstrating MKS SERVO42 motor control via UART.
//!
//! This example shows how to:
//! - Connect to the motor via serial port
//! - Configure microstepping
//! - Move the motor to specific positions
//! - Read encoder feedback
//!
//! Set the `MKS_ENV_SERVO42C_UART` environment variable to your serial port path.

use mks_servo42_rs::{Driver, RotationDirection};
use serial::{SerialPort, SerialPortSettings};
use std::env;
use std::thread;
use std::time::Duration;

/// Microstepping configuration (index 4 = 4 microsteps per step)
const MICROSTEPS: u8 = 4;

fn main() {
    dotenv::dotenv().ok();

    // Get serial port from environment
    let port_path = env::var("MKS_ENV_SERVO42C_UART")
        .expect("Set MKS_ENV_SERVO42C_UART to your serial port path");

    println!("Connecting to: {}", port_path);

    // Open and configure serial port
    let mut port = serial::open(&port_path).expect("Failed to open serial port");
    port.reconfigure(&|settings: &mut dyn SerialPortSettings| {
        settings.set_baud_rate(serial::Baud38400)?;
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    })
    .expect("Failed to configure serial port");
    port.set_timeout(Duration::from_millis(100)).unwrap();

    let mut driver = Driver::default();

    // === Setup ===
    println!("\n=== Setup ===");

    send(&mut port, driver.set_subdivision(MICROSTEPS).unwrap());
    send(&mut port, driver.enable_motor(true));

    // Go to zero position
    send(&mut port, driver.set_zero_speed(1).unwrap());
    send(&mut port, driver.go_to_zero());
    thread::sleep(Duration::from_secs(2));

    // === Move 360° clockwise ===
    println!("\n=== Move 360° Clockwise ===");

    let start_angle = read_encoder(&mut port, &mut driver);
    println!("Start: {:.1}°", start_angle);

    let pulses = mks_servo42_rs::angle_to_steps(360.0, MICROSTEPS as f32);
    send(
        &mut port,
        driver
            .run_motor(RotationDirection::Clockwise, 1, pulses)
            .unwrap(),
    );
    thread::sleep(Duration::from_secs(3));

    let end_angle = read_encoder(&mut port, &mut driver);
    println!("End: {:.1}°", end_angle);
    println!("Moved: {:.1}°", (end_angle - start_angle).abs());

    // === Move 360° counter-clockwise ===
    println!("\n=== Move 360° Counter-Clockwise ===");

    let start_angle = read_encoder(&mut port, &mut driver);
    send(
        &mut port,
        driver
            .run_motor(RotationDirection::CounterClockwise, 1, pulses)
            .unwrap(),
    );
    thread::sleep(Duration::from_secs(3));

    let end_angle = read_encoder(&mut port, &mut driver);
    println!("Moved: {:.1}°", (end_angle - start_angle).abs());

    // === Cleanup ===
    println!("\n=== Done ===");
    send(&mut port, driver.stop());
    send(&mut port, driver.enable_motor(false));
}

/// Send command and read response
fn send<S: SerialPort + std::io::Read + std::io::Write>(port: &mut S, cmd: &[u8]) -> Vec<u8> {
    println!("TX: {:02x?}", cmd);
    port.write_all(cmd).expect("Write failed");
    thread::sleep(Duration::from_millis(100));

    let mut buf = [0u8; 64];
    match port.read(&mut buf) {
        Ok(n) if n > 0 => {
            println!("RX: {:02x?}", &buf[..n]);
            buf[..n].to_vec()
        }
        _ => Vec::new(),
    }
}

/// Read encoder and return angle in degrees
fn read_encoder(port: &mut impl SerialPort, driver: &mut Driver) -> f32 {
    let response = send(port, driver.read_encoder_value());
    mks_servo42_rs::parse_encoder_response(&response)
        .expect("Failed to parse encoder")
        .to_degrees()
}
