//! Test utilities for MKS SERVO42 E2E tests

use mks_servo42_rs::Driver;
use serial::{SerialPort, SerialPortSettings};
use std::env;
use std::io::{Read, Write};
use std::thread;
use std::time::Duration;

use lazy_static::lazy_static;
use std::sync::Mutex;

use dotenvy::dotenv;
pub fn init_env() {
    // Load .env file silently (ignore if not found)
    let _ = dotenv().ok();
}

// Global mutex to ensure tests run serially
lazy_static! {
    pub static ref TEST_MUTEX: Mutex<()> = Mutex::new(());
}

/// Default baud rate for tests (matches examples/base.rs)
pub const DEFAULT_BAUD_RATE: serial::BaudRate = serial::Baud38400;

/// Default timeout for serial operations
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(500);

/// Short pause between commands
pub const SHORT_PAUSE: Duration = Duration::from_millis(100);

/// Long pause for movement operations
pub const LONG_PAUSE: Duration = Duration::from_secs(2);

/// Test result type
pub type TestResult<T> = Result<T, TestError>;

/// Test error type
#[derive(Debug)]
#[allow(dead_code)]
pub enum TestError {
    Serial(String),
    Servo(String),
    Protocol(String),
    Safety(String),
}

impl From<&str> for TestError {
    fn from(err: &str) -> Self {
        Self::Safety(err.to_string())
    }
}

impl From<String> for TestError {
    fn from(err: String) -> Self {
        Self::Safety(err)
    }
}

impl From<std::io::Error> for TestError {
    fn from(err: std::io::Error) -> Self {
        Self::Safety(err.to_string())
    }
}

impl From<serial::Error> for TestError {
    fn from(err: serial::Error) -> Self {
        Self::Serial(err.to_string())
    }
}

impl From<mks_servo42_rs::Error> for TestError {
    fn from(err: mks_servo42_rs::Error) -> Self {
        Self::Servo(format!("{:?}", err))
    }
}

/// Serial port wrapper for testing
pub struct TestSerialPort {
    port: Box<dyn SerialPort + Send>,
}

impl std::fmt::Debug for TestSerialPort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TestSerialPort").finish()
    }
}

impl TestSerialPort {
    /// Open serial port from environment variable
    pub fn from_env() -> TestResult<Self> {
        let port_path = env::var("MKS_ENV_SERVO42C_UART").map_err(|_| {
            TestError::Protocol("MKS_ENV_SERVO42C_UART environment variable not set".into())
        })?;

        println!("Connecting to serial port: {}", port_path);

        let mut port = serial::open(&port_path)?;

        port.reconfigure(&|settings: &mut dyn SerialPortSettings| {
            settings.set_baud_rate(DEFAULT_BAUD_RATE)?;
            settings.set_char_size(serial::Bits8);
            settings.set_parity(serial::ParityNone);
            settings.set_stop_bits(serial::Stop1);
            settings.set_flow_control(serial::FlowNone);
            Ok(())
        })?;

        port.set_timeout(DEFAULT_TIMEOUT)?;

        let boxed_port: Box<dyn SerialPort + Send> = Box::new(port);

        Ok(Self { port: boxed_port })
    }

    /// Clear input buffer logic
    pub fn clear_input_buffer(&mut self) -> TestResult<()> {
        self.port.set_timeout(Duration::from_millis(50))?;
        let mut buffer = [0u8; 1024];
        loop {
            match self.port.read(&mut buffer) {
                Ok(n) if n > 0 => {
                    println!("Drained {} bytes: {:02x?}", n, &buffer[..n]);
                }
                _ => break,
            }
        }
        self.port.set_timeout(DEFAULT_TIMEOUT)?;
        Ok(())
    }

    /// Send command and log it
    pub fn send_command(&mut self, command: &[u8]) -> TestResult<()> {
        // Drain any pending bytes before sending new command
        self.clear_input_buffer()?;
        println!("TX: {:02x?}", command);
        self.port.write_all(command)?;
        Ok(())
    }

    /// Read response with timeout handling and robustness
    pub fn read_response(&mut self) -> TestResult<Vec<u8>> {
        let mut buffer = [0u8; 256];
        match self.port.read(&mut buffer) {
            Ok(n) if n > 0 => {
                let raw_response = buffer[..n].to_vec();
                println!("RX Raw: {:02x?}", raw_response);

                // Find start of valid frame (0xE0..0xE9 are valid addresses)
                // This accounts for leading garbage.
                if let Some(start_idx) = raw_response
                    .iter()
                    .position(|&b| (0xE0..=0xE9).contains(&b))
                {
                    let response = raw_response[start_idx..].to_vec();
                    if start_idx > 0 {
                        println!(
                            "RX Cleaned (stripped {} bytes): {:02x?}",
                            start_idx, response
                        );
                    } else {
                        println!("RX Clean: {:02x?}", response);
                    }
                    Ok(response)
                } else {
                    println!("RX (No Header): {:02x?}", raw_response);
                    Ok(raw_response)
                }
            }
            Ok(_) => {
                println!("RX: (empty)");
                Ok(Vec::new())
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                println!("RX: (timeout)");
                Ok(Vec::new())
            }
            Err(e) => {
                println!("RX Error: {:?}", e);
                Err(TestError::Safety(e.to_string()))
            }
        }
    }

    /// Send command and read response with pause
    pub fn send_and_read(&mut self, command: &[u8]) -> TestResult<Vec<u8>> {
        self.send_command(command)?;
        thread::sleep(SHORT_PAUSE);
        self.read_response()
    }

    /// Send command, pause, and discard response
    pub fn send_only(&mut self, command: &[u8]) -> TestResult<()> {
        self.send_command(command)?;
        thread::sleep(SHORT_PAUSE);
        let _ = self.read_response();
        Ok(())
    }
}

/// Test context containing driver and serial port
#[derive(Debug)]
pub struct TestContext {
    pub driver: Driver,
    pub serial: TestSerialPort,
}

impl TestContext {
    /// Create new test context
    pub fn new() -> TestResult<Self> {
        let driver = Driver::default();
        let serial = TestSerialPort::from_env()?;

        Ok(Self { driver, serial })
    }

    /// Reset driver state (create new instance)
    #[allow(dead_code)]
    pub fn reset_driver(&mut self) {
        self.driver = Driver::default();
    }
}

/// Helper to parse encoder response
#[allow(dead_code)]
pub fn parse_encoder_response(data: &[u8]) -> TestResult<f32> {
    match mks_servo42_rs::parse_encoder_response(data) {
        Ok(encoder_value) => Ok(encoder_value.to_degrees()),
        Err(e) => Err(TestError::Protocol(format!(
            "Parse error: {:?}",
            e.as_str()
        ))),
    }
}

/// Helper to parse motor shaft angle response
#[allow(dead_code)]
pub fn parse_motor_shaft_angle_response(data: &[u8]) -> TestResult<f32> {
    match mks_servo42_rs::parse_motor_shaft_angle_response(data) {
        Ok(angle) => Ok(angle.to_degrees()),
        Err(e) => Err(TestError::Protocol(format!(
            "Parse error: {:?}",
            e.as_str()
        ))),
    }
}

/// Helper to parse motor shaft angle error response
#[allow(dead_code)]
pub fn parse_motor_shaft_angle_error_response(data: &[u8]) -> TestResult<f32> {
    match mks_servo42_rs::parse_motor_shaft_angle_error(data) {
        Ok(error) => Ok(error.to_degrees()),
        Err(e) => Err(TestError::Protocol(format!(
            "Parse error: {:?}",
            e.as_str()
        ))),
    }
}

/// Helper to parse EN pin status response
#[allow(dead_code)]
pub fn parse_en_pin_status_response(data: &[u8]) -> TestResult<mks_servo42_rs::EnPinStatus> {
    match mks_servo42_rs::parse_en_pin_status_response(data) {
        Ok(status) => Ok(status),
        Err(e) => Err(TestError::Protocol(format!(
            "Parse error: {:?}",
            e.as_str()
        ))),
    }
}

/// Helper to parse shaft status response
#[allow(dead_code)]
pub fn parse_shaft_status_response(data: &[u8]) -> TestResult<mks_servo42_rs::ShaftStatus> {
    match mks_servo42_rs::parse_shaft_status_response(data) {
        Ok(status) => Ok(status),
        Err(e) => Err(TestError::Protocol(format!("Parse error: {:?}", e))),
    }
}

/// Check if response indicates success
#[allow(dead_code)]
pub fn check_success_response(data: &[u8]) -> TestResult<bool> {
    if data.len() >= 3 {
        // Response format: [address, 0x01 (success), checksum]
        Ok(data[1] == 0x01)
    } else {
        Err(TestError::from("Response too short"))
    }
}
