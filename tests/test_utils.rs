//! Test utilities for MKS SERVO42 E2E tests

use encoder_client::EncoderClient;
use mks_servo42_rs::{Driver, EnLogic, RotationDirection, SaveClearStatus, ZeroMode};
use serial::{SerialPort, SerialPortSettings};
use std::env;
use std::io::{Read, Write};
use std::thread;
use std::time::{Duration, Instant};

use lazy_static::lazy_static;
use std::sync::Mutex;

use dotenvy::dotenv;
pub fn init_env() {
    let _ = dotenv().ok();
}

lazy_static! {
    pub static ref TEST_MUTEX: Mutex<()> = Mutex::new(());
}

pub const DEFAULT_BAUD_RATE: serial::BaudRate = serial::Baud38400;
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(500);
pub const SHORT_PAUSE: Duration = Duration::from_millis(200);

#[allow(dead_code)]
pub const LONG_PAUSE: Duration = Duration::from_secs(2);

pub type TestResult<T> = Result<T, TestError>;

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

pub struct TestSerialPort {
    port: Box<dyn SerialPort + Send>,
}

impl std::fmt::Debug for TestSerialPort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TestSerialPort").finish()
    }
}

impl TestSerialPort {
    pub fn from_env() -> TestResult<Self> {
        let port_path = env::var("MKS_ENV_SERVO42C_UART")
            .map_err(|_| TestError::Protocol("MKS_ENV_SERVO42C_UART env var not set".into()))?;
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
        Ok(Self {
            port: Box::new(port),
        })
    }

    pub fn send_and_read(&mut self, command: &[u8]) -> TestResult<Vec<u8>> {
        self.port.set_timeout(Duration::from_millis(20))?;
        let mut drain_buf = [0u8; 64];
        while let Ok(n) = self.port.read(&mut drain_buf) {
            if n == 0 {
                break;
            }
            println!("Pre-TX drain: {:02x?}", &drain_buf[..n]);
        }
        self.port.set_timeout(DEFAULT_TIMEOUT)?;
        println!("TX: {:02x?}", command);
        self.port.write_all(command)?;
        thread::sleep(SHORT_PAUSE);
        self.read_response()
    }

    pub fn send_only(&mut self, command: &[u8]) -> TestResult<()> {
        println!("TX: {:02x?}", command);
        self.port.write_all(command)?;
        thread::sleep(SHORT_PAUSE);
        let _ = self.read_response();
        Ok(())
    }

    fn read_response(&mut self) -> TestResult<Vec<u8>> {
        let mut buffer = [0u8; 256];
        match self.port.read(&mut buffer) {
            Ok(n) if n > 0 => {
                let raw = buffer[..n].to_vec();
                println!("RX Raw: {:02x?}", raw);
                if let Some(start) = raw.iter().position(|&b| (0xE0..=0xE9).contains(&b)) {
                    let response = raw[start..].to_vec();
                    if start > 0 {
                        println!("RX Cleaned (stripped {} bytes): {:02x?}", start, response);
                    } else {
                        println!("RX Clean: {:02x?}", response);
                    }
                    Ok(response)
                } else {
                    println!("RX (No Header): {:02x?}", raw);
                    Ok(raw)
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
            Err(e) => Err(TestError::Safety(e.to_string())),
        }
    }
}

fn require_ack(response: &[u8], context: &str) -> TestResult<()> {
    if response.len() >= 3 && response[1] == 0x01 {
        Ok(())
    } else {
        Err(TestError::Protocol(format!(
            "{} failed: {:02x?}",
            context, response
        )))
    }
}

#[derive(Debug)]
pub struct TestContext {
    pub driver: Driver,
    pub serial: TestSerialPort,
    pub encoder: EncoderClient,
    pub encoder_channel: usize,
    pub encoder_steps_per_rev: i32,
}

impl TestContext {
    pub fn new() -> TestResult<Self> {
        let driver = Driver::default();
        let serial = TestSerialPort::from_env()?;

        let encoder_port = env::var("PICO_ENCODER_UART")
            .map_err(|_| TestError::Protocol("PICO_ENCODER_UART env var not set".into()))?;
        let encoder_channel = env::var("MKS_ENCODER_CHANNEL")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(4);
        let encoder_steps_per_rev = env::var("PICO_ENCODER_STEPS_PER_REVOLUTION")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(800);

        println!("Starting external encoder on {}...", encoder_port);
        let encoder = EncoderClient::spawn(&encoder_port).map_err(|e| {
            TestError::Protocol(format!(
                "Failed to connect encoder on {}: {}",
                encoder_port, e
            ))
        })?;
        thread::sleep(Duration::from_millis(200));

        Ok(Self {
            driver,
            serial,
            encoder,
            encoder_channel,
            encoder_steps_per_rev,
        })
    }

    #[allow(dead_code)]
    pub fn reset_driver(&mut self) {
        self.driver = Driver::default();
    }

    // ── External encoder ────────────────────────────────────────────────────

    pub fn encoder_count(&self) -> i32 {
        self.encoder.get_counts()[self.encoder_channel]
    }

    pub fn encoder_delta_degrees(&self, counts_before: i32) -> f64 {
        (self.encoder_count() - counts_before).abs() as f64 / self.encoder_steps_per_rev as f64
            * 360.0
    }

    pub fn wait_for_move_complete(
        &self,
        baseline: i32,
        start: Instant,
        timeout: Duration,
    ) -> TestResult<f64> {
        let motion_deadline = start + Duration::from_secs(5);
        loop {
            if (self.encoder_count() - baseline).abs() >= 2 {
                break; // motion already started (or finished for fast moves)
            }
            if Instant::now() > motion_deadline {
                return Err(TestError::Servo(
                    "External encoder: motion never detected within 5s".into(),
                ));
            }
            thread::sleep(Duration::from_millis(50));
        }
        let mut prev = self.encoder_count();
        loop {
            if start.elapsed() > timeout {
                return Err(TestError::Servo("Timeout waiting for motor to stop".into()));
            }
            thread::sleep(Duration::from_millis(50));
            let cur = self.encoder_count();
            if (cur - prev).abs() < 2 {
                break;
            }
            prev = cur;
        }
        Ok(start.elapsed().as_secs_f64())
    }

    // ── Motor enable / stop ─────────────────────────────────────────────────

    pub fn enable_motor(&mut self) -> TestResult<()> {
        self.serial.send_only(self.driver.enable_motor(true))
    }

    pub fn disable_motor(&mut self) -> TestResult<()> {
        self.serial.send_only(self.driver.enable_motor(false))
    }

    pub fn stop_motor(&mut self) -> TestResult<()> {
        self.serial.send_only(self.driver.stop())
    }

    pub fn setup_motor(&mut self) -> TestResult<()> {
        let cmd = self.driver.set_stall_protection(false);
        self.serial.send_and_read(cmd)?;
        self.enable_motor()
    }

    // ── Configuration setters ───────────────────────────────────────────────

    pub fn set_subdivision(&mut self, usteps: u8) -> TestResult<()> {
        let cmd = self.driver.set_subdivision(usteps)?;
        let resp = self.serial.send_and_read(cmd)?;
        require_ack(&resp, "set_subdivision")
    }

    pub fn set_max_torque(&mut self, torque: u16) -> TestResult<()> {
        let cmd = self.driver.set_max_torque(torque)?;
        let resp = self.serial.send_and_read(cmd)?;
        require_ack(&resp, "set_max_torque")
    }

    pub fn set_position_kp(&mut self, kp: u16) -> TestResult<()> {
        let resp = self.serial.send_and_read(self.driver.set_position_kp(kp))?;
        require_ack(&resp, "set_position_kp")
    }

    pub fn set_position_ki(&mut self, ki: u16) -> TestResult<()> {
        let resp = self.serial.send_and_read(self.driver.set_position_ki(ki))?;
        require_ack(&resp, "set_position_ki")
    }

    pub fn set_position_kd(&mut self, kd: u16) -> TestResult<()> {
        let resp = self.serial.send_and_read(self.driver.set_position_kd(kd))?;
        require_ack(&resp, "set_position_kd")
    }

    pub fn set_acceleration(&mut self, acc: u16) -> TestResult<()> {
        let resp = self
            .serial
            .send_and_read(self.driver.set_acceleration(acc))?;
        require_ack(&resp, "set_acceleration")
    }

    pub fn set_current_limit(&mut self, index: u8) -> TestResult<()> {
        let cmd = self.driver.set_current_limit(index)?;
        let resp = self.serial.send_and_read(cmd)?;
        require_ack(&resp, "set_current_limit")
    }

    pub fn set_enable_logic(&mut self, logic: EnLogic) -> TestResult<()> {
        let resp = self
            .serial
            .send_and_read(self.driver.set_enable_logic(logic))?;
        require_ack(&resp, "set_enable_logic")
    }

    pub fn set_direction(&mut self, dir: RotationDirection) -> TestResult<()> {
        let resp = self.serial.send_and_read(self.driver.set_direction(dir))?;
        require_ack(&resp, "set_direction")
    }

    pub fn set_auto_screen_off(&mut self, on: bool) -> TestResult<()> {
        let resp = self
            .serial
            .send_and_read(self.driver.set_auto_screen_off(on))?;
        require_ack(&resp, "set_auto_screen_off")
    }

    pub fn set_stall_protection(&mut self, on: bool) -> TestResult<()> {
        let resp = self
            .serial
            .send_and_read(self.driver.set_stall_protection(on))?;
        require_ack(&resp, "set_stall_protection")
    }

    pub fn set_interpolation(&mut self, on: bool) -> TestResult<()> {
        let resp = self
            .serial
            .send_and_read(self.driver.set_interpolation(on))?;
        require_ack(&resp, "set_interpolation")
    }

    // ── Zero mode ───────────────────────────────────────────────────────────

    pub fn set_zero_mode(&mut self, mode: ZeroMode) -> TestResult<()> {
        let resp = self.serial.send_and_read(self.driver.set_zero_mode(mode))?;
        require_ack(&resp, "set_zero_mode")
    }

    pub fn set_zero_direction(&mut self, dir: RotationDirection) -> TestResult<()> {
        self.serial
            .send_and_read(self.driver.set_zero_direction(dir))?;
        Ok(())
    }

    pub fn set_zero_speed(&mut self, speed: u8) -> TestResult<()> {
        let cmd = self.driver.set_zero_speed(speed)?;
        let resp = self.serial.send_and_read(cmd)?;
        require_ack(&resp, "set_zero_speed")
    }

    pub fn set_current_as_zero(&mut self) -> TestResult<()> {
        self.serial
            .send_and_read(self.driver.set_current_as_zero())?;
        Ok(())
    }

    pub fn go_to_zero(&mut self) -> TestResult<()> {
        self.serial.send_and_read(self.driver.go_to_zero())?;
        Ok(())
    }

    pub fn save_clear_status(&mut self, status: SaveClearStatus) -> TestResult<()> {
        let resp = self
            .serial
            .send_and_read(self.driver.save_clear_status(status))?;
        if resp.is_empty() {
            return Err(TestError::Protocol(
                "No response for save_clear_status".into(),
            ));
        }
        Ok(())
    }

    // ── Movement ────────────────────────────────────────────────────────────

    pub fn run_motor(
        &mut self,
        direction: RotationDirection,
        speed: u8,
        pulses: u32,
    ) -> TestResult<()> {
        let cmd = self.driver.run_motor(direction, speed, pulses)?;
        let response = self.serial.send_and_read(cmd)?;
        require_ack(&response, "run_motor")
    }

    pub fn run_constant_speed(
        &mut self,
        direction: RotationDirection,
        speed: u8,
    ) -> TestResult<()> {
        let cmd = self.driver.run_with_constant_speed(direction, speed)?;
        self.serial.send_only(cmd)
    }

    // ── Reads ────────────────────────────────────────────────────────────────

    pub fn read_pulse_count(&mut self) -> TestResult<i32> {
        let response = self.serial.send_and_read(self.driver.read_pulse_count())?;
        parse_pulse_count_response(&response)
    }

    pub fn read_encoder_angle(&mut self) -> TestResult<f64> {
        let response = self
            .serial
            .send_and_read(self.driver.read_encoder_value())?;
        parse_encoder_response(&response).map(|v| v as f64)
    }

    pub fn read_shaft_angle(&mut self) -> TestResult<f64> {
        let response = self
            .serial
            .send_and_read(self.driver.read_motor_shaft_angle())?;
        parse_motor_shaft_angle_response(&response).map(|v| v as f64)
    }

    pub fn read_shaft_angle_error(&mut self) -> TestResult<f64> {
        let response = self
            .serial
            .send_and_read(self.driver.read_motor_shaft_angle_error())?;
        parse_motor_shaft_angle_error_response(&response).map(|v| v as f64)
    }

    pub fn read_en_pin_status(&mut self) -> TestResult<mks_servo42_rs::EnPinStatus> {
        let response = self
            .serial
            .send_and_read(self.driver.read_en_pin_status())?;
        parse_en_pin_status_response(&response)
    }

    pub fn read_shaft_status(&mut self) -> TestResult<mks_servo42_rs::ShaftStatus> {
        let response = self.serial.send_and_read(self.driver.read_shaft_status())?;
        parse_shaft_status_response(&response)
    }

    pub fn read_release_status(&mut self) -> TestResult<u8> {
        let response = self
            .serial
            .send_and_read(self.driver.read_release_status())?;
        if response.len() >= 3 {
            Ok(response[1])
        } else {
            Err(TestError::Protocol(format!(
                "Invalid release status response: {:02x?}",
                response
            )))
        }
    }
}

#[allow(dead_code)]
pub fn parse_encoder_response(data: &[u8]) -> TestResult<f32> {
    match mks_servo42_rs::parse_encoder_response(data) {
        Ok(v) => Ok(v.to_degrees()),
        Err(e) => Err(TestError::Protocol(format!(
            "Parse error: {:?}",
            e.as_str()
        ))),
    }
}

#[allow(dead_code)]
pub fn parse_motor_shaft_angle_response(data: &[u8]) -> TestResult<f32> {
    match mks_servo42_rs::parse_motor_shaft_angle_response(data) {
        Ok(v) => Ok(v.to_degrees()),
        Err(e) => Err(TestError::Protocol(format!(
            "Parse error: {:?}",
            e.as_str()
        ))),
    }
}

#[allow(dead_code)]
pub fn parse_motor_shaft_angle_error_response(data: &[u8]) -> TestResult<f32> {
    match mks_servo42_rs::parse_motor_shaft_angle_error(data) {
        Ok(v) => Ok(v.to_degrees()),
        Err(e) => Err(TestError::Protocol(format!(
            "Parse error: {:?}",
            e.as_str()
        ))),
    }
}

#[allow(dead_code)]
pub fn parse_pulse_count_response(data: &[u8]) -> TestResult<i32> {
    match mks_servo42_rs::parse_pulse_count_response(data) {
        Ok(pulses) => Ok(pulses),
        Err(e) => Err(TestError::Protocol(format!(
            "Parse error: {:?}",
            e.as_str()
        ))),
    }
}

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

#[allow(dead_code)]
pub fn parse_shaft_status_response(data: &[u8]) -> TestResult<mks_servo42_rs::ShaftStatus> {
    match mks_servo42_rs::parse_shaft_status_response(data) {
        Ok(status) => Ok(status),
        Err(e) => Err(TestError::Protocol(format!("Parse error: {:?}", e))),
    }
}

#[allow(dead_code)]
pub fn check_success_response(data: &[u8]) -> TestResult<bool> {
    if data.len() >= 3 {
        Ok(data[1] == 0x01)
    } else {
        Err(TestError::from("Response too short"))
    }
}
