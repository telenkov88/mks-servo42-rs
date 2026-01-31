use dotenv::dotenv;
use serial::{SerialPort, SerialPortSettings};
use std::env;
use std::fmt;
use std::io::Error as IoError;
use std::thread;
use std::time::Duration;

const SUBDIVISION_CODE: u8 = 4;
const MICROSTEPS: f32 = 4.0; // Index 4/16 results in 4 microsteps/step

enum Error {
    Servo(mks_servo42_rs::Error),
    Io(IoError),
    Serial(serial::Error),
    Protocol(String),
}

impl fmt::Debug for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Servo(servo) => write!(f, "Servo {{ {:?} }}", servo.as_str()),
            Self::Io(io) => write!(f, "Io {{ {io:?} }}"),
            Self::Serial(serial) => write!(f, "Serial {{ {serial:?} }}"),
            Self::Protocol(msg) => write!(f, "Protocol {{ {msg} }}"),
        }
    }
}

impl From<mks_servo42_rs::Error> for Error {
    fn from(other: mks_servo42_rs::Error) -> Self {
        Self::Servo(other)
    }
}

impl From<IoError> for Error {
    fn from(other: IoError) -> Self {
        Self::Io(other)
    }
}

impl From<serial::Error> for Error {
    fn from(other: serial::Error) -> Self {
        Self::Serial(other)
    }
}

fn send_command(s: &mut impl SerialPort, cmd: &[u8]) -> Result<(), Error> {
    println!("TX: {cmd:02x?}");
    s.write_all(cmd)?;
    Ok(())
}

fn read_response(s: &mut impl SerialPort) -> Result<Vec<u8>, Error> {
    let mut buf = [0u8; 256];
    match s.read(&mut buf) {
        Ok(n) if n > 0 => {
            let response = buf[..n].to_vec();
            println!("RX: {:02x?}", response);
            Ok(response)
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
            println!("RX Error: {e:?}");
            Err(Error::Io(e))
        }
    }
}

fn parse_encoder(data: &[u8]) -> Result<f32, Error> {
    match mks_servo42_rs::parse_encoder_response(data) {
        Ok(ev) => Ok(ev.to_degrees()),
        Err(e) => Err(Error::Protocol(format!("Parse error: {:?}", e.as_str()))),
    }
}

fn main() -> Result<(), Error> {
    dotenv().ok();

    let mks_env_servo42c_uart = env::var("MKS_ENV_SERVO42C_UART")
        .expect("MKS_ENV_SERVO42C_UART path to serial port must be set");

    let mut driver = mks_servo42_rs::Driver::default();

    println!("Connecting to serial port: {}", mks_env_servo42c_uart);
    let mut s = serial::open(&mks_env_servo42c_uart)?;

    s.reconfigure(&|port: &mut dyn SerialPortSettings| {
        port.set_baud_rate(serial::Baud38400)?;
        port.set_char_size(serial::Bits8);
        port.set_parity(serial::ParityNone);
        port.set_stop_bits(serial::Stop1);
        port.set_flow_control(serial::FlowNone);
        Ok(())
    })?;

    s.set_timeout(Duration::from_millis(100))?;

    let pause_short = Duration::from_millis(500);
    let pause_move = Duration::from_secs(2);

    println!("--- Setup ---");
    println!("Setting Work Mode to UART...");
    send_command(&mut s, driver.set_work_mode(mks_servo42_rs::WorkMode::Uart))?;
    thread::sleep(pause_short);
    let _ = read_response(&mut s);

    println!(
        "Setting subdivision to {} (Microsteps: {})...",
        SUBDIVISION_CODE, MICROSTEPS
    );
    send_command(&mut s, driver.set_subdivision(SUBDIVISION_CODE)?)?;
    thread::sleep(pause_short);
    let _ = read_response(&mut s);

    println!("Enabling Motor...");
    send_command(&mut s, driver.enable_motor(true))?;
    thread::sleep(pause_short);
    let _ = read_response(&mut s);

    println!("Step 1: Homing / Go to Zero...");
    send_command(&mut s, driver.set_zero_speed(0x01)?)?;
    thread::sleep(Duration::from_millis(100));
    send_command(&mut s, driver.go_to_zero())?;

    thread::sleep(pause_move);
    let _ = read_response(&mut s);

    println!("Reading Start Encoder Value...");
    send_command(&mut s, driver.read_encoder_value())?;
    thread::sleep(pause_short);
    let start_resp = read_response(&mut s)?;
    if start_resp.is_empty() {
        return Err(Error::Protocol(
            "No response for start encoder value".into(),
        ));
    }
    let start_angle = parse_encoder(&start_resp)?;
    println!("Start Angle: {:.2}", start_angle);

    let target_angle = 360.0;
    let pulses = mks_servo42_rs::angle_to_steps(target_angle, MICROSTEPS);
    println!(
        "Step 2: Moving {:.2} degrees ({} steps)...",
        target_angle, pulses
    );

    send_command(
        &mut s,
        driver.run_position(mks_servo42_rs::direction::Direction::Forward, 1, pulses)?,
    )?;

    thread::sleep(pause_move);
    let _ = read_response(&mut s);

    println!("Step 3: Reading Encoder Value...");
    send_command(&mut s, driver.read_encoder_value())?;
    thread::sleep(pause_short);

    let response = read_response(&mut s)?;

    let end_angle = parse_encoder(&response)?;
    println!("End Angle: {:.2}", end_angle);

    let moved_angle = (end_angle - start_angle).abs();

    println!("Moved Angle: {:.2} degrees", moved_angle);

    let tolerance = target_angle * 0.10;
    let diff = (moved_angle - target_angle).abs();

    if diff <= tolerance {
        println!("SUCCESS: Position within tolerance (diff: {:.2} deg)", diff);
    } else {
        println!(
            "WARNING: Position outside tolerance! (diff: {:.2} deg, expected: {:.2}, got: {:.2})",
            diff, target_angle, moved_angle
        );
    }

    println!("------------ Test 2 ------------");
    println!(
        "Setting subdivision to {} (Microsteps: {})...",
        SUBDIVISION_CODE, MICROSTEPS
    );
    send_command(&mut s, driver.set_subdivision(SUBDIVISION_CODE)?)?;
    thread::sleep(pause_short);
    let _ = read_response(&mut s);

    println!("Enabling Motor...");
    send_command(&mut s, driver.enable_motor(true))?;
    thread::sleep(pause_short);
    let _ = read_response(&mut s);

    println!("Step 1: Homing / Go to Zero...");
    send_command(&mut s, driver.set_zero_speed(0x01)?)?;
    thread::sleep(Duration::from_millis(100));
    send_command(&mut s, driver.go_to_zero())?;

    thread::sleep(pause_move);
    let _ = read_response(&mut s);

    println!("Reading Start Encoder Value...");
    send_command(&mut s, driver.read_encoder_value())?;
    thread::sleep(pause_short);
    let start_resp = read_response(&mut s)?;
    if start_resp.is_empty() {
        return Err(Error::Protocol(
            "No response for start encoder value".into(),
        ));
    }
    let start_angle = parse_encoder(&start_resp)?;
    println!("Start Angle: {:.2}", start_angle);

    let target_angle = 360.0;
    let pulses = mks_servo42_rs::angle_to_steps(target_angle, MICROSTEPS);
    println!(
        "Step 2: Moving {:.2} degrees ({} steps)...",
        target_angle, pulses
    );

    send_command(
        &mut s,
        driver.run_position(mks_servo42_rs::direction::Direction::Reverse, 1, pulses)?,
    )?;

    thread::sleep(pause_move);
    let _ = read_response(&mut s);

    println!("Step 3: Reading Encoder Value...");
    send_command(&mut s, driver.read_encoder_value())?;
    thread::sleep(pause_short);

    let response = read_response(&mut s)?;

    let end_angle = parse_encoder(&response)?;
    println!("End Angle: {:.2}", end_angle);

    let moved_angle = (end_angle - start_angle).abs();

    println!("Moved Angle: {:.2} degrees", moved_angle);

    let tolerance = target_angle * 0.10;
    let diff = (moved_angle - target_angle).abs();

    if diff <= tolerance {
        println!("SUCCESS: Position within tolerance (diff: {:.2} deg)", diff);
    } else {
        println!(
            "WARNING: Position outside tolerance! (diff: {:.2} deg, expected: {:.2}, got: {:.2})",
            diff, target_angle, moved_angle
        );
    }

    Ok(())
}
