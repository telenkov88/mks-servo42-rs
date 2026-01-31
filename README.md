# mks-servo42-rs

A generic, `no_std` Rust driver for **MKS SERVO42** closed-loop stepper motors.

This library provides a type-safe interface for generating the serial protocol commands used by the MKS SERVO42C firmware (V1.0+). It is transport-agnostic, meaning it generates byte buffers that you can send over any serial interface (UART, USB-Serial, etc.).

Usage:
```rust
use mks_servo42::{Driver, Direction, MotorType};

fn main() {
    // 1. Create a driver instance (default address 0xE0)
    let mut motor = Driver::default();

    // 2. Configure motor (e.g., set as 1.8° stepper)
    let config_cmd = motor.set_motor_type(MotorType::Deg18).unwrap();
    // write_serial(config_cmd); 

    // 3. Move motor: Forward at speed 20
    let move_cmd = motor.run_speed(Direction::Forward, 20).unwrap();
    // write_serial(move_cmd);
}
```

## License
MIT or Apache-2.0
