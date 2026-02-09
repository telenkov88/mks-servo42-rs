# mks-servo42-rs

A generic, `no_std` Rust driver for **MKS SERVO42** closed-loop stepper motors.

This library provides a type-safe interface for generating the serial protocol commands used by the MKS SERVO42C firmware (V1.0+). It is transport-agnostic, meaning it generates byte buffers that you can send over any serial interface (UART, USB-Serial, etc.).

## Features

- **`no_std` compatible** - Works in embedded environments
- **Transport-agnostic** - Generates raw command bytes, you handle the serial communication
- **UART mode focused** - Designed for `CR_UART` control mode (closed-loop UART interface)
- **Type-safe** - Rust enums for directions, modes, and status values
- **Response parsing** - Helpers for parsing encoder values, shaft status, and other responses

## Supported Commands

### Read Commands
| Command | Code | Description |
|---------|------|-------------|
| `read_encoder_value` | 0x30 | Read encoder position (carry + value) |
| `read_pulse_count` | 0x33 | Read received pulse count |
| `read_motor_shaft_angle` | 0x36 | Read motor shaft angle |
| `read_motor_shaft_angle_error` | 0x39 | Read shaft angle error |
| `read_en_pin_status` | 0x3A | Read EN pin status |
| `read_release_status` | 0x3D | Read release status |
| `read_shaft_status` | 0x3E | Read shaft blocked/unblocked status |

### Configuration Commands
| Command | Code | Description |
|---------|------|-------------|
| `calibrate_encoder` | 0x80 | Calibrate encoder (motor must be unloaded) |
| `set_current_limit` | 0x83 | Set current limit (0-15 → 0-3000mA) |
| `set_subdivision` | 0x84 | Set microstepping (1-256) |
| `set_enable_logic` | 0x85 | Set EN pin logic (Low/High/AlwaysOn) |
| `set_direction` | 0x86 | Set default rotation direction |
| `set_auto_screen_off` | 0x87 | Enable/disable auto screen off |
| `set_stall_protection` | 0x88 | Enable/disable stall protection |
| `set_interpolation` | 0x89 | Enable/disable step interpolation |

### Zero Mode Commands
| Command | Code | Description |
|---------|------|-------------|
| `set_zero_mode` | 0x90 | Set zero mode (Disable/DirMode/NearMode) |
| `set_current_as_zero` | 0x91 | Set current position as zero |
| `set_zero_speed` | 0x92 | Set return-to-zero speed (0-4) |
| `set_zero_direction` | 0x93 | Set return-to-zero direction |
| `go_to_zero` | 0x94 | Trigger return to zero |

### PID/Motion Commands
| Command | Code | Description |
|---------|------|-------------|
| `set_position_kp` | 0xA1 | Set position Kp (default 0x650) |
| `set_position_ki` | 0xA2 | Set position Ki (default 1) |
| `set_position_kd` | 0xA3 | Set position Kd (default 0x650) |
| `set_acceleration` | 0xA4 | Set acceleration (default 0x11E) |
| `set_max_torque` | 0xA5 | Set max torque (0-0x4B0) |

### Motor Control Commands
| Command | Code | Description |
|---------|------|-------------|
| `enable_motor` | 0xF3 | Enable/disable motor in UART mode |
| `run_with_constant_speed` | 0xF6 | Run at constant speed with direction |
| `stop` | 0xF7 | Stop motor immediately |
| `run_motor` | 0xFD | Move relative pulses at speed |
| `save_clear_status` | 0xFF | Save (0xC8) or clear (0xCA) status |

## Intentionally Unsupported Commands

These commands are intentionally not implemented as this crate is designed for **UART mode only**:

| Command | Code | Reason |
|---------|------|--------|
| Set motor type | 0x81 | Hardware config, set via screen |
| Set work mode | 0x82 | This crate assumes CR_UART mode |
| Set baud rate | 0x8A | Changing baud rate would break communication |
| Set UART address | 0x8B | Address change requires reconnection |
| Restore defaults | 0x3F | Would reset to non-UART mode |

## Usage Example

```rust
use mks_servo42_rs::{Driver, RotationDirection};

let mut driver = Driver::default();

// Enable motor
let cmd = driver.enable_motor(true);
// Send `cmd` bytes over your serial interface...

// Run at constant speed
let cmd = driver.run_with_constant_speed(RotationDirection::Clockwise, 10)?;

// Move 1000 pulses
let cmd = driver.run_motor(RotationDirection::Clockwise, 20, 1000)?;

// Stop
let cmd = driver.stop();
```

## License

MIT or Apache-2.0
