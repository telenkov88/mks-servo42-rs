# mks-servo42-rs

A generic, `no_std` Rust driver for **MKS SERVO42** closed-loop stepper motors.

This library provides a type-safe interface for generating the serial protocol commands used by the MKS SERVO42C firmware (V1.0+). It is transport-agnostic, meaning it generates byte buffers that you can send over any serial interface (UART, USB-Serial, etc.).

## License
MIT or Apache-2.0
