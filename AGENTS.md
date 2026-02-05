# AGENTS.md - mks-servo42-rs

This document provides guidelines for AI agents and developers working on the `mks-servo42-rs` project - a generic, `no_std` Rust driver for MKS SERVO42 closed-loop stepper motors.

## Project Overview

- **Name**: mks-servo42-rs
- **Description**: Type-safe interface for generating serial protocol commands for MKS SERVO42C firmware (V1.0+)
- **Type**: `no_std` Rust library (embedded driver)
- **License**: MIT OR Apache-2.0
- **Repository**: https://github.com/telenkov88/mks-servo42-rs
- **Documentation**: https://docs.rs/mks-servo42-rs

## Build System

### Cargo Commands

```bash
# Build the project
cargo build

# Build for release
cargo build --release

# Run tests
cargo test

# Run specific test
cargo test test_name

# Run integration tests (requires hardware)
cargo test --test integration

# Format code
cargo fmt

# Lint with clippy (strict warnings as errors)
cargo clippy --all-targets -- -D warnings

# Clean build artifacts
cargo clean
```

### Makefile Targets

```bash
# Build
make build

# Run tests
make test

# Lint
make lint

# Format
make fmt

# Clean
make clean

# Run example
make run
```

## Code Style & Conventions

### Linting Configuration

The project uses strict linting rules configured in `Cargo.toml`:

**Rust Lints (`[lints.rust]`)**:
- `unsafe_code = "forbid"` - No unsafe code allowed
- Missing debug/copy implementations: warnings
- Trivial casts/numeric casts: warnings
- Unused imports/qualifications: warnings

**Workspace Rust Lints (`[workspace.lints.rust]`)**:
- Multiple lints set to `"deny"` including:
  - `ambiguous-glob-reexports`
  - `const-item-mutation`
  - `dangling-pointers-from-temporaries`
  - `unconditional-recursion`
  - `unsafe-op-in-unsafe-fn` (denied, but unsafe is already forbidden)

**Clippy Lints (`[workspace.lints.clippy]`)**:
- `alloc-instead-of-core = "deny"` - Must use core, not alloc
- `std-instead-of-core = "deny"` - Must use core, not std
- `undocumented-unsafe-blocks = "deny"` - Unsafe blocks require docs
- All other clippy lints: `"warn"` by default

### Formatting

- Use default `rustfmt` settings (no custom `rustfmt.toml` found)
- Run `cargo fmt` before committing

### Import Patterns

**Module Organization**:
```rust
// Core modules
pub mod direction;
pub mod enums;
mod errors;  // private module
pub mod helpers;
pub mod response;

// Re-exports at crate root
pub use enums::{BaudRate, EnLogic, MotorType, SaveClearStatus, WorkMode, ZeroMode};
pub use errors::Error;
pub use helpers::{...};
pub use response::{InvalidResponse, Response};
```

**Import Style**:
- Use absolute paths within the crate (`crate::module::item`)
- Group imports by source (std, external, internal)
- Avoid wildcard imports except for re-exports

### Naming Conventions

**Constants**: `SCREAMING_SNAKE_CASE`
```rust
pub const DEFAULT_ADDRESS: u8 = 0xE0;
pub const MAX_SPEED: u8 = 0x7F;
pub const CURRENT_STEP_MA: u16 = 200;
```

**Types**: `PascalCase`
- Structs: `EncoderValue`, `Driver`, `ShaftErrValue`
- Enums: `MotorType`, `WorkMode`, `BaudRate`
- Traits: (not currently used)

**Functions/Methods**: `snake_case`
- Public API: `angle_to_steps`, `parse_encoder_response`
- Private: `validate_buffer`, `calculate_checksum`

**Variables**: `snake_case`
- Local variables: `encoder_value`, `buffer_idx`
- Parameters: `data: &[u8]`, `angle: f32`

**Private Modules**: `snake_case`
```rust
mod cmd;  // Private command constants module
```

### Error Handling

**Custom Error Type** (`src/errors.rs`):
```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidAddress,
    InvalidSpeed,
    InvalidCurrentLimit,
    InvalidPacket,
    // ... other variants
}
```

**Result Pattern**:
```rust
type Result<T> = core::result::Result<T, Error>;

pub fn parse_encoder_response(data: &[u8]) -> Result<EncoderValue> {
    // ...
    Err(Error::InvalidPacket)
}
```

**Error Methods**:
- Implement `as_str()` for error descriptions
- No `Display` trait (embedded focus, no alloc)

### Documentation

**Module Documentation** (`//!`):
```rust
//! A generic, `no_std` Rust driver for **MKS SERVO42** closed-loop stepper motors.
//!
//! This library provides a type-safe interface for generating the serial protocol commands
//! used by the MKS SERVO42C firmware (V1.0+). It is transport-agnostic...
```

**Item Documentation** (`///`):
```rust
/// Main driver for communicating with an MKS SERVO42 motor.
///
/// This struct manages the slave address and an internal buffer used to
/// construct serial protocol commands.
#[derive(Debug, Copy, Clone)]
pub struct Driver {
    address: u8,
    buffer: [u8; CMD_BUFFER_SIZE],
}
```

**Must-use Annotations**:
```rust
#[must_use]
pub fn angle_to_steps(angle: f32, microsteps: f32) -> u32 {
    // ...
}
```

### Type Safety & `no_std` Constraints

**Key Constraints**:
- `#![no_std]` - No standard library dependencies
- `core` only, no `alloc` or `std`
- All types must be `Copy`/`Clone` where possible
- No dynamic allocation
- Fixed-size arrays for buffers

**Buffer Management**:
```rust
const CMD_BUFFER_SIZE: usize = 10;
pub struct Driver {
    buffer: [u8; CMD_BUFFER_SIZE],
}
```

## Testing

### Test Organization

**Unit Tests**: Inline with source code using `#[cfg(test)]`
**Integration Tests**: In `tests/` directory:
- `integration.rs` - Hardware integration tests
- `safety.rs` - Safety validation utilities
- `test_utils.rs` - Test helpers and context

### Test Patterns

**Hardware Tests** (require serial connection):
```rust
//! Integration tests for MKS SERVO42 driver
//!
//! These tests require real hardware connected via serial port.
//! Set MKS_ENV_SERVO42C_UART environment variable to serial port path.

#[test]
fn test_motor_enable_disable() -> TestResult<()> {
    let mut ctx = TestContext::new()?;
    // Test logic...
}
```

**Safety First**:
- Tests use minimal speed and small movements
- Validation functions check safe parameters
- Explicit timeouts and delays

**Custom Test Result**:
```rust
type TestResult<T> = Result<T, TestError>;
```

### Running Tests

```bash
# Run all tests
cargo test

# Run specific test module
cargo test --test integration

# Run single test
cargo test test_motor_enable_disable

# Run tests with environment variable
MKS_ENV_SERVO42C_UART=/dev/ttyUSB0 cargo test --test integration
```

## Development Workflow

### 1. Code Changes
- Follow existing patterns and naming conventions
- Ensure `no_std` compatibility
- Add comprehensive documentation
- Include `#[must_use]` where appropriate

### 2. Validation
```bash
# Format code
cargo fmt

# Run linter (warnings as errors)
cargo clippy --all-targets -- -D warnings

# Build in debug mode
cargo build

# Build in release mode
cargo build --release

# Run tests
cargo test
```

### 3. Hardware Testing
- Set `MKS_ENV_SERVO42C_UART` environment variable
- Use `examples/base.rs` as reference
- Test with minimal speed/angle first
- Verify protocol compliance

## Architecture Patterns

### Command Generation
- Command constants in private `cmd` module
- Buffer-based command construction
- Checksum calculation inline
- Protocol-compliant byte ordering

### Data Structures
- `Copy` + `Clone` types for embedded use
- `repr(u8)` enums for protocol compatibility
- `derive(Debug)` for all public types
- Fixed-size arrays, no vectors

### Protocol Compliance
- Address range: 0xE0-0xE9
- Speed range: 0-0x7F
- Checksum validation
- Big-endian byte ordering for multi-byte values

## External Dependencies

**Production**:
- `dotenv = "0.15.0"` (for examples only)

**Development**:
- `serial = "0.4"` (for integration tests)

**Key Constraints**:
- Must be `no_std` compatible
- Minimal dependencies
- No allocator required

## Examples

See `examples/base.rs` for complete usage example:
- Serial port configuration
- Command sequencing
- Response parsing
- Error handling
- Safety checks

## Agent Guidelines

When working on this project as an AI agent:

1. **Always maintain `no_std` compatibility**
2. **Never introduce `unsafe` code** (lint forbids it)
3. **Follow the existing module structure**
4. **Use the strict linting rules** (warnings as errors)
5. **Test with hardware when modifying protocol**
6. **Keep dependencies minimal**
7. **Document all public APIs**
8. **Use `#[must_use]` for pure functions**

## File Structure

```
.
├── Cargo.toml           # Project configuration with strict lints
├── Cargo.lock
├── Makefile             # Common tasks
├── README.md
├── LICENSE
├── src/
│   ├── lib.rs           # Main library entry point
│   ├── enums.rs         # Protocol enums (MotorType, WorkMode, etc.)
│   ├── direction.rs     # Direction constants
│   ├── errors.rs        # Error type and definitions
│   ├── helpers.rs       # Utility functions and parsers
│   └── response.rs      # Response parsing types
├── tests/
│   ├── integration.rs   # Hardware integration tests
│   ├── safety.rs        # Safety validation
│   └── test_utils.rs    # Test utilities
└── examples/
    └── base.rs          # Complete usage example
```

## Notes

- No Cursor/Copilot rules found (`.cursor/rules/`, `.cursorrules`, `.github/copilot-instructions.md`)
- Project follows modern Rust 2021 edition
- Strict safety and linting rules enforced
- Hardware-focused with clear separation of concerns