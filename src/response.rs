use core::convert::TryFrom;

/// Error returned when a byte cannot be converted to a `Response`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidResponse;

/// Common response from the motor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Response {
    /// Command execution failed.
    Failure = 0x00,
    /// Command execution succeeded.
    Success = 0x01,
}

impl TryFrom<u8> for Response {
    type Error = InvalidResponse;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Self::Failure),
            0x01 => Ok(Self::Success),
            _ => Err(InvalidResponse),
        }
    }
}

impl Response {
    #[must_use]
    pub const fn is_success(self) -> bool {
        matches!(self, Self::Success)
    }

    #[must_use]
    pub const fn is_failure(self) -> bool {
        matches!(self, Self::Failure)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn test_response_try_from_success() {
        let response = Response::try_from(0x01).unwrap();
        assert_eq!(response, Response::Success);
        assert!(response.is_success());
        assert!(!response.is_failure());
    }

    #[test]
    fn test_response_try_from_failure() {
        let response = Response::try_from(0x00).unwrap();
        assert_eq!(response, Response::Failure);
        assert!(response.is_failure());
        assert!(!response.is_success());
    }

    #[test]
    fn test_response_try_from_invalid() {
        let result = Response::try_from(0x02);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), InvalidResponse);
    }

    #[test]
    fn test_response_values() {
        assert_eq!(Response::Failure as u8, 0x00);
        assert_eq!(Response::Success as u8, 0x01);
    }
}
