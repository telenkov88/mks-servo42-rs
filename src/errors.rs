/// Crate errors.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
    /// Provided value is out of range.
    InvalidValue,
    /// Checksum mismatch in received packet.
    Checksum,
    /// Received packet has invalid format or length.
    InvalidPacket,
}

impl Error {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InvalidValue => "Invalid value",
            Self::Checksum => "Checksum mismatch",
            Self::InvalidPacket => "Invalid packet format",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    #[test]
    fn test_error_display() {
        assert_eq!(Error::InvalidValue.as_str(), "Invalid value");
        assert_eq!(Error::Checksum.as_str(), "Checksum mismatch");
        assert_eq!(Error::InvalidPacket.as_str(), "Invalid packet format");
    }

    #[test]
    fn test_error_debug() {
        assert_eq!(std::format!("{:?}", Error::InvalidValue), "InvalidValue");
        assert_eq!(std::format!("{:?}", Error::Checksum), "Checksum");
        assert_eq!(std::format!("{:?}", Error::InvalidPacket), "InvalidPacket");
    }

    #[test]
    fn test_error_clone() {
        let err = Error::InvalidValue;
        let cloned = err.clone();
        assert_eq!(err, cloned);
    }

    #[test]
    fn test_error_copy() {
        let err = Error::Checksum;
        let copied = err; // Copy trait
        assert_eq!(err, copied);
    }

    #[test]
    fn test_error_equality() {
        assert_eq!(Error::InvalidValue, Error::InvalidValue);
        assert_ne!(Error::InvalidValue, Error::Checksum);
    }
}
