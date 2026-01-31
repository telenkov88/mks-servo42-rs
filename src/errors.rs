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
