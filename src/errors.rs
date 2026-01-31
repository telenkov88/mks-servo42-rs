#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
    InvalidValue,
    Checksum,
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
