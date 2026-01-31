#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Direction {
    Forward = 0x80,
    Reverse = 0x00,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_direction_values() {
        assert_eq!(Direction::Forward as u8, 0x80);
        assert_eq!(Direction::Reverse as u8, 0x00);
    }
}
