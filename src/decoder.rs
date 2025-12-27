use esp_hal::gpio::Level;
use esp_hal::rmt::PulseCode;
use log::warn;

pub const PAYLOAD_LEN: usize = 36;

pub const PULSE_MIN: u16 = 300; // us
pub const PULSE_MAX: u16 = 650; // us

pub const MIN_HIGH: u16 = 1650;
pub const MAX_HIGH: u16 = 2150;
pub const MIN_LOW: u16 = 800;
pub const MAX_LOW: u16 = 1100;

#[derive(Debug)]
pub enum DecodeError {
    WrongPayloadLen(usize),
    SampleOutOfRange(u16),
    PulseOutOfRange(u16),
    WrongChannel(u8),
    TempOutOfRange(i32, i32),
}

#[derive(Debug)]
pub struct Parsed {
    model: [u8; 32],
    pub sign: i32,
    pub temp_int: i32,
    pub temp_decimal: i32,
    pub humidity: i32,
    pub battery_ok: u8,
    pub channel: u8,
    pub id: u8,
}

impl Default for Parsed {
    fn default() -> Self {
        Parsed::new("Unknown", 1, 10, 0, 80, 1, 0, 0)
    }
}

impl Parsed {
    #[allow(clippy::too_many_arguments)]
    fn new(
        model: &str,
        sign: i32,
        temp_int: i32,
        temp_decimal: i32,
        humidity: i32,
        battery_ok: u8,
        channel: u8,
        id: u8,
    ) -> Self {
        let mut model_arr = [0u8; 32];
        let bytes = model.as_bytes();
        let len = bytes.len().min(32);
        model_arr[..len].copy_from_slice(&bytes[..len]);
        Parsed {
            model: model_arr,
            sign,
            temp_int,
            temp_decimal,
            humidity,
            battery_ok,
            channel,
            id,
        }
    }
    pub fn model(&self) -> &str {
        let len = self
            .model
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.model.len());

        str::from_utf8(&self.model[..len]).unwrap_or("")
    }

    pub fn equal(&self, a: &Parsed) -> bool {
        self.sign == a.sign
            && self.temp_int == a.temp_int
            && self.temp_decimal == a.temp_decimal
            && self.humidity == a.humidity
    }
}

fn decode_range(samples: &[u16], start: usize, size: usize) -> Result<u32, DecodeError> {
    let mut value: u32 = 0;
    for sample in &samples[start..start + size] {
        if (MIN_HIGH..MAX_HIGH).contains(sample) {
            value <<= 1;
            value |= 1;
        } else if (MIN_LOW..MAX_LOW).contains(sample) {
            value <<= 1;
        } else {
            warn!("Range: {} - {}", start, start + size);
            return Err(DecodeError::SampleOutOfRange(*sample));
        }
    }
    Ok(value)
}

pub fn decode(pulses: &[PulseCode], ch: u8, len: usize) -> Result<Parsed, DecodeError> {
    // Currently we support only Nexus-TH which has 36 bit of payload
    if len != PAYLOAD_LEN + 1 {
        return Err(DecodeError::WrongPayloadLen(len));
    }

    for entry in &pulses[..len] {
        if let Level::High = entry.level1()
            && !(PULSE_MIN..PULSE_MAX).contains(&entry.length1())
        {
            return Err(DecodeError::PulseOutOfRange(entry.length1()));
        }
        if let Level::High = entry.level2()
            && !(PULSE_MIN..PULSE_MAX).contains(&entry.length2())
        {
            return Err(DecodeError::PulseOutOfRange(entry.length2()));
        }
    }

    let mut samples: [u16; PAYLOAD_LEN + 1] = [0; PAYLOAD_LEN + 1];
    for (idx, entry) in pulses.iter().enumerate() {
        if idx == len {
            break;
        }
        samples[idx] = if let Level::Low = entry.level1() {
            entry.length1()
        } else {
            entry.length2()
        };
    }

    let mut sign = 1;
    let mut temp_10x: i32 = decode_range(&samples, 12, 12)? as i32;
    // Handle negative temp
    if temp_10x > 2048 {
        sign = -1;
        temp_10x = 4096 - temp_10x;
    }
    let temp_int = temp_10x / 10;
    let temp_decimal = temp_10x % 10;

    if !(0..60).contains(&temp_int) {
        return Err(DecodeError::TempOutOfRange(sign, temp_int));
    }

    let mut humidity: i32 = decode_range(&samples, 28, 8)? as i32;
    // Clamp humidity
    if humidity > 100 {
        humidity = 100;
    }
    let battery_ok: u8 = decode_range(&samples, 8, 1)? as u8;
    let channel: u8 = (decode_range(&samples, 10, 2)? + 1) as u8;
    let id: u8 = decode_range(&samples, 0, 8)? as u8;

    if ch != channel {
        return Err(DecodeError::WrongChannel(channel));
    }

    let res = Parsed::new(
        "Nexus-TH",
        sign,
        temp_int,
        temp_decimal,
        humidity,
        battery_ok,
        channel,
        id,
    );

    Ok(res)
}
