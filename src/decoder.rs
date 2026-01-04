use esp_hal::gpio::Level;
use esp_hal::rmt::PulseCode;
use packed_struct::prelude::*;

pub const PAYLOAD_LEN_BITS: usize = 36;
// Payload is 36 bits, 36 / 5 = 4.5 bytes, round up to 5 bytes
pub const PAYLOAD_LEN_BYTES: usize = 5;

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
    TempOutOfRange(i8, u16),
    UnpackFailed,
}

#[derive(Debug)]
pub struct SensorData {
    model: [u8; 32],
    pub sign: i8,
    pub temp_int: u16,
    pub temp_decimal: u16,
    pub humidity: u8,
    pub battery_ok: bool,
    pub channel: u8,
    pub id: u8,
}

impl Default for SensorData {
    fn default() -> Self {
        SensorData::new("Unknown", 1, 10, 0, 80, true, 0, 0)
    }
}

impl SensorData {
    #[allow(clippy::too_many_arguments)]
    fn new(
        model: &str,
        sign: i8,
        temp_int: u16,
        temp_decimal: u16,
        humidity: u8,
        battery_ok: bool,
        channel: u8,
        id: u8,
    ) -> Self {
        let mut model_arr = [0u8; 32];
        let bytes = model.as_bytes();
        let len = bytes.len().min(32);
        model_arr[..len].copy_from_slice(&bytes[..len]);
        SensorData {
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

    pub fn equal(&self, a: &SensorData) -> bool {
        self.sign == a.sign
            && self.temp_int == a.temp_int
            && self.temp_decimal == a.temp_decimal
            && self.humidity == a.humidity
    }
}

impl From<NexusTHPayload> for SensorData {
    fn from(pld: NexusTHPayload) -> Self {
        let mut sign = 1;
        let mut temp_10x: u16 = pld.temp_10x.into();
        // Handle negative temp
        if temp_10x > 2048 {
            sign = -1;
            temp_10x = 4096 - temp_10x;
        }
        let temp_int: u16 = temp_10x / 10;
        let temp_decimal: u16 = temp_10x % 10;

        let mut humidity: u8 = pld.humidity.into();
        // Clamp humidity
        if humidity > 100 {
            humidity = 100;
        }

        SensorData::new(
            "Nexus-TH",
            sign,
            temp_int,
            temp_decimal,
            humidity,
            pld.battery_ok,
            pld.channel.into(),
            pld.id.into(),
        )
    }
}

#[derive(PackedStruct, Debug)]
#[packed_struct(bit_numbering = "msb0")]
struct NexusTHPayload {
    #[packed_field(bits = "0:7")]
    id: Integer<u8, packed_bits::Bits<8>>,
    #[packed_field(bits = "8:8")]
    battery_ok: bool,
    #[packed_field(bits = "9:9")]
    _unknown_0: Integer<u8, packed_bits::Bits<1>>,
    #[packed_field(bits = "10:11")]
    channel: Integer<u8, packed_bits::Bits<2>>,
    #[packed_field(endian = "msb", bits = "12:23")]
    temp_10x: Integer<u16, packed_bits::Bits<12>>,
    #[packed_field(bits = "24:27")]
    _unknown_1: Integer<u8, packed_bits::Bits<4>>,
    #[packed_field(bits = "28:35")]
    humidity: Integer<u8, packed_bits::Bits<8>>,
}

pub fn decode(pulses: &[PulseCode], ch: u8, len: usize) -> Result<SensorData, DecodeError> {
    // len should be number of bits + terminator
    if len != PAYLOAD_LEN_BITS + 1 {
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

    let mut samples: [u16; PAYLOAD_LEN_BITS] = [0; PAYLOAD_LEN_BITS];
    for (idx, entry) in pulses.iter().enumerate() {
        if idx == len || idx == PAYLOAD_LEN_BITS {
            break;
        }
        samples[idx] = if let Level::Low = entry.level1() {
            entry.length1()
        } else {
            entry.length2()
        };
    }

    let mut decoded: [u8; PAYLOAD_LEN_BYTES] = [0; PAYLOAD_LEN_BYTES];
    for (idx, value) in samples.iter().enumerate() {
        if (MIN_HIGH..MAX_HIGH).contains(value) {
            decoded[idx / 8] |= 1 << (7 - idx % 8);
        } else if (MIN_LOW..MAX_LOW).contains(value) {
            decoded[idx / 8] &= !(1 << (7 - idx % 8));
        } else {
            return Err(DecodeError::SampleOutOfRange(*value));
        }
    }

    let unpacked = NexusTHPayload::unpack(&decoded).map_err(|_| DecodeError::UnpackFailed)?;

    let res: SensorData = unpacked.into();

    if !(0..60).contains(&res.temp_int) {
        return Err(DecodeError::TempOutOfRange(res.sign, res.temp_int));
    }

    if ch != res.channel {
        return Err(DecodeError::WrongChannel(res.channel));
    }

    Ok(res)
}
