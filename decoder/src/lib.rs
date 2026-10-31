// SPDX-License-Identifier: MIT

#![cfg_attr(not(test), no_std)]

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

/// Codes in one complete frame: one per bit, each a carrier pulse followed by
/// the gap that encodes the bit, plus the closing pulse that ends the last
/// gap, which the RMT stores with a zero-length second half as the end marker.
pub const FRAME_LEN: usize = PAYLOAD_LEN_BITS + 1;

/// Leading codes `decode` will discard before a frame. Two things put extra
/// codes in front of a frame on the receiver and each costs exactly one:
///
/// - The RXB6's data slicer chatters while its AGC recovers from the gap
///   between repeats, so a weak signal arrives with a junk pulse such as
///   `High 650 / Low 150` ahead of the first real pulse.
/// - The RMT was re-armed during the gap before the first pulse, so the first
///   code holds a partial gap and the whole train pairs as (gap, pulse)
///   instead of (pulse, gap). `decode` already reads the gap from whichever
///   half is low, so the shifted frame decodes once the partial code is gone.
///
/// Both at once is two codes. The trim is fixed by the length alone, so a
/// frame is never searched for; a frame with more leading codes than this is
/// rejected as `WrongPayloadLen`.
pub const MAX_LEADING_JUNK: usize = 2;

/// Hardware-agnostic stand-in for `esp_hal::gpio::Level`, so this crate
/// doesn't need `esp_hal` (and can be built/tested on the host) except for
/// the `#[ram]` placement of hot functions on-device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PulseLevel {
    Low,
    High,
}

/// Hardware-agnostic stand-in for `esp_hal::rmt::PulseCode`: two
/// level/length pairs, one RMT-captured edge-to-edge measurement each.
#[derive(Debug, Clone, Copy)]
pub struct Pulse {
    pub level1: PulseLevel,
    pub length1: u16,
    pub level2: PulseLevel,
    pub length2: u16,
}

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
    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
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
    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    pub fn model(&self) -> &str {
        let len = self
            .model
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.model.len());

        str::from_utf8(&self.model[..len]).unwrap_or("")
    }

    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    pub fn equal(&self, a: &SensorData) -> bool {
        self.sign == a.sign
            && self.temp_int == a.temp_int
            && self.temp_decimal == a.temp_decimal
            && self.humidity == a.humidity
    }
}

impl From<NexusTHPayload> for SensorData {
    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
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

        let channel: u8 = pld.channel.into();
        SensorData::new(
            "Nexus-TH",
            sign,
            temp_int,
            temp_decimal,
            humidity,
            pld.battery_ok,
            channel + 1,
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

/// Decode the first `len` codes of a received train. `len` may exceed a
/// frame by up to [`MAX_LEADING_JUNK`] codes; the excess is dropped from the
/// front, see that constant for why.
#[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
pub fn decode(pulses: &[Pulse], ch: u8, len: usize) -> Result<SensorData, DecodeError> {
    if !(FRAME_LEN..=FRAME_LEN + MAX_LEADING_JUNK).contains(&len) {
        return Err(DecodeError::WrongPayloadLen(len));
    }
    let frame = pulses
        .get(len - FRAME_LEN..len)
        .ok_or(DecodeError::WrongPayloadLen(len))?;
    decode_frame(frame, ch)
}

/// Decode exactly one frame of [`FRAME_LEN`] codes.
#[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
fn decode_frame(pulses: &[Pulse], ch: u8) -> Result<SensorData, DecodeError> {
    debug_assert_eq!(pulses.len(), FRAME_LEN);

    for entry in pulses {
        if let PulseLevel::High = entry.level1
            && !(PULSE_MIN..PULSE_MAX).contains(&entry.length1)
        {
            return Err(DecodeError::PulseOutOfRange(entry.length1));
        }
        if let PulseLevel::High = entry.level2
            && !(PULSE_MIN..PULSE_MAX).contains(&entry.length2)
        {
            return Err(DecodeError::PulseOutOfRange(entry.length2));
        }
    }

    let mut samples: [u16; PAYLOAD_LEN_BITS] = [0; PAYLOAD_LEN_BITS];
    for (idx, entry) in pulses.iter().take(PAYLOAD_LEN_BITS).enumerate() {
        samples[idx] = if let PulseLevel::Low = entry.level1 {
            entry.length1
        } else {
            entry.length2
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

#[cfg(test)]
mod tests {
    use super::*;

    fn bit_at(bytes: &[u8; PAYLOAD_LEN_BYTES], idx: usize) -> bool {
        let byte = bytes[idx / 8];
        (byte >> (7 - idx % 8)) & 1 == 1
    }

    // Mirrors decode()'s own bit-extraction, in reverse: turns packed
    // payload bytes into the RMT pulse train that would decode back to them.
    fn payload_to_pulses(bytes: &[u8; PAYLOAD_LEN_BYTES]) -> [Pulse; PAYLOAD_LEN_BITS + 1] {
        let terminator = Pulse {
            level1: PulseLevel::High,
            length1: 400,
            level2: PulseLevel::Low,
            length2: 3000,
        };
        let mut pulses = [terminator; PAYLOAD_LEN_BITS + 1];
        for (i, pulse) in pulses.iter_mut().take(PAYLOAD_LEN_BITS).enumerate() {
            let bit_len = if bit_at(bytes, i) { 2000 } else { 900 };
            *pulse = Pulse {
                level1: PulseLevel::High,
                length1: 400,
                level2: PulseLevel::Low,
                length2: bit_len,
            };
        }
        pulses
    }

    // The same train as the RMT records it when re-armed during the gap
    // before the first pulse: a partial gap first, then every code pairs a
    // bit's gap with the pulse that follows it, and the end marker is on its
    // own. One code longer than a clean frame.
    fn payload_to_offset_pulses(bytes: &[u8; PAYLOAD_LEN_BYTES]) -> Vec<Pulse> {
        let mut pulses = vec![Pulse {
            level1: PulseLevel::Low,
            length1: 300,
            level2: PulseLevel::High,
            length2: 400,
        }];
        for i in 0..PAYLOAD_LEN_BITS {
            let bit_len = if bit_at(bytes, i) { 2000 } else { 900 };
            pulses.push(Pulse {
                level1: PulseLevel::Low,
                length1: bit_len,
                level2: PulseLevel::High,
                length2: 400,
            });
        }
        pulses.push(Pulse {
            level1: PulseLevel::Low,
            length1: 0,
            level2: PulseLevel::Low,
            length2: 0,
        });
        pulses
    }

    // The slicer chatter seen on the board ahead of a weak repeat's first pulse.
    const JUNK: Pulse = Pulse {
        level1: PulseLevel::High,
        length1: 650,
        level2: PulseLevel::Low,
        length2: 150,
    };

    fn with_leading(junk: &[Pulse], frame: &[Pulse]) -> Vec<Pulse> {
        junk.iter().chain(frame).copied().collect()
    }

    fn assert_same_reading(a: &SensorData, b: &SensorData) {
        assert!(a.equal(b), "{a:?} != {b:?}");
        assert_eq!(a.id, b.id);
        assert_eq!(a.battery_ok, b.battery_ok);
    }

    fn valid_payload_bytes(
        id: u8,
        battery_ok: bool,
        channel: u8,
        temp_10x: u16,
        humidity: u8,
    ) -> [u8; PAYLOAD_LEN_BYTES] {
        let payload = NexusTHPayload {
            id: id.into(),
            battery_ok,
            _unknown_0: 0.into(),
            channel: channel.into(),
            temp_10x: temp_10x.into(),
            _unknown_1: 0.into(),
            humidity: humidity.into(),
        };
        payload.pack().unwrap()
    }

    #[test]
    fn decodes_valid_payload() {
        let bytes = valid_payload_bytes(174, true, 0, 101, 91);
        let pulses = payload_to_pulses(&bytes);

        let res = decode(&pulses, 1, PAYLOAD_LEN_BITS + 1).unwrap();

        assert_eq!(res.model(), "Nexus-TH");
        assert_eq!(res.id, 174);
        assert_eq!(res.channel, 1);
        assert!(res.battery_ok);
        assert_eq!(res.sign, 1);
        assert_eq!(res.temp_int, 10);
        assert_eq!(res.temp_decimal, 1);
        assert_eq!(res.humidity, 91);
    }

    #[test]
    fn decodes_negative_temperature() {
        // -10.1C => 4096 - 101
        let bytes = valid_payload_bytes(1, true, 0, 4096 - 101, 50);
        let pulses = payload_to_pulses(&bytes);

        let res = decode(&pulses, 1, PAYLOAD_LEN_BITS + 1).unwrap();

        assert_eq!(res.sign, -1);
        assert_eq!(res.temp_int, 10);
        assert_eq!(res.temp_decimal, 1);
    }

    #[test]
    fn clamps_humidity_above_100() {
        let bytes = valid_payload_bytes(1, true, 0, 200, 150);
        let pulses = payload_to_pulses(&bytes);

        let res = decode(&pulses, 1, PAYLOAD_LEN_BITS + 1).unwrap();

        assert_eq!(res.humidity, 100);
    }

    #[test]
    fn rejects_wrong_payload_len() {
        let bytes = valid_payload_bytes(1, true, 0, 200, 50);
        let pulses = payload_to_pulses(&bytes);

        let err = decode(&pulses[..10], 1, 10).unwrap_err();

        assert!(matches!(err, DecodeError::WrongPayloadLen(10)));
    }

    #[test]
    fn rejects_out_of_range_pulse() {
        let bytes = valid_payload_bytes(1, true, 0, 200, 50);
        let mut pulses = payload_to_pulses(&bytes);
        pulses[0].length1 = 700; // outside PULSE_MIN..PULSE_MAX

        let err = decode(&pulses, 1, PAYLOAD_LEN_BITS + 1).unwrap_err();

        assert!(matches!(err, DecodeError::PulseOutOfRange(700)));
    }

    #[test]
    fn rejects_out_of_range_sample() {
        let bytes = valid_payload_bytes(1, true, 0, 200, 50);
        let mut pulses = payload_to_pulses(&bytes);
        pulses[0].length2 = 500; // between the LOW and HIGH sample ranges

        let err = decode(&pulses, 1, PAYLOAD_LEN_BITS + 1).unwrap_err();

        assert!(matches!(err, DecodeError::SampleOutOfRange(500)));
    }

    #[test]
    fn rejects_wrong_channel() {
        let bytes = valid_payload_bytes(1, true, 0, 200, 50); // channel field 0 => channel 1
        let pulses = payload_to_pulses(&bytes);

        let err = decode(&pulses, 2, PAYLOAD_LEN_BITS + 1).unwrap_err();

        assert!(matches!(err, DecodeError::WrongChannel(1)));
    }

    #[test]
    fn decodes_with_one_leading_junk_code() {
        let bytes = valid_payload_bytes(174, true, 0, 101, 91);
        let clean = payload_to_pulses(&bytes);
        let pulses = with_leading(&[JUNK], &clean);
        assert_eq!(pulses.len(), FRAME_LEN + 1);

        let expected = decode(&clean, 1, FRAME_LEN).unwrap();
        let res = decode(&pulses, 1, pulses.len()).unwrap();

        assert_same_reading(&res, &expected);
    }

    #[test]
    fn decodes_with_half_symbol_offset() {
        let bytes = valid_payload_bytes(174, true, 0, 4096 - 101, 91);
        let clean = payload_to_pulses(&bytes);
        let pulses = payload_to_offset_pulses(&bytes);
        assert_eq!(pulses.len(), FRAME_LEN + 1);

        let expected = decode(&clean, 1, FRAME_LEN).unwrap();
        let res = decode(&pulses, 1, pulses.len()).unwrap();

        assert_same_reading(&res, &expected);
        assert_eq!(res.sign, -1);
    }

    #[test]
    fn decodes_with_junk_and_half_symbol_offset() {
        let bytes = valid_payload_bytes(7, false, 1, 235, 40);
        let clean = payload_to_pulses(&bytes);
        let pulses = with_leading(&[JUNK], &payload_to_offset_pulses(&bytes));
        assert_eq!(pulses.len(), FRAME_LEN + MAX_LEADING_JUNK);

        let expected = decode(&clean, 2, FRAME_LEN).unwrap();
        let res = decode(&pulses, 2, pulses.len()).unwrap();

        assert_same_reading(&res, &expected);
    }

    #[test]
    fn rejects_more_leading_codes_than_allowed() {
        let bytes = valid_payload_bytes(1, true, 0, 200, 50);
        let pulses = with_leading(&[JUNK; MAX_LEADING_JUNK + 1], &payload_to_pulses(&bytes));

        let err = decode(&pulses, 1, pulses.len()).unwrap_err();

        assert!(
            matches!(err, DecodeError::WrongPayloadLen(len) if len == FRAME_LEN + MAX_LEADING_JUNK + 1)
        );
    }

    #[test]
    fn leading_junk_does_not_mask_a_bad_frame() {
        let bytes = valid_payload_bytes(1, true, 0, 200, 50);
        let mut clean = payload_to_pulses(&bytes);
        clean[5].length1 = 700; // a bad pulse inside the frame proper
        let pulses = with_leading(&[JUNK], &clean);

        let err = decode(&pulses, 1, pulses.len()).unwrap_err();

        // The junk code's 650 us pulse is out of range too, but it is the
        // frame's own fault that is reported.
        assert!(matches!(err, DecodeError::PulseOutOfRange(700)));
    }

    #[test]
    fn rejects_len_beyond_the_buffer() {
        let bytes = valid_payload_bytes(1, true, 0, 200, 50);
        let pulses = payload_to_pulses(&bytes);

        let err = decode(&pulses, 1, FRAME_LEN + 1).unwrap_err();

        assert!(matches!(err, DecodeError::WrongPayloadLen(len) if len == FRAME_LEN + 1));
    }

    #[test]
    fn rejects_temp_out_of_range() {
        let bytes = valid_payload_bytes(1, true, 0, 600, 50); // 60.0C, out of 0..60
        let pulses = payload_to_pulses(&bytes);

        let err = decode(&pulses, 1, PAYLOAD_LEN_BITS + 1).unwrap_err();

        assert!(matches!(err, DecodeError::TempOutOfRange(1, 60)));
    }
}
