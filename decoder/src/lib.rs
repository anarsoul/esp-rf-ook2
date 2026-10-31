// SPDX-License-Identifier: MIT

#![cfg_attr(not(test), no_std)]

use packed_struct::prelude::*;

pub const PAYLOAD_LEN_BITS: usize = 36;
// Payload is 36 bits, 36 / 5 = 4.5 bytes, round up to 5 bytes
pub const PAYLOAD_LEN_BYTES: usize = 5;

/// Bit periods, one rising edge to the next: a carrier pulse of about 500 us
/// and then a gap of about 1000 us for a zero or 2000 us for a one. The bit
/// lives in the period, not in the pulse or gap on their own: the RXB6's
/// slicer moves the edge between them with signal strength and interrupts
/// both with glitches, but every real pulse starts on the sensor's grid.
pub const PERIOD_0: u16 = 1500;
pub const PERIOD_1: u16 = 2500;
/// How far a period, or the sum of two with a lost edge between them, may
/// sit from nominal. The sensor on the bench is within 50 us.
pub const PERIOD_TOL: u16 = 200;

/// Stand-in for a pulse the capture started in the middle of: its recorded
/// length is short by however long it had already been high.
pub const PULSE_NOMINAL: u16 = 500;
/// A high run shorter than this never counts as a pulse. The RXB6 chatters
/// with 50-250 us highs inside gaps and between repeats; the sensor's pulses
/// are 500-600 us.
pub const PULSE_MIN: u16 = 250;
/// A high run this long where a pulse should be is a pulse, the zero gap
/// after it and the next pulse run together: the slicer stays high through
/// a short gap while its threshold recovers from the silence between
/// repeats, which is the usual state of the first few bits of a repeat.
/// Other lost edges are told apart by the period alone, see [`classify`].
pub const STUCK_HIGH_0: core::ops::RangeInclusive<u16> = 1800..=2300;
/// Humidity above this is a corrupt frame, not a reading. The sensor does
/// report a little over 100 when saturated, never more than about 110; a
/// frame read one bit early gives 172 for a reading of 88.
pub const HUMIDITY_MAX: u8 = 110;

/// A low run at least this long before a pulse is the gap between repeats,
/// nominally 4000 us, so a frame may start at that pulse. The longest gap
/// inside a frame is a one at about 2000 us.
pub const SYNC_MIN: u16 = 2200;

/// Hardware-agnostic stand-in for `esp_hal::gpio::Level`, so this crate
/// doesn't need `esp_hal` (and can be built/tested on the host) except for
/// the `#[ram]` placement of hot functions on-device).
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// The capture ended before the frame did, after this many bits: the
    /// receiver was armed partway into the repeat, a pulse was lost at the
    /// end, or the capture overflowed.
    Truncated(u8),
    /// After bit `.0`, no pulse started at a bit period or at a sum of two;
    /// `.1` is the distance to the first pulse past the last window.
    PeriodOutOfRange(u8, u16),
    /// Two bits with the edge between them lost, whose total `.1` fits a
    /// zero then a one as well as a one then a zero.
    Ambiguous(u8, u16),
    /// Bits 24..27 were not the constant 0xF every Nexus-TH frame carries.
    BadConstant(u8),
    /// Bit 9, always 0 on Nexus-TH, was set.
    ReservedBitSet,
    /// Humidity above [`HUMIDITY_MAX`].
    HumidityOutOfRange(u8),
    /// A pulse started on the grid after the closing pulse: the frame was
    /// started a bit early, on a glitch, and every bit read was really the
    /// one before it. Such a frame passes the payload checks more often
    /// than one would think: the bench sensor's reading shifted right by
    /// one still had 0xF where the constant nibble is read, and showed up
    /// as a second sensor, id 105, at 3.8C and 100% humidity.
    StartedEarly,
    WrongChannel(u8),
    TempOutOfRange(i8, u16),
    UnpackFailed,
}

#[derive(Debug, Clone, Copy)]
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

/// Distinct readings a burst can carry. The sensor repeats one frame, so
/// anything beyond the first is a corrupt decode; the protocol has no
/// checksum and a flipped low bit passes validation as a nearby reading.
pub const MAX_CANDIDATES: usize = 4;

/// Decodes a reading needs before it is trusted. One is a possible bit
/// flip; two identical decodes of a 36-bit frame are not a coincidence.
pub const MIN_VOTES: u8 = 2;

/// Tally of the decodes from one transmission burst. Readings are grouped
/// with [`SensorData::equal`]; [`Vote::winner`] names the one the burst
/// agrees on, and the caller decides when the burst is over.
#[derive(Debug, Default)]
pub struct Vote {
    candidates: [Option<(SensorData, u8)>; MAX_CANDIDATES],
}

impl Vote {
    /// Count one decode. Once [`MAX_CANDIDATES`] distinct readings are
    /// held, further new ones are dropped: they are noise, and a real
    /// reading already present keeps counting.
    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    pub fn add(&mut self, data: SensorData) {
        for slot in self.candidates.iter_mut() {
            match slot {
                Some((seen, count)) if seen.equal(&data) => {
                    *count = count.saturating_add(1);
                    return;
                }
                Some(_) => {}
                None => {
                    *slot = Some((data, 1));
                    return;
                }
            }
        }
    }

    /// Whether any decode has been counted.
    pub fn is_empty(&self) -> bool {
        self.candidates[0].is_none()
    }

    /// Decodes counted so far, agreeing or not.
    pub fn decodes(&self) -> u32 {
        self.candidates
            .iter()
            .flatten()
            .map(|(_, count)| u32::from(*count))
            .sum()
    }

    /// The reading with the most decodes, if it has at least [`MIN_VOTES`]
    /// and no other reading ties with it.
    pub fn winner(&self) -> Option<SensorData> {
        let mut best: Option<(SensorData, u8)> = None;
        let mut tied = false;
        for (data, count) in self.candidates.iter().flatten() {
            match best {
                Some((_, best_count)) if *count < best_count => {}
                Some((_, best_count)) if *count == best_count => tied = true,
                _ => {
                    best = Some((*data, *count));
                    tied = false;
                }
            }
        }
        match best {
            Some((data, count)) if count >= MIN_VOTES && !tied => Some(data),
            _ => None,
        }
    }

    /// Forget the burst.
    pub fn clear(&mut self) {
        self.candidates = [None; MAX_CANDIDATES];
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

        // The sensor reports a little over 100 when saturated; that is a
        // reading, clamped. Beyond HUMIDITY_MAX is rejected before this.
        let humidity: u8 = u8::from(pld.humidity).min(100);

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
    /// Always 0; with the constant nibble, the only fixed bits in the frame.
    #[packed_field(bits = "9:9")]
    reserved: Integer<u8, packed_bits::Bits<1>>,
    #[packed_field(bits = "10:11")]
    channel: Integer<u8, packed_bits::Bits<2>>,
    #[packed_field(endian = "msb", bits = "12:23")]
    temp_10x: Integer<u16, packed_bits::Bits<12>>,
    /// Always 0xF on Nexus-TH; the protocol has no checksum, so this nibble
    /// is the only fixed pattern a corrupt frame can be caught against.
    #[packed_field(bits = "24:27")]
    constant: Integer<u8, packed_bits::Bits<4>>,
    #[packed_field(bits = "28:35")]
    humidity: Integer<u8, packed_bits::Bits<8>>,
}

/// The level runs of a capture: each code holds two, and the capture ends
/// at the first zero-length run. The RMT writes one to close a capture, at
/// whichever level the line was stuck past the idle threshold, so it may
/// read as a high; a capture that overflowed has none, and gets one from
/// the zeroed buffer instead. Anything after it is stale.
struct Runs<'a> {
    codes: &'a [Pulse],
    len: usize,
}

impl<'a> Runs<'a> {
    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    fn new(codes: &'a [Pulse]) -> Self {
        let mut len = 0;
        for code in codes {
            if code.length1 == 0 {
                break;
            }
            len += 1;
            if code.length2 == 0 {
                break;
            }
            len += 1;
        }
        Runs { codes, len }
    }

    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    fn get(&self, i: usize) -> (PulseLevel, u16) {
        let code = &self.codes[i / 2];
        if i.is_multiple_of(2) {
            (code.level1, code.length1)
        } else {
            (code.level2, code.length2)
        }
    }
}

/// What the distance from one pulse to a later one says.
enum Period {
    /// `n` bits, in the low bits of `bits`, most significant first.
    Bits {
        bits: u8,
        n: u8,
    },
    /// Not on the grid: a glitch, keep looking.
    Skip,
    Ambiguous,
    TooLong,
}

/// Classify the distance `period` from the start of a pulse of length `high`
/// to the start of a later high run. Besides the two bit periods, sums of
/// two are accepted, for an edge the receiver lost: the slicer stayed high
/// through a gap, or dropped a pulse. 3000 and 5000 are a pair of zeros or
/// ones; 4000 is a zero then a one when the high run is a pulse, a zero gap
/// and a pulse run together ([`STUCK_HIGH_0`]), and ambiguous otherwise.
#[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
fn classify(period: u32, high: u16) -> Period {
    let near = |nominal: u32| period.abs_diff(nominal) <= u32::from(PERIOD_TOL);
    let p0 = u32::from(PERIOD_0);
    let p1 = u32::from(PERIOD_1);
    if near(p0) {
        Period::Bits { bits: 0b0, n: 1 }
    } else if near(p1) {
        Period::Bits { bits: 0b1, n: 1 }
    } else if near(p0 + p0) {
        Period::Bits { bits: 0b00, n: 2 }
    } else if near(p0 + p1) {
        if STUCK_HIGH_0.contains(&high) {
            Period::Bits { bits: 0b01, n: 2 }
        } else {
            Period::Ambiguous
        }
    } else if near(p1 + p1) {
        Period::Bits { bits: 0b11, n: 2 }
    } else if period > p1 + p1 + u32::from(PERIOD_TOL) {
        Period::TooLong
    } else {
        Period::Skip
    }
}

/// Decode a capture: the codes the RMT recorded, up to the end marker or
/// the end of the slice, see [`Runs`]. A frame is looked for from every
/// pulse that may start one: the capture's first, since a clean gap between
/// repeats ends the capture ahead of it, and any pulse after a low of at
/// least [`SYNC_MIN`], for captures that ran on through a glitched gap. The
/// first candidate that yields 36 bits and a valid payload wins; otherwise
/// the error of the candidate that got furthest is returned.
#[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
pub fn decode(codes: &[Pulse], ch: u8) -> Result<SensorData, DecodeError> {
    let runs = Runs::new(codes);
    let mut best: Option<(u8, DecodeError)> = None;
    let mut low_before: Option<u16> = None;
    for i in 0..runs.len {
        let (level, len) = runs.get(i);
        if level == PulseLevel::Low {
            low_before = Some(len);
            continue;
        }
        // The first high run is the capture's first pulse: whole if a
        // partial gap came first, cut short if the receiver was armed
        // during it and it is the first run and shorter than a pulse.
        let first = i < 2;
        let partial = i == 0 && len < PULSE_NOMINAL;
        let candidate = first || low_before.is_some_and(|low| low >= SYNC_MIN);
        low_before = None;
        if !candidate {
            continue;
        }
        let (mut start, mut partial) = (i, partial);
        loop {
            match decode_from(&runs, start, partial, ch) {
                Ok(data) => return Ok(data),
                Err(failure) => {
                    if best.is_none_or(|(p, _)| failure.progress > p) {
                        best = Some((failure.progress, failure.err));
                    }
                    // A frame that started a bit early really starts at
                    // the pulse it took for bit 1; try from there.
                    match failure.retry_from {
                        Some(pulse) => (start, partial) = (pulse, false),
                        None => break,
                    }
                }
            }
        }
    }
    Err(best.map_or(DecodeError::Truncated(0), |(_, err)| err))
}

/// Why a candidate start did not yield a frame.
struct Failure {
    /// Bits read before giving up; a full frame for payload errors.
    progress: u8,
    err: DecodeError,
    /// Another start worth trying because of this failure.
    retry_from: Option<usize>,
}

impl Failure {
    fn at(progress: usize, err: DecodeError) -> Self {
        Failure {
            progress: progress as u8,
            err,
            retry_from: None,
        }
    }
}

/// Decode one frame whose first pulse is run `start`. `partial` says that
/// pulse was cut short by the receiver being armed during it, so the first
/// period is taken from the gap and a nominal pulse instead.
#[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
fn decode_from(runs: &Runs, start: usize, partial: bool, ch: u8) -> Result<SensorData, Failure> {
    let mut decoded = [0u8; PAYLOAD_LEN_BYTES];
    let mut nbits: usize = 0;
    let mut pulse = start;
    let mut second_pulse = start;
    while nbits < PAYLOAD_LEN_BITS {
        let high = runs.get(pulse).1;
        let mut distance = u32::from(high);
        let mut k = pulse + 1;
        let (next, bits, n) = loop {
            if k >= runs.len {
                return Err(Failure::at(nbits, DecodeError::Truncated(nbits as u8)));
            }
            let (level, len) = runs.get(k);
            if level == PulseLevel::High && len >= PULSE_MIN {
                let period = if partial && nbits == 0 {
                    distance - u32::from(high) + u32::from(PULSE_NOMINAL)
                } else {
                    distance
                };
                let clipped = period.min(u32::from(u16::MAX)) as u16;
                match classify(period, high) {
                    Period::Bits { bits, n } if nbits + usize::from(n) <= PAYLOAD_LEN_BITS => {
                        break (k, bits, n);
                    }
                    // A pair where only one bit is left means the frame's
                    // closing pulse was lost and this is the next repeat.
                    Period::Bits { .. } | Period::TooLong => {
                        return Err(Failure::at(
                            nbits,
                            DecodeError::PeriodOutOfRange(nbits as u8, clipped),
                        ));
                    }
                    Period::Ambiguous => {
                        return Err(Failure::at(
                            nbits,
                            DecodeError::Ambiguous(nbits as u8, clipped),
                        ));
                    }
                    Period::Skip => {}
                }
            }
            distance += u32::from(len);
            k += 1;
        };
        if nbits == 0 {
            second_pulse = next;
        }
        for b in (0..n).rev() {
            if (bits >> b) & 1 == 1 {
                decoded[nbits / 8] |= 1 << (7 - nbits % 8);
            }
            nbits += 1;
        }
        pulse = next;
    }

    let done = PAYLOAD_LEN_BITS;

    // The frame must be followed by silence. A pulse on the grid after the
    // closing pulse is the real closing pulse of a frame that was started a
    // bit early, see `DecodeError::StartedEarly`.
    let high = runs.get(pulse).1;
    let mut distance = u32::from(high);
    let mut k = pulse + 1;
    while k < runs.len {
        let (level, len) = runs.get(k);
        if level == PulseLevel::High && len >= PULSE_MIN {
            match classify(distance, high) {
                Period::Bits { .. } | Period::Ambiguous => {
                    return Err(Failure {
                        progress: done as u8,
                        err: DecodeError::StartedEarly,
                        retry_from: Some(second_pulse),
                    });
                }
                Period::TooLong => break,
                Period::Skip => {}
            }
        }
        distance += u32::from(len);
        k += 1;
    }

    let unpacked = NexusTHPayload::unpack(&decoded)
        .map_err(|_| Failure::at(done, DecodeError::UnpackFailed))?;

    let constant: u8 = unpacked.constant.into();
    if constant != 0xF {
        return Err(Failure::at(done, DecodeError::BadConstant(constant)));
    }

    if u8::from(unpacked.reserved) != 0 {
        return Err(Failure::at(done, DecodeError::ReservedBitSet));
    }

    let humidity = u8::from(unpacked.humidity);
    if humidity > HUMIDITY_MAX {
        return Err(Failure::at(done, DecodeError::HumidityOutOfRange(humidity)));
    }

    let res: SensorData = unpacked.into();

    if !(0..60).contains(&res.temp_int) {
        return Err(Failure::at(
            done,
            DecodeError::TempOutOfRange(res.sign, res.temp_int),
        ));
    }

    if ch != res.channel {
        return Err(Failure::at(done, DecodeError::WrongChannel(res.channel)));
    }

    Ok(res)
}

#[cfg(test)]
mod tests {
    use super::*;
    use PulseLevel::{High, Low};

    type Run = (PulseLevel, u16);

    // The sensor as the bench board records it.
    const PULSE: u16 = 550;
    const GAP_0: u16 = 950;
    const GAP_1: u16 = 1950;

    fn bit_at(bytes: &[u8; PAYLOAD_LEN_BYTES], idx: usize) -> bool {
        let byte = bytes[idx / 8];
        (byte >> (7 - idx % 8)) & 1 == 1
    }

    // The runs the receiver sees for one frame: a pulse and a gap per bit,
    // then the pulse that closes the last gap.
    fn frame_runs(bytes: &[u8; PAYLOAD_LEN_BYTES]) -> Vec<Run> {
        let mut runs = Vec::new();
        for i in 0..PAYLOAD_LEN_BITS {
            runs.push((High, PULSE));
            runs.push((Low, if bit_at(bytes, i) { GAP_1 } else { GAP_0 }));
        }
        runs.push((High, PULSE));
        runs
    }

    // Pack runs into codes the way the RMT does, two per code, and close
    // the capture with a zero-length run at `end_level`: the level the line
    // sat at past the idle threshold. The other half of that code is left
    // stale, as on the board.
    fn to_codes(runs: &[Run], end_level: PulseLevel) -> Vec<Pulse> {
        let mut halves = runs.to_vec();
        halves.push((end_level, 0));
        if halves.len() % 2 == 1 {
            halves.push((High, 579));
        }
        halves
            .chunks(2)
            .map(|c| Pulse {
                level1: c[0].0,
                length1: c[0].1,
                level2: c[1].0,
                length2: c[1].1,
            })
            .collect()
    }

    fn codes(runs: &[Run]) -> Vec<Pulse> {
        to_codes(runs, Low)
    }

    fn frame_codes(bytes: &[u8; PAYLOAD_LEN_BYTES]) -> Vec<Pulse> {
        codes(&frame_runs(bytes))
    }

    // Replace runs `range` of a frame with `with`.
    fn splice(runs: &[Run], range: core::ops::Range<usize>, with: &[Run]) -> Vec<Run> {
        let mut out = runs[..range.start].to_vec();
        out.extend_from_slice(with);
        out.extend_from_slice(&runs[range.end..]);
        out
    }

    // Indices of the pulse and gap runs of bit `bit`.
    fn pulse_of(bit: usize) -> usize {
        2 * bit
    }
    fn gap_of(bit: usize) -> usize {
        2 * bit + 1
    }

    // Slicer chatter as seen on the board between repeats.
    const NOISE: [Run; 8] = [
        (High, 300),
        (Low, 400),
        (High, 200),
        (Low, 600),
        (High, 700),
        (Low, 1200),
        (High, 150),
        (Low, 90),
    ];

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
            reserved: 0.into(),
            channel: channel.into(),
            temp_10x: temp_10x.into(),
            constant: 0xF.into(),
            humidity: humidity.into(),
        };
        payload.pack().unwrap()
    }

    // A reading whose id starts 0b0001_0011, like the sensor on the bench,
    // and the clean decode of it to compare damaged captures against.
    fn sample() -> ([u8; PAYLOAD_LEN_BYTES], SensorData) {
        let bytes = valid_payload_bytes(0b0001_0011, true, 0, 76, 87);
        let expected = decode(&frame_codes(&bytes), 1).unwrap();
        (bytes, expected)
    }

    #[test]
    fn decodes_valid_payload() {
        let bytes = valid_payload_bytes(174, true, 0, 101, 91);

        let res = decode(&frame_codes(&bytes), 1).unwrap();

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

        let res = decode(&frame_codes(&bytes), 1).unwrap();

        assert_eq!(res.sign, -1);
        assert_eq!(res.temp_int, 10);
        assert_eq!(res.temp_decimal, 1);
    }

    #[test]
    fn clamps_humidity_a_little_above_100() {
        let bytes = valid_payload_bytes(1, true, 0, 200, HUMIDITY_MAX);

        let res = decode(&frame_codes(&bytes), 1).unwrap();

        assert_eq!(res.humidity, 100);
    }

    #[test]
    fn rejects_humidity_far_above_100() {
        let bytes = valid_payload_bytes(1, true, 0, 200, HUMIDITY_MAX + 1);

        let err = decode(&frame_codes(&bytes), 1).unwrap_err();

        assert_eq!(err, DecodeError::HumidityOutOfRange(HUMIDITY_MAX + 1));
    }

    #[test]
    fn rejects_wrong_constant_nibble() {
        let mut bytes = valid_payload_bytes(1, true, 0, 200, 50);
        bytes[3] &= 0x0F; // bits 24..27 are the high nibble of byte 3

        let err = decode(&frame_codes(&bytes), 1).unwrap_err();

        assert_eq!(err, DecodeError::BadConstant(0));
    }

    #[test]
    fn rejects_reserved_bit() {
        let mut bytes = valid_payload_bytes(1, true, 0, 200, 50);
        bytes[1] |= 0x40; // bit 9

        let err = decode(&frame_codes(&bytes), 1).unwrap_err();

        assert_eq!(err, DecodeError::ReservedBitSet);
    }

    // The bench sensor's frame as recorded with a 300 us glitch 1500 us
    // ahead of it. Read from the glitch, every bit is the one before it:
    // id 105, battery ok, channel 1, 3.8C, constant nibble still 0xF and
    // humidity 172. The decoder must notice the real closing pulse on the
    // grid after the one it took for closing, and start over one bit later.
    #[test]
    fn retries_a_frame_started_a_bit_early() {
        let bytes = valid_payload_bytes(211, true, 0, 77, 88);
        let expected = decode(&frame_codes(&bytes), 1).unwrap();
        let runs = splice(&frame_runs(&bytes), 0..0, &[(High, 300), (Low, 950)]);

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
        assert_eq!(res.id, 211);
    }

    // The same with the real closing pulse lost: nothing follows the shifted
    // frame, so it is the battery bit landing on the reserved bit that
    // rejects it.
    #[test]
    fn reserved_bit_catches_a_shifted_frame_without_its_closing_pulse() {
        let bytes = valid_payload_bytes(211, true, 0, 77, 88);
        let mut runs = splice(&frame_runs(&bytes), 0..0, &[(High, 300), (Low, 950)]);
        runs.pop();

        let err = decode(&codes(&runs), 1).unwrap_err();

        assert_eq!(err, DecodeError::ReservedBitSet);
    }

    // Chatter after the closing pulse that is off the grid is not a bit.
    #[test]
    fn allows_off_grid_chatter_after_the_frame() {
        let (bytes, expected) = sample();
        let mut runs = frame_runs(&bytes);
        runs.extend_from_slice(&[
            (Low, 700),
            (High, 300),
            (Low, 400),
            (High, 600),
            (Low, 2500),
        ]);

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    #[test]
    fn rejects_wrong_channel() {
        let bytes = valid_payload_bytes(1, true, 0, 200, 50); // channel field 0 => channel 1

        let err = decode(&frame_codes(&bytes), 2).unwrap_err();

        assert_eq!(err, DecodeError::WrongChannel(1));
    }

    #[test]
    fn rejects_temp_out_of_range() {
        let bytes = valid_payload_bytes(1, true, 0, 600, 50); // 60.0C, out of 0..60

        let err = decode(&frame_codes(&bytes), 1).unwrap_err();

        assert_eq!(err, DecodeError::TempOutOfRange(1, 60));
    }

    // The receiver was armed during the gap before the frame, so the
    // capture opens with the tail of that gap and every code pairs a gap
    // with the pulse after it.
    #[test]
    fn decodes_after_partial_gap() {
        let (bytes, expected) = sample();
        let runs = splice(&frame_runs(&bytes), 0..0, &[(Low, 300)]);

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // Armed during the first pulse: only its last 137 us were recorded.
    #[test]
    fn decodes_from_partial_first_pulse() {
        let (bytes, expected) = sample();
        let runs = splice(&frame_runs(&bytes), 0..1, &[(High, 137)]);

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // The line sat high past the idle threshold after the frame, so the end
    // marker reads as a high, and the buffer holds an older capture after it.
    #[test]
    fn ignores_end_marker_level_and_stale_data() {
        let (bytes, expected) = sample();
        let mut capture = to_codes(&frame_runs(&bytes), High);
        capture.extend_from_slice(&codes(&NOISE));

        let res = decode(&capture, 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // An overflowed capture has no end marker; the zeroed buffer ends it.
    #[test]
    fn decodes_overflowed_capture_from_zeroed_buffer() {
        let (bytes, expected) = sample();
        let mut runs = frame_runs(&bytes);
        runs.push((Low, 400)); // even number of halves, no marker
        let mut capture = runs
            .chunks(2)
            .map(|c| Pulse {
                level1: c[0].0,
                length1: c[0].1,
                level2: c[1].0,
                length2: c[1].1,
            })
            .collect::<Vec<_>>();
        let zero = Pulse {
            level1: Low,
            length1: 0,
            level2: Low,
            length2: 0,
        };
        capture.extend_from_slice(&[zero; 4]);

        let res = decode(&capture, 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // A frame that fills the slice exactly, with nothing after its last pulse.
    #[test]
    fn decodes_frame_that_ends_with_the_slice() {
        let (bytes, expected) = sample();
        let runs = splice(&frame_runs(&bytes), 0..0, &[(Low, 300)]); // 74 halves
        let capture: Vec<Pulse> = runs
            .chunks(2)
            .map(|c| Pulse {
                level1: c[0].0,
                length1: c[0].1,
                level2: c[1].0,
                length2: c[1].1,
            })
            .collect();

        let res = decode(&capture, 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // Slicer chatter inside a zero gap, as recorded on the board: the gap
    // comes through as five runs that add up to the right length.
    #[test]
    fn skips_high_glitches_in_a_gap() {
        let (bytes, expected) = sample();
        let runs = frame_runs(&bytes);
        let bit = (0..PAYLOAD_LEN_BITS).find(|&b| !bit_at(&bytes, b)).unwrap();
        let g = gap_of(bit);
        let runs = splice(
            &runs,
            g..g + 1,
            &[(Low, 247), (High, 160), (Low, 53), (High, 362), (Low, 128)],
        );

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // The slicer dropped out for 50 us in the middle of a pulse.
    #[test]
    fn skips_a_low_glitch_in_a_pulse() {
        let (bytes, expected) = sample();
        let runs = frame_runs(&bytes);
        let p = pulse_of(7);
        let runs = splice(&runs, p..p + 1, &[(High, 300), (Low, 50), (High, 200)]);

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // The first two bits of the sample are zeros. The slicer stayed high
    // from the first pulse through the first gap and second pulse and 160 us
    // into the second gap: two rising edges 3000 us apart.
    #[test]
    fn recovers_two_zeros_under_a_stuck_high() {
        let (bytes, expected) = sample();
        let runs = frame_runs(&bytes);
        let runs = splice(&runs, 0..4, &[(High, 2160), (Low, 840)]);

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // Bits 0 and 1 are a zero then a one: 4000 us between edges, told from
    // a one then a zero by the 2020 us high that cannot hold a one gap.
    #[test]
    fn recovers_zero_then_one_under_a_stuck_high() {
        let bytes = valid_payload_bytes(0b0100_0000, true, 0, 76, 87);
        let expected = decode(&frame_codes(&bytes), 1).unwrap();
        let runs = splice(&frame_runs(&bytes), 0..4, &[(High, 2020), (Low, 1980)]);

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // A zero then a one with the pulse between them dropped instead: a
    // normal pulse then 3450 us of silence fits both orders, so the frame
    // is refused. (Humidity 85 puts the pair at bits 34 and 35.)
    #[test]
    fn rejects_an_ambiguous_lost_pulse() {
        let bytes = valid_payload_bytes(0b0001_0011, true, 0, 76, 85);
        let p = pulse_of(34);
        let runs = splice(&frame_runs(&bytes), p..p + 4, &[(High, PULSE), (Low, 3450)]);

        let err = decode(&codes(&runs), 1).unwrap_err();

        assert_eq!(err, DecodeError::Ambiguous(34, 4000));
    }

    // A one whose pulse and most of whose gap came through as one high.
    #[test]
    fn recovers_a_one_under_a_stuck_high() {
        let bytes = valid_payload_bytes(0b1000_0000, true, 0, 76, 87);
        let expected = decode(&frame_codes(&bytes), 1).unwrap();
        let runs = splice(&frame_runs(&bytes), 0..2, &[(High, 2191), (Low, 309)]);

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // Two ones with the pulse between them dropped: 5000 us is unambiguous.
    #[test]
    fn recovers_two_ones_with_a_dropped_pulse() {
        let bytes = valid_payload_bytes(0b1100_0000, true, 0, 76, 87);
        let expected = decode(&frame_codes(&bytes), 1).unwrap();
        let runs = splice(&frame_runs(&bytes), 0..4, &[(High, PULSE), (Low, 4450)]);

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // The closing pulse was lost, so the last gap ran into the silence
    // between repeats and bit 35 cannot be read.
    #[test]
    fn reports_truncation_when_the_closing_pulse_is_lost() {
        let (bytes, _) = sample();
        let mut runs = frame_runs(&bytes);
        runs.pop();

        let err = decode(&codes(&runs), 1).unwrap_err();

        assert_eq!(err, DecodeError::Truncated(35));
    }

    #[test]
    fn decodes_a_frame_after_noise_and_a_sync_gap() {
        let (bytes, expected) = sample();
        let mut runs = NOISE.to_vec();
        runs.push((Low, 2500));
        runs.extend(frame_runs(&bytes));

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // Only the capture's first pulse and pulses after a sync gap may start
    // a frame, or chatter would be searched for frames all day.
    #[test]
    fn does_not_look_for_a_frame_after_noise_without_a_sync_gap() {
        let (bytes, _) = sample();
        let mut runs = NOISE.to_vec();
        runs.push((Low, 1500));
        runs.extend(frame_runs(&bytes));

        assert!(decode(&codes(&runs), 1).is_err());
    }

    // A glitch split the gap between repeats, so the capture ran on into
    // the next one; its tail does no harm.
    #[test]
    fn decodes_the_first_of_two_repeats_in_one_capture() {
        let (bytes, expected) = sample();
        let mut runs = frame_runs(&bytes);
        runs.extend_from_slice(&[(Low, 1391), (High, 154), (Low, 2369)]);
        runs.extend(frame_runs(&bytes));

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    #[test]
    fn decodes_the_second_repeat_when_the_first_is_damaged() {
        let bytes = valid_payload_bytes(0b0100_0000, true, 0, 76, 87);
        let expected = decode(&frame_codes(&bytes), 1).unwrap();
        let mut runs = splice(&frame_runs(&bytes), 0..4, &[(High, PULSE), (Low, 3450)]);
        runs.push((Low, 2500));
        runs.extend(frame_runs(&bytes));

        let res = decode(&codes(&runs), 1).unwrap();

        assert_same_reading(&res, &expected);
    }

    // When nothing decodes, the error names how far the best attempt got.
    #[test]
    fn reports_the_error_of_the_furthest_candidate() {
        let (bytes, _) = sample();
        let runs = frame_runs(&bytes);
        let g = gap_of(20);
        let runs = splice(&runs, g..g + 1, &[(Low, 1200)]); // period 1750: off the grid
        let mut capture = NOISE.to_vec();
        capture.push((Low, 2500));
        capture.extend(runs);

        let err = decode(&codes(&capture), 1).unwrap_err();

        assert!(
            matches!(err, DecodeError::PeriodOutOfRange(20, _)),
            "{err:?}"
        );
    }

    #[test]
    fn rejects_noise() {
        assert!(decode(&codes(&NOISE), 1).is_err());
        assert!(decode(&[], 1).is_err());
    }

    fn reading(temp_int: u16, temp_decimal: u16, humidity: u8) -> SensorData {
        SensorData::new("Nexus-TH", 1, temp_int, temp_decimal, humidity, true, 1, 0)
    }

    #[test]
    fn vote_needs_two_agreeing_decodes() {
        let mut vote = Vote::default();
        assert!(vote.is_empty());
        assert!(vote.winner().is_none());
        vote.add(reading(21, 3, 45));
        assert!(!vote.is_empty());
        assert!(vote.winner().is_none(), "one decode may be a bit flip");
        vote.add(reading(21, 3, 45));
        let w = vote.winner().expect("two identical decodes agree");
        assert!(w.equal(&reading(21, 3, 45)));
    }

    #[test]
    fn vote_ignores_a_corrupt_decode_among_good_ones() {
        let mut vote = Vote::default();
        vote.add(reading(21, 3, 45));
        vote.add(reading(21, 7, 45)); // flipped bit in the decimal
        vote.add(reading(21, 3, 45));
        vote.add(reading(21, 3, 45));
        assert_eq!(vote.decodes(), 4);
        assert!(vote.winner().unwrap().equal(&reading(21, 3, 45)));
    }

    #[test]
    fn vote_rejects_a_tie() {
        let mut vote = Vote::default();
        vote.add(reading(21, 3, 45));
        vote.add(reading(21, 3, 46));
        vote.add(reading(21, 3, 45));
        vote.add(reading(21, 3, 46));
        assert!(vote.winner().is_none());
        vote.add(reading(21, 3, 46));
        assert!(vote.winner().unwrap().equal(&reading(21, 3, 46)));
    }

    #[test]
    fn vote_keeps_counting_a_known_reading_when_full() {
        let mut vote = Vote::default();
        for h in 0..MAX_CANDIDATES as u8 {
            vote.add(reading(21, 3, 40 + h));
        }
        vote.add(reading(21, 3, 99)); // dropped: no slot left
        vote.add(reading(21, 3, 99));
        assert!(vote.winner().is_none());
        vote.add(reading(21, 3, 42)); // already held, still counts
        assert!(vote.winner().unwrap().equal(&reading(21, 3, 42)));
    }

    #[test]
    fn vote_clear_forgets_the_burst() {
        let mut vote = Vote::default();
        vote.add(reading(21, 3, 45));
        vote.add(reading(21, 3, 45));
        vote.clear();
        assert!(vote.is_empty());
        assert!(vote.winner().is_none());
    }
}
