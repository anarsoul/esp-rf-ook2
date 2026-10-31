// SPDX-License-Identifier: MIT

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::TIMG0;
use esp_hal::ram;
use esp_hal::rmt::{
    CHANNEL_RAM_SIZE, Channel as RmtChannel, PulseCode, Rmt, Rx, RxChannelConfig, RxChannelCreator,
};
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::timer::timg::{MwdtStage, TimerGroup, Wdt};

use embassy_sync::channel::Channel;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use log::{debug, info, warn};

use esp_rf_ook2::MQTT_TOPIC;
use esp_rf_ook2::decoder::{DecodeError, Pulse, PulseLevel, SensorData, Vote, decode};
use esp_rf_ook2::mqtt::Mqtt;
use esp_rf_ook2::ntpc::Ntpc;
use esp_rf_ook2::wifi::Wifi;

use embassy_futures::select::{Either, select};
use embassy_net::Stack;

use static_cell::StaticCell;

use alloc::format;
use alloc::string::String;
use core::fmt::Write as _;

extern crate alloc;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static SHARED_STACK: StaticCell<Mutex<NoopRawMutex, Stack<'static>>> = StaticCell::new();

/// Silence after a decode that ends the burst. Nexus-TH repeats its frame
/// about every 80 ms, a dozen times, so a burst lasts about a second, and
/// then goes quiet for about a minute. Only successful decodes reset the
/// clock, and with a weak signal most repeats fail, so the gap must cover
/// a whole burst between two decodes: the bench board saw one decode at
/// each end of a burst 0.5 s apart, which a 500 ms gap split in two.
const BURST_GAP: Duration = Duration::from_secs(2);

/// RMT memory blocks for the receive channel. The ESP32 has eight 64-code
/// blocks shared by its eight channels and only channel 0 is in use, so
/// taking four is free. The ESP32 cannot wrap a receive, so a capture that
/// outgrows its memory is lost whole; with one block, 27 codes of slicer
/// chatter ahead of a frame were enough to do that.
const RX_MEM_BLOCKS: u8 = 4;
/// Codes one capture can hold, and the size of each capture buffer.
const RX_CODES: usize = RX_MEM_BLOCKS as usize * CHANNEL_RAM_SIZE;

/// A capture at least this long that fails to decode is logged at debug
/// level with its error and codes: it is most of a frame. Shorter captures
/// are noise blips and are only counted. Nothing per capture goes out at
/// info: the console runs at 115200 baud, printing blocks this task, and
/// the receiver is dead from the end of a capture until this task re-arms
/// it. With the RXB6 chattering through 70 captures a second, a line per
/// capture cost enough dead time to miss the start of repeats.
const DUMP_MIN_CODES: usize = 20;

/// The receiver is never quiet, so the tally is also logged this often
/// while no burst is in progress: once per transmission interval.
const TALLY_PERIOD: Duration = Duration::from_secs(60);

/// What became of every capture. Logged when a burst closes, and once a
/// minute otherwise, so the repeats that never turned into a decode are
/// accounted for instead of vanishing: a burst showing one decode and no
/// errors was losing the other eleven to captures nobody logged.
#[derive(Default)]
struct Tally {
    captures: u32,
    decoded: u32,
    truncated: u32,
    off_grid: u32,
    ambiguous: u32,
    bad_constant: u32,
    shifted: u32,
    other: u32,
    /// Valid frames from a sensor other than the one locked onto.
    foreign: u32,
    rmt_errors: u32,
}

impl Tally {
    fn is_empty(&self) -> bool {
        self.captures == 0
    }

    fn decoded(&mut self) {
        self.captures += 1;
        self.decoded += 1;
    }

    fn foreign(&mut self) {
        self.captures += 1;
        self.foreign += 1;
    }

    fn failed(&mut self, err: &DecodeError) {
        self.captures += 1;
        match err {
            DecodeError::Truncated(_) => self.truncated += 1,
            DecodeError::PeriodOutOfRange(..) => self.off_grid += 1,
            DecodeError::Ambiguous(..) => self.ambiguous += 1,
            DecodeError::BadConstant(_)
            | DecodeError::ReservedBitSet
            | DecodeError::HumidityOutOfRange(_) => self.bad_constant += 1,
            DecodeError::StartedEarly => self.shifted += 1,
            _ => self.other += 1,
        }
    }

    /// An RMT error is counted on top of what its data then decodes to.
    fn rmt_error(&mut self) {
        self.rmt_errors += 1;
    }
}

impl core::fmt::Display for Tally {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{} captures: {} decoded, {} truncated, {} off grid, {} ambiguous, \
             {} bad payload, {} started early, {} other, {} other sensor; {} RMT errors",
            self.captures,
            self.decoded,
            self.truncated,
            self.off_grid,
            self.ambiguous,
            self.bad_constant,
            self.shifted,
            self.other,
            self.foreign,
            self.rmt_errors
        )
    }
}

/// A reading as `7.6C 88% id 211 battery ok`.
fn describe(data: &SensorData) -> String {
    format!(
        "{}{}.{}C {}% id {} battery {}",
        if data.sign < 0 { "-" } else { "" },
        data.temp_int,
        data.temp_decimal,
        data.humidity,
        data.id,
        if data.battery_ok { "ok" } else { "low" }
    )
}

/// Codes in a capture buffer up to and including the one holding the end
/// marker, or all of them if it overflowed without one.
fn codes_in(buffer: &[PulseCode]) -> usize {
    buffer
        .iter()
        .position(|c| c.length1() == 0 || c.length2() == 0)
        .map_or(buffer.len(), |i| i + 1)
}

/// The codes of a capture as `H480/L950 H500/L1900 ...`, one entry per
/// code, levels as H/L and lengths in microseconds.
fn format_codes(codes: &[PulseCode]) -> String {
    fn level(l: esp_hal::gpio::Level) -> char {
        match l {
            esp_hal::gpio::Level::Low => 'L',
            esp_hal::gpio::Level::High => 'H',
        }
    }
    let mut out = String::with_capacity(codes.len() * 12);
    for c in codes {
        let _ = write!(
            out,
            " {}{}/{}{}",
            level(c.level1()),
            c.length1(),
            level(c.level2()),
            c.length2()
        );
    }
    out
}

/// A burst's winning reading, the time it was received and how many
/// frames of the burst decoded, agreeing or not. The count goes out with
/// the reading so a weakening signal shows in the broker's record, not
/// only on a serial console nobody is watching.
static MQTT_CHANNEL: Channel<CriticalSectionRawMutex, (SensorData, i64, u32), 2> = Channel::new();

fn get_timestamp() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis()
}

fn to_pulse_level(level: esp_hal::gpio::Level) -> PulseLevel {
    match level {
        esp_hal::gpio::Level::Low => PulseLevel::Low,
        esp_hal::gpio::Level::High => PulseLevel::High,
    }
}

fn to_pulse(pulse: PulseCode) -> Pulse {
    Pulse {
        level1: to_pulse_level(pulse.level1()),
        length1: pulse.length1(),
        level2: to_pulse_level(pulse.level2()),
        length2: pulse.length2(),
    }
}

#[embassy_executor::task]
async fn mqtt_publisher(mqtt: &'static mut Mqtt, now: i64) {
    let mut last_publish: i64 = now;
    loop {
        let (data, timestamp, decodes) = MQTT_CHANNEL.receive().await;

        if timestamp - last_publish > 300_000_000 {
            // Last successful publish was over 5 minutes ago, so something is wrong.
            // Panic and trigger watchdog reload to recover
            panic!("No successful publishes in 300 seconds!");
        }

        debug!("Publishing...");
        let date_time = jiff::Timestamp::from_microsecond(timestamp)
            .unwrap()
            .strftime("%Y-%m-%d %H:%M:%S UTC");
        let topic = format!("{}/{}", MQTT_TOPIC, data.model());
        let data = format!(
            "{{\"time\" : \"{}\", \"model\" : \"{}\", \"id\" : {}, \"channel\" : {}, \"battery_ok\" : {}, \"temperature_C\" : {}{}.{}, \"humidity\" : {}, \"decodes\" : {}, \"uptime\" : {} }}",
            date_time,
            data.model(),
            data.id,
            data.channel,
            data.battery_ok,
            { if data.sign < 0 { "-" } else { "" } },
            data.temp_int,
            data.temp_decimal,
            data.humidity,
            decodes,
            get_timestamp(),
        );
        match mqtt.publish(topic.as_str(), data.as_str()).await {
            Ok(_) => {
                last_publish = timestamp;
                info!(
                    "Published at {}",
                    jiff::Timestamp::from_microsecond(timestamp).unwrap()
                );
            }
            Err(e) => {
                warn!("Failed to publish MQTT message: {:?}", e);
            }
        };
    }
}

/// The receive loop: captures, decodes, tallies and hands winning bursts
/// to the publisher. Everything that runs once is in `main`.
///
/// Not `#[ram]`: the attribute only sets the link section of the function
/// item, which for an `async fn` is the stub that builds the future. The
/// body is a separate coroutine that rustc places in `.text` regardless
/// (and here inlines into the executor's poll), so it stays in flash.
async fn run(
    mut wdt: Wdt<TIMG0<'static>>,
    rtc: Rtc<'static>,
    mut ntpc: Ntpc,
    mut channel: RmtChannel<'static, Async, Rx>,
) -> ! {
    let mut last_time_sync = Instant::now();

    // Two capture buffers: the RMT fills one while the other is decoded.
    // Nexus-TH leaves about 4 ms between repeats and the receiver ends a
    // frame 2.6 ms into that gap (the idle threshold above), so the next
    // capture must be armed within the remaining 1.4 ms or its first code
    // is a partial gap. Arming first and decoding afterwards keeps decode,
    // logging and MQTT hand-off out of that window.
    let mut buffers: [[PulseCode; RX_CODES]; 2] = [[PulseCode::default(); RX_CODES]; 2];
    // Which buffer the next capture goes into; the other holds `finished`.
    let mut filling = 0usize;
    // Result of the capture that just completed into the other buffer.
    let mut finished: Option<Result<usize, esp_hal::rmt::Error>> = None;

    // Decodes of the burst in progress, and when the last one arrived.
    // A burst is tallied as a whole once it has gone quiet: the sensor
    // repeats one frame a dozen times, a weak signal yields only some of
    // them, and the protocol has no checksum, so a single decode is not
    // trusted on its own and one corrupt decode must not veto the rest.
    let mut vote = Vote::default();
    // The sensor this receiver reports: the id of the first burst whose
    // decodes agreed. Every Nexus-TH within range shares channel 1 and the
    // frame has no other identity, and a neighbour's sensor was published
    // for a while before ids were logged. The id changes with a battery
    // swap, so a new battery means a reset to lock again; the publisher's
    // five-minute staleness panic does that by itself.
    let mut sensor_id: Option<u8> = None;
    let mut last_decode: Option<Instant> = None;
    // Every capture's fate, and when it was last logged.
    let mut tally = Tally::default();
    let mut last_tally = Instant::now();

    loop {
        wdt.feed();
        // Re-sync time every hour. Checked against the cheap monotonic
        // clock; reading the RTC costs a register handshake and sits on the
        // re-arm path below.
        if last_time_sync.elapsed() > Duration::from_secs(3_600) {
            info!("Re-syncing time via NTP...");
            let time = ntpc.get_time().await.expect("Failed to get NTP time");
            rtc.set_current_time_us(time * 1_000_000);
            last_time_sync = Instant::now();
            let last_ts = jiff::Timestamp::from_microsecond(rtc.current_time_us() as i64).unwrap();

            info!("now is {last_ts}");
        }

        // Receive the data as series of PulseCode. For Nexus-TH, it will be
        // 36 symbols + terminator. Each symbol is a carrier pulse of about
        // 500 us followed by a gap of about 1000 us for a 0 or 2000 us for
        // a 1; the decoder reads the bit from the period of the two, since
        // a weak signal moves the edge between them but not the period.
        //
        // On ESP32 RMT can count the length of pulses for us, simplifying the decoding.
        // `receive` starts the hardware before the future is first polled, so
        // the receiver is armed from here on while the previous capture is
        // decoded below.
        let [first, second] = &mut buffers;
        let (fill, done) = if filling == 0 {
            (first, second)
        } else {
            (second, first)
        };
        // A capture that overflows has no end marker; zeroes after its data
        // give the decoder one, see `decoder::decode`.
        fill.fill(PulseCode::default());
        let a = channel.receive(fill);
        let b = Timer::after(Duration::from_secs(1));

        // An errored capture is decoded all the same: on overflow the
        // driver has copied what the hardware got before giving up, and it
        // may well hold a frame.
        if let Some(result) = finished.take() {
            let count = codes_in(done);
            if let Err(e) = result {
                tally.rmt_error();
                debug!("RMT receive error: {e:?}, {count} codes");
            }
            let pulses: [Pulse; RX_CODES] = core::array::from_fn(|i| to_pulse(done[i]));
            match decode(&pulses, 1) {
                Ok(parsed) => {
                    // Nothing is printed here: the burst is reported as a
                    // whole once it is over, see BURST_GAP. A line per decode
                    // cost dead time in the middle of the burst.
                    debug!("Decoded {}", describe(&parsed));
                    if sensor_id.is_some_and(|id| id != parsed.id) {
                        tally.foreign();
                    } else {
                        tally.decoded();
                        vote.add(parsed);
                        last_decode = Some(Instant::now());
                    }
                }
                Err(e) => {
                    tally.failed(&e);
                    if count >= DUMP_MIN_CODES {
                        debug!("{count} codes, {e:?}:{}", format_codes(&done[..count]));
                    }
                }
            }
        }

        // The burst is over once no decode has arrived for BURST_GAP;
        // this runs at least once a second thanks to the timeout below.
        let quiet_for = |t: Option<Instant>| t.is_some_and(|since| since.elapsed() > BURST_GAP);
        if quiet_for(last_decode) {
            // The one line per burst: its outcome, and the tally since the
            // last line, which holds the burst's own captures.
            match vote.winner() {
                Some(data) => {
                    if sensor_id.is_none() {
                        sensor_id = Some(data.id);
                        info!("Locked to sensor id {}", data.id);
                    }
                    info!(
                        "Burst: {} decodes, {}; {tally}",
                        vote.decodes(),
                        describe(&data)
                    );
                    MQTT_CHANNEL
                        .send((data, rtc.current_time_us() as i64, vote.decodes()))
                        .await;
                }
                None => warn!(
                    "Burst: {} decode(s), no two in agreement, dropped; {tally}",
                    vote.decodes()
                ),
            }
            tally = Tally::default();
            last_tally = Instant::now();
            vote.clear();
            last_decode = None;
        } else if last_decode.is_none() && last_tally.elapsed() > TALLY_PERIOD {
            if !tally.is_empty() {
                info!("Tally: {tally}");
            }
            tally = Tally::default();
            last_tally = Instant::now();
        }

        match select(a, b).await {
            Either::First(res) => {
                finished = Some(res);
                filling ^= 1;
            }
            // Nothing arrived; the dropped future stopped the receiver.
            Either::Second(_) => finished = None,
        }
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Heap for the Wi-Fi driver and the rest. The reclaimed region is the
    // largest single block; the second one is headroom for the driver's
    // buffers and the per-publish allocations.
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 72 * 1024);
    esp_alloc::heap_allocator!(size: 32 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // Arm watchdog timer
    let mut wdt = timg0.wdt;
    wdt.set_timeout(
        MwdtStage::Stage0,
        esp_hal::time::Duration::from_millis(30_000),
    );
    wdt.enable();
    wdt.feed();

    let rtc = Rtc::new(peripherals.RTC_TIMER);
    let wifi =
        Wifi::new(peripherals.WIFI, Rng::new(), spawner).expect("Failed to initialize Wi-Fi");

    wdt.feed();

    let shared_stack = SHARED_STACK.init(Mutex::new(wifi.stack));

    wdt.feed();
    let a = wifi.wait_for_ip();
    let b = Timer::after(Duration::from_secs(20));

    let res = select(a, b).await;
    match res {
        Either::First(_) => {
            info!("Got IP address!");
        }
        Either::Second(_) => {
            panic!("Timed out waiting for IP address");
        }
    }

    wdt.feed();
    let mut ntpc = Ntpc::new(shared_stack);

    let time = ntpc.get_time().await.expect("Failed to get NTP time");
    rtc.set_current_time_us(time * 1_000_000);

    let now = jiff::Timestamp::from_microsecond(rtc.current_time_us() as i64).unwrap();

    info!("now is {now}");

    let freq = Rate::from_mhz(80);

    let rmt = Rmt::new(peripherals.RMT, freq).unwrap().into_async();
    let rx_config = RxChannelConfig::default()
        .with_clk_divider(80) // tick will be 1us (1MHz)
        // A gap this long ends the capture. The longest gap inside a frame
        // is a 1 bit at about 2150 us and the gap after a frame is at least
        // 3000 us, so this sits between the two; the lower it is, the sooner
        // the capture ends and the more of the inter-repeat gap is left to
        // re-arm the receiver in.
        .with_idle_threshold(2600)
        // The glitch filter counts APB clock periods (80 MHz), not channel
        // ticks, and the register is 8 bits wide, so 255 is the most it can
        // do: about 3.2 us. Anything longer is left to the decoder.
        .with_filter_threshold(255)
        .with_memsize(RX_MEM_BLOCKS);

    let channel = rmt
        .channel0
        .configure_rx(&rx_config)
        .expect("Failed to configure RMT RX channel")
        .with_pin(peripherals.GPIO21);

    let mqtt = &mut *mk_static!(Mqtt, Mqtt::new(shared_stack));
    spawner.spawn(
        mqtt_publisher(mqtt, rtc.current_time_us() as i64)
            .expect("Failed to spawn MQTT sender task"),
    );

    run(wdt, rtc, ntpc, channel).await
}
