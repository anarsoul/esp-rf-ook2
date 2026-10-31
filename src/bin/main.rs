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
use esp_hal::clock::CpuClock;
use esp_hal::ram;
use esp_hal::rmt::{PulseCode, Rmt, RxChannelConfig, RxChannelCreator};
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::timer::timg::{MwdtStage, TimerGroup};

use embassy_sync::channel::Channel;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use log::{debug, info, warn};

use esp_rf_ook2::MQTT_TOPIC;
use esp_rf_ook2::decoder::{DecodeError, Pulse, PulseLevel, SensorData, decode};
use esp_rf_ook2::mqtt::Mqtt;
use esp_rf_ook2::ntpc::Ntpc;
use esp_rf_ook2::wifi::Wifi;

use embassy_futures::select::{Either, select};
use embassy_net::Stack;

use static_cell::StaticCell;

use alloc::format;

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

static MQTT_CHANNEL: Channel<CriticalSectionRawMutex, (SensorData, i64), 2> = Channel::new();

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
        let (data, timestamp) = MQTT_CHANNEL.receive().await;

        if timestamp - last_publish > 300_000_000 {
            // Last successful publish was over 5 minutes ago, so something is wrong.
            // Panic and trigger watchdog reload to recover
            panic!("No successful publishes in 300 seconds!");
        }

        info!("Publishing...");
        let date_time = jiff::Timestamp::from_microsecond(timestamp)
            .unwrap()
            .strftime("%Y-%m-%d %H:%M:%S UTC");
        let topic = format!("{}/{}", MQTT_TOPIC, data.model());
        let data = format!(
            "{{\"time\" : \"{}\", \"model\" : \"{}\", \"id\" : {}, \"channel\" : {}, \"battery_ok\" : {}, \"temperature_C\" : {}{}.{}, \"humidity\" : {}, \"uptime\" : {} }}",
            date_time,
            data.model(),
            data.id,
            data.channel,
            data.battery_ok,
            { if data.sign < 0 { "-" } else { "" } },
            data.temp_int,
            data.temp_decimal,
            data.humidity,
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

    let mut last_time_sync = Instant::now();
    let now = jiff::Timestamp::from_microsecond(rtc.current_time_us() as i64).unwrap();

    info!("now is {now}");

    let freq = Rate::from_mhz(80);

    let rmt = Rmt::new(peripherals.RMT, freq).unwrap().into_async();
    let rx_config = RxChannelConfig::default()
        .with_clk_divider(80) // tick will be 1us (1MHz)
        .with_idle_threshold(3000) // timeout after 3ms of inactivity
        .with_filter_threshold(100); // filter out pulses shorter than 100us

    let mut channel = rmt
        .channel0
        .configure_rx(&rx_config)
        .expect("Failed to configure RMT RX channel")
        .with_pin(peripherals.GPIO21);
    // Two capture buffers: the RMT fills one while the other is decoded.
    // Nexus-TH leaves about 4 ms between repeats and the receiver ends a
    // frame 3 ms into that gap, so the next capture must be armed within
    // the remaining millisecond or its first code is a partial gap. Arming
    // first and decoding afterwards keeps decode, logging and MQTT
    // hand-off out of that window.
    let mut buffers: [[PulseCode; 64]; 2] = [[PulseCode::default(); 64]; 2];
    // Which buffer the next capture goes into; the other holds `finished`.
    let mut filling = 0usize;
    // Result of the capture that just completed into the other buffer.
    let mut finished: Option<Result<usize, esp_hal::rmt::Error>> = None;

    let mqtt = &mut *mk_static!(Mqtt, Mqtt::new(shared_stack));
    spawner.spawn(
        mqtt_publisher(mqtt, rtc.current_time_us() as i64)
            .expect("Failed to spawn MQTT sender task"),
    );

    let mut measurement = SensorData::default();
    let mut measurement_cnt = 0;
    let mut last_publish = rtc.current_time_us();

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
        // 36 symbols + terminator. High pulse (carrier present) has a fixed width of
        // 350-650 uS (actual width likely depends on battery voltage),
        // The actual data is encoded in the lenght of the low pulse (pauses)
        // 1 is 1650-2150 uS, 0 is 800-1100 uS
        //
        // On ESP32 RMT can count the lenght of pulses for us, simplifying the decoding.
        // `receive` starts the hardware before the future is first polled, so
        // the receiver is armed from here on while the previous capture is
        // decoded below.
        let [first, second] = &mut buffers;
        let (fill, done) = if filling == 0 {
            (first, second)
        } else {
            (second, first)
        };
        let a = channel.receive(fill);
        let b = Timer::after(Duration::from_secs(1));

        match finished.take() {
            None => {}
            Some(Ok(symbol_count)) => {
                let pulses: [Pulse; 64] = core::array::from_fn(|i| to_pulse(done[i]));
                match decode(&pulses, 1, symbol_count) {
                    Ok(parsed) => {
                        info!(
                            "Temperature: {}{}.{}C, Humidity: {}%",
                            { if parsed.sign < 0 { "-" } else { "" } },
                            parsed.temp_int,
                            parsed.temp_decimal,
                            parsed.humidity
                        );
                        if !measurement.equal(&parsed) {
                            measurement = parsed;
                            measurement_cnt = 1;
                        } else {
                            let now = rtc.current_time_us();
                            if measurement_cnt == 3 && now - last_publish > 5_000_000 {
                                MQTT_CHANNEL.send((parsed, now as i64)).await;
                                last_publish = now;
                            } else if measurement_cnt < 3 {
                                measurement_cnt += 1;
                            }
                        }
                    }
                    Err(e) => match e {
                        DecodeError::WrongPayloadLen(_len) => {}
                        _ => {
                            warn!("Decode error: {:?}", e);
                        }
                    },
                }
            }
            Some(Err(e)) => {
                // Overflows and framing errors on a noisy line are routine;
                // keep them out of the default log.
                debug!("RMT receive error: {e:?}");
            }
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
