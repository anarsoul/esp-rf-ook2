#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::ram;
use esp_hal::rmt::{PulseCode, Rmt, RxChannelConfig, RxChannelCreator};
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::timer::timg::{MwdtStage, TimerGroup};
use esp_radio::Controller;

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use log::{info, warn};

use esp_rf_ook2::decoder::{DecodeError, Parsed, decode};
use esp_rf_ook2::mqtt::Mqtt;
use esp_rf_ook2::ntpc::Ntpc;
use esp_rf_ook2::wifi::Wifi;
use esp_rf_ook2::{RX_BUFFER_SIZE, TX_BUFFER_SIZE};

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

static RX_BUF: StaticCell<Mutex<NoopRawMutex, [u8; RX_BUFFER_SIZE]>> = StaticCell::new();
static TX_BUF: StaticCell<Mutex<NoopRawMutex, [u8; TX_BUFFER_SIZE]>> = StaticCell::new();
static SHARED_STACK: StaticCell<Mutex<NoopRawMutex, Stack<'static>>> = StaticCell::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // Arm watchdog timer
    let mut wdt = timg0.wdt;
    wdt.set_timeout(
        MwdtStage::Stage0,
        esp_hal::time::Duration::from_millis(30_000),
    );
    wdt.enable();
    wdt.feed();

    let rtc = Rtc::new(peripherals.LPWR);
    let radio_init = &*mk_static!(
        Controller<'static>,
        esp_radio::init().expect("Failed to init radio")
    );
    let wifi = Wifi::new(radio_init, peripherals.WIFI, Rng::new(), spawner)
        .await
        .expect("Failed to initialize Wi-Fi");

    wdt.feed();

    let shared_stack = SHARED_STACK.init(Mutex::new(wifi.stack));
    // Sockets cannot share the buffers, so users have to make sure that the socket is
    // closed before releasing the mutex.
    let rx_buf = RX_BUF.init(Mutex::new([0; RX_BUFFER_SIZE]));
    let tx_buf = TX_BUF.init(Mutex::new([0; TX_BUFFER_SIZE]));

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
    let mut ntpc = Ntpc::new(shared_stack, rx_buf, tx_buf);

    let time = ntpc.get_time().await.expect("Failed to get NTP time");
    rtc.set_current_time_us(time * 1_000_000);

    let mut last_time = rtc.current_time_us();
    let last_ts = jiff::Timestamp::from_microsecond(rtc.current_time_us() as i64).unwrap();

    info!("now is {last_ts}");

    let freq = Rate::from_mhz(80);

    let rmt = Rmt::new(peripherals.RMT, freq).unwrap().into_async();
    let rx_config = RxChannelConfig::default()
        .with_clk_divider(80) // tick will be 1us (1MHz)
        .with_idle_threshold(3000) // timeout after 3ms of inactivity
        .with_filter_threshold(100); // filter out pulses shorter than 100us

    let mut channel = rmt
        .channel0
        .configure_rx(peripherals.GPIO21, rx_config)
        .expect("Failed to configure RMT RX channel");
    let mut data: [PulseCode; 64] = [PulseCode::default(); 64];

    let mut mqtt = Mqtt::new(shared_stack, rx_buf, tx_buf);

    let mut measurement = Parsed::default();
    let mut measurement_cnt = 0;
    let mut last_publish = rtc.current_time_us();

    loop {
        wdt.feed();
        // Re-sync time every 10_000 seconds (~2.7 hours)
        if rtc.current_time_us() - last_time > 10_000_000_000 {
            info!("Re-syncing time via NTP...");
            let time = ntpc.get_time().await.expect("Failed to get NTP time");
            rtc.set_current_time_us(time * 1_000_000);
            last_time = rtc.current_time_us();
            let last_ts = jiff::Timestamp::from_microsecond(rtc.current_time_us() as i64).unwrap();

            info!("now is {last_ts}");
        }

        if rtc.current_time_us() - last_publish > 360_000_000 {
            // Last successful publish was over 5 minutes ago, so something is wrong.
            // Panic and trigger watchdog reload to recover
            panic!("No successful publishes in 360 seconds!");
        }

        // Receive the data as series of PulseCode. For Nexus-TH, it will be
        // 36 symbols + terminator. High pulse (carrier present) has a fixed width of
        // 350-650 uS (actual width likely depends on battery voltage),
        // The actual data is encoded in the lenght of the low pulse (pauses)
        // 1 is 1650-2150 uS, 0 is 800-1100 uS
        //
        // On ESP32 RMT can count the lenght of pulses for us, simplifying the decoding
        let a = channel.receive(&mut data);
        let b = Timer::after(Duration::from_secs(1));

        let either = select(a, b).await;
        wdt.feed();
        let res = match either {
            Either::First(res) => res,
            Either::Second(_) => {
                continue;
            }
        };
        match res {
            Ok(symbol_count) => match decode(&data, 1, symbol_count) {
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
                            info!("Publishing...");
                            let date_time = jiff::Timestamp::from_microsecond(now as i64)
                                .unwrap()
                                .strftime("%Y-%m-%d %H:%M:%S UTC");
                            let topic = format!("sensors/{}", parsed.model());
                            let data = format!(
                                "{{\"time\" : \"{}\", \"model\" : \"{}\", \"id\" : {}, \"channel\" : {}, \"battery_ok\" : {}, \"temperature_C\" : {}{}.{}, \"humidity\" : {} }}",
                                date_time,
                                parsed.model(),
                                parsed.id,
                                parsed.channel,
                                parsed.battery_ok,
                                { if parsed.sign < 0 { "-" } else { "" } },
                                parsed.temp_int,
                                parsed.temp_decimal,
                                parsed.humidity
                            );
                            match mqtt.publish(topic.as_str(), data.as_str()).await {
                                Ok(_) => {
                                    last_publish = now;
                                    info!(
                                        "Published at {}",
                                        jiff::Timestamp::from_microsecond(now as i64).unwrap()
                                    );
                                }
                                Err(e) => {
                                    warn!("Failed to publish MQTT message: {:?}", e);
                                }
                            };
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
            },
            Err(_e) => {}
        }
    }
}
