#![no_std]

pub mod decoder;
pub mod mqtt;
pub mod ntpc;
pub mod wifi;

extern crate alloc;

pub const RX_BUFFER_SIZE: usize = 4096;
pub const TX_BUFFER_SIZE: usize = 4096;

pub const SSID: &str = env!("SSID");
pub const PASSWORD: &str = env!("PASSWORD");

pub const NTP_SERVER: &str = "pool.ntp.org";
pub const TIMEZONE: &str = "UTC";

pub const MQTT_SERVER: &str = env!("MQTT_SERVER");
pub const MQTT_LOGIN: &str = env!("MQTT_LOGIN");
pub const MQTT_PASSWORD: &str = env!("MQTT_PASSWORD");

pub const MQTT_TOPIC: &str = env!("MQTT_TOPIC");

#[unsafe(no_mangle)]
pub fn custom_halt() -> ! {
    esp_hal::system::software_reset();
}

#[unsafe(no_mangle)]
pub extern "Rust" fn _esp_println_timestamp() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis()
}
