#![no_std]

pub mod decoder;
pub mod ntpc;
pub mod wifi;

pub const RX_BUFFER_SIZE: usize = 4096;
pub const TX_BUFFER_SIZE: usize = 4096;

pub const SSID: &str = env!("SSID");
pub const PASSWORD: &str = env!("PASSWORD");

const NTP_SERVER: &str = "pool.ntp.org";

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
