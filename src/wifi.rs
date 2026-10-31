// SPDX-License-Identifier: MIT

//! Wi-Fi station: keeps the station on the strongest access point of the
//! configured SSID and runs DHCP only while it is connected.
//!
//! esp-radio does not roam. With two access points on one SSID the driver's
//! default `Fast` scan takes the first beacon it hears on any channel and
//! happily associates with a far AP at -80 dBm, so the choice of AP is made
//! here, at connect time, following what esp32clock settled on:
//!
//! - `ScanMethod::AllChannels` makes the driver scan every channel before
//!   connecting and, with the by-signal sort order esp-radio sets, pick the
//!   strongest BSSID. It costs a second or two per connect.
//! - Before each connect the task scans for our SSID itself and logs every
//!   candidate with BSSID, channel and RSSI, so a bad connect can be read
//!   against the choice the driver had.
//! - The transmit ceiling is raised from esp-radio's 5 dBm default, which
//!   is low enough that a far AP hears the station at the noise floor and
//!   never answers the authentication request.
//! - A run of failed connects ends in a panic, which resets the chip
//!   (`custom_halt`): the driver avoids the AP that last dropped it and can
//!   keep trying another that cannot hear us, and only a reset clears that.

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{ConfigV4, DhcpConfig, Runner, Stack, StackResources};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::rng::Rng;
use esp_radio::wifi::scan::{ScanConfig, ScanTypeConfig};
use esp_radio::wifi::sta::{ScanMethod, StationConfig};
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, ConnectionError, ControllerConfig, DisconnectReason,
    Interface, WifiController, WifiError,
};
use heapless::String;
use log::{error, info, warn};
use static_cell::StaticCell;

use crate::{PASSWORD, SSID};

/// DHCP option 12, so the router's DNS can resolve us by name.
const HOSTNAME: &str = "esp-rf-ook2";

/// Transmit power ceiling in quarter dBm. esp-radio starts the radio at 20
/// (5 dBm), far below ESP-IDF's 80 (20 dBm) default; 68 (17 dBm) is the
/// value esp32clock measured as the knee on its board. Raise it towards 80
/// if the serial log shows the AP hearing us much weaker than we hear it.
pub const WIFI_MAX_TX_POWER_QDBM: i8 = 68;

/// Frames the driver queues for the network stack before it drops new
/// ones. esp-radio's default of 5 overflowed on bursts of LAN broadcasts
/// (esp32clock saw eight frames in 3 ms), which cost the DHCP offer and a
/// 10 s retry at boot. The queue holds references to the driver's own
/// buffers, so a slot costs a few bytes of heap.
const RX_QUEUE_FRAMES: usize = 32;

/// Pause between a lost connection or a failed connect and the next try.
const RECONNECT_DELAY: Duration = Duration::from_secs(5);

/// Wait after the driver reports the station connected before DHCP starts.
/// The driver says connected before it can pass a frame; a discover sent at
/// that moment is lost and smoltcp waits its full 10 s discover timeout
/// before the next one.
const LINK_SETTLE: Duration = Duration::from_millis(500);

/// How often the station logs its signal and AP while connected.
const STATUS_PERIOD: Duration = Duration::from_secs(60);

/// Failed connects in a row before the chip is reset.
const CONNECT_RESET_ATTEMPTS: u32 = 10;

/// Per-channel dwell of the candidate scan. The driver default of 10 to
/// 20 ms misses every AP but a strong one, since a far AP's probe response
/// takes longer than that to arrive; ESP-IDF's own default is 120 ms.
const SCAN_DWELL_MIN: esp_hal::time::Duration = esp_hal::time::Duration::from_millis(120);
const SCAN_DWELL_MAX: esp_hal::time::Duration = esp_hal::time::Duration::from_millis(200);

/// Port of the throwaway socket in `poke`.
const POKE_PORT: u16 = 12346;

/// Sockets the stack can hold at once: DHCP, DNS, the one NTP or MQTT
/// socket the mutex allows, the `poke` socket, and headroom.
static RESOURCES: StaticCell<StackResources<6>> = StaticCell::new();

pub struct Wifi {
    pub stack: Stack<'static>,
}

#[derive(Debug)]
pub enum Error {
    Controller(WifiError),
}

impl Wifi {
    /// Bring up the driver and the network stack and spawn the tasks that
    /// keep them running. Returns at once; the stack has no address until
    /// the connection task has associated and DHCP has answered, see
    /// [`Wifi::wait_for_ip`].
    pub fn new(
        wifi: esp_hal::peripherals::WIFI<'static>,
        rng: Rng,
        spawner: Spawner,
    ) -> Result<Self, Error> {
        let controller_config = ControllerConfig::default().with_rx_queue_size(RX_QUEUE_FRAMES);
        let controller = WifiController::new(wifi, controller_config).map_err(Error::Controller)?;
        let interface = Interface::station();

        let seed = (rng.random() as u64) << 32 | rng.random() as u64;
        let resources = RESOURCES.init(StackResources::new());

        // No IPv4 configuration yet: the connection task starts DHCP once
        // the station is associated and the link has settled.
        let (stack, runner) =
            embassy_net::new(interface, embassy_net::Config::default(), resources, seed);

        spawner.spawn(net_task(runner).expect("spawn net task"));
        spawner.spawn(connection(controller, stack).expect("spawn connection task"));

        Ok(Self { stack })
    }

    /// Wait until the stack has an IPv4 address.
    pub async fn wait_for_ip(&self) {
        info!("Waiting for network stack to be ready...");
        self.stack.wait_config_up().await;
        if let Some(config) = self.stack.config_v4() {
            info!(
                "Got IP: {} gateway {:?} dns {:?}",
                config.address, config.gateway, config.dns_servers
            );
        }
    }
}

/// A MAC address for the log.
struct Bssid([u8; 6]);

impl core::fmt::Display for Bssid {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let b = self.0;
        write!(
            f,
            "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            b[0], b[1], b[2], b[3], b[4], b[5]
        )
    }
}

/// The station configuration from the build-time credentials. An empty
/// password means an open network.
fn station_config() -> Result<Config, WifiError> {
    let auth = if PASSWORD.is_empty() {
        AuthenticationMethodConfig::Open
    } else {
        AuthenticationMethodConfig::Wpa2Personal(PASSWORD.try_into()?)
    };
    Ok(Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into()?)
            .with_authentication(auth)
            .with_scan_method(ScanMethod::AllChannels),
    ))
}

/// Log every access point on our SSID with its channel and signal. About
/// two seconds per scan at the dwell above.
async fn log_candidates(controller: &mut WifiController<'static>) {
    let Ok(ssid) = SSID.try_into() else { return };
    let scan = ScanConfig::default()
        .with_ssid(ssid)
        .with_scan_type(ScanTypeConfig::Active {
            min: SCAN_DWELL_MIN,
            max: SCAN_DWELL_MAX,
        });
    match controller.scan_async(&scan).await {
        Ok(aps) if aps.is_empty() => warn!("WiFi: scan found no AP for {SSID}"),
        Ok(aps) => {
            for ap in aps {
                info!(
                    "WiFi: candidate {} channel {} rssi {}",
                    Bssid(ap.bssid),
                    ap.channel,
                    ap.signal_strength
                );
            }
        }
        Err(e) => warn!("WiFi: scan failed: {e:?}"),
    }
}

/// The signal and the AP as the driver sees them. Both come from the last
/// beacon received, so a station that has stopped hearing its AP shows a
/// stale RSSI here.
fn log_status(controller: &WifiController<'static>, since: Instant) {
    let up = since.elapsed().as_secs();
    match (controller.rssi(), controller.ap_info()) {
        (Ok(rssi), Ok(ap)) => {
            info!(
                "WiFi: up {up} s, rssi {rssi} dBm, bssid {} channel {}",
                Bssid(ap.bssid),
                ap.channel
            )
        }
        (rssi, ap) => warn!("WiFi: up {up} s, rssi {rssi:?}, ap {ap:?}"),
    }
}

/// What a connect failure most likely means, for the log.
fn failure_hint(e: &ConnectionError) -> &'static str {
    match e {
        ConnectionError::Failed(info) => match info.reason {
            DisconnectReason::NoAccessPointFound => ", no such network in range",
            DisconnectReason::FourWayHandshakeTimeout | DisconnectReason::HandshakeTimeout => {
                ", wrong password?"
            }
            _ => "",
        },
        _ => "",
    }
}

/// Start DHCP with our hostname. The runner is poked because embassy-net
/// does not wake it for a configuration change; with nothing else pending
/// it would send the discover only when the next packet happened to arrive.
fn start_dhcp(stack: Stack<'static>) {
    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname = String::try_from(HOSTNAME).ok();
    stack.set_config_v4(ConfigV4::Dhcp(dhcp_config));
    poke(stack);
}

/// Drop DHCP and the address with it.
fn stop_dhcp(stack: Stack<'static>) {
    stack.set_config_v4(ConfigV4::None);
}

/// Wake the stack runner. Any socket operation does; a bind on a throwaway
/// UDP socket is the cheapest.
fn poke(stack: Stack<'static>) {
    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; 1];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_buf = [0u8; 1];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    let _ = socket.bind(POKE_PORT);
}

/// Keep the station connected and run DHCP only while it is.
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>, stack: Stack<'static>) -> ! {
    info!("Start connection task");
    // SSID and password are build-time constants, so a bad one is a build
    // misconfiguration rather than something to retry.
    let config = station_config().expect("SSID or PASSWORD is not a valid Wi-Fi credential");

    let mut failures: u32 = 0;
    loop {
        if let Err(e) = controller.set_config(&config) {
            error!("WiFi: set_config: {e:?}");
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }
        // The driver resets the ceiling to its default on every start,
        // which `set_config` does, so it is raised again each time.
        match controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM) {
            Ok(()) => info!(
                "WiFi: max tx power {} dBm",
                WIFI_MAX_TX_POWER_QDBM as f32 / 4.0
            ),
            Err(e) => warn!("WiFi: set_max_tx_power: {e:?}"),
        }
        log_candidates(&mut controller).await;
        info!("WiFi: connecting to {SSID}");
        match controller.connect_async().await {
            Ok(info) => {
                info!(
                    "WiFi: connected to {} channel {}",
                    Bssid(info.bssid),
                    info.channel
                );
                failures = 0;
            }
            Err(e) => {
                failures += 1;
                warn!(
                    "WiFi: connect failed: {e:?}{} (attempt {failures} of {CONNECT_RESET_ATTEMPTS})",
                    failure_hint(&e)
                );
                if failures >= CONNECT_RESET_ATTEMPTS {
                    panic!("WiFi: no connection after {failures} attempts, resetting");
                }
                Timer::after(RECONNECT_DELAY).await;
                continue;
            }
        }
        let since = Instant::now();
        Timer::after(LINK_SETTLE).await;
        start_dhcp(stack);
        loop {
            match select(
                controller.wait_for_disconnect_async(),
                Timer::after(STATUS_PERIOD),
            )
            .await
            {
                Either::First(Ok(info)) => {
                    warn!(
                        "WiFi: disconnected from {} after {} s: {:?}, rssi {}",
                        Bssid(info.bssid),
                        since.elapsed().as_secs(),
                        info.reason,
                        info.rssi
                    );
                    break;
                }
                Either::First(Err(e)) => {
                    warn!(
                        "WiFi: disconnect wait failed after {} s: {e:?}",
                        since.elapsed().as_secs()
                    );
                    break;
                }
                Either::Second(()) => log_status(&controller, since),
            }
        }
        stop_dhcp(stack);
        Timer::after(RECONNECT_DELAY).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) -> ! {
    runner.run().await
}
