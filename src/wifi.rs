use esp_hal::rng::Rng;
use esp_radio::{
    Controller,
    wifi::{
        ClientConfig, ModeConfig, ScanConfig, WifiController, WifiDevice, WifiEvent, WifiStaState,
    },
};

use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use heapless::String;
use log::{info, warn};
use static_cell::StaticCell;

use crate::{PASSWORD, SSID};

static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
static LINK_STATE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

pub struct Wifi {
    pub stack: Stack<'static>,
}

#[derive(Debug)]
pub enum Error {}

impl Wifi {
    pub async fn new(
        radio_init: &'static Controller<'static>,
        wifi: esp_hal::peripherals::WIFI<'static>,
        rng: Rng,
        spawner: Spawner,
    ) -> Result<Self, Error> {
        let config = esp_radio::wifi::Config::default().with_rx_queue_size(10);
        let (wifi_controller, interfaces) = esp_radio::wifi::new(radio_init, wifi, config)
            .expect("Failed to initialize Wi-Fi controller");

        let wifi_interface = interfaces.sta;

        let mut dhcp_config: DhcpConfig = Default::default();
        let hostname: String<32> = String::try_from("esp-rf-ook2").unwrap();
        dhcp_config.hostname = Some(hostname);
        let config = embassy_net::Config::dhcpv4(dhcp_config);

        let seed = (rng.random() as u64) << 32 | rng.random() as u64;

        let resources = RESOURCES.init(StackResources::new());

        spawner.spawn(connection(wifi_controller)).ok();
        info!("Waiting for link to come up...");
        loop {
            let link_is_up = LINK_STATE.wait().await;
            Timer::after(Duration::from_millis(500)).await;
            if link_is_up {
                break;
            }
        }
        info!("Link is up, starting stack");

        let (stack, runner) = embassy_net::new(wifi_interface, config, resources, seed);
        spawner.spawn(net_task(runner)).ok();

        Ok(Self { stack })
    }

    pub async fn wait_for_ip(&self) -> Result<(), Error> {
        info!("Waiting for network stack to be ready...");
        loop {
            if self.stack.is_link_up() {
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        info!("Waiting to get IP address...");
        loop {
            if let Some(config) = self.stack.config_v4() {
                info!("Got IP: {}", config.address);
                break;
            }
            info!("Waiting...");
            Timer::after(Duration::from_millis(1000)).await;
        }
        Ok(())
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("Start connection task");
    info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        if esp_radio::wifi::sta_state() == WifiStaState::Connected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into()),
            );
            controller.set_config(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");

            info!("Scan");
            let scan_config = ScanConfig::default().with_max(10);
            let result = controller
                .scan_with_config_async(scan_config)
                .await
                .unwrap();
            for ap in result {
                info!("{ap:?}");
            }
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => {
                info!("Wifi connected!");
                LINK_STATE.signal(true);
            }
            Err(e) => {
                warn!("Failed to connect to wifi: {:?}", e);
                LINK_STATE.signal(false);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
