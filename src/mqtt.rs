use embassy_net::{Stack, dns::DnsQueryType, tcp::TcpSocket};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

use crate::{MQTT_LOGIN, MQTT_PASSWORD, MQTT_SERVER, RX_BUFFER_SIZE, TX_BUFFER_SIZE};

use log::{info, warn};

use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig as MqttClientConfig},
    packet::v5::publish_packet::QualityOfService::QoS0,
    utils::rng_generator::CountingRng,
};

#[derive(Debug)]
pub enum Error {
    DnsResolveFailed,
    ConnectionFailed,
    PublishFailed,
    DisconnectFailed,
}

pub struct Mqtt {
    stack: &'static Mutex<NoopRawMutex, Stack<'static>>,
    rx_buf: &'static Mutex<NoopRawMutex, [u8; RX_BUFFER_SIZE]>,
    tx_buf: &'static Mutex<NoopRawMutex, [u8; TX_BUFFER_SIZE]>,
}

impl Mqtt {
    pub fn new(
        stack: &'static Mutex<NoopRawMutex, Stack<'static>>,
        rx_buf: &'static Mutex<NoopRawMutex, [u8; RX_BUFFER_SIZE]>,
        tx_buf: &'static Mutex<NoopRawMutex, [u8; TX_BUFFER_SIZE]>,
    ) -> Self {
        Mqtt {
            stack,
            rx_buf,
            tx_buf,
        }
    }

    pub async fn publish(&mut self, topic: &str, data: &str) -> Result<(), Error> {
        let stack = self.stack.lock().await;
        let mut tx_buf = self.tx_buf.lock().await;
        let mut rx_buf = self.rx_buf.lock().await;

        let addr = stack
            .dns_query(MQTT_SERVER, DnsQueryType::A)
            .await
            .map_err(|_| Error::DnsResolveFailed)?
            .first()
            .copied()
            .ok_or(Error::DnsResolveFailed)?;

        let mut socket = TcpSocket::new(*stack, &mut *rx_buf, &mut *tx_buf);
        socket.set_timeout(Some(Duration::from_secs(10)));
        socket
            .connect((addr, 1883))
            .await
            .map_err(|_| Error::ConnectionFailed)?;

        let mut config = MqttClientConfig::new(
            rust_mqtt::client::client_config::MqttVersion::MQTTv5,
            CountingRng(20000),
        );
        config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
        config.add_client_id("esp-rf-ook2");
        config.max_packet_size = 100;
        config.keep_alive = 30;

        config.add_username(MQTT_LOGIN);
        config.add_password(MQTT_PASSWORD);

        let mut writebuf = [0; 256];
        let mut readbuf = [0; 256];
        let mut client = {
            let writebuf_len = writebuf.len();
            let readbuf_len = readbuf.len();
            MqttClient::<_, 5, _>::new(
                &mut socket,
                &mut writebuf,
                writebuf_len,
                &mut readbuf,
                readbuf_len,
                config,
            )
        };

        client.connect_to_broker().await.map_err(|e| {
            warn!("Error: {:?}", e);
            Error::ConnectionFailed
        })?;

        info!("Connected to MQTT broker");

        client
            .send_message(topic, data.as_bytes(), QoS0, false)
            .await
            .map_err(|_| Error::PublishFailed)?;

        info!("Published to topic {}", topic);

        client
            .disconnect()
            .await
            .map_err(|_| Error::DisconnectFailed)?;

        socket.close();
        // Give stack some time to process the socket closure
        Timer::after(Duration::from_millis(100)).await;

        Ok(())
    }
}
