// SPDX-License-Identifier: MIT

use core::num::NonZero;

use embassy_net::{IpAddress, Stack, dns::DnsQueryType, tcp::TcpSocket};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

use crate::{MQTT_LOGIN, MQTT_PASSWORD, MQTT_SERVER, RX_BUFFER_SIZE, TX_BUFFER_SIZE};

use log::{debug, warn};

use rust_mqtt::{
    Bytes,
    buffer::BumpBuffer,
    client::{
        Client,
        options::{ConnectOptions, DisconnectOptions, PublicationOptions, TopicReference},
    },
    config::KeepAlive,
    types::{MqttBinary, MqttString, TopicName},
};

/// Space for the dynamically sized fields of the packets the broker sends
/// us. We publish at QoS 0, so that is a CONNACK and nothing else.
const PACKET_BUFFER_SIZE: usize = 512;

const CLIENT_ID: &str = "esp-rf-ook2";
const KEEP_ALIVE_SECS: u16 = 30;

#[derive(Debug)]
pub enum Error {
    DnsResolveFailed,
    ConnectionFailed,
    /// The topic or a credential is not a valid MQTT string.
    InvalidArgument,
    PublishFailed,
    DisconnectFailed,
    FlushError,
}

pub struct Mqtt {
    stack: &'static Mutex<NoopRawMutex, Stack<'static>>,
    addr: Option<IpAddress>,
}

impl Mqtt {
    pub fn new(stack: &'static Mutex<NoopRawMutex, Stack<'static>>) -> Self {
        Mqtt { stack, addr: None }
    }

    pub async fn publish(&mut self, topic: &str, data: &str) -> Result<(), Error> {
        let stack = self.stack.lock().await;
        let mut tx_buf: [u8; TX_BUFFER_SIZE] = [0; TX_BUFFER_SIZE];
        let mut rx_buf: [u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE];

        // Cache address after first resolution
        if self.addr.is_none() {
            let addr = stack
                .dns_query(MQTT_SERVER, DnsQueryType::A)
                .await
                .map_err(|e| {
                    warn!("DNS resolve failed: {:?}", e);
                    Error::DnsResolveFailed
                })?
                .first()
                .copied()
                .ok_or(Error::DnsResolveFailed)?;
            self.addr = Some(addr);
        }

        let addr = self.addr.unwrap();

        let mut socket = TcpSocket::new(*stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(10)));
        socket.connect((addr, 1883)).await.map_err(|e| {
            self.addr = None; // Clear cached address on failure
            warn!("Error: {:?}", e);
            Error::ConnectionFailed
        })?;

        let res = self.publish_inner(&mut socket, topic, data).await;

        // We publish at QoS0, so nothing is acknowledged at the MQTT layer:
        // the only evidence the message actually left the device is the
        // socket draining. Treat a failed flush as a failed publish so the
        // caller doesn't record a success that never made it to the broker.
        let flushed = socket.flush().await.map_err(|e| {
            self.addr = None; // Clear cached address on failure
            warn!("Failed to flush socket: {:?}", e);
            Error::FlushError
        });

        Timer::after(Duration::from_millis(100)).await;
        socket.close();
        // Give stack some time to process the socket closure
        Timer::after(Duration::from_millis(100)).await;
        socket.abort();

        // Keep the original error if there was one -- a flush failure that
        // follows it is a downstream symptom, not the root cause.
        res.and(flushed)
    }

    async fn publish_inner<'a>(
        &mut self,
        socket: &mut TcpSocket<'a>,
        topic: &str,
        data: &str,
    ) -> Result<(), Error> {
        let topic =
            TopicName::new(MqttString::try_from(topic).map_err(|_| Error::InvalidArgument)?)
                .ok_or(Error::InvalidArgument)?;
        let user_name = MqttString::try_from(MQTT_LOGIN).map_err(|_| Error::InvalidArgument)?;
        let password = MqttBinary::try_from(MQTT_PASSWORD).map_err(|_| Error::InvalidArgument)?;
        let client_id = MqttString::try_from(CLIENT_ID).map_err(|_| Error::InvalidArgument)?;

        let mut packet_buf = [0u8; PACKET_BUFFER_SIZE];
        let mut buffer = BumpBuffer::new(&mut packet_buf);
        // One connection, one QoS 0 publish, no subscriptions: the smallest
        // session the client allows. The socket is lent, not moved, so it
        // can be flushed and closed by the caller whatever happens here.
        let mut client = Client::<'_, '_, _, _, 0, 1, 1, 0, 0, 0, 0>::new(&mut buffer);

        let connect_options = ConnectOptions::new()
            .clean_start()
            .keep_alive(KeepAlive::Seconds(
                NonZero::new(KEEP_ALIVE_SECS).expect("keep-alive is non-zero"),
            ))
            .user_name(user_name)
            .password(password);

        client
            .connect(&mut *socket, &connect_options, Some(client_id))
            .await
            .map_err(|e| {
                self.addr = None; // Clear cached address on failure
                warn!("Error: {:?}", e);
                Error::ConnectionFailed
            })?;

        debug!("Connected to MQTT broker");

        let publication = PublicationOptions::new(TopicReference::Name(topic.as_borrowed()));
        client
            .publish(&publication, Bytes::from(data.as_bytes()))
            .await
            .map_err(|e| {
                self.addr = None; // Clear cached address on failure
                warn!("Error: {:?}", e);
                Error::PublishFailed
            })?;

        debug!("Published to topic {}", topic.as_ref().as_str());

        client
            .disconnect(&DisconnectOptions::new())
            .await
            .map_err(|e| {
                self.addr = None; // Clear cached address on failure
                warn!("Error: {:?}", e);
                Error::DisconnectFailed
            })?;

        Ok(())
    }
}
