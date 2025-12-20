use core::net::{IpAddr, SocketAddr};
use embassy_net::{
    Stack,
    dns::DnsQueryType,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

use crate::{NTP_SERVER, RX_BUFFER_SIZE, TX_BUFFER_SIZE};

use sntpc::{Error, NtpContext, NtpTimestampGenerator, get_time};

use embassy_futures::select::{Either, select};

use log::info;

#[derive(Clone, Copy)]
struct Timestamp {
    current_time_us: u64,
}

impl NtpTimestampGenerator for Timestamp {
    fn init(&mut self) {
        self.current_time_us = 0;
    }

    fn timestamp_sec(&self) -> u64 {
        self.current_time_us / 1_000_000
    }

    fn timestamp_subsec_micros(&self) -> u32 {
        (self.current_time_us % 1_000_000) as u32
    }
}

pub struct Ntpc {
    stack: &'static Mutex<NoopRawMutex, Stack<'static>>,
    rx_buf: &'static Mutex<NoopRawMutex, [u8; RX_BUFFER_SIZE]>,
    tx_buf: &'static Mutex<NoopRawMutex, [u8; TX_BUFFER_SIZE]>,
}

impl Ntpc {
    pub fn new(
        stack: &'static Mutex<NoopRawMutex, Stack<'static>>,
        rx_buf: &'static Mutex<NoopRawMutex, [u8; RX_BUFFER_SIZE]>,
        tx_buf: &'static Mutex<NoopRawMutex, [u8; TX_BUFFER_SIZE]>,
    ) -> Self {
        Ntpc {
            stack,
            rx_buf,
            tx_buf,
        }
    }

    pub async fn get_time(&mut self) -> Option<u64> {
        let stack = self.stack.lock().await;
        let mut tx_buf = self.tx_buf.lock().await;
        let mut rx_buf = self.rx_buf.lock().await;

        let mut rx_meta = [PacketMetadata::EMPTY; 16];
        let mut tx_meta = [PacketMetadata::EMPTY; 16];

        let ntp_addrs = stack.dns_query(NTP_SERVER, DnsQueryType::A).await.unwrap();

        if ntp_addrs.is_empty() {
            panic!("Failed to resolve NTP server address");
        }

        let mut socket = UdpSocket::new(
            *stack,
            &mut rx_meta,
            &mut *rx_buf,
            &mut tx_meta,
            &mut *tx_buf,
        );

        socket.bind(123).unwrap();

        let addr: IpAddr = ntp_addrs[0].into();

        let a = get_time(
            SocketAddr::from((addr, 123)),
            &socket,
            NtpContext::new(Timestamp { current_time_us: 0 }),
        );
        let b = Timer::after(Duration::from_secs(5));

        let result = select(a, b).await;

        let result = match result {
            Either::First(res) => res,
            Either::Second(_) => Err(Error::Network),
        };

        let res = match result {
            Ok(time) => {
                info!("NTP time: {}", time.sec());
                Some(time.sec() as u64)
            }
            Err(e) => {
                info!("Failed to get NTP time: {:?}", e);
                None
            }
        };

        socket.close();
        // Give stack some time to process the socket closure
        Timer::after(Duration::from_millis(100)).await;

        res
    }
}
