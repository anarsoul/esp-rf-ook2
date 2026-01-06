use core::net::SocketAddr;
use embassy_net::{
    IpAddress, Stack,
    dns::DnsQueryType,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

use crate::{NTP_SERVER, RX_BUFFER_SIZE, TX_BUFFER_SIZE};

use sntpc::{NtpContext, NtpTimestampGenerator, get_time};

use embassy_futures::select::{Either, select};

use log::warn;

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
    addr: Option<IpAddress>,
}

#[derive(Debug)]
pub enum NtpcError {
    DnsResolveFailed,
    SocketBindFailed,
    NetworkError,
    Timeout,
}

impl Ntpc {
    pub fn new(stack: &'static Mutex<NoopRawMutex, Stack<'static>>) -> Self {
        Ntpc { stack, addr: None }
    }

    pub async fn get_time(&mut self) -> Result<u64, NtpcError> {
        let stack = self.stack.lock().await;
        let mut tx_buf: [u8; TX_BUFFER_SIZE] = [0; TX_BUFFER_SIZE];
        let mut rx_buf: [u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE];

        let mut rx_meta = [PacketMetadata::EMPTY; 16];
        let mut tx_meta = [PacketMetadata::EMPTY; 16];

        // Cache address after first resolution
        if self.addr.is_none() {
            let addr = stack
                .dns_query(NTP_SERVER, DnsQueryType::A)
                .await
                .map_err(|_| NtpcError::DnsResolveFailed)?
                .first()
                .copied()
                .ok_or(NtpcError::DnsResolveFailed)?;
            self.addr = Some(addr);
        }

        let addr = self.addr.unwrap();

        let mut socket =
            UdpSocket::new(*stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);

        socket.bind(123).map_err(|e| {
            self.addr = None; // Clear cached address on failure
            warn!("Failed to bind NTP socket: {:?}", e);
            NtpcError::SocketBindFailed
        })?;

        let a = get_time(
            SocketAddr::from((addr, 123)),
            &socket,
            NtpContext::new(Timestamp { current_time_us: 0 }),
        );
        let b = Timer::after(Duration::from_secs(5));

        let result = select(a, b).await;

        let result = match result {
            Either::First(res) => {
                res.map(|r| r.sec() as u64).map_err(|e| {
                    self.addr = None; // Clear cached address on failure
                    warn!("NTP get_time error: {:?}", e);
                    NtpcError::Timeout
                })
            }
            Either::Second(_) => Err(NtpcError::Timeout),
        };

        socket.flush().await;
        Timer::after(Duration::from_millis(100)).await;
        socket.close();
        // Give stack some time to process the socket closure
        Timer::after(Duration::from_millis(100)).await;

        result
    }
}
