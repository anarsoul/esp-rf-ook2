# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A `no_std` Rust firmware for ESP32 (Xtensa) that decodes OOK-modulated 433MHz
signals from a Nexus-TH thermal/humidity sensor, received via an RXB6 module
on GPIO21, and publishes readings to MQTT over WiFi. It's a rewrite of
[esp-rf-ook](https://github.com/anarsoul/esp-rf-ook) using `esp-hal` +
`embassy` + `esp-rtos` instead of ESP-IDF/C.

The Nexus-TH protocol details (pulse timings, 36-bit payload layout) are
documented in README.md and mirrored in the constants/comments in
`src/decoder.rs`.

## Build environment

This targets `xtensa-esp32-none-elf` and requires the espressif Rust
toolchain (`channel = "esp"` in `rust-toolchain.toml`), not upstream
stable/nightly. `.cargo/config.toml` sets the target, `build-std`, and the
runner (`espflash flash --monitor`).

Run `source ~/export-esp.sh` before any `cargo build`/`clippy`/`run` —
it puts the xtensa-gcc toolchain on `PATH`, which the linker needs. Without
it the build fails at the link step, not at compile time.

Required env vars at build time (baked in via `env!()` in `src/lib.rs`, so
missing ones are a hard build failure): `SSID`, `PASSWORD`, `MQTT_SERVER`,
`MQTT_LOGIN`, `MQTT_PASSWORD`, `MQTT_TOPIC`. Locally these live in `.env`
(gitignored); CI (`.github/workflows/rust_ci.yml`) sets dummy values
directly in the workflow env.

## Common commands

```sh
cargo build --release        # what CI builds; debug uses opt-level "s" too, since unoptimized debug is too slow on-device
cargo fmt --all -- --check
cargo clippy --all-features --workspace -- -D warnings
cargo run --release          # builds, flashes over USB via espflash, and opens serial monitor (see .cargo/config.toml runner)
```

There is no test suite — this crate is `no_std`/embedded and has no
`#[test]` targets; correctness is checked via `build`, `fmt`, and `clippy`
in CI, matching the three CI matrix jobs.

`ESP_LOG` is set to `info` by default via `.cargo/config.toml`.

## Architecture

`src/bin/main.rs` is the entry point and owns the top-level control flow:
init peripherals → bring up WiFi → sync time via NTP → arm a hardware
watchdog → loop reading RMT pulse trains and decoding/publishing sensor
data. Library code lives in `src/lib.rs` and is split into independent
modules, each owning one concern:

- **`decoder.rs`** — pure, no-I/O signal decoding. Converts raw RMT
  `PulseCode` pulse-width samples into a `SensorData` via a `packed_struct`
  bitfield (`NexusTHPayload`), with range/channel/checksum-style validation
  returning a `DecodeError`. Safe to reason about and modify without
  touching networking code.
- **`wifi.rs`** — owns the `esp_radio` WiFi controller and the
  `embassy-net` `Stack`. Runs its own reconnect loop as a spawned embassy
  task (`connection`) driven by a `Signal`, plus a `net_task` running the
  `embassy-net` `Runner`. Exposes a `Wifi` struct with `stack: Stack<'static>`.
- **`ntpc.rs`** / **`mqtt.rs`** — both follow the same pattern: hold a
  shared `&'static Mutex<NoopRawMutex, Stack<'static>>` (the WiFi stack is
  shared across NTP/MQTT via a mutex since embassy-net sockets need
  exclusive stack access), cache a DNS-resolved `IpAddress` after first
  lookup, and clear the cached address on any I/O failure so the next call
  re-resolves. Each socket (UDP for NTP, TCP for MQTT) is opened fresh per
  call and explicitly flushed/closed/aborted with short delays to let the
  smoltcp stack process closure before the buffers go out of scope — do not
  "simplify" this away, it's there because these are stack-allocated
  buffers, not owned/leaked ones (see `1bd2c1d Stop sharing buffers for the
  sockets`).

Cross-task coordination in `main.rs` uses an `embassy_sync::channel::Channel`
(`MQTT_CHANNEL`) to hand decoded `SensorData` + timestamp from the main RMT
receive loop to a separate `mqtt_publisher` embassy task, so a slow/blocked
MQTT publish never stalls signal decoding. `mk_static!` (local macro) is
the stable-Rust substitute for `static_cell::make_static!` — used anywhere
a value needs `'static` lifetime for a spawned task.

The hardware watchdog (`Mwdt`, 30s timeout) is fed (`wdt.feed()`) at each
loop iteration and around blocking operations; the `mqtt_publisher` task
deliberately `panic!`s if no publish has succeeded in 5 minutes so the
watchdog resets the device rather than running silently degraded.

`radium-0.7.0/` is a vendored crate pulled in via `[patch.crates-io]` in
`Cargo.toml` — not part of this project's own code, don't edit it as if it
were.
