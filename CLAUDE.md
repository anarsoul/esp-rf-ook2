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
`decoder/src/lib.rs`.

This is a two-member Cargo workspace: the root package (`esp-rf-ook2`,
`no_std`, xtensa-only) and `decoder/` (the pure protocol-decode logic,
`no_std` on-device but plain-testable on the host — see "Testing" below).

## Build environment

This targets `xtensa-esp32-none-elf` and requires the espressif Rust
toolchain (`channel = "esp"` in `rust-toolchain.toml`), not upstream
stable/nightly. `.cargo/config.toml` sets the target, `build-std`, and the
runner (`espflash flash --monitor`).

`rust-toolchain.toml` can't pin a version — espup always installs the
toolchain as plain `esp` — so the real pin is the `version:` input to
`esp-rs/xtensa-toolchain` in the CI workflow, currently `1.97.0`. If a
local build behaves differently from CI, check `rustc +esp -vV` against
that and `espup update` if it's behind.

Run `source ~/export-esp.sh` before any `cargo build`/`clippy`/`run` —
it puts the xtensa-gcc toolchain on `PATH`, which the linker needs. Without
it the build fails at the link step, not at compile time.

`.cargo/config.toml` also sets `ESP_HAL_CONFIG_MIN_CHIP_REVISION="0"`.
esp-hal 1.2 dropped support for ESP32 silicon below v3.0 (esp-hal PR
#5812, issue #5656: pre-v3.0 chips have a dual-core read erratum that
corrupts `Instant::now`, issue #4903, and no workaround was accepted).
The default of 300 is baked into the ELF metadata, which espflash 4.7+
refuses to flash onto an older chip, and `esp_hal::init` asserts it at
boot. The board this project runs on is an ESP32 v1.0, so the floor is
lowered; this firmware only ever runs the first core. Do not remove it.

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

The root crate has no test suite — it's `no_std`/embedded with no `#[test]`
targets; correctness there is checked via `build`, `fmt`, and `clippy` in
CI, matching the three CI matrix jobs.

`decoder/` does have host-runnable unit tests (`decoder/src/lib.rs`,
`#[cfg(test)] mod tests`), since it has no hardware dependency:

```sh
cargo +stable test-decoder   # alias in .cargo/config.toml
```

The `+stable` is load-bearing, not incidental. `.cargo/config.toml` sets
`[unstable] build-std`, which is required for the xtensa target (the esp
toolchain ships no prebuilt `core`/`alloc` for `xtensa-esp32-none-elf`)
but breaks any host-target build: it recompiles `core` from source, which
then collides with the prebuilt `std` the test harness links against
("duplicate lang item: sized"). `build-std` can't be scoped to one target
and can't be un-set by a nested config — Cargo *joins* array-valued config
keys across files rather than letting the deeper one replace them. The
stable toolchain sidesteps all of it by ignoring `[unstable]` keys
outright, and since the decoder crate is plain stable Rust, testing it
there is honest rather than a workaround.

Relatedly, keep linker `rustflags` under `[target.xtensa-esp32-none-elf]`
and *not* `[build]` — `[build] rustflags` applies to every target, so
`-nostartfiles` would leak into host test binaries and produce an
executable with no entry point that segfaults before `main`.

CI runs the tests as a separate `decoder-tests` job (on stable) in
`.github/workflows/rust_ci.yml`.

`ESP_LOG` is set to `info` by default via `.cargo/config.toml`.

## Architecture

`src/bin/main.rs` is the entry point and owns the top-level control flow:
init peripherals → bring up WiFi → sync time via NTP → arm a hardware
watchdog → loop reading RMT pulse trains and decoding/publishing sensor
data. Library code lives in `src/lib.rs` and is split into independent
modules, each owning one concern:

- **`decoder` crate** (`decoder/src/lib.rs`) — pure, no-I/O signal
  decoding, and the one part of this codebase with unit tests. Converts a
  `Pulse`/`PulseLevel` pulse-width sample train into a `SensorData` via a
  `packed_struct` bitfield (`NexusTHPayload`), with range/channel/checksum-
  style validation returning a `DecodeError`. It tolerates up to
  `MAX_LEADING_JUNK` (2) extra codes ahead of a frame: a slicer-chatter
  junk pulse from a weak signal, or the partial gap recorded when the RMT
  was re-armed mid-gap, which shifts every code to (gap, pulse). Both were
  seen on the board to turn most of a Nexus burst's repeats into 38/39-code
  trains that a strict 37-code check threw away. `Pulse`/`PulseLevel` are
  local stand-ins for `esp_hal::rmt::PulseCode`/`esp_hal::gpio::Level` —
  this crate has no `esp_hal` dependency on the host target (only
  target-gated for `xtensa`, to keep the `#[ram]` placement of hot
  functions on-device); `main.rs` converts `PulseCode` → `Pulse` at the
  RMT call site (`to_pulse`/`to_pulse_level`). It also holds `Vote`, the
  per-burst tally `main.rs` feeds every successful decode into: the sensor
  repeats one frame about a dozen times per burst, a weak signal yields
  only some of them, and the protocol has no checksum, so a reading is
  published only when at least `MIN_VOTES` (2) decodes in the burst agree
  (grouped by `SensorData::equal`, which ignores id/channel/battery) and
  nothing ties with it. The old "same reading four times in a row" counter
  carried state across bursts and never published a reading that changed
  every minute. Safe to reason about and modify without touching
  networking code.
- **`wifi.rs`** — owns the `esp_radio` WiFi controller and the
  `embassy-net` `Stack`. Runs its own reconnect loop as a spawned embassy
  task (`connection`), plus a `net_task` running the `embassy-net`
  `Runner`. Exposes a `Wifi` struct with `stack: Stack<'static>`. The
  station picks its access point deliberately because esp-radio does not
  roam and its default `Fast` scan takes the first beacon it hears:
  `ScanMethod::AllChannels` makes the driver scan everything and connect to
  the strongest BSSID, a candidate scan before each connect logs every AP
  on the SSID with BSSID/channel/RSSI, and the transmit ceiling is raised
  from esp-radio's 5 dBm default (`WIFI_MAX_TX_POWER_QDBM`). The stack is
  created with no IPv4 config; DHCP is switched on 500 ms after each
  connect (the driver says "connected" before it can pass a frame, and a
  lost discover costs smoltcp's 10 s timeout) and off at disconnect. Ten
  failed connects in a row panic, which resets the chip via `custom_halt`.
  This mirrors `~/src/rust/esp32clock/src/net.rs`, the reference for Wi-Fi
  handling on esp-radio.
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
(`MQTT_CHANNEL`) to hand the burst's winning `SensorData` + timestamp from
the main RMT receive loop to a separate `mqtt_publisher` embassy task, so a
slow/blocked MQTT publish never stalls signal decoding. A burst is closed
`BURST_GAP` (2 s) after its last successful decode (failed repeats do not
extend it, and a burst lasts about a second); the loop's 1 s receive
timeout guarantees that check runs even when the air goes quiet. `mk_static!` (local macro) is
the stable-Rust substitute for `static_cell::make_static!` — used anywhere
a value needs `'static` lifetime for a spawned task.

The hardware watchdog (`Mwdt`, 30s timeout) is fed (`wdt.feed()`) at each
loop iteration and around blocking operations; the `mqtt_publisher` task
deliberately `panic!`s if no publish has succeeded in 5 minutes so the
watchdog resets the device rather than running silently degraded.
