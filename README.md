# Teensy CAN Bring-Up

Standalone Rust bring-up project for Teensy 4.0 CAN testing.

This repo contains:

- Teensy 4.0 / i.MX RT1062 CAN firmware
- a local Teensy flashing runner
- a host-side terminal dashboard for the CAN log stream

The project uses the published `teensy4-bsp` crate and does not require a sibling `../teensy4-rs` checkout.

## Clone project

```bash
git clone https://github.com/OIEIEIO/teensy-can-bringup
cd teensy-can-bringup
```
Example project folder after cloning:

```text
~/teensy-can-bringup
```

## Project layout

```text
.
├── src/main.rs                  # Teensy CAN firmware
├── .cargo/config.toml           # Embedded target and local runner config
├── build.rs                     # Teensy 4 linker/runtime setup
├── tools/runner.rs              # Local Teensy flashing runner
├── tools/can-dashboard/         # Host-side CAN dashboard
└── docs/Bring-Up-Notes.MD       # Bring-up notes
```

## Requirements

Required host tools:

```text
Rust toolchain
thumbv7em-none-eabihf Rust target
rust-objcopy
teensy_loader_cli
```

Install the required Rust target:

```bash
rustup target add thumbv7em-none-eabihf
```

## Build firmware

From the repo root:

```bash
cd ~/teensy-rust-test/teensy-can-bringup

cargo build --release --target thumbv7em-none-eabihf
```

## Flash Teensy

From the repo root:

```bash
cd ~/teensy-rust-test/teensy-can-bringup

cargo run --release --target thumbv7em-none-eabihf
```

When prompted, press the Teensy program/reset button.

The local Cargo runner builds the HEX file and calls `teensy_loader_cli`.

## Build dashboard

From the repo root:

```bash
cd ~/teensy-can-bringup

cargo build \
  --release \
  --manifest-path tools/can-dashboard/Cargo.toml \
  --target x86_64-unknown-linux-gnu
```

The dashboard binary is built under the tools workspace target directory:

```text
tools/target/x86_64-unknown-linux-gnu/release/can-dashboard
```

## Run dashboard

This is the tested dashboard run command:

```bash
cd ~/teensy-can-bringup/tools/can-dashboard

cat /dev/ttyACM0 | tee /tmp/can_raw2.log | ~/teensy-rust-test/teensy-can-bringup/tools/target/x86_64-unknown-linux-gnu/release/can-dashboard
```

This reads the Teensy serial stream, saves a raw log copy to `/tmp/can_raw2.log`, and pipes the same stream into the dashboard.

## ComChan compatibility

The firmware emits line-oriented CAN bring-up records over the serial/log stream.

The output is compatible with ComChan-style serial monitoring workflows, and the included dashboard consumes the same record stream.

Dashboard display includes:

```text
CAN status
CAN1/CAN2 TX/RX state
frame grid
message watch
error counters
timing/rate/utilization
decoded frame data
```

## Current status

Confirmed working:

```text
Teensy 4.0 firmware build
Teensy flash through local runner
CAN dashboard build
Dashboard run from /dev/ttyACM0 pipe
Standalone crates.io BSP dependency
No active ../teensy4-rs project dependency
```

## License

This project is licensed under the MIT License.
See [LICENSE](LICENSE) for details.
