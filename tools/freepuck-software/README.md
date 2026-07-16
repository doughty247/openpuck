# freepuck-software

Software-only BLE bridge for the Steam Controller 2. Pairs directly over
Bluetooth and presents the controller to the OS as a virtual XInput,
DualSense-equivalent, or Switch-Pro-layout controller — no ESP32 dongle, no
Steam running, no additional hardware.

The report parser, output formatters, and GATT command builders live in
[`crates/steam-protocol`](../../crates/steam-protocol) — a `no_std` crate
this bridge shares with the repo's ESP32-S3 firmware (`src/controller.rs`
and parts of `src/bluetooth.rs` there are thin wrappers around the same
crate), rather than two independently-maintained copies. Only the BLE
transport (`ble.rs`, `btleplug`-based here vs. `trouble-host` in the
firmware) and output device wiring (ViGEmBus/uinput here vs. USB gadget mode
in the firmware) are platform-specific and stay local to each. See
[`docs/ble-protocol.md`](docs/ble-protocol.md) for the protocol reference.

## Status

Phase 1 (BLE connect/parse/print) builds and runs cleanly, including on this
development container (verified by starting a D-Bus session and confirming
`btleplug` correctly detects "no adapter" rather than crashing). Phases 2-5
(virtual controller output, rumble forwarding) are implemented and build, but
**have not been exercised against a real Steam Controller, ViGEmBus install,
or game** — that requires hardware this environment doesn't have. Treat
anything beyond `scan` as needing a first real-hardware pass before trusting
it.

## Usage

```bash
# Print live input state to stdout — no virtual controller, just validates
# the BLE connection and parser.
cargo run -- scan

# Present the controller to the OS as a virtual controller.
cargo run -- run --mode xinput      # Xbox 360 (ViGEmBus / uinput) — most reliable
cargo run -- run --mode dualsense   # DualShock 4-equivalent — PC only, see below
cargo run -- run --mode switch      # Nintendo button layout — local PC only, see below
```

### Linux

Needs `/dev/uinput` access (root, or add yourself to the `input` group and
have a udev rule granting write access) and BlueZ running. System packages:
`libdbus-1-dev`, `libudev-dev` (build-time only).

### Windows

Needs [ViGEmBus](https://github.com/ViGEm/ViGEmBus/releases) installed
separately — it's a signed kernel driver and this tool does not (and cannot)
bundle it.

### macOS

Not implemented. HID injection on macOS requires additional entitlements and
is more restricted than Windows/Linux; deferred.

## Honest limitations

- **DualSense mode is PC-only.** A PS5 will not recognize this as a genuine
  DualSense under any circumstances — Sony requires cryptographic
  authentication this bridge does not and cannot implement. On Windows,
  ViGEmBus emulates a DualShock 4 (its closest available Sony target), which
  has no gyro/touchpad fields and no rumble-notification support in the
  `vigem-client` crate used here. On Linux, `--mode dualsense` builds a
  generic uinput gamepad with DualSense-like vendor/product IDs and button
  naming — real gyro/touchpad/rumble-over-hidraw are not emulated.
- **Switch mode does not bridge to a real Nintendo Switch console**, on
  either platform. The original ESP32 firmware's Switch mode works because
  the dongle is a USB *device* plugged into the Switch's dock — a desktop PC
  cannot present itself as a USB device to external host hardware the way a
  microcontroller can. `--mode switch` on Windows returns an error explaining
  this (ViGEmBus has no Switch Pro target at all). On Linux it builds a local
  generic uinput gamepad using Nintendo-style button naming/layout — useful
  only for PC games that expect that convention, not for playing on an actual
  Switch.
- **Rumble forwarding only works in Windows `--mode xinput`.** ViGEmBus's
  Xbox 360 target supports a rumble notification callback; its DualShock 4
  target does not (in the `vigem-client` version used here). The Linux
  `uinput` crate this bridge uses has no force-feedback (`EV_FF`) support at
  all, so no output mode forwards rumble on Linux. Games will not feel
  vibration through this bridge on Linux, or through DualSense/Switch mode on
  Windows.
- **Latency.** A software BLE bridge on a desktop host adds OS Bluetooth
  stack latency on top of the 7.5 ms BLE radio leg — this has not been
  measured yet (needs real hardware) and should not be assumed equivalent to
  the dedicated-radio ESP32 dongle until it is.
- **Gyro/accelerometer/real trackpad are provisional, not hardware-confirmed.**
  `parse_steam` now reads them from a full-length (45+ byte) notification,
  based on cross-referencing [safijari/openpuck](https://github.com/safijari/openpuck) —
  an independent reverse-engineering of the same physical controller over a
  *different* transport (2.4GHz RF, not BLE). The byte offsets line up
  exactly where that project's documented report layout says they should
  once you account for BLE dropping the leading report-ID byte, which is
  reasonable evidence, but nobody has captured a real BLE notification from
  this project to confirm it. Treat gyro/accel/trackpad data (and the
  digital L2/R2 click bits) as likely-correct-but-unverified until someone
  runs `scan` against real hardware and checks. Short (17-byte) reports
  still fall back to zeroed IMU and a stick-deflection proxy for trackpad
  position, same as before this existed.

## Development

```bash
# The repo root pins an embedded (xtensa-esp32s3-none-elf) build target for
# the ESP32 firmware, which this subproject must NOT inherit — always pass
# --target explicitly (or use the cargo aliases below once you set them up):
cargo build --target x86_64-unknown-linux-gnu   # or your platform's host triple
cargo test --target x86_64-unknown-linux-gnu
```
