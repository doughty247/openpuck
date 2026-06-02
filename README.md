# OpenPuck

Steam Controller 2026 bridge firmware for ESP32-S3.

## TL;DR (Gamers)

- Plug an ESP32-S3 dongle into PC / Switch / PS5
- Pair your Steam Controller
- Cycle output mode with BOOT: XInput -> Switch Pro -> DualSense
- XInput works best right now

Current caveats:

- No native Steam Controller mode yet
- Input mode toggling is still finicky
- Rumble currently reliable only in XInput mode


## What It Does

OpenPuck reads Steam Controller BLE input and exposes a USB gamepad profile to the host:

- XInput
- Switch Pro
- DualSense

Mode selection is persisted in flash.

## Build Firmware

```bash
cargo install espup && espup install
. ~/export-esp.sh
cargo +esp build --release
espflash save-image --chip esp32s3 --merge \
  target/xtensa-esp32s3-none-elf/release/openpuck \
  openpuck-v0.0.1-b1.bin
```

## Build Flasher

```bash
cargo build --manifest-path tools/flash/Cargo.toml --release --target x86_64-unknown-linux-gnu
```

## Flash

External firmware image:

```bash
tools/flash/target/x86_64-unknown-linux-gnu/release/flash ./openpuck-v0.0.1-b1.bin
```

Packaged firmware mode (if embedded at build time):

```bash
tools/flash/target/x86_64-unknown-linux-gnu/release/flash --embedded
```

Useful flags:

- `--embedded`
- `--external`
- `--port PORT`

If `--port` is not provided, the flasher auto-detects a single connected board via `espflash list-ports`.

## Cross-Platform Flasher Packaging

Self-contained Linux build (embed firmware + Linux espflash):

```bash
OPENPUCK_EMBED_FIRMWARE=$PWD/openpuck-v0.0.1-b1.bin \
OPENPUCK_EMBED_ESPFLASH=$(command -v espflash) \
  cargo build --manifest-path tools/flash/Cargo.toml --release --target x86_64-unknown-linux-gnu
```

Self-contained Windows build (embed firmware + Windows espflash.exe):

```bash
rustup target add x86_64-pc-windows-msvc
OPENPUCK_EMBED_FIRMWARE=$PWD/openpuck-v0.0.1-b1.bin \
OPENPUCK_EMBED_ESPFLASH=$PWD/espflash.exe \
  cargo build --manifest-path tools/flash/Cargo.toml --release --target x86_64-pc-windows-msvc
```


MIT
