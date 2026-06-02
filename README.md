# OpenPuck

Steam Controller 2026 bridge firmware for ESP32-S3.

## TL;DR

- Download the flasher for your OS from Releases.
- Plug in your ESP32-S3 board.
- Run the flasher first. Do not build anything unless you want to develop.
- Pair Steam Controller 2026 and play.
- If output mode acts weird, tap BOOT to cycle modes. XInput is the most reliable.

## Run Flasher First

1. Download release assets:
   - openpuck-v0.0.1-b1.bin
   - openpuck-flash for your platform
2. Connect the board with USB.
3. Run the flasher (embedded is the default):

Linux:

```bash
./openpuck-flash
```

Windows (PowerShell):

```powershell
.\openpuck-flash.exe
```

Use external firmware only when needed:

Linux:

```bash
./openpuck-flash --external ./openpuck-v0.0.1-b1.bin
```

Windows (PowerShell):

```powershell
.\openpuck-flash.exe --external .\openpuck-v0.0.1-b1.bin
```

Common flags:

- --embedded
- --external
- --port PORT

If --port is not set, the flasher auto-detects a single connected board.

## Scope and Caveats

- Steam Controller 2026 only
- USB output modes: XInput, Switch Pro, DualSense
- No native Steam Controller USB mode yet
- Input mode toggling can still be finicky
- Rumble is currently most reliable in XInput

## Build From Source (Optional)

Only needed for development.

```bash
cargo install espup && espup install
. ~/export-esp.sh
cargo +esp build --release
espflash save-image --chip esp32s3 --merge \
  target/xtensa-esp32s3-none-elf/release/openpuck \
  openpuck-v0.0.1-b1.bin
```

MIT
