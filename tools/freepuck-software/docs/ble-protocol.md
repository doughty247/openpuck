# Steam Controller 2 BLE Protocol Reference

Reference for `tools/freepuck-software`, extracted from the validated,
hardware-tested implementation in this repo's firmware
(`src/bluetooth.rs`, `src/controller.rs`). This supersedes any
byte-offset notes elsewhere that predate this file — where they
disagree, this document and the firmware source are authoritative
because they're the versions actually exercised against real hardware.

## Connection

- Advertised name prefix: `Steam Ctrl` (e.g. `Steam Ctrl (BT) FXA9961102DD6`)
  or `Steam Controller Puck` (SC2 uses the same Valve GATT service either way).
- Custom Valve GATT service, not standard HID-over-GATT for input.
- Target connection interval: 7.5 ms (BLE 4.x floor).

## GATT UUIDs

All Valve custom UUIDs share the base `100f6c00-1735-4313-b402-38567131e5f3`
with only one byte varying:

| Name | UUID | Notes |
|---|---|---|
| Valve service | `100f6c32-1735-4313-b402-38567131e5f3` | Primary custom service |
| Triton input characteristic | `100f6c7a-1735-4313-b402-38567131e5f3` | Steam Controller 2 input reports (notify) |
| D0G input characteristic | `100f6c33-1735-4313-b402-38567131e5f3` | Gen 1 (2015) input reports — **not supported** by this bridge |
| Valve report characteristic | `100f6c34-1735-4313-b402-38567131e5f3` | Fallback command channel |

Standard Bluetooth SIG HID-over-GATT service (`0x1812`) is also present and is
used to locate the feature-report (settings/haptics/lizard-off) and
output-report (rumble) characteristics. Which specific characteristic handle
plays which role isn't fixed across BLE stacks, so both the firmware and
`freepuck-software` score candidates by their properties (readable +
write-with-response tends to be the feature characteristic; write-without-response
and not readable tends to be the output/rumble characteristic) rather than
relying on a hardcoded ATT handle.

## Input report (Triton input characteristic notification)

Minimum 17 bytes. Byte 0 is a rolling sequence counter.

| Bytes | Field |
|---|---|
| 0 | sequence counter |
| 1 | button byte 0: bit0=A, bit1=B, bit2=X, bit3=Y, bit4=QAM ("..."), bit5=R3, bit6=Start |
| 2 | button byte 1: bit1=R1, bit2=dpad down, bit3=dpad right, bit4=dpad left, bit5=dpad up, bit6=Select, bit7=L3 |
| 3 | button byte 2: bit0=Home, bit1=Touchpad click, bit3=L1 |
| 5-6 | left trigger, u16 LE, `lt = (raw >> 7) as u8`, digital press when `raw > 8000` |
| 7-8 | right trigger, u16 LE, same scaling |
| 9-10 | left stick X, i16 LE, positive = right |
| 11-12 | left stick Y, i16 LE, positive = up |
| 13-14 | right stick X, i16 LE, positive = right |
| 15-16 | right stick Y, i16 LE, positive = up |

IMU (gyro/accel) fields are not populated by `parse_steam` — the packet
format for `SETTING_IMU_MODE` has not been confirmed against hardware.

## Commands (feature characteristic)

- `0x81` — CMD_CLEAR_DIGITAL_MAPPINGS (single byte, sent once on connect).
- `0x87 0x06 0x07 0x07 0x00 0x08 0x07 0x00` — CMD_SET_SETTINGS base config (sent once on connect).
- Haptics enable (settings IDs — sent on connect and every keepalive tick):
  `[0x87, 0x09, 70, 0x01, 0x00, 76, 0x06, 0x00, 79, 0x04, 0x00]`
  (`SETTING_HAPTICS_ENABLED=70`, `SETTING_HAPTIC_MASTER_GAIN_DB=76` at +6dB,
  `SETTING_HAPTIC_INTENSITY=79` at INSANE=4 — **79, not 77**, a common bug in
  other implementations).
- Lizard-mode off (64-byte feature report, sent every keepalive tick):
  `[0x87, 0x03, 0x09, 0x00, 0x00, ...zero-padded to 64 bytes]`
  (`SETTING_LIZARD_MODE=0x09`).
- Keepalive interval: 3 seconds. The controller's built-in trackpad-to-keyboard
  mapping ("lizard mode") re-enables itself if this isn't resent.

## Rumble (output characteristic)

10-byte report, report ID `0x80`:

```
[0x80, 0x00, intensity_lo, intensity_hi, left_lo, left_hi, left_gain, right_lo, right_hi, right_gain]
```

- `intensity = 0` triggers the controller's internal LRA emulator — pass
  magnitudes straight through rather than trying to compute LRA-specific values.
- `left_gain` / `right_gain` = `0x06` (+6 dB, maximum boost).
- Speed values are boosted before sending: raw `1..65535` maps to `12000..65535`
  so low rumble magnitudes are still felt on the LRA (`scale_rumble` in
  `src/bluetooth.rs` / `ble::build_triton_rumble_with_id`).
- Hardware safety timeout: ~50 ms. Resend every 40 ms while rumble is active
  (matches SDL's `TRITON_RUMBLE_RESEND_INTERVAL_MS`).

## What's provisional

Nothing above is provisional — this file only documents fields the firmware
already parses and exercises against real hardware. Byte ranges not listed
here (most of bytes 3-4, parts of the packet past byte 17) are not yet mapped.
