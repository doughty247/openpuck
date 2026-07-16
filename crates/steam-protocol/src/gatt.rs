//! Valve GATT protocol constants and command builders for the Steam
//! Controller 2 (Triton): the pure, transport-agnostic parts of the BLE
//! command layer, shared between the firmware's `trouble-host`-based GATT
//! client (`src/bluetooth.rs`) and the desktop bridge's `btleplug`-based
//! client (`tools/freepuck-software/src/ble.rs`).
//!
//! UUIDs are exposed as raw 16-byte arrays in the controller's own
//! little-endian wire order (as `trouble-host`'s `Uuid::Uuid128` expects
//! them directly) rather than as a typed `Uuid`, because the two BLE
//! stacks use incompatible `Uuid` types (`trouble_host::prelude::Uuid` vs.
//! the `uuid` crate's `Uuid`) — this crate depends on neither, so each
//! consumer wraps these bytes in whatever type its own stack needs. The
//! desktop bridge reverses the byte order to get the `uuid` crate's
//! standard big-endian string form; see `ble.rs` there for that transform.

/// Every Valve custom UUID below shares this base with only one byte
/// varying (byte 12, 0-indexed): `100f6c00-1735-4313-b402-38567131e5f3`.
pub const VALVE_SERVICE_UUID_BYTES: [u8; 16] =
    [0xf3, 0xe5, 0x31, 0x71, 0x56, 0x38, 0x02, 0xb4, 0x13, 0x43, 0x35, 0x17, 0x32, 0x6c, 0x0f, 0x10];
/// Steam Controller 2 (Triton) input characteristic. Gen 1 (D0G, 2015)
/// advertises a different suffix and is not supported by the desktop
/// bridge (the firmware's RF path handles Gen 1 separately).
pub const TRITON_INPUT_UUID_BYTES: [u8; 16] =
    [0xf3, 0xe5, 0x31, 0x71, 0x56, 0x38, 0x02, 0xb4, 0x13, 0x43, 0x35, 0x17, 0x7a, 0x6c, 0x0f, 0x10];
pub const D0G_INPUT_UUID_BYTES: [u8; 16] =
    [0xf3, 0xe5, 0x31, 0x71, 0x56, 0x38, 0x02, 0xb4, 0x13, 0x43, 0x35, 0x17, 0x33, 0x6c, 0x0f, 0x10];
/// Valve "report" characteristic — fallback command channel when the
/// standard HID feature characteristic can't be identified.
pub const VALVE_REPORT_UUID_BYTES: [u8; 16] =
    [0xf3, 0xe5, 0x31, 0x71, 0x56, 0x38, 0x02, 0xb4, 0x13, 0x43, 0x35, 0x17, 0x34, 0x6c, 0x0f, 0x10];

// Standard Bluetooth SIG HID-over-GATT 16-bit UUIDs, little-endian
// (low byte, high byte), as `trouble-host`'s `Uuid::Uuid16` expects them.
pub const HID_SERVICE_UUID_U16_LE: [u8; 2] = [0x12, 0x18]; // 0x1812
pub const HID_REPORT_UUID_U16_LE: [u8; 2] = [0x4D, 0x2A]; // 0x2A4D
pub const HID_CONTROL_POINT_UUID_U16_LE: [u8; 2] = [0x4C, 0x2A]; // 0x2A4C
pub const HID_PROTOCOL_MODE_UUID_U16_LE: [u8; 2] = [0x4E, 0x2A]; // 0x2A4E

// Steam Controller 2 (Triton) command bytes.
pub const TRITON_CMD_RUMBLE: u8 = 0x80; // HID output report reference ID for rumble
pub const TRITON_CMD_SET_SETTINGS: u8 = 0x87;
pub const TRITON_SETTING_LIZARD_MODE: u8 = 0x09;
pub const TRITON_SETTING_HAPTICS_ENABLED: u8 = 70;
pub const TRITON_SETTING_HAPTIC_MASTER_GAIN_DB: u8 = 76;
pub const TRITON_SETTING_HAPTIC_INTENSITY: u8 = 79; // not 77 -- a common bug in other implementations

/// Every 3 seconds, or the built-in lizard mode (trackpad-to-keyboard mapping) re-enables.
pub const LIZARD_KEEPALIVE_INTERVAL_MS: u64 = 3000;
/// Steam Controller 2 haptics hardware safety timeout is ~50ms; resend sustained rumble
/// faster than that to keep it going (matches SDL's TRITON_RUMBLE_RESEND_INTERVAL_MS).
pub const HAPTICS_RESEND_INTERVAL_MS: u64 = 40;

fn scale_rumble(val: u16) -> u16 {
    if val == 0 {
        0
    } else {
        // Boost low values so they are felt on the LRA, mapping 1..65535 to 12000..65535.
        let min_val = 12000u32;
        let max_val = 65535u32;
        let scaled = min_val + ((val as u32) * (max_val - min_val) / 65535);
        scaled as u16
    }
}

/// Full Triton haptic rumble output report, including report ID 0x80.
/// Layout matches SDL's MsgHapticRumble payload:
///   type:u8, intensity:u16, left.speed:u16, left.gain:i8, right.speed:u16, right.gain:i8
pub fn build_triton_rumble_with_id(left_speed: u16, right_speed: u16) -> [u8; 10] {
    let scaled_left = scale_rumble(left_speed);
    let scaled_right = scale_rumble(right_speed);
    [
        TRITON_CMD_RUMBLE,
        0x00, // type
        0x00, // intensity (low) - 0 triggers internal hardware LRA emulator
        0x00, // intensity (high)
        (scaled_left & 0xFF) as u8,
        (scaled_left >> 8) as u8,
        0x06, // left gain dB (maximum boost)
        (scaled_right & 0xFF) as u8,
        (scaled_right >> 8) as u8,
        0x06, // right gain dB (maximum boost)
    ]
}

pub fn build_triton_lizard_off() -> [u8; 64] {
    let mut buf = [0u8; 64];
    buf[0] = TRITON_CMD_SET_SETTINGS;
    buf[1] = 0x03;
    buf[2] = TRITON_SETTING_LIZARD_MODE;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf
}

pub fn build_triton_haptics_enable() -> [u8; 11] {
    [
        TRITON_CMD_SET_SETTINGS,
        0x09, // 3 settings * 3 bytes each
        TRITON_SETTING_HAPTICS_ENABLED,
        0x01,
        0x00, // ON
        TRITON_SETTING_HAPTIC_MASTER_GAIN_DB,
        0x06,
        0x00, // +6 dB (maximum master gain)
        TRITON_SETTING_HAPTIC_INTENSITY,
        0x04,
        0x00, // INSANE
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lizard_off_has_documented_header() {
        let buf = build_triton_lizard_off();
        assert_eq!(buf.len(), 64);
        assert_eq!(&buf[0..5], &[0x87, 0x03, 0x09, 0x00, 0x00]);
        assert!(buf[5..].iter().all(|&b| b == 0));
    }

    #[test]
    fn haptics_enable_uses_setting_79_not_77() {
        let buf = build_triton_haptics_enable();
        assert_eq!(buf, [0x87, 0x09, 70, 0x01, 0x00, 76, 0x06, 0x00, 79, 0x04, 0x00]);
    }

    #[test]
    fn rumble_report_has_report_id_and_gain() {
        // Layout: [0]=cmd [1]=type [2..4]=intensity [4..6]=left speed LE
        // [6]=left gain [7..9]=right speed LE [9]=right gain.
        let buf = build_triton_rumble_with_id(0xFFFF, 0);
        assert_eq!(buf[0], TRITON_CMD_RUMBLE);
        assert_eq!(buf[6], 0x06, "left gain is always set regardless of magnitude");
        assert_eq!(buf[9], 0x06, "right gain is always set regardless of magnitude");
        // A zero right speed stays zero even though the gain byte is fixed.
        assert_eq!((buf[7], buf[8]), (0x00, 0x00));
    }

    #[test]
    fn rumble_scaling_boosts_low_nonzero_values_and_preserves_zero() {
        assert_eq!(scale_rumble(0), 0);
        assert!(scale_rumble(1) >= 12000, "even a tiny nonzero magnitude should be boosted onto the LRA's felt range");
        assert_eq!(scale_rumble(65535), 65535);
    }
}
