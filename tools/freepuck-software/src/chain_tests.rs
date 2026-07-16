//! End-to-end tests driving the whole software chain — BLE GATT discovery,
//! init commands, input parsing, and output encoding — against
//! [`mock_ble::MockController`], a stand-in for a real Steam Controller 2
//! built from the byte-exact protocol this repo's firmware already
//! validates against hardware (see `docs/ble-protocol.md`). No real
//! Bluetooth is involved; this proves the bridge's own logic is correct,
//! not that a physical controller behaves as documented — that still needs
//! a real-hardware pass (see the crate README's Status section).

#![cfg(test)]

use crate::ble::{self, ControllerPeripheral};
use crate::controller;
use crate::mock_ble::MockController;
use btleplug::api::{CharPropFlags, WriteType};
use futures::StreamExt;

/// Builds a 17-byte Triton input report using the exact byte layout from
/// `docs/ble-protocol.md`: seq counter, 3 button bytes, LT/RT as u16 LE,
/// then four i16 LE stick axes.
#[allow(clippy::too_many_arguments)]
fn steam_report(seq: u8, btn0: u8, btn1: u8, btn2: u8, lt: u16, rt: u16, lsx: i16, lsy: i16, rsx: i16, rsy: i16) -> Vec<u8> {
    let mut p = vec![0u8; 17];
    p[0] = seq;
    p[1] = btn0;
    p[2] = btn1;
    p[3] = btn2;
    p[5..7].copy_from_slice(&lt.to_le_bytes());
    p[7..9].copy_from_slice(&rt.to_le_bytes());
    p[9..11].copy_from_slice(&lsx.to_le_bytes());
    p[11..13].copy_from_slice(&lsy.to_le_bytes());
    p[13..15].copy_from_slice(&rsx.to_le_bytes());
    p[15..17].copy_from_slice(&rsy.to_le_bytes());
    p
}

async fn connect(mock: &MockController) -> ble::SteamGatt<MockController> {
    ble::connect_and_discover(mock.clone()).await.expect("mock connect_and_discover should succeed")
}

#[tokio::test]
async fn resolves_gatt_characteristics_by_property_scoring() {
    let mock = MockController::steam_controller_2("Steam Ctrl (BT) TEST0001");
    assert!(ble::matches_steam_controller(mock.advertised_name()));
    let gatt = connect(&mock).await;

    assert_eq!(gatt.input_char.uuid, ble::TRITON_INPUT_UUID);

    // Feature (settings) characteristic: readable + write-with-response, no write-without-response.
    assert!(gatt.feature_char.properties.contains(CharPropFlags::READ));
    assert!(gatt.feature_char.properties.contains(CharPropFlags::WRITE));
    assert!(!gatt.feature_char.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE));

    // Output (rumble) characteristic: write-without-response, not readable — the opposite shape.
    assert!(gatt.output_char.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE));
    assert!(!gatt.output_char.properties.contains(CharPropFlags::READ));

    assert_ne!(gatt.feature_char, gatt.output_char, "scoring must not pick the same characteristic for both roles");
}

#[tokio::test]
async fn rejects_gen1_steam_controller() {
    let mock = MockController::gen1_steam_controller("Steam Ctrl (BT) OLDUNIT");
    let result = ble::connect_and_discover(mock).await;
    assert!(result.is_err(), "Gen 1 (D0G) controllers are explicitly unsupported and should be rejected");
}

#[tokio::test]
async fn init_commands_send_the_documented_byte_sequence() {
    let mock = MockController::steam_controller_2("Steam Ctrl (BT) TEST0001");
    let gatt = connect(&mock).await;

    ble::send_init_commands(&gatt).await.expect("init commands should succeed against the mock");

    let writes = mock.recorded_writes();
    assert_eq!(writes.len(), 3, "expected exactly clear + settings + haptics-enable, got {writes:?}");

    assert_eq!(writes[0].characteristic_uuid, gatt.feature_char.uuid);
    assert_eq!(writes[0].data, vec![0x81], "CMD_CLEAR_DIGITAL_MAPPINGS");

    assert_eq!(writes[1].data, vec![0x87, 0x06, 0x07, 0x07, 0x00, 0x08, 0x07, 0x00], "CMD_SET_SETTINGS base config");

    assert_eq!(
        writes[2].data,
        vec![0x87, 0x09, 70, 0x01, 0x00, 76, 0x06, 0x00, 79, 0x04, 0x00],
        "haptics-enable: setting 70 (HAPTICS_ENABLED)=1, 76 (MASTER_GAIN_DB)=6, 79 (INTENSITY)=4 — not 77, a known bug in other implementations"
    );

    // This mock's feature characteristic only advertises WRITE (not
    // WRITE_WITHOUT_RESPONSE — see MockController::steam_controller_2), so
    // write_best_effort must fall back to a with-response write.
    assert_eq!(writes[0].write_type, WriteType::WithResponse);
}

#[tokio::test]
async fn lizard_keepalive_tick_resends_lizard_off_and_haptics_enable() {
    let mock = MockController::steam_controller_2("Steam Ctrl (BT) TEST0001");
    let gatt = connect(&mock).await;
    ble::send_init_commands(&gatt).await.unwrap();

    ble::send_lizard_tick(&gatt).await.expect("keepalive tick should succeed against the mock");

    let writes = mock.recorded_writes();
    assert_eq!(writes.len(), 5, "3 init writes + lizard-off + haptics-enable");

    let lizard_off = &writes[3].data;
    assert_eq!(lizard_off.len(), 64, "SDL sends Triton lizard-off as a 64-byte feature report");
    assert_eq!(&lizard_off[0..5], &[0x87, 0x03, 0x09, 0x00, 0x00], "CMD_SET_SETTINGS, SETTING_LIZARD_MODE=off");
    assert!(lizard_off[5..].iter().all(|&b| b == 0), "rest of the 64-byte report is zero-padded");

    assert_eq!(writes[4].data, vec![0x87, 0x09, 70, 0x01, 0x00, 76, 0x06, 0x00, 79, 0x04, 0x00]);
}

/// Simulates a realistic play sequence — idle, press face buttons, push a
/// stick, pull a trigger, dpad + QAM — as a real Steam Controller 2 would
/// report it over BLE, and checks the parsed `GamepadState` and encoded
/// output reports at each step. This is the "how well does the chain work"
/// test: mock BLE peripheral -> GATT discovery -> notification stream ->
/// parse_steam -> GamepadState -> state_to_{xinput,dualsense,switch}.
#[tokio::test]
async fn full_chain_realistic_button_sequence() {
    let mock = MockController::steam_controller_2("Steam Ctrl (BT) TEST0001");
    let gatt = connect(&mock).await;
    ble::send_init_commands(&gatt).await.unwrap();

    let mut notifications = gatt.peripheral.notifications().await.unwrap();

    // Frame 1: idle — controller just connected, nothing pressed, sticks centred.
    mock.push_input_report(&steam_report(0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
    let n = notifications.next().await.unwrap();
    assert_eq!(n.uuid, ble::TRITON_INPUT_UUID);
    let s = controller::parse_steam(&n.value).expect("idle frame should parse");
    assert!(!s.a && !s.b && !s.x && !s.y);
    assert_eq!((s.lx, s.ly, s.rx, s.ry), (128, 128, 128, 128));
    assert_eq!((s.lt, s.rt), (0, 0));

    // Frame 2: player presses A and B (btn0 bits 0,1).
    mock.push_input_report(&steam_report(1, 0x03, 0, 0, 0, 0, 0, 0, 0, 0));
    let s = controller::parse_steam(&notifications.next().await.unwrap().value).unwrap();
    assert!(s.a && s.b && !s.x && !s.y);
    let xinput = controller::state_to_xinput(&s);
    assert_eq!(xinput[3] & 0x30, 0x30, "A and B bits set in the XInput report");

    // Frame 3: buttons released, left stick pushed hard right, right trigger
    // barely touched (light pull, below the digital-press threshold).
    mock.push_input_report(&steam_report(2, 0, 0, 0, 0, 5000, 30000, 0, 0, 0));
    let s = controller::parse_steam(&notifications.next().await.unwrap().value).unwrap();
    assert!(s.lx > 200, "left stick should read strongly right, got {}", s.lx);
    assert_eq!(s.ly, 128, "Y axis untouched should stay centred");
    assert!(s.rt > 0 && s.rt < 255, "a light trigger pull should be a small non-zero value, got {}", s.rt);
    assert!(!s.r2_dig, "5000 is below the 8000 digital-press threshold and should not register a digital press");
    let ds = controller::state_to_dualsense(&s, false);
    assert_eq!(ds[1], s.lx, "DualSense report LX byte should carry the same stick value");

    // Frame 4: right trigger pulled past the digital-press threshold, QAM
    // ("...") button clicked, dpad up+right held simultaneously.
    mock.push_input_report(&steam_report(3, 0x10, 0x20 | 0x08, 0, 0, 40000, 0, 0, 0, 0));
    let s = controller::parse_steam(&notifications.next().await.unwrap().value).unwrap();
    assert!(s.qam);
    assert!(s.dpad_up && s.dpad_right && !s.dpad_down && !s.dpad_left);
    assert!(s.r2_dig, "40000 is above the 8000 digital-press threshold");
    let switch = controller::state_to_switch(&s);
    assert_eq!(switch[2], 1, "dpad up+right should encode to hat value 1 in state_to_switch");
    let remapped = controller::remap_steam_for_switch(&s);
    assert!(remapped.touchpad, "Switch mode should route QAM to the Capture-equivalent touchpad bit");
}
