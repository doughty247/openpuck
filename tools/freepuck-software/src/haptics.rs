//! Rumble bridge: forwards host haptics intent (from a virtual controller's
//! rumble callback) back to the Steam Controller over BLE.
//!
//! The Steam Controller 2's haptics hardware safety timeout is ~50ms, so
//! sustained rumble must be resent faster than that — this loop resends
//! every `ble::HAPTICS_RESEND_INTERVAL` (40ms) while the intent is non-zero,
//! and stops resending (letting the timeout silence the motors) once it's zero.

use crate::ble::{self, ControllerPeripheral, SteamGatt};
use crate::controller::HapticsIntent;
use tokio::sync::watch;

/// Runs forever, watching `intent_rx` for updates from an output backend's
/// rumble callback and writing Triton rumble commands to the BLE output
/// characteristic. Call via `tokio::spawn`.
pub async fn run_rumble_bridge<P: ControllerPeripheral>(gatt: SteamGatt<P>, mut intent_rx: watch::Receiver<HapticsIntent>) {
    let mut interval = tokio::time::interval(ble::HAPTICS_RESEND_INTERVAL);
    let mut current = *intent_rx.borrow();

    loop {
        tokio::select! {
            changed = intent_rx.changed() => {
                if changed.is_err() {
                    return; // sender dropped, backend shut down
                }
                current = *intent_rx.borrow();
                if let Err(e) = send_rumble(&gatt, current).await {
                    log::warn!("rumble write failed: {e:#}");
                }
            }
            _ = interval.tick() => {
                if current != HapticsIntent::default() {
                    if let Err(e) = send_rumble(&gatt, current).await {
                        log::warn!("rumble resend failed: {e:#}");
                    }
                }
            }
        }
    }
}

pub(crate) async fn send_rumble<P: ControllerPeripheral>(gatt: &SteamGatt<P>, intent: HapticsIntent) -> anyhow::Result<()> {
    let payload = ble::build_triton_rumble_with_id(intent.left_motor, intent.right_motor);
    ble::write_best_effort(&gatt.peripheral, &gatt.output_char, &payload).await
}
