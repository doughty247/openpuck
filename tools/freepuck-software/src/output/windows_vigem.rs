//! Windows virtual controller output via ViGEmBus (the `vigem-client` crate).
//!
//! `xinput` mode creates a real ViGEmBus Xbox 360 target: any XInput game
//! recognizes it, and rumble is forwarded back over BLE via ViGEmBus's
//! notification thread.
//!
//! `dualsense` mode creates a ViGEmBus DualShock 4 target (the closest thing
//! ViGEmBus supports to a DualSense). `vigem-client`'s `DS4Report` has no
//! gyro/touchpad fields and this crate exposes no DS4 rumble-notification
//! API, so haptics are not forwarded in this mode. A PS5 would not recognize
//! this as a genuine DualSense regardless — Sony requires cryptographic
//! authentication this bridge does not implement. DualSense mode is PC-only.
//!
//! `switch` mode is not supported: ViGEmBus has no Switch Pro Controller
//! emulation target, and even if it did, a Windows PC cannot present itself
//! as a USB device to a physical Switch console the way the ESP32 hardware
//! dongle can. `run` returns an error for this mode.
//!
//! ViGEmBus must be installed separately (https://github.com/ViGEm/ViGEmBus)
//! — it's a signed kernel driver and cannot be bundled with this tool.

use crate::controller::{dpad_to_hat, GamepadState, HapticsIntent};
use crate::output::OutputMode;
use anyhow::{bail, Context, Result};
use std::sync::mpsc::{Receiver, Sender};
use vigem_client::{Client, DualShock4Wired, TargetId, XButtons, XGamepad, Xbox360Wired, DS4Report};

fn xgamepad_from_state(s: &GamepadState) -> XGamepad {
    let mut raw = 0u16;
    if s.dpad_up { raw |= XButtons::UP; }
    if s.dpad_down { raw |= XButtons::DOWN; }
    if s.dpad_left { raw |= XButtons::LEFT; }
    if s.dpad_right { raw |= XButtons::RIGHT; }
    if s.start { raw |= XButtons::START; }
    if s.select { raw |= XButtons::BACK; }
    if s.l3 { raw |= XButtons::LTHUMB; }
    if s.r3 { raw |= XButtons::RTHUMB; }
    if s.l1 { raw |= XButtons::LB; }
    if s.r1 { raw |= XButtons::RB; }
    if s.home { raw |= XButtons::GUIDE; }
    if s.a { raw |= XButtons::A; }
    if s.b { raw |= XButtons::B; }
    if s.x { raw |= XButtons::X; }
    if s.y { raw |= XButtons::Y; }

    // GamepadState: 128=centre, 0=up/left. XInput i16: 0=centre, positive=right/up (Y inverted).
    let lx = (s.lx as i16 - 128).saturating_mul(258);
    let ly = (128i16 - s.ly as i16).saturating_mul(258);
    let rx = (s.rx as i16 - 128).saturating_mul(258);
    let ry = (128i16 - s.ry as i16).saturating_mul(258);

    XGamepad {
        buttons: XButtons(raw),
        left_trigger: s.lt,
        right_trigger: s.rt,
        thumb_lx: lx,
        thumb_ly: ly,
        thumb_rx: rx,
        thumb_ry: ry,
    }
}

/// Matches ViGEmBus's DS4_REPORT bit layout: buttons bits 0-3 = hat
/// (8 = centred), 4=Square, 5=Cross, 6=Circle, 7=Triangle, 8=L1, 9=R1,
/// 10=L2 digital, 11=R2 digital, 12=Share, 13=Options, 14=L3, 15=R3.
/// special: bit0=PS, bit1=Touchpad click.
fn ds4report_from_state(s: &GamepadState) -> DS4Report {
    let hat = dpad_to_hat(s.dpad_up, s.dpad_down, s.dpad_left, s.dpad_right) as u16;
    let mut buttons = hat & 0x0F;
    if s.x { buttons |= 1 << 4; }
    if s.a { buttons |= 1 << 5; }
    if s.b { buttons |= 1 << 6; }
    if s.y { buttons |= 1 << 7; }
    if s.l1 { buttons |= 1 << 8; }
    if s.r1 { buttons |= 1 << 9; }
    if s.l2_dig { buttons |= 1 << 10; }
    if s.r2_dig { buttons |= 1 << 11; }
    if s.select { buttons |= 1 << 12; }
    if s.start { buttons |= 1 << 13; }
    if s.l3 { buttons |= 1 << 14; }
    if s.r3 { buttons |= 1 << 15; }

    let mut special = 0u8;
    if s.home { special |= 0x01; }
    if s.touchpad || s.qam { special |= 0x02; }

    DS4Report {
        thumb_lx: s.lx,
        thumb_ly: s.ly,
        thumb_rx: s.rx,
        thumb_ry: s.ry,
        buttons,
        special,
        trigger_l: s.lt,
        trigger_r: s.rt,
    }
}

/// Runs the ViGEmBus output backend on the calling thread until `state_rx`
/// disconnects. Blocking — call via `std::thread::spawn` or
/// `tokio::task::spawn_blocking`, not directly from an async task.
pub fn run(mode: OutputMode, state_rx: Receiver<GamepadState>, haptics_tx: Sender<HapticsIntent>) -> Result<()> {
    let client = Client::connect().context(
        "failed to connect to ViGEmBus — is the driver installed? https://github.com/ViGEm/ViGEmBus/releases",
    )?;

    match mode {
        OutputMode::Xinput => run_xbox360(client, state_rx, haptics_tx),
        OutputMode::Dualsense => run_ds4(client, state_rx),
        OutputMode::Switch => bail!(
            "Switch Pro output is not supported on Windows: ViGEmBus has no Switch Pro emulation \
             target, and a PC cannot present itself as a USB device to a physical Switch console \
             the way the hardware dongle firmware can. Use --mode xinput or --mode dualsense instead."
        ),
    }
}

fn run_xbox360(client: Client, state_rx: Receiver<GamepadState>, haptics_tx: Sender<HapticsIntent>) -> Result<()> {
    let mut target = Xbox360Wired::new(client, TargetId::XBOX360_WIRED);
    target.plugin().context("failed to plug in virtual Xbox 360 controller")?;
    target.wait_ready().context("virtual Xbox 360 controller did not become ready")?;
    log::info!("virtual Xbox 360 controller plugged in");

    let notification = target
        .request_notification()
        .context("failed to request rumble notifications from ViGEmBus")?;
    // XInput convention: large_motor = strong/left, small_motor = weak/right.
    let _rumble_thread = notification.spawn_thread(move |_req, note| {
        let intent = HapticsIntent {
            left_motor: (note.large_motor as u16) * 257,
            right_motor: (note.small_motor as u16) * 257,
            left_trigger_fx: 0,
            right_trigger_fx: 0,
        };
        let _ = haptics_tx.send(intent);
    });

    while let Ok(state) = state_rx.recv() {
        let gamepad = xgamepad_from_state(&state);
        if let Err(e) = target.update(&gamepad) {
            log::warn!("Xbox 360 target update failed: {e:?}");
        }
    }
    Ok(())
}

fn run_ds4(client: Client, state_rx: Receiver<GamepadState>) -> Result<()> {
    let mut target = DualShock4Wired::new(client, TargetId::DUALSHOCK4_WIRED);
    target.plugin().context("failed to plug in virtual DualShock 4 controller")?;
    target.wait_ready().context("virtual DualShock 4 controller did not become ready")?;
    log::info!("virtual DualShock 4 controller plugged in (DualSense-equivalent; PC-only, no rumble)");

    while let Ok(state) = state_rx.recv() {
        let report = ds4report_from_state(&state);
        if let Err(e) = target.update(&report) {
            log::warn!("DualShock 4 target update failed: {e:?}");
        }
    }
    Ok(())
}
