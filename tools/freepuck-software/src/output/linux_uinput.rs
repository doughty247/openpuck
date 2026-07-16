//! Linux virtual controller output via `/dev/uinput`.
//!
//! `xinput` mode spoofs the Xbox 360 wired controller's USB vendor/product
//! id (045E:028E) so SDL's GameControllerDB and the kernel's `xpad` quirks
//! recognize it — this is the well-tested, known-good path.
//!
//! `dualsense` and `switch` modes build a generic evdev gamepad with
//! Sony/Nintendo vendor/product ids and Nintendo-style button naming, but
//! **uinput operates at the evdev abstraction, not raw HID reports** — it
//! cannot replicate a real DualSense/Switch Pro Controller's HID report
//! format (gyro, touchpad, PS5 auth, rumble-over-hidraw). Games that talk to
//! those controllers via SDL's hidraw backend rather than the generic
//! joystick layer will not recognize this as a real DualSense/Switch Pro —
//! it behaves like a generic gamepad with the equivalent button layout. See
//! the crate README for details. Rumble forwarding is also not implemented
//! for uinput: the `uinput` crate this bridge uses does not expose the
//! force-feedback (EV_FF) ioctls needed to read rumble commands back from
//! the kernel, so Phase 5 haptics only work in the Windows ViGEmBus backend.

use crate::controller::GamepadState;
use crate::output::OutputMode;
use anyhow::{Context, Result};
use tokio::sync::watch;
use uinput::event::absolute::{Hat, Position};
use uinput::event::controller::GamePad;
use uinput::{Device, Event};

fn device_identity(mode: OutputMode) -> (&'static str, u16, u16) {
    match mode {
        OutputMode::Xinput => ("FreePuck Virtual Xbox 360 Controller", 0x045E, 0x028E),
        OutputMode::Dualsense => ("FreePuck Virtual Gamepad (DualSense layout)", 0x054C, 0x0CE6),
        OutputMode::Switch => ("FreePuck Virtual Gamepad (Switch Pro layout)", 0x057E, 0x2009),
    }
}

fn build_device(mode: OutputMode) -> Result<Device> {
    let (name, vendor, product) = device_identity(mode);

    let mut builder = uinput::default()
        .context("failed to open /dev/uinput — is the uinput kernel module loaded and do you have write access (usually needs root or the `input` group)?")?
        .name(name)
        .context("invalid device name")?;
    builder = builder.vendor(vendor).product(product).bus(0x03 /* BUS_USB */).version(1);

    for button in [
        GamePad::South, GamePad::East, GamePad::North, GamePad::West,
        GamePad::TL, GamePad::TR, GamePad::TL2, GamePad::TR2,
        GamePad::Select, GamePad::Start, GamePad::Mode,
        GamePad::ThumbL, GamePad::ThumbR,
        GamePad::C, // touchpad-click / Capture equivalent for dualsense/switch modes
    ] {
        builder = builder.event(button).context("failed to register button event")?;
    }

    for axis in [Position::X, Position::Y, Position::RX, Position::RY, Position::Z, Position::RZ] {
        builder = builder.event(axis).context("failed to register axis event")?;
        builder = builder.min(0).max(255);
    }
    for hat in [Hat::X0, Hat::Y0] {
        builder = builder.event(hat).context("failed to register hat event")?;
        builder = builder.min(-1).max(1);
    }

    builder.create().context("failed to create uinput device")
}

/// Pure mapping from a `GamepadState` (plus output mode) to the uinput
/// `(Event, value)` pairs that should be sent for it. Split out from
/// `write_state` so the button/axis mapping logic is unit-testable without a
/// real `/dev/uinput` device — this sandbox has no uinput kernel module, so
/// direct device tests aren't possible here; this is the next best thing.
fn compute_events(mode: OutputMode, s: &GamepadState) -> Vec<(Event, i32)> {
    // On dualsense/switch layouts, route the Steam Controller's QAM ("...")
    // button to the touchpad-click / Capture equivalent, matching what the
    // firmware's remap_steam_for_switch / DualSense steam_touchpad_mode do.
    let s = if mode == OutputMode::Xinput { *s } else { crate::controller::remap_steam_for_switch(s) };

    let hat_x = if s.dpad_left { -1 } else if s.dpad_right { 1 } else { 0 };
    let hat_y = if s.dpad_up { -1 } else if s.dpad_down { 1 } else { 0 };

    vec![
        (GamePad::South.into(), s.a as i32),
        (GamePad::East.into(), s.b as i32),
        (GamePad::West.into(), s.x as i32),
        (GamePad::North.into(), s.y as i32),
        (GamePad::TL.into(), s.l1 as i32),
        (GamePad::TR.into(), s.r1 as i32),
        (GamePad::TL2.into(), s.l2_dig as i32),
        (GamePad::TR2.into(), s.r2_dig as i32),
        (GamePad::Select.into(), s.select as i32),
        (GamePad::Start.into(), s.start as i32),
        (GamePad::Mode.into(), s.home as i32),
        (GamePad::ThumbL.into(), s.l3 as i32),
        (GamePad::ThumbR.into(), s.r3 as i32),
        (GamePad::C.into(), s.touchpad as i32),
        (Position::X.into(), s.lx as i32),
        (Position::Y.into(), s.ly as i32),
        (Position::RX.into(), s.rx as i32),
        (Position::RY.into(), s.ry as i32),
        (Position::Z.into(), s.lt as i32),
        (Position::RZ.into(), s.rt as i32),
        (Hat::X0.into(), hat_x),
        (Hat::Y0.into(), hat_y),
    ]
}

fn write_state(device: &mut Device, mode: OutputMode, s: &GamepadState) -> Result<()> {
    for (event, value) in compute_events(mode, s) {
        device.send(event, value)?;
    }
    device.synchronize()?;
    Ok(())
}

/// Runs the uinput output backend until `state_rx`'s sender is dropped.
pub async fn run(mode: OutputMode, mut state_rx: watch::Receiver<GamepadState>) -> Result<()> {
    let mut device = build_device(mode)?;
    log::info!("uinput virtual gamepad created ({:?} mode)", mode);

    loop {
        if state_rx.changed().await.is_err() {
            return Ok(()); // BLE task shut down
        }
        let state = *state_rx.borrow();
        write_state(&mut device, mode, &state)?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn value_of(events: &[(Event, i32)], target: Event) -> i32 {
        events
            .iter()
            .find(|(e, _)| *e == target)
            .unwrap_or_else(|| panic!("event {target:?} not present in computed event list"))
            .1
    }

    #[test]
    fn xinput_mode_maps_face_buttons_directly() {
        let mut s = GamepadState::default_centred();
        s.a = true;
        s.y = true;
        let events = compute_events(OutputMode::Xinput, &s);
        assert_eq!(value_of(&events, GamePad::South.into()), 1);
        assert_eq!(value_of(&events, GamePad::North.into()), 1);
        assert_eq!(value_of(&events, GamePad::East.into()), 0);
        assert_eq!(value_of(&events, GamePad::West.into()), 0);
    }

    #[test]
    fn xinput_mode_does_not_remap_qam_to_touchpad() {
        let mut s = GamepadState::default_centred();
        s.qam = true;
        let events = compute_events(OutputMode::Xinput, &s);
        assert_eq!(value_of(&events, GamePad::C.into()), 0, "xinput has no touchpad-equivalent button");
    }

    #[test]
    fn dualsense_and_switch_modes_route_qam_to_touchpad_click() {
        let mut s = GamepadState::default_centred();
        s.qam = true;
        for mode in [OutputMode::Dualsense, OutputMode::Switch] {
            let events = compute_events(mode, &s);
            assert_eq!(value_of(&events, GamePad::C.into()), 1, "{mode:?} should route QAM to the touchpad/Capture button");
        }
    }

    #[test]
    fn dpad_combinations_map_to_hat_axes() {
        let mut s = GamepadState::default_centred();
        s.dpad_up = true;
        s.dpad_right = true;
        let events = compute_events(OutputMode::Xinput, &s);
        assert_eq!(value_of(&events, Hat::X0.into()), 1, "right should be +1 on the X hat");
        assert_eq!(value_of(&events, Hat::Y0.into()), -1, "up should be -1 on the Y hat");

        let mut s2 = GamepadState::default_centred();
        s2.dpad_down = true;
        s2.dpad_left = true;
        let events2 = compute_events(OutputMode::Xinput, &s2);
        assert_eq!(value_of(&events2, Hat::X0.into()), -1);
        assert_eq!(value_of(&events2, Hat::Y0.into()), 1);

        let events3 = compute_events(OutputMode::Xinput, &GamepadState::default_centred());
        assert_eq!(value_of(&events3, Hat::X0.into()), 0, "no dpad input should centre the hat");
        assert_eq!(value_of(&events3, Hat::Y0.into()), 0);
    }

    #[test]
    fn stick_and_trigger_values_pass_through_unscaled() {
        let mut s = GamepadState::default_centred();
        s.lx = 200;
        s.rt = 77;
        let events = compute_events(OutputMode::Xinput, &s);
        assert_eq!(value_of(&events, Position::X.into()), 200);
        assert_eq!(value_of(&events, Position::RZ.into()), 77);
        assert_eq!(value_of(&events, Position::Y.into()), 128, "untouched Y should stay centred");
    }
}
