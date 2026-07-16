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
use uinput::Device;

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

fn write_state(device: &mut Device, mode: OutputMode, s: &GamepadState) -> Result<()> {
    // On dualsense/switch layouts, route the Steam Controller's QAM ("...")
    // button to the touchpad-click / Capture equivalent, matching what the
    // firmware's remap_steam_for_switch / DualSense steam_touchpad_mode do.
    let s = if mode == OutputMode::Xinput { *s } else { crate::controller::remap_steam_for_switch(s) };
    let s = &s;

    device.send(GamePad::South, s.a as i32)?;
    device.send(GamePad::East, s.b as i32)?;
    device.send(GamePad::West, s.x as i32)?;
    device.send(GamePad::North, s.y as i32)?;
    device.send(GamePad::TL, s.l1 as i32)?;
    device.send(GamePad::TR, s.r1 as i32)?;
    device.send(GamePad::TL2, s.l2_dig as i32)?;
    device.send(GamePad::TR2, s.r2_dig as i32)?;
    device.send(GamePad::Select, s.select as i32)?;
    device.send(GamePad::Start, s.start as i32)?;
    device.send(GamePad::Mode, s.home as i32)?;
    device.send(GamePad::ThumbL, s.l3 as i32)?;
    device.send(GamePad::ThumbR, s.r3 as i32)?;
    device.send(GamePad::C, s.touchpad as i32)?;

    device.send(Position::X, s.lx as i32)?;
    device.send(Position::Y, s.ly as i32)?;
    device.send(Position::RX, s.rx as i32)?;
    device.send(Position::RY, s.ry as i32)?;
    device.send(Position::Z, s.lt as i32)?;
    device.send(Position::RZ, s.rt as i32)?;

    let hat_x = if s.dpad_left { -1 } else if s.dpad_right { 1 } else { 0 };
    let hat_y = if s.dpad_up { -1 } else if s.dpad_down { 1 } else { 0 };
    device.send(Hat::X0, hat_x)?;
    device.send(Hat::Y0, hat_y)?;

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
