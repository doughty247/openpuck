//! Virtual controller output backends.
//!
//! `xinput` is the primary, best-supported mode on every platform: on
//! Windows it's a real ViGEmBus Xbox 360 target that any XInput game
//! recognizes; on Linux it's a uinput device spoofing the Xbox 360 wired
//! controller's USB vendor/product id so SDL's GameControllerDB matches it.
//!
//! `dualsense` and `switch` are best-effort. See each module's doc comment
//! for what does and doesn't work — notably neither is a real bridge to a
//! PS5 or a physical Nintendo Switch console; see the crate README.

#[cfg(target_os = "linux")]
pub mod linux_uinput;
#[cfg(windows)]
pub mod windows_vigem;

use clap::ValueEnum;

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum OutputMode {
    Xinput,
    Dualsense,
    Switch,
}
