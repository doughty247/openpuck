//! Steam Controller 2 protocol codec, shared between this repo's ESP32-S3
//! firmware (`no_std`, `src/`) and the desktop `freepuck-software` bridge
//! (`std`, `tools/freepuck-software/`).
//!
//! `no_std` so both consumers can use it unmodified — `std` binaries can
//! always depend on a `no_std` crate (`std` is a superset of `core`), but
//! the reverse isn't true. This crate has no dependencies of its own and
//! touches no I/O: it's pure data transformation, which is exactly the
//! part that had drifted between the two consumers and picked up real bugs
//! along the way (see `codec`'s doc comment). Each consumer keeps its own
//! platform-specific BLE transport (`trouble-host` for the firmware,
//! `btleplug` for the desktop bridge) and output device wiring — those
//! can't be shared, since they're built on fundamentally different stacks.

// Real no_std for the firmware; `cargo test` on this crate alone links std
// anyway (the test harness requires it) so tests use the normal prelude
// (Vec, vec!, etc.) with no extra ceremony.
#![cfg_attr(not(test), no_std)]

pub mod codec;
pub mod gatt;
