//! Re-exports the shared Steam Controller 2 protocol codec.
//!
//! The actual parser, output formatters, and their tests live in
//! `crates/steam-protocol` — the same implementation the ESP32-S3 firmware
//! uses (`src/controller.rs` there is the same kind of re-export shim).
//! Kept as a local module (rather than having every call site reference
//! `steam_protocol::codec::` directly) so this crate's internal `crate::controller::X`
//! paths didn't need to change when the implementation moved out.

pub use steam_protocol::codec::*;
