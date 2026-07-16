//! Re-exports the shared Steam Controller 2 protocol codec.
//!
//! The actual parser, output formatters, and their tests live in
//! `crates/steam-protocol` — the same implementation the desktop
//! `freepuck-software` bridge uses (`tools/freepuck-software/src/controller.rs`
//! is the same kind of re-export shim). Having one implementation instead of
//! two independently-maintained copies is the point: cross-referencing an
//! independent reverse-engineering of the same controller
//! (safijari/openpuck) found three real bugs that existed in both copies —
//! see `steam_protocol::codec::parse_steam`'s doc comment.
//!
//! CAUTION: this change could not be compiled against the real target in
//! the environment that made it — no ESP32-S3/xtensa Rust toolchain was
//! available (`rustup target add xtensa-esp32s3-none-elf` fails outright;
//! that target needs the separate `espup`-built compiler fork). The shared
//! crate itself (`crates/steam-protocol`) was fully built and tested on a
//! host target, including a real `no_std` build, which exercises the same
//! code this shim re-exports — but the firmware's *use* of it (this file,
//! and the delegated builders in `bluetooth.rs`) has not been built even
//! once. Build with the real toolchain before trusting it.

pub use steam_protocol::codec::*;
