// Flash-backed MAC address storage for Steam Controller pairing.
// Calls ESP32-S3 ROM SPI flash functions directly to avoid embedded-storage
// trait-resolution issues. Addresses are from the ESP32-S3 ROM table.
//
// Layout at MAC_FLASH_OFFSET: [MAGIC: 4 bytes][ADDR_KIND: 1 byte][MAC: 6 bytes]

use core::sync::atomic::{AtomicU8, Ordering};

/// Offset into flash where we store the paired MAC address.
/// Factory app partition ends at 0xFF0000 on a 16 MB flash, so this sector is free.
const MAC_FLASH_OFFSET: u32 = 0xFF_0000;
const SECTOR_NUM: u32 = MAC_FLASH_OFFSET / 4096;
const MAGIC: [u8; 4] = [0xDE, 0xAD, 0xC0, 0xDE];
#[allow(dead_code)]
const RECORD_SIZE: usize = 11; // 4 magic + 1 kind + 6 mac

// ESP32-S3 ROM SPI flash functions (ROM table addresses).
// Run inside a critical section by the ROM itself.
extern "C" {
    #[link_name = "esp_rom_spiflash_read"]
    fn rom_read(src_addr: u32, data: *mut u32, len: u32) -> i32;
    #[link_name = "esp_rom_spiflash_erase_sector"]
    fn rom_erase_sector(sector_number: u32) -> i32;
    #[link_name = "esp_rom_spiflash_write"]
    fn rom_write(dest_addr: u32, data: *const u32, len: u32) -> i32;
}

/// Read a previously paired MAC from flash.
/// Returns `Some((addr_kind, [u8; 6]))` when the magic is valid.
pub fn load_mac() -> Option<(u8, [u8; 6])> {
    let mut buf = [0u8; 16]; // 16-byte aligned read (ROM needs u32 alignment)
    let rc = unsafe { rom_read(MAC_FLASH_OFFSET, buf.as_mut_ptr() as *mut u32, 16) };
    if rc != 0 {
        log::warn!("Flash read error {}", rc);
        return None;
    }
    if buf[0..4] != MAGIC {
        return None;
    }
    let kind = buf[4];
    let mut mac = [0u8; 6];
    mac.copy_from_slice(&buf[5..11]);
    log::info!("Loaded MAC from flash: {:02x?} (kind={})", mac, kind);
    Some((kind, mac))
}

/// Erase the sector and write a new MAC address record.
pub fn save_mac(addr_kind: u8, mac: &[u8; 6]) {
    // Erase the 4 KB sector
    let rc = unsafe { rom_erase_sector(SECTOR_NUM) };
    if rc != 0 {
        log::error!("Flash erase error {}", rc);
        return;
    }

    // Build a 16-byte aligned write buffer (ROM write needs u32 alignment + multiple-of-4 size)
    let mut buf = [0xFFu8; 16];
    buf[0..4].copy_from_slice(&MAGIC);
    buf[4] = addr_kind;
    buf[5..11].copy_from_slice(mac);

    let rc = unsafe { rom_write(MAC_FLASH_OFFSET, buf.as_ptr() as *const u32, 16) };
    if rc != 0 {
        log::error!("Flash write error {}", rc);
    } else {
        log::info!("MAC {:02x?} saved to flash", mac);
    }
}

/// Erase the stored MAC so the device performs a fresh scan on next boot.
#[allow(dead_code)]
pub fn clear_mac() {
    let rc = unsafe { rom_erase_sector(SECTOR_NUM) };
    if rc == 0 {
        log::info!("Stored MAC cleared");
    } else {
        log::error!("Clear flash error {}", rc);
    }
}

const MODE_FLASH_OFFSET: u32 = 0xFF_1000;
const MODE_SECTOR_NUM: u32 = MODE_FLASH_OFFSET / 4096;
const MODE_MAGIC: [u8; 4] = [0xBA, 0xBE, 0xC0, 0xDE];

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum OutputMode {
    XInput    = 0,
    SwitchPro = 1,
    DualSense = 2,
}

const OUTPUT_MODE_UNINIT: u8 = 0xFF;
static OUTPUT_MODE_CACHE: AtomicU8 = AtomicU8::new(OUTPUT_MODE_UNINIT);

fn output_mode_from_u8(mode: u8) -> Option<OutputMode> {
    match mode {
        0 => Some(OutputMode::XInput),
        1 => Some(OutputMode::SwitchPro),
        2 => Some(OutputMode::DualSense),
        _ => None,
    }
}

/// Fast RAM-backed output mode read for hot paths.
///
/// If cache has not been initialized yet, defaults to DualSense until a flash
/// load/save path populates it.
pub fn current_output_mode() -> OutputMode {
    output_mode_from_u8(OUTPUT_MODE_CACHE.load(Ordering::Relaxed)).unwrap_or(OutputMode::DualSense)
}

fn cache_output_mode(mode: OutputMode) {
    OUTPUT_MODE_CACHE.store(mode as u8, Ordering::Relaxed);
}

/// Read output mode from flash, defaulting to DualSense if not found or invalid.
pub fn load_output_mode() -> OutputMode {
    let mut buf = [0u8; 16];
    let rc = unsafe { rom_read(MODE_FLASH_OFFSET, buf.as_mut_ptr() as *mut u32, 16) };
    if rc != 0 {
        log::warn!("Output mode read failed (rc={}), defaulting to DualSense", rc);
        cache_output_mode(OutputMode::DualSense);
        return OutputMode::DualSense;
    }
    if buf[0..4] != MODE_MAGIC {
        log::warn!("Output mode magic missing, defaulting to DualSense");
        cache_output_mode(OutputMode::DualSense);
        return OutputMode::DualSense;
    }
    let mode = match output_mode_from_u8(buf[4]) {
        Some(mode) => mode,
        None => {
            log::warn!("Output mode byte {} invalid, resetting flash to DualSense", buf[4]);
            save_output_mode(OutputMode::DualSense);
            OutputMode::DualSense
        }
    };
    cache_output_mode(mode);
    log::info!("Loaded output mode {:?} from flash", mode);
    mode
}

/// Erase sector and save active output mode.
pub fn save_output_mode(mode: OutputMode) {
    let rc = unsafe { rom_erase_sector(MODE_SECTOR_NUM) };
    if rc != 0 {
        log::error!("Flash erase error {}", rc);
        return;
    }
    let mut buf = [0xFFu8; 16];
    buf[0..4].copy_from_slice(&MODE_MAGIC);
    buf[4] = mode as u8;

    let rc = unsafe { rom_write(MODE_FLASH_OFFSET, buf.as_ptr() as *const u32, 16) };
    if rc != 0 {
        log::error!("Flash write error {}", rc);
    } else {
        cache_output_mode(mode);
        log::info!("Output mode {:?} saved to flash", mode);
    }
}

