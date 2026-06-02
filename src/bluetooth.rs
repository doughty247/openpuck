// BLE connection and client task module using trouble-host

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use esp_radio::ble::controller::BleConnector;
use bt_hci::controller::ExternalController;
use trouble_host::prelude::*;
use core::sync::atomic::{AtomicU8, Ordering};

type BleController = ExternalController<BleConnector<'static>, 20>;

/// Vendor-specific prefix that all Steam Controllers advertise.
/// Observed advertisement name: "Steam Ctrl (BT) FXA9961102DD6"
const STEAM_NAME_PREFIX: &str = "Steam Ctrl";
const STEAM_PUCK_NAME_PREFIX: &str = "Steam Controller Puck";

/// Returns the device type code (BleDevice variant as u8) for a given advertisement name.
fn match_device_name(name: Option<&str>) -> Option<u8> {
    let n = name?;
    if n.starts_with(STEAM_NAME_PREFIX)     { Some(0) }
    else if n.starts_with(STEAM_PUCK_NAME_PREFIX) { Some(0) }  // SC2 uses Valve GATT, same path
    else { None }
}

/// Tracks which BLE device type is currently connected.
/// Read by the coordinator task to route input reports to the correct parser.
pub static CURRENT_BLE_DEVICE: AtomicU8 = AtomicU8::new(0);

/// Signal for forwarding raw HID reports to the USB coordinator task.
pub static BLE_REPORTS: Signal<CriticalSectionRawMutex, [u8; 64]> = Signal::new();

/// Connection state signal used by the coordinator to clear stale USB state on disconnect.
pub static BLE_CONNECTED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Haptics intents produced by USB OUT parsing and consumed by Steam BLE writer.
pub static BLE_HAPTICS: Channel<CriticalSectionRawMutex, crate::controller::HapticsIntent, 16> = Channel::new();

/// Signal sent from the EventHandler to the client task when a target device is found.
/// Carries (addr_kind_byte, mac_bytes, device_type).
static SCAN_RESULT: Signal<CriticalSectionRawMutex, (u8, [u8; 6], u8)> = Signal::new();

/// Optionally set by the client task before scanning; the EventHandler matches this
/// exact MAC (for reconnect) OR any device with STEAM_NAME_PREFIX (for first-pair).
static SAVED_TARGET: Signal<CriticalSectionRawMutex, (u8, [u8; 6])> = Signal::new();

// Steam Controller 2 (Triton) output report commands
const TRITON_CMD_RUMBLE: u8 = 0x80;  // HID report reference ID for output rumble report
const TRITON_CMD_SET_SETTINGS: u8 = 0x87;
const TRITON_SETTING_LIZARD_MODE: u8 = 0x09;
const TRITON_SETTING_HAPTICS_ENABLED: u8 = 70;
const TRITON_SETTING_HAPTIC_MASTER_GAIN_DB: u8 = 76;
const TRITON_SETTING_HAPTIC_INTENSITY: u8 = 79;
const TARGET_CONN_INTERVAL_US: u64 = 7_500;
const TRITON_HID_OUTPUT_HANDLE_HINT: u16 = 0x0067;
const TRITON_HID_FEATURE_HANDLE_HINT: u16 = 0x0085;

// Deterministic protocol selection for troubleshooting:
// 1 = Triton output report 0x80 to selected rumble characteristic (default)
// 2 = Feature report 0xEB to Valve command characteristic
// 3 = 64-byte feature-wrapped 0x80 to Valve command characteristic
// 4 = Raw 0x80 to Valve command characteristic
// 5 = Send all formats (legacy shotgun mode)
// 6 = Feature haptic pulse (0x8F) to Valve command characteristic
// 7 = Triton output haptic pulse (0x81)
// 8 = Triton output haptic command click (0x82)
const RUMBLE_PROTOCOL_MODE: u8 = 1;
const AUTO_RUMBLE_CYCLE: bool = false;
const AUTO_RUMBLE_LEVEL: u16 = 0xFFFF;
const AUTO_RUMBLE_TOGGLE_MS: u64 = 600;
const AUTO_RUMBLE_MODE_DWELL_MS: u64 = 4000;

fn rumble_mode_name(mode: u8) -> &'static str {
    match mode {
        1 => "triton-0x80-on-rumble-char",
        2 => "feature-0xeb-on-valve-char",
        3 => "feature-wrapped-0x80-64b-on-valve-char",
        4 => "raw-0x80-on-valve-char",
        5 => "all-formats",
        6 => "feature-haptic-pulse-0x8f",
        7 => "triton-out-haptic-pulse-0x81",
        8 => "triton-out-haptic-command-click-0x82",
        _ => "unknown",
    }
}

fn log_conn_interval_verdict(prefix: &str, interval: embassy_time::Duration) {
    let us = interval.as_micros();
    if us <= TARGET_CONN_INTERVAL_US {
        log::info!(
            "{} interval accepted at {}us (<= {}us target)",
            prefix,
            us,
            TARGET_CONN_INTERVAL_US
        );
    } else {
        log::warn!(
            "{} interval is {}us (> {}us target)",
            prefix,
            us,
            TARGET_CONN_INTERVAL_US
        );
    }
}
fn scale_rumble(val: u16) -> u16 {
    if val == 0 {
        0
    } else {
        // Boost low values so they are felt on the LRA,
        // mapping 1..65535 to 12000..65535.
        let min_val = 12000u32;
        let max_val = 65535u32;
        let scaled = min_val + ((val as u32) * (max_val - min_val) / 65535);
        scaled as u16
    }
}

/// Build a Steam Controller 2 (Triton) haptic rumble payload without report ID.
/// Layout matches SDL's MsgHapticRumble payload:
///   type:u8, intensity:u16, left.speed:u16, left.gain:i8, right.speed:u16, right.gain:i8
/// HID output report chars may already imply report ID via Report Reference (0x2908).
fn build_triton_rumble(left_speed: u16, right_speed: u16) -> [u8; 9] {
    let scaled_left = scale_rumble(left_speed);
    let scaled_right = scale_rumble(right_speed);
    [
        0x00, // type
        0x00, // intensity (low) - 0 triggers internal hardware emulator
        0x00, // intensity (high)
        (scaled_left & 0xFF) as u8,
        (scaled_left >> 8) as u8,
        0x06, // left gain dB (maximum boost)
        (scaled_right & 0xFF) as u8,
        (scaled_right >> 8) as u8,
        0x06, // right gain dB (maximum boost)
    ]
}

/// Full Triton haptic rumble output report including report ID 0x80.
fn build_triton_rumble_with_id(left_speed: u16, right_speed: u16) -> [u8; 10] {
    let payload = build_triton_rumble(left_speed, right_speed);
    let mut out = [0u8; 10];
    out[0] = TRITON_CMD_RUMBLE;
    out[1..].copy_from_slice(&payload);
    out
}

fn build_triton_lizard_off() -> [u8; 64] {
    let mut buf = [0u8; 64];
    buf[0] = TRITON_CMD_SET_SETTINGS;
    buf[1] = 0x03;
    buf[2] = TRITON_SETTING_LIZARD_MODE;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf
}

fn build_triton_haptics_enable() -> [u8; 11] {
    [
        TRITON_CMD_SET_SETTINGS,
        0x09, // 3 settings * 3 bytes each
        TRITON_SETTING_HAPTICS_ENABLED,
        0x01,
        0x00, // ON
        TRITON_SETTING_HAPTIC_MASTER_GAIN_DB,
        0x06,
        0x00, // +6 dB (maximum master gain)
        TRITON_SETTING_HAPTIC_INTENSITY,
        0x04,
        0x00, // INSANE
    ]
}

// Compatibility fallback for Steam feature-report rumble path used by some devices.
fn build_feature_rumble_0xeb(left_speed: u16, right_speed: u16) -> [u8; 11] {
    [
        0xEB,
        0x09,
        0x00,
        0x00,
        0x00,
        (left_speed & 0xFF) as u8,
        (left_speed >> 8) as u8,
        (right_speed & 0xFF) as u8,
        (right_speed >> 8) as u8,
        0x02,
        0x00,
    ]
}

// Some stacks tunnel output reports through feature-report framing.
fn build_feature_wrapped_rumble_0x80(left_speed: u16, right_speed: u16) -> [u8; 64] {
    let mut buf = [0u8; 64];
    buf[0] = TRITON_CMD_RUMBLE;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = (left_speed & 0xFF) as u8;
    buf[5] = (left_speed >> 8) as u8;
    buf[6] = 0x00;
    buf[7] = (right_speed & 0xFF) as u8;
    buf[8] = (right_speed >> 8) as u8;
    buf[9] = 0x00;
    buf
}

// Legacy feature-report haptic pulse command.
// MsgFireHapticPulse payload: which_pad, pulse_duration, pulse_interval, pulse_count, dBgain, priority
fn build_feature_haptic_pulse_0x8f() -> [u8; 12] {
    [
        0x8F,
        0x0A,
        0x03, // both pads
        0x40,
        0x00, // pulse_duration = 64
        0x20,
        0x00, // pulse_interval = 32
        0x10,
        0x00, // pulse_count = 16
        0x00,
        0x00, // dBgain = 0
        0x00, // priority = normal
    ]
}

fn build_feature_haptic_pulse_0x8f_64() -> [u8; 64] {
    let mut buf = [0u8; 64];
    let short = build_feature_haptic_pulse_0x8f();
    buf[..short.len()].copy_from_slice(&short);
    buf
}

fn build_triton_haptic_pulse_0x81() -> [u8; 8] {
    [
        0x81,
        0x03, // both sides
        0x40,
        0x00, // on_us
        0x20,
        0x00, // off_us
        0x10,
        0x00, // repeat_count
    ]
}

fn build_triton_haptic_command_click_0x82() -> [u8; 4] {
    [
        0x82,
        0x03, // both sides
        0x02, // click
        0x18, // gain_db
    ]
}

// ---------------------------------------------------------------------------
// Scan EventHandler
// ---------------------------------------------------------------------------

struct SteamScanHandler;

impl EventHandler for SteamScanHandler {
    fn on_adv_reports(&self, mut reports: bt_hci::param::LeAdvReportsIter) {
        let saved = SAVED_TARGET.try_take();
        while let Some(Ok(report)) = reports.next() {
            let addr_kind = report.addr_kind.0;
            let mac: [u8; 6] = report.addr.0;
            let name = get_local_name(report.data);
            log::debug!("BLE ADV: kind={} addr={:02x?} name={:?}", addr_kind, mac, name);

            // Exact saved MAC for fast reconnect, OR match any supported device name.
            let device_type = match_device_name(name);
            let is_match = if let Some((_, saved_mac)) = saved {
                mac == saved_mac || device_type.is_some()
            } else {
                device_type.is_some()
            };

            if is_match {
                let dt = device_type.unwrap_or(0);
                log::info!("BLE device seen: kind={} mac={:02x?} name={:?} type={}", addr_kind, mac, name, dt);
                SCAN_RESULT.signal((addr_kind, mac, dt));
                return;
            }
        }
        if let Some(s) = saved { SAVED_TARGET.signal(s); }
    }

    fn on_ext_adv_reports(&self, mut reports: bt_hci::param::LeExtAdvReportsIter) {
        let saved = SAVED_TARGET.try_take();
        while let Some(Ok(report)) = reports.next() {
            let mac: [u8; 6] = report.addr.0;
            let name = get_local_name(report.data);
            log::debug!("BLE EXT ADV: addr={:02x?} name={:?}", mac, name);
            let device_type = match_device_name(name);
            let is_match = if let Some((_, saved_mac)) = saved {
                mac == saved_mac || device_type.is_some()
            } else {
                device_type.is_some()
            };
            if is_match {
                let dt = device_type.unwrap_or(0);
                log::info!("BLE device seen (ext): mac={:02x?} type={}", mac, dt);
                SCAN_RESULT.signal((0, mac, dt));
                return;
            }
        }
        if let Some(s) = saved { SAVED_TARGET.signal(s); }
    }
}

static SCAN_HANDLER: SteamScanHandler = SteamScanHandler;

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

/// BLE host runner – processes all HCI events. Uses the scan handler so that
/// advertisement callbacks are dispatched to `SteamScanHandler::on_adv_reports`.
#[embassy_executor::task]
pub async fn ble_runner_task(mut runner: Runner<'static, BleController, DefaultPacketPool>) {
    log::info!("Starting BLE runner task");
    loop {
        if let Err(e) = runner.run_with_handler(&SCAN_HANDLER).await {
            log::error!("BLE runner error: {:?}", e);
        }
    }
}

/// BLE client task.
/// Always scans first to confirm the controller is advertising,
/// then immediately connects. This prevents `connect()` hanging forever.
#[embassy_executor::task]
pub async fn ble_client_task(
    stack: &'static Stack<'static, BleController, DefaultPacketPool>,
    central: Central<'static, BleController, DefaultPacketPool>,
) {
    log::info!("Starting BLE client task");
    BLE_CONNECTED.signal(false);

    let saved = crate::storage::load_mac();
    if saved.is_some() {
        log::info!("Saved MAC found – will scan to confirm device is advertising before connecting");
    } else {
        log::info!("No saved MAC – will scan for '{}'", STEAM_NAME_PREFIX);
    }

    // Unified loop: scan → connect → handle → repeat
    let mut central = central;
    let mut current_saved = saved;
    let mut consecutive_connect_failures = 0u8;
    loop {
        // --- Scan until we see the target device actively advertising ---
        let (kind, mac, device_type, returned_central) = scan_for_controller(central, current_saved).await;
        central = returned_central;

        // Always save/update MAC (handles address rotation)
        let needs_save = current_saved.map(|(_, m)| m != mac).unwrap_or(true);
        if needs_save {
            log::info!("Saving new MAC {:02x?} to flash", mac);
            crate::storage::save_mac(kind, &mac);
            current_saved = Some((kind, mac));
        }

        // --- Connect immediately while device is advertising ---
        // Wait 3s for controller to fully enter pairing mode before handshake
        log::info!("Device seen – waiting 3s for controller to settle…");
        embassy_time::Timer::after_secs(3).await;
        log::info!("Connecting to {:02x?}", mac);
        let addr = Address::random(mac);
        let filter_accept_list = &[(addr.kind, &addr.addr)];
        let conn_config = ConnectConfig {
            scan_config: ScanConfig {
                active: true,
                filter_accept_list,
                ..Default::default()
            },
            connect_params: RequestedConnParams {
                min_connection_interval: embassy_time::Duration::from_micros(7500),
                max_connection_interval: embassy_time::Duration::from_micros(7500),
                ..Default::default()
            },
        };

        match central.connect(&conn_config).await {
            Ok(connection) => {
                log::info!("Connected!");
                BLE_CONNECTED.signal(true);
                handle_connection(stack, connection, device_type).await;
                BLE_CONNECTED.signal(false);
                consecutive_connect_failures = 0;
                log::info!("Disconnected – rescanning in 2 s");
            }
            Err(e) => {
                log::error!("connect() failed: {:?}", e);
                BLE_CONNECTED.signal(false);
                consecutive_connect_failures = consecutive_connect_failures.saturating_add(1);
                if consecutive_connect_failures >= 3 {
                    log::warn!("Repeated connect failures, clearing saved MAC and forcing fresh pairing scan");
                    crate::storage::clear_mac();
                    current_saved = None;
                    consecutive_connect_failures = 0;
                }
            }
        }
        embassy_time::Timer::after_secs(2).await;
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Runs BLE scanning until a supported controller is found advertising.
/// - With `saved_mac`: tries exact MAC first, falls back to name scan after 30s.
/// - Without `saved_mac`: matches any supported controller name.
/// Returns `(addr_kind, mac, device_type, central)`.
async fn scan_for_controller(
    central: Central<'static, BleController, DefaultPacketPool>,
    saved_mac: Option<(u8, [u8; 6])>,
) -> (u8, [u8; 6], u8, Central<'static, BleController, DefaultPacketPool>) {
    let mut scanner = Scanner::new(central);
    let scan_config = ScanConfig { active: true, ..Default::default() };

    // Phase A: try saved MAC (up to 30s), then fall back to name
    let mut using_saved = saved_mac.is_some();
    let mut deadline_ticks = 6u32; // 6 × 5s = 30s before fallback

    if using_saved {
        let (_, mac) = saved_mac.unwrap();
        log::info!("Waiting for controller {:02x?} to advertise (30s before fallback)…", mac);
        SAVED_TARGET.signal(saved_mac.unwrap());
    } else {
        log::info!("Scanning for '{}' – press Steam button on controller", STEAM_NAME_PREFIX);
    }

    let result = loop {
        match scanner.scan(&scan_config).await {
            Ok(session) => {
                let found = embassy_futures::select::select(
                    SCAN_RESULT.wait(),
                    embassy_time::Timer::after_secs(5),
                ).await;
                drop(session);
                match found {
                    embassy_futures::select::Either::First(r) => break r,
                    embassy_futures::select::Either::Second(_) => {
                        if using_saved {
                            deadline_ticks -= 1;
                            if deadline_ticks == 0 {
                                // Saved MAC not seen – controller has new address, fall back to name
                                log::warn!("Saved MAC not seen in 30s – falling back to name scan");
                                // Clear stale saved target so handler uses name prefix
                                let _ = SAVED_TARGET.try_take();
                                using_saved = false;
                                log::info!("Now scanning for '{}' prefix…", STEAM_NAME_PREFIX);
                            } else {
                                log::info!("Still waiting ({} × 5s remaining)…", deadline_ticks);
                            }
                        } else {
                            log::info!("Still scanning for Steam Controller…");
                        }
                    }
                }
            }
            Err(e) => {
                log::error!("scan() error: {:?}", e);
                embassy_time::Timer::after_secs(1).await;
            }
        }
    };

    let central = scanner.into_inner();
    (result.0, result.1, result.2, central)
}

/// Parse standard BLE AD structures to find the local name (type 0x08 or 0x09).
fn get_local_name(data: &[u8]) -> Option<&str> {
    let mut iter = data;
    while iter.len() >= 2 {
        let len = iter[0] as usize;
        if len == 0 || iter.len() < 1 + len {
            break;
        }
        let ad_type = iter[1];
        let payload = &iter[2..1 + len];
        if ad_type == 0x09 || ad_type == 0x08 {
            if let Ok(name) = core::str::from_utf8(payload) {
                return Some(name);
            }
        }
        iter = &iter[1 + len..];
    }
    None
}

// ---------------------------------------------------------------------------
// GATT connection handler
// ---------------------------------------------------------------------------

async fn handle_connection(
    stack: &'static Stack<'static, BleController, DefaultPacketPool>,
    connection: Connection<'static, DefaultPacketPool>,
    device_type: u8,
) {
    CURRENT_BLE_DEVICE.store(device_type, Ordering::Relaxed);
    log::info!("handle_connection: device_type={}", device_type);
    // --- Step 1: Initiate "Just Works" BLE pairing (needed for HID NOTIFY) ---
    log::info!("Requesting BLE security (Just Works pairing)…");
    if let Err(e) = connection.request_security() {
        log::error!("request_security failed: {:?}", e);
        return;
    }

    // Wait for PairingComplete (or PairingFailed / Disconnected)
    loop {
        match connection.next().await {
            ConnectionEvent::PairingComplete { security_level, .. } => {
                log::info!("Paired! Security level: {:?}", security_level);
                break;
            }
            ConnectionEvent::PairingFailed(e) => {
                log::error!("Pairing failed: {:?}", e);
                return;
            }
            ConnectionEvent::Disconnected { reason } => {
                log::warn!("Disconnected during pairing: {:?}", reason);
                return;
            }
            other => {
                log::debug!("Connection event during pairing: {:?}", other);
            }
        }
    }

    // --- Step 2: GATT service discovery and subscription ---
    if device_type == 0 {
    let client = match GattClient::<_, _, 64>::new(stack, &connection).await {
        Ok(c) => c,
        Err(e) => { log::error!("GattClient::new failed: {:?}", e); return; }
    };

    let client_operations = async {
        const VALVE_SERVICE_UUID: Uuid = Uuid::Uuid128([
            0xf3, 0xe5, 0x31, 0x71, 0x56, 0x38, 0x02, 0xb4, 0x13, 0x43, 0x35, 0x17, 0x32, 0x6c, 0x0f, 0x10
        ]);
        const HID_SERVICE_UUID: Uuid = Uuid::Uuid16([0x12, 0x18]);
        const HID_REPORT_UUID: Uuid = Uuid::Uuid16([0x4D, 0x2A]);
        const HID_CONTROL_POINT_UUID: Uuid = Uuid::Uuid16([0x4C, 0x2A]);
        const HID_PROTOCOL_MODE_UUID: Uuid = Uuid::Uuid16([0x4E, 0x2A]);
        const TRITON_INPUT_UUID: Uuid = Uuid::Uuid128([
            0xf3, 0xe5, 0x31, 0x71, 0x56, 0x38, 0x02, 0xb4, 0x13, 0x43, 0x35, 0x17, 0x7a, 0x6c, 0x0f, 0x10
        ]);
        const D0G_INPUT_UUID: Uuid = Uuid::Uuid128([
            0xf3, 0xe5, 0x31, 0x71, 0x56, 0x38, 0x02, 0xb4, 0x13, 0x43, 0x35, 0x17, 0x33, 0x6c, 0x0f, 0x10
        ]);
        const REPORT_UUID: Uuid = Uuid::Uuid128([
            0xf3, 0xe5, 0x31, 0x71, 0x56, 0x38, 0x02, 0xb4, 0x13, 0x43, 0x35, 0x17, 0x34, 0x6c, 0x0f, 0x10
        ]);

        log::info!(
            "Rumble mode {} ({})",
            RUMBLE_PROTOCOL_MODE,
            rumble_mode_name(RUMBLE_PROTOCOL_MODE)
        );
        if AUTO_RUMBLE_CYCLE {
            log::warn!(
                "AUTO_RUMBLE_CYCLE enabled: cycling modes 1..8 every {}ms with {}ms on/off pulses at level {}",
                AUTO_RUMBLE_MODE_DWELL_MS,
                AUTO_RUMBLE_TOGGLE_MS,
                AUTO_RUMBLE_LEVEL
            );
        }

        log::info!("Enumerating full GATT topology for Steam controller…");
        match client.services().await {
            Ok(all_services) => {
                for svc in all_services.iter() {
                    log::info!("GATT service: {:?}", svc);
                    match client.characteristics::<32>(svc).await {
                        Ok(chars) => {
                            for characteristic in chars.iter() {
                                log::info!(
                                    "  char handle=0x{:04x} props=0x{:02x}",
                                    characteristic.handle,
                                    characteristic.props.to_raw()
                                );
                            }
                        }
                        Err(e) => {
                            log::warn!("  failed to enumerate characteristics for service {:?}: {:?}", svc, e);
                        }
                    }
                }
            }
            Err(e) => {
                log::warn!("Full GATT services enumeration failed: {:?}", e);
            }
        }

        log::info!("Discovering Valve custom service…");
        let services = match client.services_by_uuid(&VALVE_SERVICE_UUID).await {
            Ok(s) => s,
            Err(e) => { log::error!("services_by_uuid for Valve: {:?}", e); return; }
        };
        let service = match services.first() {
            Some(s) => s.clone(),
            None => { log::error!("Valve custom service not found!"); return; }
        };

        log::info!("Discovering characteristics by UUID…");
        let triton_input_char = client.characteristic_by_uuid::<[u8]>(&service, &TRITON_INPUT_UUID).await.ok();
        let d0g_input_char = client.characteristic_by_uuid::<[u8]>(&service, &D0G_INPUT_UUID).await.ok();
        let report_char = client.characteristic_by_uuid::<[u8]>(&service, &REPORT_UUID).await.ok();

        // Detect controller generation by which input characteristic is present.
        // SC2 (Triton) advertises 0x...7a; Gen1 (D0G, 2015) advertises 0x...33.
        let is_triton = triton_input_char.is_some();
        if !is_triton {
            log::warn!("Gen 1 Steam Controller (D0G) detected — only Steam Controller 2 (Triton) is supported. Disconnecting.");
            return;
        }

        let input_char = match triton_input_char.or(d0g_input_char) {
            Some(ic) => ic,
            None => { log::error!("Valve input characteristic not found"); return; }
        };

        log::info!("Discovering HID report characteristics for Triton output and feature reports…");
        let (hid_output_char, hid_feature_char) = match client.services_by_uuid(&HID_SERVICE_UUID).await {
            Ok(services) => match services.first() {
                Some(hid_service) => {
                    let control_point = client.characteristic_by_uuid::<[u8]>(hid_service, &HID_CONTROL_POINT_UUID).await.ok();
                    let protocol_mode = client.characteristic_by_uuid::<[u8]>(hid_service, &HID_PROTOCOL_MODE_UUID).await.ok();

                    match client.characteristics::<32>(hid_service).await {
                        Ok(chars) => {
                            let mut best_output: Option<(i32, Characteristic<[u8]>)> = None;
                            let mut best_feature: Option<(i32, Characteristic<[u8]>)> = None;
                            let mut hinted_output: Option<Characteristic<[u8]>> = None;
                            let mut hinted_feature: Option<Characteristic<[u8]>> = None;
                            for characteristic in chars {
                                let props = characteristic.props;
                                let writable = props.any(&[CharacteristicProp::Write, CharacteristicProp::WriteWithoutResponse]);
                                let write_no_resp = props.any(&[CharacteristicProp::WriteWithoutResponse]);
                                let write_with_resp = props.any(&[CharacteristicProp::Write]);
                                let readable = props.any(&[CharacteristicProp::Read]);
                                let notifiable = props.any(&[CharacteristicProp::Notify, CharacteristicProp::Indicate]);
                                let is_control_point = control_point.as_ref().map(|c| c.handle == characteristic.handle).unwrap_or(false);
                                let is_protocol_mode = protocol_mode.as_ref().map(|c| c.handle == characteristic.handle).unwrap_or(false);

                                log::info!(
                                    "HID char handle=0x{:04x} props=0x{:02x} writable={} readable={} notifiable={} cp={} pm={}",
                                    characteristic.handle,
                                    props.to_raw(),
                                    writable,
                                    readable,
                                    notifiable,
                                    is_control_point,
                                    is_protocol_mode,
                                );

                                if !writable || is_control_point || is_protocol_mode {
                                    continue;
                                }

                                if characteristic.handle == TRITON_HID_OUTPUT_HANDLE_HINT {
                                    hinted_output = Some(characteristic);
                                    continue;
                                }
                                if characteristic.handle == TRITON_HID_FEATURE_HANDLE_HINT {
                                    hinted_feature = Some(characteristic);
                                    continue;
                                }

                                let mut score = 0i32;
                                if readable { score += 3; }
                                if write_with_resp { score += 2; }
                                if write_no_resp { score += 2; }
                                if notifiable { score -= 2; }

                                // Output report usually prefers write-without-response and is often not readable.
                                if write_no_resp && !readable {
                                    match &best_output {
                                        Some((best_score, _)) if score <= *best_score => {}
                                        _ => best_output = Some((score, characteristic)),
                                    }
                                }

                                // Feature report is commonly writable and readable.
                                else if readable && write_with_resp {
                                    match &best_feature {
                                        Some((best_score, _)) if score <= *best_score => {}
                                        _ => best_feature = Some((score, characteristic)),
                                    }
                                }
                            }

                            let output = if let Some(ch) = hinted_output {
                                log::info!(
                                    "Selected HID output by Triton handle hint: 0x{:04x}",
                                    ch.handle
                                );
                                Some(ch)
                            } else if let Some((score, ch)) = best_output {
                                log::info!(
                                    "Selected HID output candidate handle=0x{:04x} score={} props=0x{:02x}",
                                    ch.handle,
                                    score,
                                    ch.props.to_raw()
                                );
                                Some(ch)
                            } else {
                                None
                            };

                            let feature = if let Some(ch) = hinted_feature {
                                log::info!(
                                    "Selected HID feature by Triton handle hint: 0x{:04x}",
                                    ch.handle
                                );
                                Some(ch)
                            } else if let Some((score, ch)) = best_feature {
                                log::info!(
                                    "Selected HID feature candidate handle=0x{:04x} score={} props=0x{:02x}",
                                    ch.handle,
                                    score,
                                    ch.props.to_raw()
                                );
                                Some(ch)
                            } else {
                                None
                            };

                            if output.is_none() {
                                // Last resort: use first discovered report characteristic if writable.
                                match client.characteristic_by_uuid::<[u8]>(hid_service, &HID_REPORT_UUID).await {
                                    Ok(ch) if ch.props.any(&[CharacteristicProp::Write, CharacteristicProp::WriteWithoutResponse]) => {
                                        log::warn!("Falling back to first HID report char handle=0x{:04x}", ch.handle);
                                        (Some(ch), feature)
                                    }
                                    Ok(ch) => {
                                        log::warn!(
                                            "First HID report char not writable (handle=0x{:04x}, props=0x{:02x})",
                                            ch.handle,
                                            ch.props.to_raw()
                                        );
                                        (output, feature)
                                    }
                                    Err(e) => {
                                        log::warn!("HID report characteristic discovery failed: {:?}", e);
                                        (output, feature)
                                    }
                                }
                            } else {
                                (output, feature)
                            }
                        }
                        Err(e) => {
                            log::warn!("HID characteristic enumeration failed: {:?}", e);
                            (None, None)
                        }
                    }
                }
                None => {
                    log::warn!("HID service (0x1812) not found for Triton");
                    (None, None)
                }
            },
            Err(e) => {
                log::warn!("HID service discovery failed: {:?}", e);
                (None, None)
            }
        };

        log::info!("Subscribing to input characteristic 0x{:04x}…", input_char.handle);
        let mut listener = match client.subscribe(&input_char, false).await {
            Ok(l) => l,
            Err(e) => { log::error!("Subscribe failed: {:?}", e); return; }
        };
        log::info!("Subscribed successfully!");

        // Send initialization commands to disable lizard mode.
        // Try the report characteristic first; fall back to the input characteristic.
        let cmd_char = report_char.as_ref().unwrap_or(&input_char);
        if report_char.is_none() {
            log::warn!("report_char (0x...34) not found — using input_char for commands");
        }
        let rumble_char = match hid_output_char.as_ref() {
            Some(ch) => {
                log::info!(
                    "Using HID output report characteristic 0x{:04x} for rumble",
                    ch.handle
                );
                ch
            }
            None => {
                log::warn!(
                    "No suitable HID output char found; falling back to Valve feature char 0x{:04x}",
                    cmd_char.handle
                );
                cmd_char
            }
        };
        let feature_char = match hid_feature_char.as_ref() {
            Some(ch) => {
                log::info!(
                    "Using HID feature report characteristic 0x{:04x} for settings/feature commands",
                    ch.handle
                );
                ch
            }
            None => {
                log::warn!(
                    "No suitable HID feature char found; falling back to Valve feature char 0x{:04x}",
                    cmd_char.handle
                );
                cmd_char
            }
        };
        let mut rumble_send_ok: u32 = 0;
        let mut rumble_send_fail: u32 = 0;
        let mut last_rumble_stats = embassy_time::Instant::now();

        // Try write-without-response first (matches how the kernel driver sends feature reports)
        log::info!("Sending CMD_CLEAR_DIGITAL_MAPPINGS (0x81) to 0x{:04x}…", feature_char.handle);
        let clear_cmd = [0x81];
        match client.write_characteristic_without_response(feature_char, &clear_cmd).await {
            Ok(_) => log::info!("  0x81 write-no-resp: OK"),
            Err(e) => {
                log::warn!("  0x81 write-no-resp failed: {:?}, trying with response…", e);
                match client.write_characteristic(feature_char, &clear_cmd).await {
                    Ok(_) => log::info!("  0x81 write-with-resp: OK"),
                    Err(e2) => log::error!("  0x81 write-with-resp also failed: {:?}", e2),
                }
            }
        }

        log::info!("Sending CMD_SET_SETTINGS to 0x{:04x}…", feature_char.handle);
        let settings_cmd = [0x87, 0x06, 0x07, 0x07, 0x00, 0x08, 0x07, 0x00];
        match client.write_characteristic_without_response(feature_char, &settings_cmd).await {
            Ok(_) => log::info!("  0x87 write-no-resp: OK"),
            Err(e) => {
                log::warn!("  0x87 write-no-resp failed: {:?}, trying with response…", e);
                match client.write_characteristic(feature_char, &settings_cmd).await {
                    Ok(_) => log::info!("  0x87 write-with-resp: OK"),
                    Err(e2) => log::error!("  0x87 write-with-resp also failed: {:?}", e2),
                }
            }
        }

        let haptics_enable_cmd = build_triton_haptics_enable();
        log::info!("Sending explicit haptics-enable settings to 0x{:04x}…", feature_char.handle);
        match client.write_characteristic_without_response(feature_char, &haptics_enable_cmd).await {
            Ok(_) => log::info!("  haptics-enable write-no-resp: OK"),
            Err(e) => {
                log::warn!("  haptics-enable write-no-resp failed: {:?}, trying with response…", e);
                match client.write_characteristic(feature_char, &haptics_enable_cmd).await {
                    Ok(_) => log::info!("  haptics-enable write-with-resp: OK"),
                    Err(e2) => log::error!("  haptics-enable write-with-resp also failed: {:?}", e2),
                }
            }
        }

        // SC2 (Triton): ~50ms hardware safety timeout means rumble stops automatically.
        // Resend active rumble every 40ms (SDL's TRITON_RUMBLE_RESEND_INTERVAL_MS).
        // Also send lizard-mode disable every 3s to keep the controller in game mode.
        // SDL sends Triton lizard-off as a 64-byte feature report with report ID 1.
        let lizard_off_cmd = build_triton_lizard_off();
        let haptics_enable_cmd = build_triton_haptics_enable();
        let mut resend_ticker = embassy_time::Ticker::every(embassy_time::Duration::from_millis(40));
        let mut last_lizard = embassy_time::Instant::now();
        let mut rumble_left: u16 = 0;
        let mut rumble_right: u16 = 0;
        let mut auto_mode: u8 = 1;
        let mut last_auto_mode_switch = embassy_time::Instant::now();
        let mut auto_pulse_on = true;
        let mut last_auto_toggle = embassy_time::Instant::now();

        loop {
            match embassy_futures::select::select(
                embassy_futures::select::select(listener.next(), resend_ticker.next()),
                BLE_HAPTICS.receive(),
            ).await {
                embassy_futures::select::Either::First(inner) => {
                    match inner {
                        embassy_futures::select::Either::First(notif) => {
                            let data = notif.as_ref();
                            log::debug!("VALVE NOTIF len={}: {:02x?}", data.len(), &data[..data.len().min(30)]);

                            let mut buf = [0u8; 64];
                            buf[0..2].copy_from_slice(&input_char.handle.to_le_bytes());
                            buf[2] = data.len() as u8;
                            let len = data.len().min(61);
                            buf[3..3+len].copy_from_slice(&data[..len]);
                            BLE_REPORTS.signal(buf);
                        }
                        embassy_futures::select::Either::Second(_) => {
                            if AUTO_RUMBLE_CYCLE {
                                if last_auto_mode_switch.elapsed()
                                    >= embassy_time::Duration::from_millis(AUTO_RUMBLE_MODE_DWELL_MS)
                                {
                                    auto_mode = if auto_mode >= 8 { 1 } else { auto_mode + 1 };
                                    log::warn!(
                                        "AUTO mode switch -> {} ({})",
                                        auto_mode,
                                        rumble_mode_name(auto_mode)
                                    );
                                    last_auto_mode_switch = embassy_time::Instant::now();
                                }
                                if last_auto_toggle.elapsed()
                                    >= embassy_time::Duration::from_millis(AUTO_RUMBLE_TOGGLE_MS)
                                {
                                    auto_pulse_on = !auto_pulse_on;
                                    last_auto_toggle = embassy_time::Instant::now();
                                }
                                let level = if auto_pulse_on { AUTO_RUMBLE_LEVEL } else { 0 };
                                rumble_left = level;
                                rumble_right = level;
                            }

                            // Resend active rumble (SC2 hardware auto-stops after ~50ms)
                            if rumble_left > 0 || rumble_right > 0 {
                                let pulse = build_triton_rumble(rumble_left, rumble_right);
                                let pulse_with_id = build_triton_rumble_with_id(rumble_left, rumble_right);
                                let mut tick_ok = true;
                                let mode = if AUTO_RUMBLE_CYCLE { auto_mode } else { RUMBLE_PROTOCOL_MODE };
                                match mode {
                                    1 => {
                                        if let Err(e) = client.write_characteristic_without_response(rumble_char, &pulse).await {
                                            log::warn!("Mode1 resend failed: {:?}", e);
                                            tick_ok = false;
                                        }
                                    }
                                    2 => {
                                        let feature_pulse = build_feature_rumble_0xeb(rumble_left, rumble_right);
                                        if let Err(e) = client.write_characteristic_without_response(feature_char, &feature_pulse).await {
                                            log::warn!("Mode2 resend failed: {:?}", e);
                                            tick_ok = false;
                                        }
                                    }
                                    3 => {
                                        let wrapped_pulse = build_feature_wrapped_rumble_0x80(rumble_left, rumble_right);
                                        if let Err(e) = client.write_characteristic_without_response(feature_char, &wrapped_pulse).await {
                                            log::warn!("Mode3 resend failed: {:?}", e);
                                            tick_ok = false;
                                        }
                                    }
                                    4 => {
                                        if let Err(e) = client.write_characteristic_without_response(cmd_char, &pulse_with_id).await {
                                            log::warn!("Mode4 resend failed: {:?}", e);
                                            tick_ok = false;
                                        }
                                    }
                                    5 => {
                                        if let Err(e) = client.write_characteristic(rumble_char, &pulse).await {
                                            log::warn!("Mode5 triton resend write-with-resp failed: {:?}", e);
                                            if let Err(e2) = client.write_characteristic_without_response(rumble_char, &pulse).await {
                                                log::warn!("Mode5 triton resend write-no-resp fallback failed: {:?}", e2);
                                                tick_ok = false;
                                            }
                                        }
                                        let feature_pulse = build_feature_rumble_0xeb(rumble_left, rumble_right);
                                        if let Err(e) = client.write_characteristic(feature_char, &feature_pulse).await {
                                            log::warn!("Mode5 0xEB resend write-with-resp failed: {:?}", e);
                                            if let Err(e2) = client.write_characteristic_without_response(feature_char, &feature_pulse).await {
                                                log::warn!("Mode5 0xEB resend write-no-resp fallback failed: {:?}", e2);
                                                tick_ok = false;
                                            }
                                        }
                                        let wrapped_pulse = build_feature_wrapped_rumble_0x80(rumble_left, rumble_right);
                                        if let Err(e) = client.write_characteristic(feature_char, &wrapped_pulse).await {
                                            log::warn!("Mode5 wrapped resend write-with-resp failed: {:?}", e);
                                            if let Err(e2) = client.write_characteristic_without_response(feature_char, &wrapped_pulse).await {
                                                log::warn!("Mode5 wrapped resend write-no-resp fallback failed: {:?}", e2);
                                                tick_ok = false;
                                            }
                                        }
                                        if let Err(e) = client.write_characteristic(cmd_char, &pulse_with_id).await {
                                            log::warn!("Mode5 raw resend write-with-resp failed: {:?}", e);
                                            if let Err(e2) = client.write_characteristic_without_response(cmd_char, &pulse_with_id).await {
                                                log::warn!("Mode5 raw resend write-no-resp fallback failed: {:?}", e2);
                                                tick_ok = false;
                                            }
                                        }
                                        let pulse_8f = build_feature_haptic_pulse_0x8f();
                                        if let Err(e) = client.write_characteristic(feature_char, &pulse_8f).await {
                                            log::warn!("Mode5 0x8F resend write-with-resp failed: {:?}", e);
                                            if let Err(e2) = client.write_characteristic_without_response(feature_char, &pulse_8f).await {
                                                log::warn!("Mode5 0x8F resend write-no-resp fallback failed: {:?}", e2);
                                                tick_ok = false;
                                            }
                                        }
                                        let pulse_81 = build_triton_haptic_pulse_0x81();
                                        if let Err(e) = client.write_characteristic(rumble_char, &pulse_81).await {
                                            log::warn!("Mode5 0x81 resend write-with-resp failed: {:?}", e);
                                            if let Err(e2) = client.write_characteristic_without_response(rumble_char, &pulse_81).await {
                                                log::warn!("Mode5 0x81 resend write-no-resp fallback failed: {:?}", e2);
                                                tick_ok = false;
                                            }
                                        }
                                        let cmd_82 = build_triton_haptic_command_click_0x82();
                                        if let Err(e) = client.write_characteristic(rumble_char, &cmd_82).await {
                                            log::warn!("Mode5 0x82 resend write-with-resp failed: {:?}", e);
                                            if let Err(e2) = client.write_characteristic_without_response(rumble_char, &cmd_82).await {
                                                log::warn!("Mode5 0x82 resend write-no-resp fallback failed: {:?}", e2);
                                                tick_ok = false;
                                            }
                                        }
                                    }
                                    6 => {
                                        let pulse_8f = build_feature_haptic_pulse_0x8f();
                                        if let Err(e) = client.write_characteristic_without_response(feature_char, &pulse_8f).await {
                                            log::warn!("Mode6 0x8F short resend failed: {:?}", e);
                                            tick_ok = false;
                                        }
                                        let pulse_8f_64 = build_feature_haptic_pulse_0x8f_64();
                                        if let Err(e) = client.write_characteristic_without_response(feature_char, &pulse_8f_64).await {
                                            log::warn!("Mode6 0x8F/64B resend failed: {:?}", e);
                                            tick_ok = false;
                                        }
                                    }
                                    7 => {
                                        let pulse_81 = build_triton_haptic_pulse_0x81();
                                        if let Err(e) = client.write_characteristic_without_response(rumble_char, &pulse_81).await {
                                            log::warn!("Mode7 0x81 resend failed: {:?}", e);
                                            tick_ok = false;
                                        }
                                    }
                                    8 => {
                                        let cmd_82 = build_triton_haptic_command_click_0x82();
                                        if let Err(e) = client.write_characteristic_without_response(rumble_char, &cmd_82).await {
                                            log::warn!("Mode8 0x82 resend failed: {:?}", e);
                                            tick_ok = false;
                                        }
                                    }
                                    _ => {
                                        if let Err(e) = client.write_characteristic_without_response(rumble_char, &pulse).await {
                                            log::warn!("Default mode resend failed: {:?}", e);
                                            tick_ok = false;
                                        }
                                    }
                                }

                                if tick_ok {
                                    rumble_send_ok = rumble_send_ok.saturating_add(1);
                                } else {
                                    rumble_send_fail = rumble_send_fail.saturating_add(1);
                                }

                                if last_rumble_stats.elapsed() >= embassy_time::Duration::from_millis(2000) {
                                    log::info!(
                                        "Rumble stats mode={} ({}) ok={} fail={}",
                                        mode,
                                        rumble_mode_name(mode),
                                        rumble_send_ok,
                                        rumble_send_fail
                                    );
                                    last_rumble_stats = embassy_time::Instant::now();
                                }
                            }
                            // Lizard-mode keepalive every 3s
                            if last_lizard.elapsed() >= embassy_time::Duration::from_millis(3000) {
                                if let Err(e) = client.write_characteristic_without_response(feature_char, &lizard_off_cmd).await {
                                    log::warn!("Triton lizard keepalive failed: {:?}", e);
                                }
                                if let Err(e) = client.write_characteristic_without_response(feature_char, &haptics_enable_cmd).await {
                                    log::warn!("Triton haptics-enable keepalive failed: {:?}", e);
                                }
                                last_lizard = embassy_time::Instant::now();
                            }
                        }
                    }
                }
                embassy_futures::select::Either::Second(intent) => {
                    log::debug!(
                        "Haptics: L={} R={}",
                        intent.left_motor, intent.right_motor,
                    );
                    if AUTO_RUMBLE_CYCLE {
                        log::debug!("AUTO_RUMBLE_CYCLE active: ignoring host haptics values");
                        continue;
                    }
                    rumble_left = intent.left_motor;
                    rumble_right = intent.right_motor;
                    // Send immediately; the 40ms ticker will keep resending until motors → 0
                    let pulse = build_triton_rumble(rumble_left, rumble_right);
                    let pulse_with_id = build_triton_rumble_with_id(rumble_left, rumble_right);
                    let mut evt_ok = true;
                    match RUMBLE_PROTOCOL_MODE {
                        1 => {
                            if let Err(e) = client.write_characteristic_without_response(rumble_char, &pulse).await {
                                log::warn!("Mode1 immediate write failed: {:?}", e);
                                evt_ok = false;
                            }
                        }
                        2 => {
                            let feature_pulse = build_feature_rumble_0xeb(rumble_left, rumble_right);
                            if let Err(e) = client.write_characteristic_without_response(feature_char, &feature_pulse).await {
                                log::warn!("Mode2 immediate failed: {:?}", e);
                                evt_ok = false;
                            }
                        }
                        3 => {
                            let wrapped_pulse = build_feature_wrapped_rumble_0x80(rumble_left, rumble_right);
                            if let Err(e) = client.write_characteristic_without_response(feature_char, &wrapped_pulse).await {
                                log::warn!("Mode3 immediate failed: {:?}", e);
                                evt_ok = false;
                            }
                        }
                        4 => {
                            if let Err(e) = client.write_characteristic_without_response(cmd_char, &pulse_with_id).await {
                                log::warn!("Mode4 immediate failed: {:?}", e);
                                evt_ok = false;
                            }
                        }
                        5 => {
                            if let Err(e) = client.write_characteristic(rumble_char, &pulse).await {
                                log::warn!("Mode5 triton immediate write-with-resp failed: {:?}", e);
                                if let Err(e2) = client.write_characteristic_without_response(rumble_char, &pulse).await {
                                    log::warn!("Mode5 triton immediate write-no-resp fallback failed: {:?}", e2);
                                    evt_ok = false;
                                }
                            }
                            let feature_pulse = build_feature_rumble_0xeb(rumble_left, rumble_right);
                            if let Err(e) = client.write_characteristic(feature_char, &feature_pulse).await {
                                log::warn!("Mode5 0xEB immediate write-with-resp failed: {:?}", e);
                                if let Err(e2) = client.write_characteristic_without_response(feature_char, &feature_pulse).await {
                                    log::warn!("Mode5 0xEB immediate write-no-resp fallback failed: {:?}", e2);
                                    evt_ok = false;
                                }
                            }
                            let wrapped_pulse = build_feature_wrapped_rumble_0x80(rumble_left, rumble_right);
                            if let Err(e) = client.write_characteristic(feature_char, &wrapped_pulse).await {
                                log::warn!("Mode5 wrapped immediate write-with-resp failed: {:?}", e);
                                if let Err(e2) = client.write_characteristic_without_response(feature_char, &wrapped_pulse).await {
                                    log::warn!("Mode5 wrapped immediate write-no-resp fallback failed: {:?}", e2);
                                    evt_ok = false;
                                }
                            }
                            if let Err(e) = client.write_characteristic(cmd_char, &pulse_with_id).await {
                                log::warn!("Mode5 raw immediate write-with-resp failed: {:?}", e);
                                if let Err(e2) = client.write_characteristic_without_response(cmd_char, &pulse_with_id).await {
                                    log::warn!("Mode5 raw immediate write-no-resp fallback failed: {:?}", e2);
                                    evt_ok = false;
                                }
                            }
                            let pulse_8f = build_feature_haptic_pulse_0x8f();
                            if let Err(e) = client.write_characteristic(feature_char, &pulse_8f).await {
                                log::warn!("Mode5 0x8F immediate write-with-resp failed: {:?}", e);
                                if let Err(e2) = client.write_characteristic_without_response(feature_char, &pulse_8f).await {
                                    log::warn!("Mode5 0x8F immediate write-no-resp fallback failed: {:?}", e2);
                                    evt_ok = false;
                                }
                            }
                            let pulse_81 = build_triton_haptic_pulse_0x81();
                            if let Err(e) = client.write_characteristic(rumble_char, &pulse_81).await {
                                log::warn!("Mode5 0x81 immediate write-with-resp failed: {:?}", e);
                                if let Err(e2) = client.write_characteristic_without_response(rumble_char, &pulse_81).await {
                                    log::warn!("Mode5 0x81 immediate write-no-resp fallback failed: {:?}", e2);
                                    evt_ok = false;
                                }
                            }
                            let cmd_82 = build_triton_haptic_command_click_0x82();
                            if let Err(e) = client.write_characteristic(rumble_char, &cmd_82).await {
                                log::warn!("Mode5 0x82 immediate write-with-resp failed: {:?}", e);
                                if let Err(e2) = client.write_characteristic_without_response(rumble_char, &cmd_82).await {
                                    log::warn!("Mode5 0x82 immediate write-no-resp fallback failed: {:?}", e2);
                                    evt_ok = false;
                                }
                            }
                        }
                        6 => {
                            let pulse_8f = build_feature_haptic_pulse_0x8f();
                            if let Err(e) = client.write_characteristic_without_response(feature_char, &pulse_8f).await {
                                log::warn!("Mode6 0x8F short immediate failed: {:?}", e);
                                evt_ok = false;
                            }
                            let pulse_8f_64 = build_feature_haptic_pulse_0x8f_64();
                            if let Err(e) = client.write_characteristic_without_response(feature_char, &pulse_8f_64).await {
                                log::warn!("Mode6 0x8F/64B immediate failed: {:?}", e);
                                evt_ok = false;
                            }
                        }
                        7 => {
                            let pulse_81 = build_triton_haptic_pulse_0x81();
                            if let Err(e) = client.write_characteristic_without_response(rumble_char, &pulse_81).await {
                                log::warn!("Mode7 0x81 immediate failed: {:?}", e);
                                evt_ok = false;
                            }
                        }
                        8 => {
                            let cmd_82 = build_triton_haptic_command_click_0x82();
                            if let Err(e) = client.write_characteristic_without_response(rumble_char, &cmd_82).await {
                                log::warn!("Mode8 0x82 immediate failed: {:?}", e);
                                evt_ok = false;
                            }
                        }
                        _ => {
                            if let Err(e) = client.write_characteristic_without_response(rumble_char, &pulse).await {
                                log::warn!("Default mode immediate failed: {:?}", e);
                                evt_ok = false;
                            }
                        }
                    }

                    if evt_ok {
                        rumble_send_ok = rumble_send_ok.saturating_add(1);
                    } else {
                        rumble_send_fail = rumble_send_fail.saturating_add(1);
                    }
                }
            }
        }
    };

    let connection_event_loop = async {
        // Explicitly request low-latency connection parameters from the host side
        log::info!("Requesting low-latency connection parameters from host side...");
        let fast_params = RequestedConnParams {
            min_connection_interval: embassy_time::Duration::from_micros(7500),
            max_connection_interval: embassy_time::Duration::from_micros(7500),
            ..Default::default()
        };
        match connection.update_connection_params(stack, &fast_params).await {
            Ok(_) => {
                log::info!(
                    "Initial update_connection_params request sent (target={}us)",
                    TARGET_CONN_INTERVAL_US
                );
                log::info!(
                    "Waiting for ConnectionParamsUpdated event to confirm acceptance/rejection"
                );
            }
            Err(e) => {
                log::warn!("Initial update_connection_params request failed: {:?}", e);
            }
        }

        loop {
            match connection.next().await {
                ConnectionEvent::Disconnected { reason } => {
                    log::warn!("Connection event: Disconnected ({:?})", reason);
                    break;
                }
                ConnectionEvent::RequestConnectionParams(req) => {
                    log::info!("Connection event: RequestConnectionParams: min={:?}, max={:?}, latency={}",
                        req.params().min_connection_interval,
                        req.params().max_connection_interval,
                        req.params().max_latency
                    );
                    if let Err(e) = req.accept(None, stack).await {
                        log::error!("Failed to accept connection parameters request: {:?}", e);
                    } else {
                        log::info!("Accepted connection parameters request");
                    }
                }
                ConnectionEvent::ConnectionParamsUpdated { conn_interval, peripheral_latency, supervision_timeout } => {
                    log::info!(
                        "Connection event: ConnectionParamsUpdated: interval={:?}, latency={}, timeout={:?}",
                        conn_interval,
                        peripheral_latency,
                        supervision_timeout
                    );
                    log_conn_interval_verdict("Steam", conn_interval);
                }
                other => {
                    log::debug!("Connection event: {:?}", other);
                }
            }
        }
    };

    // Run GATT client task, client operations, and connection event loop.
    // If either client_operations or connection_event_loop exits, cancel the rest.
    embassy_futures::select::select(
        client.task(),
        embassy_futures::select::select(client_operations, connection_event_loop)
    ).await;
    } else {
        handle_hid_gatt(stack, &connection).await;
    }
}

// ---------------------------------------------------------------------------
// Standard HID GATT handler (DS4, DualSense, 8BitDo, Switch Pro)
// ---------------------------------------------------------------------------

async fn handle_hid_gatt(
    stack: &'static Stack<'static, BleController, DefaultPacketPool>,
    connection: &Connection<'static, DefaultPacketPool>,
) {
    const HID_SERVICE_UUID: Uuid = Uuid::Uuid16([0x12, 0x18]);

    let client = match GattClient::<_, _, 64>::new(stack, connection).await {
        Ok(c) => c,
        Err(e) => { log::error!("HID GattClient::new failed: {:?}", e); return; }
    };

    let client_operations = async {
        log::info!("Discovering HID service (0x1812)\u{2026}");
        let services = match client.services_by_uuid(&HID_SERVICE_UUID).await {
            Ok(s) => s,
            Err(e) => { log::error!("HID service discovery failed: {:?}", e); return; }
        };
        let service = match services.first() {
            Some(s) => s.clone(),
            None => { log::error!("HID service (0x1812) not found"); return; }
        };
        log::info!("Enumerating HID report characteristics\u{2026}");
        let characteristics = match client.characteristics::<32>(&service).await {
            Ok(c) => c,
            Err(e) => { log::error!("HID characteristics discovery failed: {:?}", e); return; }
        };
        let mut report_char = None;
        for characteristic in characteristics {
            if characteristic.props.any(&[CharacteristicProp::Notify, CharacteristicProp::Indicate]) {
                report_char = Some(characteristic);
                break;
            }
        }
        let report_char = match report_char {
            Some(ch) => ch,
            None => {
                log::error!("No notifiable HID Report characteristic (0x2A4D) found!");
                return;
            }
        };
        log::info!("Subscribing to HID Report char (handle 0x{:04x})\u{2026}", report_char.handle);
        let mut listener = match client.subscribe(&report_char, false).await {
            Ok(l) => l,
            Err(e) => { log::error!("HID subscribe failed: {:?}", e); return; }
        };
        log::info!("HID subscribed OK!");
        loop {
            let notif = listener.next().await;
            let data = notif.as_ref();
            log::debug!("HID NOTIF len={}: {:02x?}", data.len(), &data[..data.len().min(20)]);
            let mut buf = [0u8; 64];
            buf[0..2].copy_from_slice(&report_char.handle.to_le_bytes());
            buf[2] = data.len() as u8;
            let len = data.len().min(61);
            buf[3..3 + len].copy_from_slice(&data[..len]);
            BLE_REPORTS.signal(buf);
        }
    };

    let connection_event_loop = async {
        let fast_params = RequestedConnParams {
            min_connection_interval: embassy_time::Duration::from_micros(TARGET_CONN_INTERVAL_US),
            max_connection_interval: embassy_time::Duration::from_micros(TARGET_CONN_INTERVAL_US),
            ..Default::default()
        };
        match connection.update_connection_params(stack, &fast_params).await {
            Ok(_) => {
                log::info!(
                    "HID: update_connection_params request sent (target={}us)",
                    TARGET_CONN_INTERVAL_US
                );
            }
            Err(e) => {
                log::warn!("HID: update_connection_params request failed: {:?}", e);
            }
        }

        loop {
            match connection.next().await {
                ConnectionEvent::Disconnected { reason } => {
                    log::warn!("HID: Disconnected ({:?})", reason);
                    break;
                }
                ConnectionEvent::RequestConnectionParams(req) => {
                    if let Err(e) = req.accept(None, stack).await {
                        log::error!("HID: accept conn params failed: {:?}", e);
                    }
                }
                ConnectionEvent::ConnectionParamsUpdated { conn_interval, peripheral_latency, supervision_timeout } => {
                    log::info!(
                        "HID: ConnectionParamsUpdated: interval={:?}, latency={}, timeout={:?}",
                        conn_interval,
                        peripheral_latency,
                        supervision_timeout
                    );
                    log_conn_interval_verdict("HID", conn_interval);
                }
                other => { log::debug!("HID conn event: {:?}", other); }
            }
        }
    };

    embassy_futures::select::select(
        client.task(),
        embassy_futures::select::select(client_operations, connection_event_loop)
    ).await;
}


