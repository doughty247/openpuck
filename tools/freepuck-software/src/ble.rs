//! BLE central-role connection to a Steam Controller 2 (Triton), ported from
//! the OpenPuck/FreePuck firmware's `src/bluetooth.rs` (trouble-host GATT
//! server implementation) onto `btleplug` for desktop hosts.
//!
//! Everything except [`find_controller`] (which needs a real `btleplug`
//! [`Central`]/[`Manager`] talking to an actual Bluetooth adapter) is generic
//! over [`ControllerPeripheral`] — a narrow trait covering only the handful
//! of operations this module actually needs, blanket-implemented for any
//! real `btleplug::api::Peripheral`. It exists (rather than using
//! `btleplug::api::Peripheral` directly as the bound) because that trait's
//! `id()` method returns a `PeripheralId` whose inner field is private on
//! every platform backend, making it impossible to construct outside the
//! `btleplug` crate — so a mock can't implement the real trait at all. This
//! bridge trait lets `mock_ble::MockController` stand in for a real
//! peripheral in tests without touching real Bluetooth hardware.

use anyhow::{anyhow, Context, Result};
use btleplug::api::{
    BDAddr, Central, CharPropFlags, Characteristic, Manager as _, PeripheralProperties, ScanFilter,
    ValueNotification, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral as PlatformPeripheral};
use futures::stream::Stream;
use std::collections::BTreeSet;
use std::future::Future;
use std::pin::Pin;
use std::time::Duration;
use uuid::Uuid;

/// The subset of `btleplug::api::Peripheral` this module needs, narrow
/// enough that a test mock can implement it directly. See the module doc
/// comment for why this exists instead of using the real trait as the bound.
pub trait ControllerPeripheral: Clone + Send + Sync + 'static {
    fn address(&self) -> BDAddr;
    fn properties(&self) -> impl Future<Output = Result<Option<PeripheralProperties>>> + Send;
    fn connect(&self) -> impl Future<Output = Result<()>> + Send;
    fn discover_services(&self) -> impl Future<Output = Result<()>> + Send;
    fn characteristics(&self) -> BTreeSet<Characteristic>;
    fn subscribe(&self, characteristic: &Characteristic) -> impl Future<Output = Result<()>> + Send;
    fn write(
        &self,
        characteristic: &Characteristic,
        data: &[u8],
        write_type: WriteType,
    ) -> impl Future<Output = Result<()>> + Send;
    fn notifications(
        &self,
    ) -> impl Future<Output = Result<Pin<Box<dyn Stream<Item = ValueNotification> + Send>>>> + Send;
}

impl<T: btleplug::api::Peripheral + 'static> ControllerPeripheral for T {
    fn address(&self) -> BDAddr {
        btleplug::api::Peripheral::address(self)
    }
    async fn properties(&self) -> Result<Option<PeripheralProperties>> {
        Ok(btleplug::api::Peripheral::properties(self).await?)
    }
    async fn connect(&self) -> Result<()> {
        Ok(btleplug::api::Peripheral::connect(self).await?)
    }
    async fn discover_services(&self) -> Result<()> {
        Ok(btleplug::api::Peripheral::discover_services(self).await?)
    }
    fn characteristics(&self) -> BTreeSet<Characteristic> {
        btleplug::api::Peripheral::characteristics(self)
    }
    async fn subscribe(&self, characteristic: &Characteristic) -> Result<()> {
        Ok(btleplug::api::Peripheral::subscribe(self, characteristic).await?)
    }
    async fn write(&self, characteristic: &Characteristic, data: &[u8], write_type: WriteType) -> Result<()> {
        Ok(btleplug::api::Peripheral::write(self, characteristic, data, write_type).await?)
    }
    async fn notifications(&self) -> Result<Pin<Box<dyn Stream<Item = ValueNotification> + Send>>> {
        Ok(btleplug::api::Peripheral::notifications(self).await?)
    }
}

/// Vendor-specific prefixes that all Steam Controllers advertise.
/// Observed advertisement name: "Steam Ctrl (BT) FXA9961102DD6"
const STEAM_NAME_PREFIX: &str = "Steam Ctrl";
const STEAM_PUCK_NAME_PREFIX: &str = "Steam Controller Puck";

// Valve custom GATT service and characteristics (Steam Controller 2 / Triton).
// UUID bytes below are the firmware's little-endian wire order, reversed to
// the standard big-endian UUID string form used by the `uuid` crate.
pub const VALVE_SERVICE_UUID: Uuid = uuid::uuid!("100f6c32-1735-4313-b402-38567131e5f3");
/// Steam Controller 2 (Triton) input characteristic. Gen 1 (D0G, 2015)
/// advertises a different suffix and is not supported by this bridge.
pub const TRITON_INPUT_UUID: Uuid = uuid::uuid!("100f6c7a-1735-4313-b402-38567131e5f3");
pub const D0G_INPUT_UUID: Uuid = uuid::uuid!("100f6c33-1735-4313-b402-38567131e5f3");
/// Valve "report" characteristic — fallback command channel when the
/// standard HID feature characteristic can't be identified.
pub const VALVE_REPORT_UUID: Uuid = uuid::uuid!("100f6c34-1735-4313-b402-38567131e5f3");

// Standard Bluetooth SIG HID-over-GATT UUIDs.
pub const HID_SERVICE_UUID: Uuid = uuid::uuid!("00001812-0000-1000-8000-00805f9b34fb");
pub const HID_CONTROL_POINT_UUID: Uuid = uuid::uuid!("00002a4c-0000-1000-8000-00805f9b34fb");
pub const HID_PROTOCOL_MODE_UUID: Uuid = uuid::uuid!("00002a4e-0000-1000-8000-00805f9b34fb");

// Steam Controller 2 (Triton) command bytes, ported from bluetooth.rs.
pub const TRITON_CMD_RUMBLE: u8 = 0x80; // HID output report reference ID for rumble
pub const TRITON_CMD_SET_SETTINGS: u8 = 0x87;
pub const TRITON_SETTING_LIZARD_MODE: u8 = 0x09;
pub const TRITON_SETTING_HAPTICS_ENABLED: u8 = 70;
pub const TRITON_SETTING_HAPTIC_MASTER_GAIN_DB: u8 = 76;
pub const TRITON_SETTING_HAPTIC_INTENSITY: u8 = 79;

/// Every 3 seconds, or the built-in lizard mode (trackpad-to-keyboard mapping) re-enables.
pub const LIZARD_KEEPALIVE_INTERVAL: Duration = Duration::from_millis(3000);
/// Steam Controller 2 haptics hardware safety timeout is ~50ms; resend sustained rumble
/// faster than that to keep it going (matches SDL's TRITON_RUMBLE_RESEND_INTERVAL_MS).
pub const HAPTICS_RESEND_INTERVAL: Duration = Duration::from_millis(40);

pub(crate) fn matches_steam_controller(name: &str) -> bool {
    name.starts_with(STEAM_NAME_PREFIX) || name.starts_with(STEAM_PUCK_NAME_PREFIX)
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

fn scale_rumble(val: u16) -> u16 {
    if val == 0 {
        0
    } else {
        // Boost low values so they are felt on the LRA, mapping 1..65535 to 12000..65535.
        let min_val = 12000u32;
        let max_val = 65535u32;
        let scaled = min_val + ((val as u32) * (max_val - min_val) / 65535);
        scaled as u16
    }
}

/// Full Triton haptic rumble output report, including report ID 0x80.
/// Layout matches SDL's MsgHapticRumble payload:
///   type:u8, intensity:u16, left.speed:u16, left.gain:i8, right.speed:u16, right.gain:i8
pub fn build_triton_rumble_with_id(left_speed: u16, right_speed: u16) -> [u8; 10] {
    let scaled_left = scale_rumble(left_speed);
    let scaled_right = scale_rumble(right_speed);
    [
        TRITON_CMD_RUMBLE,
        0x00, // type
        0x00, // intensity (low) - 0 triggers internal hardware LRA emulator
        0x00, // intensity (high)
        (scaled_left & 0xFF) as u8,
        (scaled_left >> 8) as u8,
        0x06, // left gain dB (maximum boost)
        (scaled_right & 0xFF) as u8,
        (scaled_right >> 8) as u8,
        0x06, // right gain dB (maximum boost)
    ]
}

/// Discovered Steam Controller GATT handles needed to read input and send commands.
/// Generic over the BLE peripheral type so tests can substitute a mock.
#[derive(Clone)]
pub struct SteamGatt<P: ControllerPeripheral> {
    pub peripheral: P,
    /// Notifies with parsed input reports (Triton or D0G input characteristic).
    pub input_char: Characteristic,
    /// HID feature-report characteristic (settings, lizard-off, haptics-enable).
    /// Falls back to the Valve report characteristic when the HID service
    /// doesn't expose a clearly writable+readable candidate.
    pub feature_char: Characteristic,
    /// HID output-report characteristic used for rumble. Falls back to the
    /// feature characteristic when no dedicated output characteristic is found.
    pub output_char: Characteristic,
}

/// [`SteamGatt`] specialized to the real `btleplug` platform backend — the
/// type production code (`main.rs`) actually uses.
pub type PlatformGatt = SteamGatt<PlatformPeripheral>;

async fn get_manager_adapter() -> Result<Adapter> {
    let manager = Manager::new().await.context("failed to init BLE manager")?;
    let adapters = manager.adapters().await.context("failed to list BLE adapters")?;
    adapters
        .into_iter()
        .next()
        .ok_or_else(|| anyhow!("no Bluetooth adapters found"))
}

/// Scans for a Steam Controller by advertised name prefix and returns it once found.
pub async fn find_controller(scan_timeout: Duration) -> Result<PlatformPeripheral> {
    let adapter = get_manager_adapter().await?;
    adapter
        .start_scan(ScanFilter::default())
        .await
        .context("failed to start BLE scan")?;

    let deadline = tokio::time::Instant::now() + scan_timeout;
    loop {
        for p in adapter.peripherals().await.context("failed to list peripherals")? {
            if let Some(props) = p.properties().await.context("failed to read peripheral properties")? {
                if let Some(name) = props.local_name {
                    if matches_steam_controller(&name) {
                        log::info!("found controller: {name} ({})", p.address());
                        let _ = adapter.stop_scan().await;
                        return Ok(p);
                    }
                }
            }
        }
        if tokio::time::Instant::now() >= deadline {
            let _ = adapter.stop_scan().await;
            return Err(anyhow!(
                "no Steam Controller found within {:?} (name prefix \"{}\" or \"{}\")",
                scan_timeout,
                STEAM_NAME_PREFIX,
                STEAM_PUCK_NAME_PREFIX
            ));
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
    }
}

/// Writes to a characteristic, preferring write-without-response and falling
/// back to write-with-response — mirrors the firmware's dual-attempt writes.
pub async fn write_best_effort<P: ControllerPeripheral>(peripheral: &P, ch: &Characteristic, data: &[u8]) -> Result<()> {
    if ch.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE) {
        if peripheral.write(ch, data, WriteType::WithoutResponse).await.is_ok() {
            return Ok(());
        }
    }
    peripheral
        .write(ch, data, WriteType::WithResponse)
        .await
        .with_context(|| format!("write to characteristic {} failed", ch.uuid))
}

/// Scores HID characteristics to guess which is the output (rumble) vs.
/// feature (settings) report, since desktop BLE stacks don't expose the raw
/// ATT handles the firmware uses as a fast-path hint.
pub(crate) fn score_hid_characteristics(
    chars: impl Iterator<Item = Characteristic>,
    control_point: Option<&Characteristic>,
    protocol_mode: Option<&Characteristic>,
) -> (Option<Characteristic>, Option<Characteristic>) {
    let mut best_output: Option<(i32, Characteristic)> = None;
    let mut best_feature: Option<(i32, Characteristic)> = None;

    for c in chars {
        let props = c.properties;
        let write_no_resp = props.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE);
        let write_with_resp = props.contains(CharPropFlags::WRITE);
        let writable = write_no_resp || write_with_resp;
        let readable = props.contains(CharPropFlags::READ);
        let is_control_point = control_point.map(|cp| cp.uuid == c.uuid).unwrap_or(false);
        let is_protocol_mode = protocol_mode.map(|pm| pm.uuid == c.uuid).unwrap_or(false);

        if !writable || is_control_point || is_protocol_mode {
            continue;
        }

        let mut score = 0i32;
        if readable { score += 3; }
        if write_with_resp { score += 2; }
        if write_no_resp { score += 2; }
        if props.contains(CharPropFlags::NOTIFY) { score -= 2; }

        // Output report usually prefers write-without-response and is often not readable.
        if write_no_resp && !readable {
            if best_output.as_ref().map(|(s, _)| score > *s).unwrap_or(true) {
                best_output = Some((score, c.clone()));
            }
        // Feature report is commonly both writable and readable.
        } else if readable && write_with_resp {
            if best_feature.as_ref().map(|(s, _)| score > *s).unwrap_or(true) {
                best_feature = Some((score, c.clone()));
            }
        }
    }

    (best_output.map(|(_, c)| c), best_feature.map(|(_, c)| c))
}

/// Connects to a discovered Steam Controller peripheral and resolves the GATT
/// characteristics needed to read input and send commands.
pub async fn connect_and_discover<P: ControllerPeripheral>(peripheral: P) -> Result<SteamGatt<P>> {
    peripheral.connect().await.context("BLE connect failed")?;
    peripheral.discover_services().await.context("GATT service discovery failed")?;

    let all_chars = peripheral.characteristics();
    let valve_chars = all_chars.iter().filter(|c| c.service_uuid == VALVE_SERVICE_UUID);
    let (triton_input, d0g_input, valve_report) = {
        let mut triton_input = None;
        let mut d0g_input = None;
        let mut valve_report = None;
        for c in valve_chars {
            if c.uuid == TRITON_INPUT_UUID { triton_input = Some(c.clone()); }
            if c.uuid == D0G_INPUT_UUID { d0g_input = Some(c.clone()); }
            if c.uuid == VALVE_REPORT_UUID { valve_report = Some(c.clone()); }
        }
        (triton_input, d0g_input, valve_report)
    };

    if triton_input.is_none() {
        return Err(anyhow!(
            "Gen 1 Steam Controller (D0G) detected or Valve service not found — \
             only Steam Controller 2 (Triton) is supported by this bridge"
        ));
    }
    let input_char = triton_input.or(d0g_input).ok_or_else(|| anyhow!("Valve input characteristic not found"))?;

    let hid_chars: Vec<Characteristic> = all_chars
        .iter()
        .filter(|c| c.service_uuid == HID_SERVICE_UUID)
        .cloned()
        .collect();
    let control_point = hid_chars.iter().find(|c| c.uuid == HID_CONTROL_POINT_UUID).cloned();
    let protocol_mode = hid_chars.iter().find(|c| c.uuid == HID_PROTOCOL_MODE_UUID).cloned();

    let (hid_output, hid_feature) =
        score_hid_characteristics(hid_chars.into_iter(), control_point.as_ref(), protocol_mode.as_ref());

    // Command channel: prefer the Valve report characteristic, fall back to the input
    // characteristic (matches firmware's `cmd_char = report_char.unwrap_or(input_char)`).
    let valve_cmd_char = valve_report.unwrap_or_else(|| input_char.clone());

    let feature_char = hid_feature.unwrap_or_else(|| {
        log::warn!("no suitable HID feature characteristic found; falling back to Valve report characteristic");
        valve_cmd_char.clone()
    });
    let output_char = hid_output.unwrap_or_else(|| {
        log::warn!("no suitable HID output characteristic found; falling back to feature characteristic");
        feature_char.clone()
    });

    log::info!(
        "GATT resolved: input={} feature={} output={}",
        input_char.uuid, feature_char.uuid, output_char.uuid
    );

    peripheral.subscribe(&input_char).await.context("subscribe to input characteristic failed")?;

    Ok(SteamGatt { peripheral, input_char, feature_char, output_char })
}

/// Sends the one-time initialization sequence: clear digital mappings, base
/// settings, and explicit haptics-enable. Ported from the firmware's
/// per-connection init writes.
pub async fn send_init_commands<P: ControllerPeripheral>(gatt: &SteamGatt<P>) -> Result<()> {
    let clear_cmd = [0x81u8];
    write_best_effort(&gatt.peripheral, &gatt.feature_char, &clear_cmd)
        .await
        .context("CMD_CLEAR_DIGITAL_MAPPINGS write failed")?;

    let settings_cmd = [0x87u8, 0x06, 0x07, 0x07, 0x00, 0x08, 0x07, 0x00];
    write_best_effort(&gatt.peripheral, &gatt.feature_char, &settings_cmd)
        .await
        .context("CMD_SET_SETTINGS write failed")?;

    let haptics_enable = build_triton_haptics_enable();
    write_best_effort(&gatt.peripheral, &gatt.feature_char, &haptics_enable)
        .await
        .context("haptics-enable write failed")?;

    Ok(())
}

/// Sends one lizard-off + haptics-enable keepalive round. Split out from
/// [`run_lizard_keepalive`]'s infinite loop so it's directly testable.
pub(crate) async fn send_lizard_tick<P: ControllerPeripheral>(gatt: &SteamGatt<P>) -> Result<()> {
    let lizard_off = build_triton_lizard_off();
    write_best_effort(&gatt.peripheral, &gatt.feature_char, &lizard_off)
        .await
        .context("lizard-mode keepalive write failed")?;

    let haptics_enable = build_triton_haptics_enable();
    write_best_effort(&gatt.peripheral, &gatt.feature_char, &haptics_enable)
        .await
        .context("haptics-enable keepalive write failed")?;

    Ok(())
}

/// Runs the lizard-mode keepalive loop forever: every 3s, re-send lizard-off
/// plus haptics-enable so the controller stays out of trackpad-to-keyboard mode.
pub async fn run_lizard_keepalive<P: ControllerPeripheral>(gatt: SteamGatt<P>) -> ! {
    let mut interval = tokio::time::interval(LIZARD_KEEPALIVE_INTERVAL);
    interval.tick().await; // first tick fires immediately; commands already sent by send_init_commands
    loop {
        interval.tick().await;
        if let Err(e) = send_lizard_tick(&gatt).await {
            log::warn!("lizard-mode keepalive round failed: {e:#}");
        }
    }
}
