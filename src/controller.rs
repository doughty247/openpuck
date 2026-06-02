// Input translation layer.
//
// Architecture:
//   BLE input device → parse_*() → GamepadState → state_to_*() → USB output bytes
//
// Input parsers  : parse_steam
// Output formats : state_to_xinput, state_to_switch, state_to_dualsense

use core::sync::atomic::{AtomicU8, Ordering};

// ------------------------------------------------------------------
// Common intermediate representation
// ------------------------------------------------------------------

/// Normalised gamepad state. Uses Xbox/XInput naming conventions.
/// Sticks: 0-255, 128 = centre, 0 = left/up, 255 = right/down.
/// Triggers: 0-255, 0 = released, 255 = fully pressed.
#[derive(Clone, Copy)]
pub struct GamepadState {
    pub a: bool, pub b: bool, pub x: bool, pub y: bool,
    pub l1: bool, pub r1: bool,
    pub lt: u8, pub rt: u8,
    pub l2_dig: bool, pub r2_dig: bool,
    pub l3: bool, pub r3: bool,
    pub start: bool, pub select: bool, pub home: bool, pub touchpad: bool, pub qam: bool,
    pub dpad_up: bool, pub dpad_down: bool, pub dpad_left: bool, pub dpad_right: bool,
    pub lx: u8, pub ly: u8, pub rx: u8, pub ry: u8,
    // Dedicated touch coordinates (reserved for future Steam -> DualSense touchpad tracking).
    #[allow(dead_code)] pub pad_x: u16,
    #[allow(dead_code)] pub pad_y: u16,
    pub pad_active: bool,
    // IMU motion data from Steam Controller BLE (raw i16, signed, 0 = no data).
    pub gyro_x: i16, pub gyro_y: i16, pub gyro_z: i16,
    pub accel_x: i16, pub accel_y: i16, pub accel_z: i16,
    #[cfg(feature = "rf")]
    #[allow(dead_code)]
    pub puck_provisional: Option<crate::puck::PuckProvisional>,
}

/// Normalized haptics intent derived from USB host output reports.
///
/// `left_motor` / `right_motor` are continuous rumble levels (0..65535).
/// `left_trigger_fx` / `right_trigger_fx` are non-zero when host requests
/// adaptive trigger behavior (best-effort translated to Steam pad pulses).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct HapticsIntent {
    pub left_motor: u16,
    pub right_motor: u16,
    pub left_trigger_fx: u8,
    pub right_trigger_fx: u8,
}

/// Steam-specific remap for Switch output mode.
/// Route the Steam QAM signal (`touchpad`) to Switch Capture.
pub fn remap_steam_for_switch(s: &GamepadState) -> GamepadState {
    let mut r = *s;
    // Steam BLE face-button bits arrive rotated versus our logical ABXY.
    // Rotate back before state_to_switch applies Nintendo ABXY ordering.
    r.a = s.b;
    r.b = s.y;
    r.y = s.x;
    r.x = s.a;

    // Steam BLE bit-positions for system buttons are swapped vs. our field names,
    // same as face buttons. Observed mapping:
    //   qam bit  → physical left stick click  (L3)
    //   home bit → physical right stick click (R3)
    //   r3 bit   → physical Steam button      (Home)
    //   l3 bit   → physical QAM button        (Capture)
    r.l3 = s.qam;
    r.r3 = s.home;
    r.home = s.r3;
    r.touchpad = s.l3;
    r
}

// ------------------------------------------------------------------
// Private helpers
// ------------------------------------------------------------------

fn read_u16_le(p: &[u8], off: usize) -> u16 {
    let lo = p.get(off    ).copied().unwrap_or(0) as u16;
    let hi = p.get(off + 1).copied().unwrap_or(0) as u16;
    (hi << 8) | lo
}

fn read_i16_le(p: &[u8], off: usize) -> i16 { read_u16_le(p, off) as i16 }

fn dpad_to_hat(up: bool, down: bool, left: bool, right: bool) -> u8 {
    match (up, down, left, right) {
        (true,  false, false, false) => 0,
        (true,  false, false, true)  => 1,
        (false, false, false, true)  => 2,
        (false, true,  false, true)  => 3,
        (false, true,  false, false) => 4,
        (false, true,  true,  false) => 5,
        (false, false, true,  false) => 6,
        (true,  false, true,  false) => 7,
        _                            => 8,
    }
}

// DualSense touch packet sequence counter (7-bit in touch contact header).
static DS_TOUCH_SEQ: AtomicU8 = AtomicU8::new(0);

// ------------------------------------------------------------------
// Input parsers  (BLE HID report → GamepadState)
// ------------------------------------------------------------------

/// Steam Controller full-state BLE report.
/// payload[0]=seq; btn bytes 1-3; triggers 5-8; sticks 9-16.
/// IMU fields are zero until SETTING_IMU_MODE packet format is confirmed.
pub fn parse_steam(payload: &[u8]) -> Option<GamepadState> {
    if payload.len() < 17 { return None; }
    let btn0 = payload[1]; let btn1 = payload[2]; let btn2 = payload[3];
    let lt_val = read_u16_le(payload, 5);
    let rt_val = read_u16_le(payload, 7);
    let ls_x = read_i16_le(payload, 9);
    let ls_y = read_i16_le(payload, 11);
    let rs_x = read_i16_le(payload, 13);
    let rs_y = read_i16_le(payload, 15);
    // Steam i16: positive = right/up. GamepadState: 128=centre, 0=up.
    let lx = ((ls_x >> 8) + 128).clamp(0, 255) as u8;
    let ly = (128 - (ls_y >> 8)).clamp(0, 255) as u8;
    let rx = ((rs_x >> 8) + 128).clamp(0, 255) as u8;
    let ry = (128 - (rs_y >> 8)).clamp(0, 255) as u8;

    // Map Steam right touchpad full-range i16 coordinates to DualSense touch area.
    let pad_x = ((((rs_x as i32) + 32768) * 1919) / 65535).clamp(0, 1919) as u16;
    let pad_y = (((32767 - rs_y as i32) * 1079) / 65535).clamp(0, 1079) as u16;
    let pad_active = rs_x.abs() > 1200 || rs_y.abs() > 1200;

    Some(GamepadState {
        a:          btn0 & 0x01 != 0,
        b:          btn0 & 0x02 != 0,
        x:          btn0 & 0x04 != 0,
        y:          btn0 & 0x08 != 0,
        l1:         btn2 & 0x08 != 0,
        r1:         btn1 & 0x02 != 0,
        lt:         (lt_val >> 7) as u8,
        rt:         (rt_val >> 7) as u8,
        l2_dig:     lt_val > 8000,
        r2_dig:     rt_val > 8000,
        l3:         btn1 & 0x80 != 0,
        r3:         btn0 & 0x20 != 0,
        start:      btn0 & 0x40 != 0,
        select:     btn1 & 0x40 != 0,
        home:       btn2 & 0x01 != 0,
        touchpad:   btn2 & 0x02 != 0,
        qam:        btn0 & 0x10 != 0, // "..." quick access button
        dpad_up:    btn1 & 0x20 != 0,
        dpad_down:  btn1 & 0x04 != 0,
        dpad_left:  btn1 & 0x10 != 0,
        dpad_right: btn1 & 0x08 != 0,
        lx, ly, rx, ry,
        pad_x, pad_y, pad_active,
        gyro_x: 0, gyro_y: 0, gyro_z: 0,
        accel_x: 0, accel_y: 0, accel_z: 0,
        #[cfg(feature = "rf")]
        puck_provisional: None,
    })
}

// ------------------------------------------------------------------
// Output formatters  (GamepadState → USB HID report bytes)
// ------------------------------------------------------------------

/// XInput (Xbox 360) report — 20 bytes.
pub fn state_to_xinput(s: &GamepadState) -> [u8; 20] {
    let mut b1 = 0u8;
    let mut b2 = 0u8;
    if s.dpad_up    { b1 |= 0x01; }
    if s.dpad_down  { b1 |= 0x02; }
    if s.dpad_left  { b1 |= 0x04; }
    if s.dpad_right { b1 |= 0x08; }
    if s.start      { b1 |= 0x10; }
    if s.select     { b1 |= 0x20; }
    if s.l3         { b1 |= 0x40; }
    if s.r3         { b1 |= 0x80; }
    if s.l1         { b2 |= 0x01; }
    if s.r1         { b2 |= 0x02; }
    if s.home       { b2 |= 0x04; }
    if s.a          { b2 |= 0x10; }
    if s.b          { b2 |= 0x20; }
    if s.x          { b2 |= 0x40; }
    if s.y          { b2 |= 0x80; }
    // GamepadState: 128=centre, 0=up. XInput i16: 0=centre, positive=right/up (Y inverted).
    let lx = (s.lx as i16 - 128).saturating_mul(258);
    let ly = (128i16 - s.ly as i16).saturating_mul(258);
    let rx = (s.rx as i16 - 128).saturating_mul(258);
    let ry = (128i16 - s.ry as i16).saturating_mul(258);
    let [lx0, lx1] = lx.to_le_bytes();
    let [ly0, ly1] = ly.to_le_bytes();
    let [rx0, rx1] = rx.to_le_bytes();
    let [ry0, ry1] = ry.to_le_bytes();
    let mut r = [0u8; 20];
    r[0] = 0x00; r[1] = 0x14;
    r[2] = b1;   r[3] = b2;
    r[4] = s.lt; r[5] = s.rt;
    r[6] = lx0;  r[7] = lx1;
    r[8] = ly0;  r[9] = ly1;
    r[10] = rx0; r[11] = rx1;
    r[12] = ry0; r[13] = ry1;
    r
}

/// Nintendo Switch Pokken-compatible HID report — 8 bytes.
pub fn state_to_switch(s: &GamepadState) -> [u8; 8] {
    let mut b1 = 0u8; let mut b2 = 0u8;
    // Nintendo face layout from logical ABXY:
    // Y=west(X), B=south(A), A=east(B), X=north(Y)
    if s.x      { b1 |= 0x01; } if s.a     { b1 |= 0x02; }
    if s.b      { b1 |= 0x04; } if s.y     { b1 |= 0x08; }
    if s.l1     { b1 |= 0x10; } if s.r1    { b1 |= 0x20; }
    if s.l2_dig { b1 |= 0x40; } if s.r2_dig{ b1 |= 0x80; }
    if s.select { b2 |= 0x01; } if s.start { b2 |= 0x02; }
    if s.l3     { b2 |= 0x04; } if s.r3    { b2 |= 0x08; }
    if s.home   { b2 |= 0x10; } if s.touchpad { b2 |= 0x20; } // Capture
    let hat = dpad_to_hat(s.dpad_up, s.dpad_down, s.dpad_left, s.dpad_right);
    [b1, b2, hat, s.lx, s.ly, s.rx, s.ry, 0x00]
}

/// Sony DualSense (PS5) USB HID report — 64 bytes.
/// Layout matches DUALSENSE_REPORT_DESC in usb.rs:
///   r[0]=ReportID(1), r[1-6]=LX/LY/RX/RY/L2/R2,
///   r[7]=hat[3:0]+pad[7:4], r[8]=buttons1-8, r[9]=buttons9-15+pad
pub fn state_to_dualsense(s: &GamepadState, steam_touchpad_mode: bool) -> [u8; 64] {
    let mut s = *s;

    // For Steam Controller in DualSense mode, parse_steam field names are already correct:
    //   l3 (btn1 & 0x80)  = physical left stick click  → DualSense L3
    //   r3 (btn0 & 0x20)  = physical right stick click → DualSense R3
    //   home (btn2 & 0x01)= physical Steam button      → DualSense PS
    //   qam (btn0 & 0x10) = physical QAM button        → DualSense Touchpad click
    // Only the qam button needs routing; everything else passes through unchanged.
    if steam_touchpad_mode {
        let src = s;
        s.touchpad = src.qam; // QAM → Touchpad click button
    }

    // Real DualSense USB report 0x01 layout (matches parse_dualsense_input):
    // r[1..6]  = LX, LY, RX, RY, L2, R2
    // r[7]     = frame counter
    // r[8]     = hat(3:0) | Square(4) | Cross(5) | Circle(6) | Triangle(7)
    // r[9]     = L1(0)|R1(1)|L2dig(2)|R2dig(3)|Share(4)|Options(5)|L3(6)|R3(7)
    // r[10]    = PS(0) | Touchpad(1) | padding(7:2)
    // r[16..22]= Gyro X/Y/Z (i16 LE, 1024 units/deg/s, hid-playstation scale)
    // r[22..28]= Accel X/Y/Z (i16 LE, 8192 units/g)
    // r[32..37]= touchpad packet
    // r[53]    = battery status byte: bits[3:0]=capacity(0-10), bits[7:4]=state
    let mut r = [0u8; 64];
    r[0] = 0x01; // Report ID
    r[1] = s.lx; r[2] = s.ly; r[3] = s.rx; r[4] = s.ry;
    r[5] = s.lt; r[6] = s.rt;
    r[7] = DS_TOUCH_SEQ.fetch_add(1, Ordering::Relaxed); // frame counter

    // Byte 8: hat in low nibble, face buttons in high nibble
    let hat = dpad_to_hat(s.dpad_up, s.dpad_down, s.dpad_left, s.dpad_right) & 0x0F;
    let mut b8 = hat;
    if s.x { b8 |= 0x10; } // Square
    if s.a { b8 |= 0x20; } // Cross
    if s.b { b8 |= 0x40; } // Circle
    if s.y { b8 |= 0x80; } // Triangle
    r[8] = b8;

    // Byte 9: L1, R1, L2dig, R2dig, Share/Create, Options, L3, R3
    let mut b9 = 0u8;
    if s.l1     { b9 |= 0x01; }
    if s.r1     { b9 |= 0x02; }
    if s.l2_dig { b9 |= 0x04; }
    if s.r2_dig { b9 |= 0x08; }
    if s.select { b9 |= 0x10; } // Share/Create
    if s.start  { b9 |= 0x20; } // Options
    if s.l3     { b9 |= 0x40; }
    if s.r3     { b9 |= 0x80; }
    r[9] = b9;

    // Byte 10: PS button, Touchpad click
    let mut b10 = 0u8;
    if s.home     { b10 |= 0x01; }
    if s.touchpad { b10 |= 0x02; }
    r[10] = b10;

    // Gyro X/Y/Z at r[16..22] (signed i16 LE, 1024 LSB per deg/s).
    // Steam Controller raw gyro is also in 1024 LSB/deg/s units so we pass through directly.
    let gx = s.gyro_x.to_le_bytes();
    let gy = s.gyro_y.to_le_bytes();
    let gz = s.gyro_z.to_le_bytes();
    r[16] = gx[0]; r[17] = gx[1];
    r[18] = gy[0]; r[19] = gy[1];
    r[20] = gz[0]; r[21] = gz[1];

    // Accel X/Y/Z at r[22..28] (signed i16 LE, 8192 LSB per g).
    // Steam Controller raw accel is in ~256 LSB/g; scale ×32 to reach 8192 LSB/g.
    let ax = (s.accel_x.saturating_mul(32)).to_le_bytes();
    let ay = (s.accel_y.saturating_mul(32)).to_le_bytes();
    let az = (s.accel_z.saturating_mul(32)).to_le_bytes();
    r[22] = ax[0]; r[23] = ax[1];
    r[24] = ay[0]; r[25] = ay[1];
    r[26] = az[0]; r[27] = az[1];

    // Touchpad data at bytes 32+
    let seq = DS_TOUCH_SEQ.load(Ordering::Relaxed) & 0x7F;
    r[32] = seq;

    // Touchpad cursor: only activate from non-steam-controller sources for now.
    // Right-stick-as-touchpad removed; Steam Controller touchpad tracking is TODO.
    let touch_active = !steam_touchpad_mode && s.pad_active;
    let tx: u16 = ((s.rx as u32 * 1919) / 255) as u16;
    let ty: u16 = ((s.ry as u32 * 1079) / 255) as u16;

    if touch_active {
        r[33] = seq; // bit7=0 => finger active, low7=id
        r[34] = (tx & 0xFF) as u8;
        r[35] = (((tx >> 8) & 0x0F) as u8) | (((ty & 0x0F) as u8) << 4);
        r[36] = ((ty >> 4) & 0xFF) as u8;
    } else {
        r[33] = 0x80; // inactive finger
    }
    r[37] = 0x80; // Finger 2 always inactive

    // Battery status byte (r[53]):
    //   bits[3:0] = capacity level 0-10 (10 = 100%)
    //   bits[7:4] = charging state (0=discharging, 1=charging, 2=complete)
    // Hosts (Steam, games) suppress haptics when battery == 0 (0x00 = 0% discharging).
    // Report 100% fully-charged (capacity=10, state=complete=2) → 0x2A.
    r[53] = 0x2A;

    r
}

/// Parse Xbox 360/XInput host OUT report into normalized haptics.
/// Common rumble frame: [0x00, 0x08, 0x00, left, right, ...]
pub fn parse_xinput_haptics_out(data: &[u8]) -> Option<HapticsIntent> {
    if data.len() < 4 {
        return None;
    }
    // Common XInput SetState frames seen on Linux hosts:
    //   [0x00, 0x08, left, right, ...]
    //   [0x00, 0x08, 0x00, left, right, ...]
    // Handle both so we do not accidentally treat a reserved 0x00 byte
    // as the left motor level.
    let (left_u8, right_u8) = if data.len() >= 5 && data[0] == 0x00 && data[1] == 0x08 && data[2] == 0x00 {
        (data[3], data[4])
    } else {
        (data[2], data[3])
    };

    let left = left_u8 as u16 * 257;
    let right = right_u8 as u16 * 257;
    Some(HapticsIntent {
        left_motor: left,
        right_motor: right,
        left_trigger_fx: 0,
        right_trigger_fx: 0,
    })
}

/// Parse DualSense USB OUT report (ID 0x02) into normalized haptics intent.
///
/// The host's adaptive-trigger blocks are translated as coarse effect levels,
/// since Steam Controller does not have physical adaptive triggers.
pub fn parse_dualsense_haptics_out(data: &[u8]) -> Option<HapticsIntent> {
    if data.is_empty() {
        return None;
    }

    // Host packet variants seen in practice:
    // - USB output report (ID 0x02): [id, valid0, valid1, right, left, ...]
    // - BT-style output report (ID 0x31): [id, seq/tag, tag, valid0, valid1, right, left, ...]
    // Only report-ID-tagged packets are trusted to avoid startup phantom rumble.
    let (right_u8, left_u8, motors_enabled) =
        if data[0] == 0x02 && data.len() >= 5 {
            let valid0 = data[1];
            let valid1 = data[2];
            let has_valid_fields = valid0 != 0 || valid1 != 0;
            let enabled = (valid0 & 0x03) != 0 || (valid1 & 0x01) != 0;
            let right = data[3];
            let left = data[4];

            // If host omits valid fields entirely, only accept an explicit zero frame as stop.
            // This avoids false positives from non-rumble startup/config packets.
            if !has_valid_fields {
                if right == 0 && left == 0 {
                    return Some(HapticsIntent::default());
                }
                return None;
            }

            (right, left, enabled)
        } else if data[0] == 0x31 && data.len() >= 7 {
            // BT-style 0x31: honor valid bits to avoid false positives.
            let valid0 = data[3];
            let valid1 = data[4];
            let enabled = (valid0 & 0x03) != 0 || (valid1 & 0x01) != 0;
            let right = data[5];
            let left = data[6];
            // Explicit stop frame.
            if !enabled && right == 0 && left == 0 {
                return Some(HapticsIntent::default());
            }
            // If valid bits do not enable motors, ignore frame.
            if !enabled {
                return None;
            }
            let (right, left) = (right, left);
            (right, left, enabled)
        } else {
            return None;
        };

    // Non-stop protection: only treat valid/explicit motor intent frames as haptics updates.
    if !motors_enabled && right_u8 == 0 && left_u8 == 0 {
        return Some(HapticsIntent::default());
    }
    if !motors_enabled {
        return None;
    }

    // DualSense convention: right = weak, left = strong.
    let right = right_u8 as u16 * 257;
    let left = left_u8 as u16 * 257;

    // Stability mode: disable adaptive-trigger translation to Steam haptics for now.
    // Trigger-only DS packets are frequent and can interfere with motor rumble intent.
    let right_fx = 0;
    let left_fx = 0;

    Some(HapticsIntent {
        left_motor: left,
        right_motor: right,
        left_trigger_fx: left_fx,
        right_trigger_fx: right_fx,
    })
}

/// Parse Switch Pro-style OUT report into normalized haptics intent.
/// Pokken-compatible mode usually does not carry rumble payloads.
pub fn parse_switch_haptics_out(_data: &[u8]) -> Option<HapticsIntent> {
    if _data.is_empty() {
        return None;
    }

    // Switch hosts can emit multiple variants:
    // - 8-byte rumble payload (Pokken-style descriptor)
    // - 9-byte payload with leading report ID
    // - shorter fallback/keepalive payloads
    let data = if _data.len() >= 9 && matches!(_data[0], 0x01 | 0x10 | 0x30) {
        &_data[1..]
    } else {
        _data
    };

    let (left_peak, right_peak) = if data.len() >= 8 {
        (
            data[..4].iter().copied().max().unwrap_or(0) as u16,
            data[4..8].iter().copied().max().unwrap_or(0) as u16,
        )
    } else if data.len() >= 4 {
        let mid = data.len() / 2;
        (
            data[..mid].iter().copied().max().unwrap_or(0) as u16,
            data[mid..].iter().copied().max().unwrap_or(0) as u16,
        )
    } else if data.len() >= 2 {
        (data[0] as u16, data[1] as u16)
    } else {
        (data[0] as u16, data[0] as u16)
    };

    if left_peak == 0 && right_peak == 0 {
        return Some(HapticsIntent::default());
    }
    Some(HapticsIntent {
        left_motor: left_peak * 257,
        right_motor: right_peak * 257,
        left_trigger_fx: 0,
        right_trigger_fx: 0,
    })
}

// Note: Wii Remote uses Classic Bluetooth (BR/EDR), not BLE.
// It cannot be supported by the trouble-host BLE-only stack.
// ---- dead code removal marker -- do not add below this line --
