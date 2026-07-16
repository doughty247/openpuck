//! Input translation layer, ported from the OpenPuck/FreePuck firmware's `src/controller.rs`.
//!
//! Architecture:
//!   BLE input notification → parse_steam() → GamepadState → state_to_*() → virtual HID bytes

use std::sync::atomic::{AtomicU8, Ordering};

// ------------------------------------------------------------------
// Common intermediate representation
// ------------------------------------------------------------------

/// Normalised gamepad state. Uses Xbox/XInput naming conventions.
/// Sticks: 0-255, 128 = centre, 0 = left/up, 255 = right/down.
/// Triggers: 0-255, 0 = released, 255 = fully pressed.
#[derive(Clone, Copy, Debug)]
pub struct GamepadState {
    pub a: bool, pub b: bool, pub x: bool, pub y: bool,
    pub l1: bool, pub r1: bool,
    pub lt: u8, pub rt: u8,
    pub l2_dig: bool, pub r2_dig: bool,
    pub l3: bool, pub r3: bool,
    pub start: bool, pub select: bool, pub home: bool, pub touchpad: bool, pub qam: bool,
    pub dpad_up: bool, pub dpad_down: bool, pub dpad_left: bool, pub dpad_right: bool,
    pub lx: u8, pub ly: u8, pub rx: u8, pub ry: u8,
    pub pad_x: u16,
    pub pad_y: u16,
    pub pad_active: bool,
    // IMU motion data from Steam Controller BLE (raw i16, signed, 0 = no data).
    pub gyro_x: i16, pub gyro_y: i16, pub gyro_z: i16,
    pub accel_x: i16, pub accel_y: i16, pub accel_z: i16,
}

/// Normalized haptics intent derived from virtual-device rumble callbacks.
///
/// `left_motor` / `right_motor` are continuous rumble levels (0..65535).
/// `left_trigger_fx` / `right_trigger_fx` are non-zero when the host requests
/// adaptive trigger behavior (best-effort translated to Steam pad pulses).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct HapticsIntent {
    pub left_motor: u16,
    pub right_motor: u16,
    pub left_trigger_fx: u8,
    pub right_trigger_fx: u8,
}

impl GamepadState {
    /// All buttons released, sticks centred (128), triggers at rest — the
    /// state to present before the first real BLE input report arrives.
    pub fn default_centred() -> Self {
        GamepadState {
            a: false, b: false, x: false, y: false,
            l1: false, r1: false,
            lt: 0, rt: 0,
            l2_dig: false, r2_dig: false,
            l3: false, r3: false,
            start: false, select: false, home: false, touchpad: false, qam: false,
            dpad_up: false, dpad_down: false, dpad_left: false, dpad_right: false,
            lx: 128, ly: 128, rx: 128, ry: 128,
            pad_x: 0, pad_y: 0, pad_active: false,
            gyro_x: 0, gyro_y: 0, gyro_z: 0,
            accel_x: 0, accel_y: 0, accel_z: 0,
        }
    }
}

/// Steam-specific remap for Switch output mode.
/// Route the Steam QAM signal (`touchpad`) to Switch Capture.
pub fn remap_steam_for_switch(s: &GamepadState) -> GamepadState {
    let mut r = *s;
    // parse_steam uses Xbox/XInput naming conventions and is already correct:
    //   state.a = bottom face button (Xbox A -> Nintendo B)
    //   state.b = right  face button (Xbox B -> Nintendo A)
    //   state.x = left   face button (Xbox X -> Nintendo Y)
    //   state.y = top    face button (Xbox Y -> Nintendo X)
    // state_to_switch handles the Xbox->Nintendo layout conversion via its bit
    // assignments, so no face-button rotation is needed here.
    r.touchpad = s.qam;
    r
}

// ------------------------------------------------------------------
// Private helpers
// ------------------------------------------------------------------

fn read_u16_le(p: &[u8], off: usize) -> u16 {
    let lo = p.get(off).copied().unwrap_or(0) as u16;
    let hi = p.get(off + 1).copied().unwrap_or(0) as u16;
    (hi << 8) | lo
}

fn read_i16_le(p: &[u8], off: usize) -> i16 { read_u16_le(p, off) as i16 }

pub(crate) fn dpad_to_hat(up: bool, down: bool, left: bool, right: bool) -> u8 {
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
// Input parsers  (BLE input notification -> GamepadState)
// ------------------------------------------------------------------

/// Steam Controller full-state BLE report (Triton input characteristic).
/// payload[0]=seq; btn bytes 1-4; triggers 5-8; sticks 9-16; right pad,
/// gyro, and accel ride in the tail of a full report (see below) and are
/// only present on the 45+ byte notifications, not the 17-byte minimum.
///
/// Field offsets and the button bitfield were cross-checked against
/// safijari/openpuck's `docs/PROTOCOL.md` and `triton.h`/`rf_link.cpp` —
/// an independent reverse-engineering of the same physical Steam
/// Controller 2, over its 2.4GHz RF link rather than BLE. Its documented
/// report 0x45 is 46 bytes starting with the report ID; this BLE
/// notification payload appears to be the same layout with that leading
/// report-ID byte stripped (every offset below lines up exactly one byte
/// earlier), which is what let the trigger/button/gyro/accel/pad fixes
/// below be derived with reasonable confidence. That alignment is
/// inferred from a differently-transported reference implementation, not
/// confirmed against a real BLE capture from this project — treat the
/// newly-added fields (real trackpad coordinates, gyro, accel, the digital
/// trigger-click bits) as provisional pending a hardware pass.
pub fn parse_steam(payload: &[u8]) -> Option<GamepadState> {
    if payload.len() < 17 { return None; }
    let btn0 = payload[1]; let btn1 = payload[2]; let btn2 = payload[3];
    // 4th button byte: left-pad touch/click, left trigger click, grip touch.
    // Always present — it falls inside the 17-byte minimum this bridge
    // already required, we just weren't reading it.
    let btn3 = payload[4];

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

    // The trigger's raw range tops out near half-scale (~0x8000) even at a
    // full pull, so `raw >> 7` alone can exceed 255 (a hard pull wraps back
    // down to a small value through a plain `as u8` cast instead of reading
    // as fully pressed) — saturate instead of wrapping.
    let lt = (lt_val >> 7).min(255) as u8;
    let rt = (rt_val >> 7).min(255) as u8;

    // Digital trigger-click has its own dedicated bit distinct from the
    // analog value; OR it with the old analog-threshold heuristic so a
    // light-but-not-quite-clicked pull still registers either way.
    let l2_dig = btn3 & 0x08 != 0 || lt_val > 8000;
    let r2_dig = btn2 & 0x80 != 0 || rt_val > 8000;

    // Touchpad click = either trackpad's own click bit. (Previously read
    // btn2 & 0x02, which is actually the L4 back-paddle button — a
    // real bug, not just an approximation.)
    let touchpad = btn2 & 0x40 != 0 || btn3 & 0x04 != 0;

    // Real right-trackpad coordinates, gyro, and accel only ride on a full
    // report; short reports fall back to the old stick-deflection proxy for
    // pad position (better than going dead) and leave IMU zeroed, same as
    // this function's behaviour before these fields existed.
    let (pad_x, pad_y, pad_active, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z) = if payload.len() >= 45 {
        let rp_x = read_i16_le(payload, 23);
        let rp_y = read_i16_le(payload, 25);
        let rp_press = read_i16_le(payload, 27);
        let pad_x = ((((rp_x as i32) + 32768) * 1919) / 65535).clamp(0, 1919) as u16;
        let pad_y = (((32767 - rp_y as i32) * 1079) / 65535).clamp(0, 1079) as u16;
        let pad_active = rp_press != 0;
        let accel_x = read_i16_le(payload, 33);
        let accel_y = read_i16_le(payload, 35);
        let accel_z = read_i16_le(payload, 37);
        let gyro_x = read_i16_le(payload, 39);
        let gyro_y = read_i16_le(payload, 41);
        let gyro_z = read_i16_le(payload, 43);
        (pad_x, pad_y, pad_active, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z)
    } else {
        let pad_x = ((((rs_x as i32) + 32768) * 1919) / 65535).clamp(0, 1919) as u16;
        let pad_y = (((32767 - rs_y as i32) * 1079) / 65535).clamp(0, 1079) as u16;
        let pad_active = rs_x.abs() > 1200 || rs_y.abs() > 1200;
        (pad_x, pad_y, pad_active, 0, 0, 0, 0, 0, 0)
    };

    Some(GamepadState {
        a:          btn0 & 0x01 != 0,
        b:          btn0 & 0x02 != 0,
        x:          btn0 & 0x04 != 0,
        y:          btn0 & 0x08 != 0,
        l1:         btn2 & 0x08 != 0,
        r1:         btn1 & 0x02 != 0,
        lt, rt,
        l2_dig, r2_dig,
        l3:         btn1 & 0x80 != 0,
        r3:         btn0 & 0x20 != 0,
        start:      btn0 & 0x40 != 0,
        select:     btn1 & 0x40 != 0,
        home:       btn2 & 0x01 != 0,
        touchpad,
        qam:        btn0 & 0x10 != 0, // "..." quick access button
        dpad_up:    btn1 & 0x20 != 0,
        dpad_down:  btn1 & 0x04 != 0,
        dpad_left:  btn1 & 0x10 != 0,
        dpad_right: btn1 & 0x08 != 0,
        lx, ly, rx, ry,
        pad_x, pad_y, pad_active,
        gyro_x, gyro_y, gyro_z,
        accel_x, accel_y, accel_z,
    })
}

// ------------------------------------------------------------------
// Output formatters  (GamepadState -> virtual HID report bytes)
//
// These match the firmware's exact USB HID report byte layouts (see
// docs/ble-protocol.md) and are covered by the test suite below. The
// current output backends (ViGEmBus, uinput) consume GamepadState fields
// directly rather than these byte layouts, since neither speaks raw HID
// reports — they're kept here as the validated reference encoding and for
// a possible future raw-hidraw output backend.
// ------------------------------------------------------------------

/// XInput (Xbox 360) report — 20 bytes. Matches XUSB_REPORT layout used by
/// ViGEmBus / vigem-client on Windows.
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

/// Sony DualSense (PS5) USB HID report — 64 bytes. PC-only: PS5 requires
/// cryptographic authentication this bridge does not implement.
pub fn state_to_dualsense(s: &GamepadState, steam_touchpad_mode: bool) -> [u8; 64] {
    let mut s = *s;

    // For Steam Controller in DualSense mode, parse_steam field names are already correct:
    //   l3 (btn1 & 0x80)  = physical left stick click  -> DualSense L3
    //   r3 (btn0 & 0x20)  = physical right stick click -> DualSense R3
    //   home (btn2 & 0x01)= physical Steam button      -> DualSense PS
    //   qam (btn0 & 0x10) = physical QAM button         -> DualSense Touchpad click
    if steam_touchpad_mode {
        let src = s;
        s.touchpad = src.qam; // QAM -> Touchpad click button
    }

    // Real DualSense USB report 0x01 layout:
    // r[1..6]  = LX, LY, RX, RY, L2, R2
    // r[7]     = frame counter
    // r[8]     = hat(3:0) | Square(4) | Cross(5) | Circle(6) | Triangle(7)
    // r[9]     = L1(0)|R1(1)|L2dig(2)|R2dig(3)|Share(4)|Options(5)|L3(6)|R3(7)
    // r[10]    = PS(0) | Touchpad(1) | padding(7:2)
    // r[16..22]= Gyro X/Y/Z (i16 LE, 1024 units/deg/s)
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
    let gx = s.gyro_x.to_le_bytes();
    let gy = s.gyro_y.to_le_bytes();
    let gz = s.gyro_z.to_le_bytes();
    r[16] = gx[0]; r[17] = gx[1];
    r[18] = gy[0]; r[19] = gy[1];
    r[20] = gz[0]; r[21] = gz[1];

    // Accel X/Y/Z at r[22..28] (signed i16 LE). Passed through unscaled: an
    // independent reverse-engineering (safijari/openpuck's mode_ps5.cpp)
    // sends the Steam Controller 2's raw accel/gyro values straight through
    // to a real DualSense-driver HID report with no rescaling at all, which
    // is why the ×32 this used to apply here was removed — neither this
    // project nor that scale factor has been confirmed against real
    // hardware, but "no scaling," matching a project others have actually
    // run, is the safer default until someone measures it.
    let ax = s.accel_x.to_le_bytes();
    let ay = s.accel_y.to_le_bytes();
    let az = s.accel_z.to_le_bytes();
    r[22] = ax[0]; r[23] = ax[1];
    r[24] = ay[0]; r[25] = ay[1];
    r[26] = az[0]; r[27] = az[1];

    // Touchpad data at bytes 32+
    let seq = DS_TOUCH_SEQ.load(Ordering::Relaxed) & 0x7F;
    r[32] = seq;

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

    // Battery status byte: report 100% fully-charged (capacity=10, state=complete=2) -> 0x2A.
    // Hosts (Steam, games) suppress haptics when battery == 0.
    r[53] = 0x2A;

    r
}

/// Parse Xbox 360/XInput host rumble callback into normalized haptics.
/// Common rumble frame: [0x00, 0x08, 0x00, left, right, ...]
pub fn parse_xinput_haptics_out(data: &[u8]) -> Option<HapticsIntent> {
    if data.len() < 4 {
        return None;
    }
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

/// Parse DualSense output report (ID 0x02) into normalized haptics intent.
///
/// The host's adaptive-trigger blocks are translated as coarse effect levels,
/// since Steam Controller does not have physical adaptive triggers.
pub fn parse_dualsense_haptics_out(data: &[u8]) -> Option<HapticsIntent> {
    if data.is_empty() {
        return None;
    }

    let (right_u8, left_u8, motors_enabled) =
        if data[0] == 0x02 && data.len() >= 5 {
            let valid0 = data[1];
            let valid1 = data[2];
            let has_valid_fields = valid0 != 0 || valid1 != 0;
            let enabled = (valid0 & 0x03) != 0 || (valid1 & 0x01) != 0;
            let right = data[3];
            let left = data[4];

            if !has_valid_fields {
                if right == 0 && left == 0 {
                    return Some(HapticsIntent::default());
                }
                return None;
            }

            (right, left, enabled)
        } else if data[0] == 0x31 && data.len() >= 7 {
            let valid0 = data[3];
            let valid1 = data[4];
            let enabled = (valid0 & 0x03) != 0 || (valid1 & 0x01) != 0;
            let right = data[5];
            let left = data[6];
            if !enabled && right == 0 && left == 0 {
                return Some(HapticsIntent::default());
            }
            if !enabled {
                return None;
            }
            (right, left, enabled)
        } else {
            return None;
        };

    if !motors_enabled && right_u8 == 0 && left_u8 == 0 {
        return Some(HapticsIntent::default());
    }
    if !motors_enabled {
        return None;
    }

    // DualSense convention: right = weak, left = strong.
    let right = right_u8 as u16 * 257;
    let left = left_u8 as u16 * 257;

    Some(HapticsIntent {
        left_motor: left,
        right_motor: right,
        left_trigger_fx: 0,
        right_trigger_fx: 0,
    })
}

/// Parse Switch Pro-style rumble payload into normalized haptics intent.
/// Pokken-compatible mode usually does not carry rumble payloads.
pub fn parse_switch_haptics_out(data: &[u8]) -> Option<HapticsIntent> {
    if data.is_empty() {
        return None;
    }

    let data = if data.len() >= 9 && matches!(data[0], 0x01 | 0x10 | 0x30) {
        &data[1..]
    } else {
        data
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

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_report(btn0: u8, btn1: u8, btn2: u8) -> Vec<u8> {
        let mut p = vec![0u8; 17];
        p[0] = 0x45; // report id (rolling counter position for real reports)
        p[1] = btn0;
        p[2] = btn1;
        p[3] = btn2;
        p
    }

    fn synthetic_report_full(btn0: u8, btn1: u8, btn2: u8, btn3: u8) -> Vec<u8> {
        let mut p = synthetic_report(btn0, btn1, btn2);
        p[4] = btn3;
        p
    }

    /// A full 45-byte (post report-ID-strip) report with sticks/pads/IMU set
    /// from the given raw i16 values, for exercising the extended fields
    /// that only appear on a full-length notification.
    #[allow(clippy::too_many_arguments)]
    fn synthetic_full_length_report(
        rp_x: i16, rp_y: i16, rp_press: i16, accel: (i16, i16, i16), gyro: (i16, i16, i16),
    ) -> Vec<u8> {
        let mut p = vec![0u8; 45];
        p[0] = 0x45;
        p[23..25].copy_from_slice(&rp_x.to_le_bytes());
        p[25..27].copy_from_slice(&rp_y.to_le_bytes());
        p[27..29].copy_from_slice(&rp_press.to_le_bytes());
        p[33..35].copy_from_slice(&accel.0.to_le_bytes());
        p[35..37].copy_from_slice(&accel.1.to_le_bytes());
        p[37..39].copy_from_slice(&accel.2.to_le_bytes());
        p[39..41].copy_from_slice(&gyro.0.to_le_bytes());
        p[41..43].copy_from_slice(&gyro.1.to_le_bytes());
        p[43..45].copy_from_slice(&gyro.2.to_le_bytes());
        p
    }

    #[test]
    fn rejects_short_payload() {
        assert!(parse_steam(&[0x45, 0x01]).is_none());
    }

    #[test]
    fn parses_face_buttons() {
        let p = synthetic_report(0x0F, 0x00, 0x00); // A|B|X|Y
        let s = parse_steam(&p).unwrap();
        assert!(s.a && s.b && s.x && s.y);
        assert!(!s.l1 && !s.r1);
    }

    #[test]
    fn parses_bumpers_and_dpad() {
        // r1 = btn1 & 0x02, l1 = btn2 & 0x08
        let p = synthetic_report(0x00, 0x02 | 0x20 | 0x04 | 0x10 | 0x08, 0x08);
        let s = parse_steam(&p).unwrap();
        assert!(s.r1 && s.l1);
        assert!(s.dpad_up && s.dpad_down && s.dpad_left && s.dpad_right);
    }

    #[test]
    fn centre_sticks_map_to_128() {
        let p = synthetic_report(0x00, 0x00, 0x00);
        let s = parse_steam(&p).unwrap();
        assert_eq!((s.lx, s.ly, s.rx, s.ry), (128, 128, 128, 128));
    }

    #[test]
    fn triggers_scale_from_u16() {
        let mut p = synthetic_report(0x00, 0x00, 0x00);
        // LT at bytes 5-6, RT at bytes 7-8, little-endian.
        p[5] = 0xFF; p[6] = 0xFF; // lt_val = 0xFFFF
        p[7] = 0x00; p[8] = 0x00; // rt_val = 0
        let s = parse_steam(&p).unwrap();
        assert_eq!(s.lt, 0xFF);
        assert_eq!(s.rt, 0x00);
        assert!(s.l2_dig);
        assert!(!s.r2_dig);
    }

    #[test]
    fn trigger_scaling_saturates_instead_of_wrapping_on_a_hard_pull() {
        let mut p = synthetic_report(0x00, 0x00, 0x00);
        // 40000 >> 7 = 312, which overflows a u8 (max 255). A plain `as u8`
        // cast wraps that to 56 -- a hard pull must read as fully pressed
        // (255), not a fifth of the way in.
        p[7..9].copy_from_slice(&40000u16.to_le_bytes());
        let s = parse_steam(&p).unwrap();
        assert_eq!(s.rt, 255);
    }

    #[test]
    fn digital_trigger_click_bits_register_even_below_the_analog_threshold() {
        // Cross-checked against safijari/openpuck's TB_L2 (payload[4] bit 3)
        // and TB_R2 (payload[3]/btn2 bit 7): a light click can set the
        // digital bit well before the analog value crosses 8000.
        let p = synthetic_report_full(0, 0, 0x80, 0x08);
        let s = parse_steam(&p).unwrap();
        assert!(s.r2_dig, "btn2 bit 7 (TB_R2) should register a digital click on its own");
        assert!(s.l2_dig, "payload[4] bit 3 (TB_L2) should register a digital click on its own");
    }

    #[test]
    fn touchpad_click_reads_the_pad_click_bits_not_the_l4_paddle() {
        // Regression test: this field used to read btn2 & 0x02, which is
        // actually the L4 back-paddle button (TB_L4), not a touchpad bit at
        // all. Cross-checked against safijari/openpuck's triton.h: right pad
        // click is btn2 (payload[3]) bit 6 (TB_RPADC), left pad click is
        // payload[4] bit 2 (TB_LPADC).
        let right_click = synthetic_report(0, 0, 0x40);
        assert!(parse_steam(&right_click).unwrap().touchpad);

        let left_click = synthetic_report_full(0, 0, 0, 0x04);
        assert!(parse_steam(&left_click).unwrap().touchpad);

        let l4_paddle_only = synthetic_report(0, 0, 0x02);
        assert!(!parse_steam(&l4_paddle_only).unwrap().touchpad, "L4 paddle must not be read as a touchpad click");
    }

    #[test]
    fn short_report_leaves_imu_zeroed_and_uses_stick_deflection_for_pad_active() {
        let mut p = synthetic_report(0, 0, 0);
        // Push the right stick hard right so the fallback pad-position proxy activates.
        p[13..15].copy_from_slice(&30000i16.to_le_bytes());
        let s = parse_steam(&p).unwrap();
        assert_eq!((s.gyro_x, s.gyro_y, s.gyro_z), (0, 0, 0));
        assert_eq!((s.accel_x, s.accel_y, s.accel_z), (0, 0, 0));
        assert!(s.pad_active, "a 17-byte report should still fall back to the stick-deflection proxy");
    }

    #[test]
    fn full_length_report_populates_real_gyro_accel_and_trackpad() {
        let p = synthetic_full_length_report(1000, -2000, 500, (11, -22, 33), (111, -222, 333));
        let s = parse_steam(&p).unwrap();
        assert_eq!((s.gyro_x, s.gyro_y, s.gyro_z), (111, -222, 333));
        assert_eq!((s.accel_x, s.accel_y, s.accel_z), (11, -22, 33));
        assert!(s.pad_active, "non-zero pad press should mark the pad active");
    }

    #[test]
    fn full_length_report_pad_inactive_when_press_is_zero() {
        let p = synthetic_full_length_report(1000, -2000, 0, (0, 0, 0), (0, 0, 0));
        let s = parse_steam(&p).unwrap();
        assert!(!s.pad_active, "zero pad press should read as not touching, even with nonzero X/Y");
    }

    #[test]
    fn xinput_report_has_correct_length_and_header() {
        let p = synthetic_report(0x01, 0x00, 0x00); // A pressed
        let s = parse_steam(&p).unwrap();
        let r = state_to_xinput(&s);
        assert_eq!(r.len(), 20);
        assert_eq!(r[0], 0x00);
        assert_eq!(r[1], 0x14);
        assert_eq!(r[3] & 0x10, 0x10); // A bit
    }

    #[test]
    fn switch_report_maps_nintendo_face_layout() {
        let p = synthetic_report(0x01, 0x00, 0x00); // logical A (Xbox) pressed
        let s = parse_steam(&p).unwrap();
        let r = state_to_switch(&s);
        assert_eq!(r[0] & 0x02, 0x02); // Xbox A -> Nintendo B bit
    }

    #[test]
    fn dualsense_report_id_and_length() {
        let p = synthetic_report(0x00, 0x00, 0x00);
        let s = parse_steam(&p).unwrap();
        let r = state_to_dualsense(&s, false);
        assert_eq!(r.len(), 64);
        assert_eq!(r[0], 0x01);
    }

    #[test]
    fn xinput_haptics_short_frame() {
        let intent = parse_xinput_haptics_out(&[0x00, 0x08, 0x80, 0x40]).unwrap();
        assert_eq!(intent.left_motor, 0x80 * 257);
        assert_eq!(intent.right_motor, 0x40 * 257);
    }

    #[test]
    fn xinput_haptics_long_frame() {
        let intent = parse_xinput_haptics_out(&[0x00, 0x08, 0x00, 0x80, 0x40]).unwrap();
        assert_eq!(intent.left_motor, 0x80 * 257);
        assert_eq!(intent.right_motor, 0x40 * 257);
    }

    #[test]
    fn dualsense_haptics_stop_frame() {
        let intent = parse_dualsense_haptics_out(&[0x02, 0x03, 0x00, 0x00, 0x00]).unwrap();
        assert_eq!(intent, HapticsIntent::default());
    }

    #[test]
    fn dualsense_haptics_active_frame() {
        let intent = parse_dualsense_haptics_out(&[0x02, 0x03, 0x00, 0x80, 0x40]).unwrap();
        assert_eq!(intent.right_motor, 0x80 * 257);
        assert_eq!(intent.left_motor, 0x40 * 257);
    }
}
