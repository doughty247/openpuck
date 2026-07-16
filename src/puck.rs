use crate::controller::GamepadState;

/// Provisional decode surface for report 0x45 fields that are not yet confirmed.
///
/// Confidence levels are noted per field group so these can be promoted later.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PuckProvisional {
    /// Medium confidence: Start/Back/Menu group likely lives in byte 11 bits 0-1.
    pub provisional_start_back_menu_bits: u8,

    /// Medium confidence: D-pad activity cluster appears across bytes 14-17.
    pub provisional_dpad_b14: u8,
    pub provisional_dpad_b15: u8,
    pub provisional_dpad_b16: u8,
    pub provisional_dpad_b17: u8,

    /// Medium confidence: LT path candidates from LT sweep captures.
    pub provisional_lt_b6: u8,
    pub provisional_lt_b7: u8,
    pub provisional_lt_b13: u8,

    /// Medium confidence: RT path candidates from RT sweep captures.
    pub provisional_rt_b8: u8,
    pub provisional_rt_b9: u8,
    pub provisional_rt_b4_high_nibble: u8,

    /// Medium confidence: right stick axes candidates.
    pub provisional_rstick_x_b15: u8,
    pub provisional_rstick_y_b17: u8,
}

#[derive(Clone, Copy)]
pub struct PuckParseResult {
    pub state: GamepadState,
    /// Was `state.puck_provisional` before `GamepadState` moved into the
    /// shared `steam-protocol` crate, which has no reason to know about
    /// this RF-only, provisional decode surface. Carried alongside instead.
    pub provisional: PuckProvisional,
    #[allow(dead_code)]
    pub counter: u8,
}

/// Stateful parser for Steam puck USB input report 0x45.
///
/// The rolling counter validator logs gaps so dropped frames are visible in logs.
#[derive(Default)]
pub struct PuckParser {
    last_counter: Option<u8>,
}

impl PuckParser {
    pub const fn new() -> Self {
        Self { last_counter: None }
    }

    pub fn parse_report_0x45(&mut self, payload: &[u8]) -> Option<PuckParseResult> {
        if payload.len() != 46 || payload[0] != 0x45 {
            return None;
        }

        let counter = payload[1];
        self.validate_counter(counter);

        // Confirmed mappings from controlled capture suite.
        let a = payload[2] & 0x01 != 0;
        let b = payload[2] & 0x02 != 0;
        let x = payload[2] & 0x04 != 0;
        let y = payload[2] & 0x08 != 0;
        let l1 = payload[4] & 0x08 != 0;
        let r1 = payload[3] & 0x02 != 0;

        let provisional = PuckProvisional {
            provisional_start_back_menu_bits: payload[11] & 0x03,
            provisional_dpad_b14: payload[14],
            provisional_dpad_b15: payload[15],
            provisional_dpad_b16: payload[16],
            provisional_dpad_b17: payload[17],
            provisional_lt_b6: payload[6],
            provisional_lt_b7: payload[7],
            provisional_lt_b13: payload[13],
            provisional_rt_b8: payload[8],
            provisional_rt_b9: payload[9],
            provisional_rt_b4_high_nibble: (payload[4] >> 4) & 0x0f,
            provisional_rstick_x_b15: payload[15],
            provisional_rstick_y_b17: payload[17],
        };

        // Profile A (default): only the six bits above are decoded; every
        // other field below stays neutral, exactly as this function has
        // always shipped. Profile B (`rf-xref-mapping`): the rest of report
        // 0x45, decoded from offsets cross-referenced against
        // safijari/openpuck's `docs/PROTOCOL.md`/`triton.h` -- a *different*
        // confidence case than the same cross-reference's BLE-side fixes
        // elsewhere in this repo, because there's no shift to infer here:
        // this function already gates on `payload[0] == 0x45`, i.e. this is
        // confirmed to be the raw report with its report-ID byte intact, so
        // the reference's offsets apply directly, byte-for-byte. Still
        // nobody has run this against real hardware -- these two profiles
        // exist specifically so that can happen as an A/B comparison rather
        // than a blind swap. Report back which one is actually correct.
        #[cfg(feature = "rf-xref-mapping")]
        let (
            l3, r3, start, select, home, touchpad, qam,
            dpad_up, dpad_down, dpad_left, dpad_right,
            l2_dig, r2_dig, lt, rt, lx, ly, rx, ry,
        ) = {
            fn u16le(p: &[u8], off: usize) -> u16 { (p[off] as u16) | ((p[off + 1] as u16) << 8) }
            fn i16le(p: &[u8], off: usize) -> i16 { u16le(p, off) as i16 }

            let btn2 = payload[4]; // report offset 4: STEAM,L4,L5,LB,RSTICK_T,RPAD_T,RPAD_C,R2
            let btn3 = payload[5]; // report offset 5: LSTICK_T,LPAD_T,LPAD_C,L2,RGRIP_T,LGRIP_T

            let lt_val = u16le(payload, 6);
            let rt_val = u16le(payload, 8);
            let ls_x = i16le(payload, 10);
            let ls_y = i16le(payload, 12);
            let rs_x = i16le(payload, 14);
            let rs_y = i16le(payload, 16);
            // Same centring/scale convention as parse_steam (BLE path):
            // Steam i16 positive = right/up; GamepadState 128=centre, 0=up.
            let lx = ((ls_x >> 8) as i32 + 128).clamp(0, 255) as u8;
            let ly = (128 - (ls_y >> 8) as i32).clamp(0, 255) as u8;
            let rx = ((rs_x >> 8) as i32 + 128).clamp(0, 255) as u8;
            let ry = (128 - (rs_y >> 8) as i32).clamp(0, 255) as u8;

            (
                payload[3] & 0x80 != 0, // l3   (TB_L3,   byte1/report[3] bit7)
                payload[2] & 0x20 != 0, // r3   (TB_R3,   byte0/report[2] bit5)
                payload[2] & 0x40 != 0, // start(TB_VIEW, byte0/report[2] bit6)
                payload[3] & 0x40 != 0, // select(TB_MENU,byte1/report[3] bit6)
                btn2 & 0x01 != 0,       // home (TB_STEAM, bit0)
                btn2 & 0x40 != 0,       // touchpad (TB_RPADC, bit6)
                payload[2] & 0x10 != 0, // qam  (TB_QAM,   byte0/report[2] bit4)
                payload[3] & 0x20 != 0, // dpad_up    (TB_DUP, bit5)
                payload[3] & 0x04 != 0, // dpad_down  (TB_DDN, bit2)
                payload[3] & 0x10 != 0, // dpad_left  (TB_DLF, bit4)
                payload[3] & 0x08 != 0, // dpad_right (TB_DRT, bit3)
                btn3 & 0x08 != 0 || lt_val > 8000, // l2_dig (TB_L2, bit3; OR analog threshold as in parse_steam)
                btn2 & 0x80 != 0 || rt_val > 8000, // r2_dig (TB_R2, bit7; OR analog threshold as in parse_steam)
                (lt_val >> 7).min(255) as u8,
                (rt_val >> 7).min(255) as u8,
                lx, ly, rx, ry,
            )
        };
        #[cfg(not(feature = "rf-xref-mapping"))]
        let (
            l3, r3, start, select, home, touchpad, qam,
            dpad_up, dpad_down, dpad_left, dpad_right,
            l2_dig, r2_dig, lt, rt, lx, ly, rx, ry,
        ) = (
            false, false, false, false, false, false, false,
            false, false, false, false,
            false, false, 0u8, 0u8, 128u8, 128u8, 128u8, 128u8,
        );

        let state = GamepadState {
            a, b, x, y, l1, r1,
            lt, rt,
            l2_dig, r2_dig,
            l3, r3,
            start, select, home, touchpad, qam,
            dpad_up, dpad_down, dpad_left, dpad_right,
            lx, ly, rx, ry,
            pad_x: 960,
            pad_y: 540,
            pad_active: false,
            gyro_x: 0, gyro_y: 0, gyro_z: 0,
            accel_x: 0, accel_y: 0, accel_z: 0,
        };

        Some(PuckParseResult { state, provisional, counter })
    }

    fn counter_gap(prev: u8, current: u8) -> Option<u8> {
        let expected = prev.wrapping_add(1);
        if current == expected {
            None
        } else {
            Some(current.wrapping_sub(expected))
        }
    }

    fn validate_counter(&mut self, current: u8) {
        if let Some(prev) = self.last_counter {
            if let Some(missed) = Self::counter_gap(prev, current) {
                let expected = prev.wrapping_add(1);
                log::warn!(
                    "Puck counter gap: prev={} expected={} got={} missed={}",
                    prev,
                    expected,
                    current,
                    missed,
                );
            }
        }
        self.last_counter = Some(current);
    }
}

#[cfg(test)]
mod tests {
    use super::PuckParser;

    fn base_report(counter: u8) -> [u8; 46] {
        let mut report = [0u8; 46];
        report[0] = 0x45;
        report[1] = counter;
        report
    }

    #[test]
    fn parses_confirmed_button_bits() {
        let mut parser = PuckParser::new();
        let mut report = base_report(0x2a);
        report[2] = 0x0f;
        report[3] = 0x02;
        report[4] = 0x08;
        report[11] = 0x03;
        report[14] = 0x12;
        report[15] = 0x34;
        report[16] = 0x56;
        report[17] = 0x78;

        let parsed = parser.parse_report_0x45(&report).expect("report should parse");

        assert!(parsed.state.a);
        assert!(parsed.state.b);
        assert!(parsed.state.x);
        assert!(parsed.state.y);
        assert!(parsed.state.l1);
        assert!(parsed.state.r1);
        assert_eq!(parsed.counter, 0x2a);

        let provisional = parsed.provisional;
        assert_eq!(provisional.provisional_start_back_menu_bits, 0x03);
        assert_eq!(provisional.provisional_dpad_b14, 0x12);
        assert_eq!(provisional.provisional_rstick_x_b15, 0x34);
        assert_eq!(provisional.provisional_dpad_b16, 0x56);
        assert_eq!(provisional.provisional_rstick_y_b17, 0x78);
    }

    #[cfg(feature = "rf-xref-mapping")]
    #[test]
    fn xref_profile_decodes_dpad_start_select_and_sticks() {
        let mut parser = PuckParser::new();
        let mut report = base_report(0x01);
        report[2] = 0x40; // TB_VIEW -> start
        report[3] = 0x40 | 0x20; // TB_MENU -> select, TB_DUP -> dpad_up
        report[4] = 0x01 | 0x40; // TB_STEAM -> home, TB_RPADC -> touchpad
        report[5] = 0x08; // TB_L2 -> l2_dig
        report[10..12].copy_from_slice(&30000i16.to_le_bytes()); // left stick hard right

        let parsed = parser.parse_report_0x45(&report).expect("report should parse");
        assert!(parsed.state.start, "TB_VIEW (report[2] bit6) should decode to start");
        assert!(parsed.state.select, "TB_MENU (report[3] bit6) should decode to select");
        assert!(parsed.state.dpad_up, "TB_DUP (report[3] bit5) should decode to dpad_up");
        assert!(parsed.state.home, "TB_STEAM (report[4] bit0) should decode to home");
        assert!(parsed.state.touchpad, "TB_RPADC (report[4] bit6) should decode to touchpad");
        assert!(parsed.state.l2_dig, "TB_L2 (report[5] bit3) should decode to l2_dig");
        assert!(parsed.state.lx > 200, "left stick should read strongly right, got {}", parsed.state.lx);
    }

    #[cfg(not(feature = "rf-xref-mapping"))]
    #[test]
    fn default_profile_leaves_dpad_start_select_neutral() {
        let mut parser = PuckParser::new();
        let mut report = base_report(0x01);
        // Set every bit the xref profile would read as start/select/dpad/etc,
        // to confirm the default profile really does ignore them all.
        report[2] = 0xF0;
        report[3] = 0xFC;
        report[4] = 0xFF;
        report[5] = 0xFF;

        let parsed = parser.parse_report_0x45(&report).expect("report should parse");
        assert!(!parsed.state.start && !parsed.state.select && !parsed.state.home && !parsed.state.touchpad);
        assert!(!parsed.state.dpad_up && !parsed.state.dpad_down && !parsed.state.dpad_left && !parsed.state.dpad_right);
        assert_eq!((parsed.state.lx, parsed.state.ly, parsed.state.rx, parsed.state.ry), (128, 128, 128, 128));
    }

    #[test]
    fn rejects_wrong_report_id_or_length() {
        let mut parser = PuckParser::new();

        assert!(parser.parse_report_0x45(&[0u8; 45]).is_none());

        let mut wrong_id = [0u8; 46];
        wrong_id[0] = 0x44;
        assert!(parser.parse_report_0x45(&wrong_id).is_none());
    }

    #[test]
    fn counter_gap_accepts_wraparound() {
        assert_eq!(PuckParser::counter_gap(0xff, 0x00), None);
        assert_eq!(PuckParser::counter_gap(0x10, 0x11), None);
    }

    #[test]
    fn counter_gap_detects_missed_frames() {
        assert_eq!(PuckParser::counter_gap(10, 13), Some(2));
        assert_eq!(PuckParser::counter_gap(0xfe, 0x01), Some(2));
    }
}
