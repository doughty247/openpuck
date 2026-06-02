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

        let state = GamepadState {
            a,
            b,
            x,
            y,
            l1,
            r1,
            lt: 0,
            rt: 0,
            l2_dig: false,
            r2_dig: false,
            l3: false,
            r3: false,
            start: false,
            select: false,
            home: false,
            touchpad: false,
            qam: false,
            dpad_up: false,
            dpad_down: false,
            dpad_left: false,
            dpad_right: false,
            lx: 128,
            ly: 128,
            rx: 128,
            ry: 128,
            pad_x: 960,
            pad_y: 540,
            pad_active: false,
            puck_provisional: Some(provisional),
        };

        Some(PuckParseResult { state, counter })
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

        let provisional = parsed.state.puck_provisional.expect("provisional fields present");
        assert_eq!(provisional.provisional_start_back_menu_bits, 0x03);
        assert_eq!(provisional.provisional_dpad_b14, 0x12);
        assert_eq!(provisional.provisional_rstick_x_b15, 0x34);
        assert_eq!(provisional.provisional_dpad_b16, 0x56);
        assert_eq!(provisional.provisional_rstick_y_b17, 0x78);
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
