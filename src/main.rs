#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
extern crate alloc;

use esp_backtrace as _;
esp_bootloader_esp_idf::esp_app_desc!();
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use static_cell::StaticCell;
use bt_hci::controller::ExternalController;
use trouble_host::prelude::*;

mod controller;
#[cfg(feature = "rf")]
mod puck;
mod usb;
mod bluetooth;
mod storage;

use controller::{parse_steam, remap_steam_for_switch, state_to_xinput, state_to_switch, state_to_dualsense,
    HapticsIntent, parse_xinput_haptics_out, parse_switch_haptics_out, parse_dualsense_haptics_out};
use usb::{usb_manager_task, queue_usb_report, USB_OUT_FRAMES, USB_REINIT};
use bluetooth::{ble_client_task, ble_runner_task, BLE_CONNECTED, BLE_REPORTS, BLE_HAPTICS};
use core::sync::atomic::{AtomicU8, Ordering};
use esp_hal::gpio::{Input, InputConfig, Pull, Output, OutputConfig, Level};

// Define concrete types for trouble-host to simplify signatures
type BleController = ExternalController<BleConnector<'static>, 20>;

/// One-shot LED mode event written by button task and consumed by led_task.
/// 0xFF = no pending event; 0=XInput, 1=SwitchPro, 2=DualSense.
static LED_MODE: AtomicU8 = AtomicU8::new(0xFF);

// Coordinator task: translates raw BLE notifications → XInput/Switch/DualSense USB reports
#[embassy_executor::task]
async fn coordinator_task() {
    log::info!("Starting translation coordinator task");
    #[cfg(feature = "rf")]
    let mut puck_parser = crate::puck::PuckParser::new();

    loop {
        let raw: [u8; 64] = BLE_REPORTS.wait().await;
        let ble_instant = embassy_time::Instant::now();
        let data_len = raw[2] as usize;
        let payload = &raw[3..3 + data_len.min(61)];
        let state = parse_steam(payload)
            .or_else(|| {
                #[cfg(feature = "rf")]
                {
                    puck_parser.parse_report_0x45(payload).map(|parsed| parsed.state)
                }
                #[cfg(not(feature = "rf"))]
                {
                    None
                }
            });
        if let Some(state) = state {
            let mode = storage::current_output_mode();
            let report: [u8; 64] = match mode {
                storage::OutputMode::XInput => {
                    let x = state_to_xinput(&state);
                    let mut r = [0u8; 64];
                    r[..20].copy_from_slice(&x);
                    r
                }
                storage::OutputMode::SwitchPro => {
                    let s_state = remap_steam_for_switch(&state);
                    let s = state_to_switch(&s_state);
                    let mut r = [0u8; 64];
                    r[..8].copy_from_slice(&s);
                    r
                }
                storage::OutputMode::DualSense => {
                    state_to_dualsense(&state, true)
                }
            };
            if mode == storage::OutputMode::SwitchPro {
                log::debug!(
                    "Switch report: b1={:02x} b2={:02x} hat={:02x} | select={} start={} l3={} r3={} home={} qam={}",
                    report[0],
                    report[1],
                    report[2],
                    state.select,
                    state.start,
                    state.l3,
                    state.r3,
                    state.home,
                    state.qam,
                );
            }
            queue_usb_report(report, ble_instant).await;
        }
    }
}

#[embassy_executor::task]
async fn rumble_bridge_task() {
    log::info!("Starting rumble bridge task");
    #[cfg(feature = "trace-haptics")]
    log::info!("Haptics trace enabled");

    let mut last_sent = HapticsIntent::default();
    let mut last_send_at = embassy_time::Instant::now();
    loop {
        let frame = USB_OUT_FRAMES.receive().await;
        let data = &frame.data[..(frame.len as usize).min(frame.data.len())];
        log::info!(
            "USB OUT frame: mode={:?} len={} data={:02x?}",
            frame.mode,
            frame.len,
            &data[..data.len().min(16)]
        );
        let intent = match frame.mode {
            storage::OutputMode::XInput => parse_xinput_haptics_out(data),
            storage::OutputMode::SwitchPro => parse_switch_haptics_out(data),
            storage::OutputMode::DualSense => parse_dualsense_haptics_out(data),
        };

        if let Some(intent) = intent {
            let changed = intent != last_sent;
            let stale = last_send_at.elapsed() >= embassy_time::Duration::from_millis(500);
            if changed || stale {
                BLE_HAPTICS.send(intent).await;
                last_sent = intent;
                last_send_at = embassy_time::Instant::now();
                #[cfg(feature = "trace-haptics")]
                log::info!(
                    "Haptics bridge: mode={:?} L={} R={} LTfx={} RTfx={}",
                    frame.mode,
                    intent.left_motor,
                    intent.right_motor,
                    intent.left_trigger_fx,
                    intent.right_trigger_fx,
                );
                #[cfg(not(feature = "trace-haptics"))]
                log::debug!(
                    "Haptics bridge: mode={:?} L={} R={} LTfx={} RTfx={}",
                    frame.mode,
                    intent.left_motor,
                    intent.right_motor,
                    intent.left_trigger_fx,
                    intent.right_trigger_fx,
                );
            }
        }
    }
}

fn neutral_report(mode: storage::OutputMode) -> [u8; 64] {
    match mode {
        storage::OutputMode::XInput => {
            let mut r = [0u8; 64];
            r[0] = 0x00;
            r[1] = 0x14;
            r
        }
        storage::OutputMode::SwitchPro => {
            let mut r = [0u8; 64];
            r[0] = 0x00;
            r[1] = 0x00;
            r[2] = 0x08; // Hat neutral
            r[3] = 128;
            r[4] = 128;
            r[5] = 128;
            r[6] = 128;
            r
        }
        storage::OutputMode::DualSense => {
            let mut r = [0u8; 64];
            r[0] = 0x01; // Report ID
            r[1] = 128; r[2] = 128; r[3] = 128; r[4] = 128;
            r[5] = 0; r[6] = 0;
            r[7] = 0x08; // Hat neutral (low nibble)
            r
        }
    }
}
/// Write one WS2812B pixel using raw GPIO register writes and CCOUNT cycle-accurate timing.
/// `cycles_per_us` is calibrated by the caller at startup.
fn ws2812_write(_pin: &mut Output<'static>, rgb: (u8, u8, u8), cycles_per_us: u32) {
    // GPIO_OUT1_W1TS / W1TC control GPIO32–63 on ESP32-S3.
    // GPIO48 = bit (48−32) = bit 16.
    const W1TS: *mut u32 = 0x6000_4014 as *mut u32;
    const W1TC: *mut u32 = 0x6000_4018 as *mut u32;
    const BIT: u32 = 1 << (48 - 32);

    // Spin for exactly `n` CPU cycles using the Xtensa CCOUNT register.
    fn spin(n: u32) {
        let start: u32;
        unsafe { core::arch::asm!("rsr.ccount {0}", out(reg) start); }
        loop {
            let now: u32;
            unsafe { core::arch::asm!("rsr.ccount {0}", out(reg) now); }
            if now.wrapping_sub(start) >= n { break; }
        }
    }

    // Derive timing from calibrated cycles_per_us (works at 80 / 160 / 240 MHz):
    //   T0H 300 ns  T0L 900 ns  T1H 800 ns  T1L 350 ns  reset 80 µs
    let t0h   = (cycles_per_us * 30)  / 100;
    let t0l   = (cycles_per_us * 90)  / 100;
    let t1h   = (cycles_per_us * 80)  / 100;
    let t1l   = (cycles_per_us * 35)  / 100;
    let reset = cycles_per_us * 80;

    critical_section::with(|_| {
        unsafe { core::ptr::write_volatile(W1TC, BIT); }
        spin(reset);

        for &byte in &[rgb.1, rgb.0, rgb.2, 0u8] {   // GRB + W=0 (SK6812MINI-E GRBW / WS2812B compatible)
            for bit in (0..8_u8).rev() {
                if (byte >> bit) & 1 == 0 {
                    unsafe { core::ptr::write_volatile(W1TS, BIT); }
                    spin(t0h);
                    unsafe { core::ptr::write_volatile(W1TC, BIT); }
                    spin(t0l);
                } else {
                    unsafe { core::ptr::write_volatile(W1TS, BIT); }
                    spin(t1h);
                    unsafe { core::ptr::write_volatile(W1TC, BIT); }
                    spin(t1l);
                }
            }
        }

        unsafe { core::ptr::write_volatile(W1TC, BIT); }
        spin(reset);
    });
}

#[embassy_executor::task]
async fn led_task(mut pin: Output<'static>) {
    // ── Calibrate CPU frequency ──────────────────────────────────────────────
    // Measure CCOUNT ticks over a known 500 µs delay_micros() call to obtain
    // the actual cycles_per_us regardless of whether the CPU is at 80/160/240 MHz.
    let cycles_per_us = {
        let start: u32;
        unsafe { core::arch::asm!("rsr.ccount {0}", out(reg) start); }
        let cal_delay = esp_hal::delay::Delay::new();
        cal_delay.delay_micros(500);
        let end: u32;
        unsafe { core::arch::asm!("rsr.ccount {0}", out(reg) end); }
        let cps = end.wrapping_sub(start) / 500;
        log::info!("LED calibration: ~{} MHz ({} cyc/µs)", cps, cps);
        cps.max(1) // guard against division-by-zero if CCOUNT is broken
    };

    // ── Blast black frames to clear any stuck state ──────────────────────────
    log::info!("LED: clearing to black (x20)...");
    for _ in 0..20 {
        ws2812_write(&mut pin, (0, 0, 0), cycles_per_us);
        embassy_time::Timer::after(embassy_time::Duration::from_millis(2)).await;
    }

    let mode = storage::load_output_mode();
    let mut color = match mode {
        storage::OutputMode::XInput    => (0,   255, 0),   // Green
        storage::OutputMode::SwitchPro => (255, 0,   0),   // Red
        storage::OutputMode::DualSense => (0,   0,   255), // Blue
    };
    log::info!("LED: mode {:?} → color {:?}", mode, color);

    // ── Loop forever, refreshing every 100 ms; on for 10s after boot/mode changes ──
    let mut bright_start = embassy_time::Instant::now();
    loop {
        // Consume pending mode-change event exactly once.
        let cur_raw = LED_MODE.swap(0xFF, Ordering::Relaxed);
        if cur_raw != 0xFF {
            color = match cur_raw {
                0 => (0,   255, 0),
                1 => (255, 0,   0),
                2 => (0,   0,   255),
                _ => color,
            };
            bright_start = embassy_time::Instant::now();
            log::debug!("LED: mode changed to {:?}", color);
        }
        let target = if bright_start.elapsed().as_secs() < 10 { color } else { (0, 0, 0) };
        ws2812_write(&mut pin, target, cycles_per_us);
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn connection_state_task() {
    let mut prev_connected = false;
    loop {
        let connected = BLE_CONNECTED.wait().await;
        if prev_connected && !connected {
            let mode = storage::current_output_mode();
            queue_usb_report(neutral_report(mode), embassy_time::Instant::now()).await;
            log::info!("BLE disconnected: sent neutral USB report for {:?}", mode);
        }
        prev_connected = connected;
    }
}

#[embassy_executor::task]
async fn button_monitor_task(mut pin: Input<'static>) {
    log::info!("Starting BOOT button monitor task");
    // Track the current mode locally so USB-reinit cache updates don't corrupt our cycle state.
    // Read once from flash at startup; afterwards advance the local variable on every press.
    let mut current = storage::load_output_mode();
    let mut last_toggle_at = embassy_time::Instant::now();
    loop {
        pin.wait_for_falling_edge().await;
        // Robust debounce: validate low state twice and ignore repeat edges briefly.
        embassy_time::Timer::after(embassy_time::Duration::from_millis(30)).await;
        if !pin.is_low() {
            continue;
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(30)).await;
        if !pin.is_low() {
            continue;
        }

        if last_toggle_at.elapsed() < embassy_time::Duration::from_millis(500) {
            // Ignore bounce/retrigger events from one physical press.
            continue;
        }

            let next = match current {
                storage::OutputMode::XInput    => storage::OutputMode::SwitchPro,
                storage::OutputMode::SwitchPro => storage::OutputMode::DualSense,
                storage::OutputMode::DualSense => storage::OutputMode::XInput,
            };

        log::info!(
            "Mode toggle: {:?} -> {:?} (USB will re-enumerate, BLE stays connected)",
            current,
            next
        );

        storage::save_output_mode(next);
        USB_REINIT.signal(());
        LED_MODE.store(next as u8, Ordering::Relaxed);
        current = next; // advance local state regardless of flash success
        last_toggle_at = embassy_time::Instant::now();

        // Wait for release to avoid multiple toggles from one hold.
        pin.wait_for_rising_edge().await;
        embassy_time::Timer::after(embassy_time::Duration::from_millis(60)).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    log::info!("Initializing OpenPuck firmware...");

    // Prime output mode cache once at boot so hot paths avoid flash reads.
    let _ = storage::load_output_mode();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize heap - the Bluetooth radio controller requires dynamic memory.
    // 72KB is the minimum recommended for BLE + BT stack on ESP32-S3.
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // Start preemptive RTOS task scheduler
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Initialize BLE Stack using esp-radio and trouble-host
    log::info!("Initializing BLE controller...");
    let ble_connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let controller = ExternalController::<_, 20>::new(ble_connector);

    static RESOURCES: StaticCell<HostResources<DefaultPacketPool, 1, 1, 1>> = StaticCell::new();
    let resources = RESOURCES.init(HostResources::new());

    let address = Address::random([0xff, 0x8f, 0x07, 0x90, 0x11, 0x22]);

    static STACK: StaticCell<Stack<'static, BleController, DefaultPacketPool>> = StaticCell::new();

    // Wrap the ESP32-S3 hardware RNG in a CryptoRng marker (it IS a CSPRNG at the hardware level).
    struct EspCryptoRng(Rng);
    impl rand_core::RngCore for EspCryptoRng {
        fn next_u32(&mut self) -> u32 { self.0.random() }
        fn next_u64(&mut self) -> u64 {
            (self.0.random() as u64) << 32 | self.0.random() as u64
        }
        fn fill_bytes(&mut self, dest: &mut [u8]) {
            for chunk in dest.chunks_mut(4) {
                let bytes = self.0.random().to_le_bytes();
                chunk.copy_from_slice(&bytes[..chunk.len()]);
            }
        }
        fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
            self.fill_bytes(dest);
            Ok(())
        }
    }
    impl rand_core::CryptoRng for EspCryptoRng {}

    let mut rng = EspCryptoRng(Rng::new());
    let stack_ref = STACK.init(
        trouble_host::new(controller, resources)
            .set_random_address(address)
            .set_random_generator_seed(&mut rng)
    );

    let Host {
        runner,
        central,
        ..
    } = stack_ref.build();

    spawner.spawn(ble_runner_task(runner).unwrap());
    spawner.spawn(ble_client_task(stack_ref, central).unwrap());

    // USB is managed by usb_manager_task, which initialises the hardware itself via peripheral
    // steal on each mode change. GPIO19/GPIO20/USB0 must NOT be consumed here.
    log::info!("Spawning USB manager task...");
    spawner.spawn(usb_manager_task().unwrap());
    spawner.spawn(rumble_bridge_task().unwrap());

    // Spawn coordinator
    spawner.spawn(coordinator_task().unwrap());
    spawner.spawn(connection_state_task().unwrap());

    // Initialize RGB LED (GPIO48)
    let led_pin = Output::new(peripherals.GPIO48, Level::Low, OutputConfig::default());
    spawner.spawn(led_task(led_pin).unwrap());

    // Set up BOOT button (GPIO0) for hardware toggling
    let boot_button = Input::new(peripherals.GPIO0, InputConfig::default().with_pull(Pull::Up));
    spawner.spawn(button_monitor_task(boot_button).unwrap());

    log::info!("OpenPuck initialization complete! System running.");
}
