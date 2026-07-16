mod ble;
mod controller;
mod haptics;
mod output;

use anyhow::{Context, Result};
use btleplug::api::Peripheral as _;
use clap::{Parser, Subcommand};
use controller::{GamepadState, HapticsIntent};
use futures::StreamExt;
use output::OutputMode;
use std::time::Duration;
use tokio::sync::watch;

#[derive(Parser)]
#[command(name = "freepuck-software", about = "Software-only BLE bridge for the Steam Controller 2")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Connect to a Steam Controller over BLE and print live input state to stdout.
    /// No virtual controller output — useful for validating the BLE connection alone.
    Scan {
        /// Seconds to scan for a controller before giving up.
        #[arg(long, default_value_t = 20)]
        timeout: u64,
    },
    /// Connect to a Steam Controller and present it to the OS as a virtual controller.
    Run {
        /// Output mode: what kind of virtual controller to present.
        #[arg(long, value_enum, default_value_t = OutputMode::Xinput)]
        mode: OutputMode,
        /// Seconds to scan for a controller before giving up.
        #[arg(long, default_value_t = 20)]
        timeout: u64,
    },
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let cli = Cli::parse();

    match cli.command {
        Command::Scan { timeout } => run_scan(Duration::from_secs(timeout)).await,
        Command::Run { mode, timeout } => run_bridge(mode, Duration::from_secs(timeout)).await,
    }
}

async fn connect(scan_timeout: Duration) -> Result<ble::SteamGatt> {
    println!("Scanning for Steam Controller (timeout {scan_timeout:?})...");
    let peripheral = ble::find_controller(scan_timeout).await.context("scan failed")?;

    println!("Connecting...");
    let gatt = ble::connect_and_discover(peripheral).await.context("connect/discover failed")?;
    ble::send_init_commands(&gatt).await.context("init commands failed")?;
    println!("Connected. Lizard mode disabled, haptics enabled.");
    Ok(gatt)
}

async fn run_scan(scan_timeout: Duration) -> Result<()> {
    let gatt = connect(scan_timeout).await?;
    println!("Streaming input (Ctrl+C to stop)...");

    tokio::spawn(ble::run_lizard_keepalive(gatt.clone()));

    let mut notifications = gatt
        .peripheral
        .notifications()
        .await
        .context("failed to get notification stream")?;

    let input_uuid = gatt.input_char.uuid;
    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("\nDisconnecting...");
                let _ = gatt.peripheral.disconnect().await;
                return Ok(());
            }
            notification = notifications.next() => {
                let Some(notification) = notification else {
                    println!("Notification stream ended (controller disconnected).");
                    return Ok(());
                };
                if notification.uuid != input_uuid {
                    continue;
                }
                match controller::parse_steam(&notification.value) {
                    Some(state) => print_state(&state),
                    None => log::warn!("short/unparseable input report ({} bytes)", notification.value.len()),
                }
            }
        }
    }
}

async fn run_bridge(mode: OutputMode, scan_timeout: Duration) -> Result<()> {
    let gatt = connect(scan_timeout).await?;

    let (state_tx, state_rx) = watch::channel(GamepadState::default_centred());
    let (haptics_tx, haptics_rx) = watch::channel(HapticsIntent::default());

    tokio::spawn(ble::run_lizard_keepalive(gatt.clone()));
    tokio::spawn(haptics::run_rumble_bridge(gatt.clone(), haptics_rx));
    spawn_output_backend(mode, state_rx, haptics_tx)?;

    println!("Virtual {mode:?} controller running (Ctrl+C to stop)...");

    let mut notifications = gatt
        .peripheral
        .notifications()
        .await
        .context("failed to get notification stream")?;
    let input_uuid = gatt.input_char.uuid;

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("\nDisconnecting...");
                let _ = gatt.peripheral.disconnect().await;
                return Ok(());
            }
            notification = notifications.next() => {
                let Some(notification) = notification else {
                    println!("Notification stream ended (controller disconnected).");
                    return Ok(());
                };
                if notification.uuid != input_uuid {
                    continue;
                }
                match controller::parse_steam(&notification.value) {
                    Some(state) => { let _ = state_tx.send(state); }
                    None => log::warn!("short/unparseable input report ({} bytes)", notification.value.len()),
                }
            }
        }
    }
}

#[cfg(target_os = "linux")]
fn spawn_output_backend(
    mode: OutputMode,
    state_rx: watch::Receiver<GamepadState>,
    _haptics_tx: watch::Sender<HapticsIntent>,
) -> Result<()> {
    // Rumble forwarding is not implemented for the Linux uinput backend — see
    // output/linux_uinput.rs for why. `_haptics_tx` is intentionally unused here.
    tokio::spawn(async move {
        if let Err(e) = output::linux_uinput::run(mode, state_rx).await {
            log::error!("uinput output backend failed: {e:#}");
        }
    });
    Ok(())
}

#[cfg(windows)]
fn spawn_output_backend(
    mode: OutputMode,
    mut state_rx: watch::Receiver<GamepadState>,
    haptics_tx: watch::Sender<HapticsIntent>,
) -> Result<()> {
    // vigem-client is a blocking/synchronous API, so it runs on its own OS
    // thread bridged to the async world via std::sync::mpsc channels.
    let (std_state_tx, std_state_rx) = std::sync::mpsc::channel::<GamepadState>();
    let (std_haptics_tx, std_haptics_rx) = std::sync::mpsc::channel::<HapticsIntent>();

    tokio::spawn(async move {
        loop {
            if state_rx.changed().await.is_err() {
                return;
            }
            let state = *state_rx.borrow();
            if std_state_tx.send(state).is_err() {
                return;
            }
        }
    });

    std::thread::spawn(move || {
        while let Ok(intent) = std_haptics_rx.recv() {
            if haptics_tx.send(intent).is_err() {
                return;
            }
        }
    });

    std::thread::spawn(move || {
        if let Err(e) = output::windows_vigem::run(mode, std_state_rx, std_haptics_tx) {
            log::error!("ViGEmBus output backend failed: {e:#}");
        }
    });

    Ok(())
}

#[cfg(not(any(target_os = "linux", windows)))]
fn spawn_output_backend(
    _mode: OutputMode,
    _state_rx: watch::Receiver<GamepadState>,
    _haptics_tx: watch::Sender<HapticsIntent>,
) -> Result<()> {
    anyhow::bail!("virtual controller output is not yet implemented on this platform (only Linux and Windows are supported)")
}

fn print_state(s: &controller::GamepadState) {
    let mut buttons = String::new();
    for (pressed, label) in [
        (s.a, "A"), (s.b, "B"), (s.x, "X"), (s.y, "Y"),
        (s.l1, "LB"), (s.r1, "RB"), (s.l3, "L3"), (s.r3, "R3"),
        (s.start, "Start"), (s.select, "Select"), (s.home, "Home"),
        (s.touchpad, "Touchpad"), (s.qam, "QAM"),
        (s.dpad_up, "Up"), (s.dpad_down, "Down"), (s.dpad_left, "Left"), (s.dpad_right, "Right"),
    ] {
        if pressed {
            buttons.push_str(label);
            buttons.push(' ');
        }
    }
    print!(
        "\rLX:{:3} LY:{:3} RX:{:3} RY:{:3} LT:{:3} RT:{:3} | {:<60}",
        s.lx, s.ly, s.rx, s.ry, s.lt, s.rt, buttons
    );
    use std::io::Write;
    let _ = std::io::stdout().flush();
}
