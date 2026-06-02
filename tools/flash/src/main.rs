use std::env;
use std::ffi::OsString;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Duration;

use espflash::connection::{Connection, ResetAfterOperation, ResetBeforeOperation};
use espflash::flasher::Flasher;
use espflash::target::ProgressCallbacks;
use serialport::UsbPortInfo;

struct StderrProgress {
    label: &'static str,
    total: usize,
    size: usize,
}

impl ProgressCallbacks for StderrProgress {
    fn init(&mut self, addr: u32, total: usize) {
        self.total = total;
        eprintln!("{}: {} bytes → {addr:#010x} ({total} chunks)…", self.label, self.size);
    }
    fn update(&mut self, current: usize) {
        if self.total > 0 {
            let pct = current * 100 / self.total;
            eprint!("\r{}: {pct:3}%   ", self.label);
        }
    }
    fn verifying(&mut self) {
        eprint!("\r{}: verifying…   ", self.label);
    }
    fn finish(&mut self, skipped: bool) {
        if skipped {
            eprintln!("\r{}: done (already up to date)              ", self.label);
        } else {
            eprintln!("\r{}: done                                   ", self.label);
        }
    }
}

include!(concat!(env!("OUT_DIR"), "/embedded_firmware.rs"));

struct Options {
    use_embedded: bool,
    external_explicit: bool,
    port: Option<String>,
    firmware_path: Option<PathBuf>,
}

fn main() -> ExitCode {
    env_logger::init();
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let raw: Vec<OsString> = env::args_os().collect();
    let program = raw
        .first()
        .map(|s| {
            Path::new(s)
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| s.to_string_lossy().into_owned())
        })
        .unwrap_or_else(|| "openpuck-flash".to_owned());

    let options = parse_args(&raw[1..], &program)?;

    if options
        .firmware_path
        .as_deref()
        .map(|p| p == Path::new("--help"))
        .unwrap_or(false)
    {
        print_help(&program);
        return Ok(());
    }

    let firmware = resolve_firmware(&options)?;
    let port = resolve_port(options.port.as_deref())?;
    flash(&port, &firmware)
}

// ---------------------------------------------------------------------------
// Firmware resolution
// ---------------------------------------------------------------------------

fn resolve_firmware(opts: &Options) -> Result<Vec<u8>, String> {
    if let Some(path) = &opts.firmware_path {
        return load_firmware_path(path);
    }
    if opts.external_explicit {
        return load_firmware_dir(Path::new("."));
    }
    if let Some(bytes) = EMBEDDED_FIRMWARE {
        return Ok(bytes.to_vec());
    }
    if opts.use_embedded {
        return Err(
            "--embedded requested but this binary was built without embedded firmware".into(),
        );
    }
    load_firmware_dir(Path::new("."))
}

fn load_firmware_path(path: &Path) -> Result<Vec<u8>, String> {
    if path.is_file() {
        fs::read(path).map_err(|e| format!("failed to read {}: {e}", path.display()))
    } else {
        load_firmware_dir(path)
    }
}

fn load_firmware_dir(dir: &Path) -> Result<Vec<u8>, String> {
    let entries =
        fs::read_dir(dir).map_err(|e| format!("failed to read {}: {e}", dir.display()))?;
    let mut bins: Vec<PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            p.is_file() && p.extension().and_then(|x| x.to_str()) == Some("bin")
        })
        .collect();
    bins.sort();
    match bins.len() {
        1 => fs::read(&bins[0])
            .map_err(|e| format!("failed to read {}: {e}", bins[0].display())),
        0 => Err(format!("no .bin firmware file found in {}", dir.display())),
        _ => Err(format!(
            "multiple .bin files in {}; pass an explicit path",
            dir.display()
        )),
    }
}

// ---------------------------------------------------------------------------
// Port resolution
// ---------------------------------------------------------------------------

fn resolve_port(explicit: Option<&str>) -> Result<String, String> {
    if let Some(p) = explicit {
        return Ok(p.to_owned());
    }
    if let Ok(p) = env::var("ESPFLASH_PORT") {
        if !p.is_empty() {
            return Ok(p);
        }
    }

    let ports = serialport::available_ports()
        .map_err(|e| format!("failed to enumerate serial ports: {e}"))?;

    let usb: Vec<_> = ports
        .iter()
        .filter(|p| matches!(p.port_type, serialport::SerialPortType::UsbPort(_)))
        .collect();

    match usb.len() {
        1 => Ok(usb[0].port_name.clone()),
        0 => Err(
            "no USB serial ports detected; connect the board or set ESPFLASH_PORT / --port".into(),
        ),
        _ => Err(format!(
            "multiple USB serial ports found: {}; use --port to specify one",
            usb.iter()
                .map(|p| p.port_name.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        )),
    }
}

// ---------------------------------------------------------------------------
// Flashing
// ---------------------------------------------------------------------------

fn flash(port_name: &str, firmware: &[u8]) -> Result<(), String> {
    // Trim trailing 0xFF (erased flash value) so we only write actual data.
    let data = trim_ff(firmware);
    eprintln!("Connecting to {} ({} bytes to write)…", port_name, data.len());

    let ports = serialport::available_ports().unwrap_or_default();
    let port_info = ports
        .iter()
        .find(|p| p.port_name == port_name)
        .and_then(|p| {
            if let serialport::SerialPortType::UsbPort(info) = &p.port_type {
                Some(info.clone())
            } else {
                None
            }
        })
        .unwrap_or_else(|| UsbPortInfo {
            vid: 0,
            pid: 0,
            serial_number: None,
            manufacturer: None,
            product: None,
        });

    let serial = serialport::new(port_name, 115_200)
        .timeout(Duration::from_secs(3))
        .open_native()
        .map_err(|e| {
            let msg = format!("failed to open {port_name}: {e}");
            #[cfg(unix)]
            if e.kind() == serialport::ErrorKind::NoDevice
                || format!("{e}").to_ascii_lowercase().contains("permission")
            {
                return format!(
                    "{msg}\n\nHint: run with sudo, or install the udev rule once:\n  \
                     echo 'SUBSYSTEM==\"tty\", ATTRS{{idVendor}}==\"1a86\", \
                     ATTRS{{idProduct}}==\"55d3\", GROUP=\"plugdev\", MODE=\"0660\"' \
                     | sudo tee /etc/udev/rules.d/99-openpuck.rules\n  \
                     sudo udevadm control --reload-rules && sudo udevadm trigger"
                );
            }
            msg
        })?;

    let connection = Connection::new(
        serial,
        port_info,
        ResetAfterOperation::HardReset,
        ResetBeforeOperation::DefaultReset,
        115_200,
    );

    eprintln!("Detecting chip…");
    let mut flasher = Flasher::connect(connection, true, false, false, None, None)
        .map_err(|e| format!("failed to connect to device on {port_name}: {e}"))?;

    let mut progress = StderrProgress { label: "Writing", total: 0, size: data.len() };
    flasher
        .write_bin_to_flash(0x0, data, &mut progress)
        .map_err(|e| format!("failed to write firmware: {e}"))?;

    eprintln!("Resetting device…");
    Ok(())
}

fn trim_ff(data: &[u8]) -> &[u8] {
    let end = data
        .iter()
        .rposition(|&b| b != 0xFF)
        .map_or(0, |i| i + 1);
    // Round up to 4-byte boundary for the flash driver.
    let aligned = (end + 3) & !3;
    &data[..aligned.min(data.len())]
}

// ---------------------------------------------------------------------------
// Argument parsing / help
// ---------------------------------------------------------------------------

fn parse_args(args: &[OsString], program: &str) -> Result<Options, String> {
    let mut use_embedded = true;
    let mut external_explicit = false;
    let mut port: Option<String> = None;
    let mut firmware_path: Option<PathBuf> = None;
    let mut i = 0usize;

    while i < args.len() {
        let arg = args[i].to_string_lossy();
        if arg == "-h" || arg == "--help" {
            return Ok(Options {
                use_embedded,
                external_explicit,
                port,
                firmware_path: Some(PathBuf::from("--help")),
            });
        } else if arg == "--embedded" {
            use_embedded = true;
            external_explicit = false;
        } else if arg == "--external" {
            use_embedded = false;
            external_explicit = true;
        } else if arg == "--port" {
            i += 1;
            let val = args
                .get(i)
                .ok_or_else(|| format!("{program}: --port requires a value"))?;
            port = Some(val.to_string_lossy().into_owned());
        } else if let Some(val) = arg.strip_prefix("--port=") {
            port = Some(val.to_owned());
        } else if arg.starts_with('-') {
            return Err(format!("{program}: unknown option: {arg}"));
        } else if firmware_path.is_some() {
            return Err("expected at most one positional argument".into());
        } else {
            firmware_path = Some(PathBuf::from(args[i].clone()));
        }
        i += 1;
    }

    // Explicit path implies external mode
    if firmware_path.is_some() && !external_explicit {
        use_embedded = false;
    }

    Ok(Options {
        use_embedded,
        external_explicit,
        port,
        firmware_path,
    })
}

fn print_help(program: &str) {
    println!(
        "Usage: {program} [--embedded|--external] [--port PORT] [firmware.bin|directory]

Default: use embedded firmware if built with one, otherwise scan current directory.
  --embedded        Use firmware compiled into this binary.
  --external        Ignore embedded firmware; use path or current directory.
  --port PORT       Serial port (overrides ESPFLASH_PORT and auto-detection).
  firmware.bin      Explicit path to a merged ESP32 firmware image.
  directory         Directory containing exactly one .bin firmware image."
    );
}
