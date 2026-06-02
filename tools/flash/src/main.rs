use std::env;
use std::ffi::OsString;
use std::fs;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

include!(concat!(env!("OUT_DIR"), "/embedded_firmware.rs"));

enum SourcePreference {
    DefaultEmbedded,
    Embedded,
    External,
}

struct Options {
    source_preference: SourcePreference,
    port: Option<OsString>,
    input: Option<PathBuf>,
}

fn main() -> ExitCode {
    match run() {
        Ok(code) => code,
        Err(message) => {
            eprintln!("error: {message}");
            ExitCode::from(1)
        }
    }
}

fn run() -> Result<ExitCode, String> {
    let mut args = env::args_os();
    let program = args.next().unwrap_or_else(|| OsString::from("flash"));
    let options = parse_args(args.collect(), &program)?;

    if matches!(options.input.as_deref(), Some(path) if path == Path::new("--help")) {
        print_help(&program);
        return Ok(ExitCode::SUCCESS);
    }

    let source = resolve_firmware_source(options.input.as_deref(), &options.source_preference)?;
    let espflash = resolve_espflash_source()?;
    let status = run_espflash(&source, &espflash, options.port)?;
    Ok(ExitCode::from(status.code().unwrap_or(1) as u8))
}

fn print_help(program: &OsString) {
    println!(
        "Usage: {} [--embedded|--external] [--port PORT] [firmware.bin|directory]\n\nDefault behavior is embedded firmware (same as --embedded).\n--external disables embedded firmware and uses the supplied path or current directory.\nIf a firmware path is provided, that merged ESP32 image is flashed.\nIf a directory is provided, the tool flashes the only .bin file found there.\n--port overrides auto-detection and ESPFLASH_PORT.\nIf no port is supplied, the tool auto-detects a single available port by asking espflash to list ports.",
        Path::new(program).file_name().and_then(|name| name.to_str()).unwrap_or("flash")
    );
}

enum FirmwareSource {
    File(PathBuf),
    Embedded { bytes: &'static [u8], name: &'static str },
}

enum EspflashSource {
    Path(PathBuf),
    Embedded { bytes: &'static [u8], name: &'static str },
}

fn parse_args(args: Vec<OsString>, program: &OsString) -> Result<Options, String> {
    let mut source_preference = SourcePreference::DefaultEmbedded;
    let mut port = env::var_os("ESPFLASH_PORT");
    let mut input: Option<PathBuf> = None;
    let mut i = 0usize;

    while i < args.len() {
        let arg = &args[i];
        if arg == "-h" || arg == "--help" {
            return Ok(Options {
                source_preference,
                port,
                input: Some(PathBuf::from("--help")),
            });
        } else if arg == "--embedded" {
            source_preference = SourcePreference::Embedded;
        } else if arg == "--external" {
            source_preference = SourcePreference::External;
        } else if arg == "--port" {
            i += 1;
            let value = args.get(i).ok_or_else(|| format!("{}: --port requires a value", display_program(program)))?;
            port = Some(value.clone());
        } else if arg.to_string_lossy().starts_with("--port=") {
            port = Some(OsString::from(arg.to_string_lossy()[7..].to_string()));
        } else if arg.to_string_lossy().starts_with('-') {
            return Err(format!("{}: unknown option {}", display_program(program), arg.to_string_lossy()));
        } else if input.is_some() {
            return Err("expected at most one positional argument: a firmware .bin path or a directory to search".into());
        } else {
            input = Some(PathBuf::from(arg));
        }
        i += 1;
    }

    Ok(Options {
        source_preference,
        port,
        input,
    })
}

fn display_program(program: &OsString) -> &str {
    Path::new(program).file_name().and_then(|name| name.to_str()).unwrap_or("flash")
}

fn resolve_firmware_source(input: Option<&Path>, preference: &SourcePreference) -> Result<FirmwareSource, String> {
    match (preference, input) {
        (SourcePreference::Embedded, Some(path)) if path != Path::new("--help") => {
            Err("cannot combine --embedded with an explicit firmware path".into())
        }
        (SourcePreference::DefaultEmbedded, Some(path)) => {
            if path.is_file() {
                Ok(FirmwareSource::File(path.to_path_buf()))
            } else {
                Ok(FirmwareSource::File(resolve_firmware_in_directory(path)?))
            }
        }
        (SourcePreference::Embedded, _) => {
            if let (Some(bytes), Some(name)) = (EMBEDDED_FIRMWARE, EMBEDDED_FIRMWARE_NAME) {
                Ok(FirmwareSource::Embedded { bytes, name })
            } else {
                Err("--embedded was requested, but this flasher was built without embedded firmware".into())
            }
        }
        (SourcePreference::External, Some(path)) => {
            if path.is_file() {
                Ok(FirmwareSource::File(path.to_path_buf()))
            } else {
                Ok(FirmwareSource::File(resolve_firmware_in_directory(path)?))
            }
        }
        (SourcePreference::External, None) => Ok(FirmwareSource::File(resolve_firmware_in_directory(Path::new("."))?)),
        (SourcePreference::DefaultEmbedded, None) => {
            if let (Some(bytes), Some(name)) = (EMBEDDED_FIRMWARE, EMBEDDED_FIRMWARE_NAME) {
                Ok(FirmwareSource::Embedded { bytes, name })
            } else {
                Err("no embedded firmware found in this flasher build; pass --external [firmware.bin|directory]".into())
            }
        }
    }
}

fn resolve_firmware_in_directory(input: &Path) -> Result<PathBuf, String> {
    if input.is_file() {
        return Ok(input.to_path_buf());
    }

    let dir = if input.as_os_str().is_empty() { Path::new(".") } else { input };
    if !dir.is_dir() {
        return Err(format!("firmware path does not exist: {}", dir.display()));
    }

    let mut bins = Vec::new();
    let entries = fs::read_dir(dir)
        .map_err(|err| format!("failed to read {}: {err}", dir.display()))?;

    for entry in entries {
        let entry = entry.map_err(|err| format!("failed to read directory entry: {err}"))?;
        let path = entry.path();
        if path.is_file() && path.extension().and_then(|ext| ext.to_str()) == Some("bin") {
            bins.push(path);
        }
    }

    bins.sort();

    match bins.len() {
        1 => Ok(bins.remove(0)),
        0 => Err(format!("no .bin firmware files found in {}", dir.display())),
        _ => Err(format!(
            "multiple .bin firmware files found in {}: {}",
            dir.display(),
            bins.iter()
                .map(|path| path.file_name().and_then(|name| name.to_str()).unwrap_or("<invalid utf-8>").to_string())
                .collect::<Vec<_>>()
                .join(", ")
        )),
    }
}

fn resolve_espflash_source() -> Result<EspflashSource, String> {
    if let (Some(bytes), Some(name)) = (EMBEDDED_ESPFLASH, EMBEDDED_ESPFLASH_NAME) {
        Ok(EspflashSource::Embedded { bytes, name })
    } else {
        Ok(EspflashSource::Path(PathBuf::from("espflash")))
    }
}

fn run_espflash(
    source: &FirmwareSource,
    espflash_source: &EspflashSource,
    explicit_port: Option<OsString>,
) -> Result<std::process::ExitStatus, String> {
    let mut cleanup_paths: Vec<PathBuf> = Vec::new();
    let firmware_temp;
    let espflash_temp;

    let firmware_path = match source {
        FirmwareSource::File(path) => path.as_path(),
        FirmwareSource::Embedded { bytes, name } => {
            firmware_temp = write_temp_file(bytes, name, false)?;
            cleanup_paths.push(firmware_temp.clone());
            &firmware_temp
        }
    };

    let espflash_path = match espflash_source {
        EspflashSource::Path(path) => path.as_path(),
        EspflashSource::Embedded { bytes, name } => {
            espflash_temp = write_temp_file(bytes, name, true)?;
            cleanup_paths.push(espflash_temp.clone());
            &espflash_temp
        }
    };

    let port = match resolve_port(espflash_path, explicit_port) {
        Ok(port) => port,
        Err(err) => {
            cleanup_temp_files(&cleanup_paths);
            return Err(err);
        }
    };

    let mut command = Command::new(espflash_path);
    command.arg("write-bin").arg("--non-interactive");

    command.arg("--port").arg(&port);

    command.arg("0x0").arg(firmware_path);

    let status = command
        .status()
        .map_err(|err| {
            cleanup_temp_files(&cleanup_paths);
            format!("failed to launch {}: {err}", espflash_path.display())
        })?;

    if !status.success() {
        cleanup_temp_files(&cleanup_paths);
        return Err(format!("espflash failed while writing {}", firmware_path.display()));
    }

    cleanup_temp_files(&cleanup_paths);

    Ok(status)
}

fn resolve_port(espflash_path: &Path, explicit_port: Option<OsString>) -> Result<OsString, String> {
    if let Some(port) = explicit_port {
        if !port.is_empty() {
            return Ok(port);
        }
    }

    let ports = list_ports(espflash_path)?;
    match ports.len() {
        1 => Ok(OsString::from(&ports[0])),
        0 => Err("no serial ports detected; connect the board or set ESPFLASH_PORT/--port".into()),
        _ => Err(format!(
            "multiple serial ports detected: {}. Set ESPFLASH_PORT or pass --port.",
            ports.join(", ")
        )),
    }
}

fn list_ports(espflash_path: &Path) -> Result<Vec<String>, String> {
    let output = Command::new(espflash_path)
        .arg("list-ports")
        .output()
        .map_err(|err| format!("failed to run {} list-ports: {err}", espflash_path.display()))?;

    if !output.status.success() {
        let mut stderr = String::new();
        stderr.push_str(&String::from_utf8_lossy(&output.stderr));
        return Err(format!("espflash list-ports failed: {}", stderr.trim()));
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    let mut ports = Vec::new();
    for line in stdout.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        if let Some(first) = trimmed.split_whitespace().next() {
            ports.push(first.to_string());
        }
    }
    Ok(ports)
}

fn write_temp_file(bytes: &[u8], name: &str, executable: bool) -> Result<PathBuf, String> {
    let mut path = env::temp_dir();
    let pid = std::process::id();
    path.push(format!("openpuck-embedded-{pid}-{name}"));

    let mut file = File::create(&path)
        .map_err(|err| format!("failed to create temp firmware {}: {err}", path.display()))?;
    file.write_all(bytes)
        .map_err(|err| format!("failed to write temp firmware {}: {err}", path.display()))?;
    file.flush()
        .map_err(|err| format!("failed to flush temp firmware {}: {err}", path.display()))?;

    if executable {
        make_executable(&path)?;
    }

    Ok(path)
}

fn cleanup_temp_files(paths: &[PathBuf]) {
    for path in paths {
        let _ = fs::remove_file(path);
    }
}

#[cfg(unix)]
fn make_executable(path: &Path) -> Result<(), String> {
    use std::os::unix::fs::PermissionsExt;

    let mut permissions = fs::metadata(path)
        .map_err(|err| format!("failed to read metadata for {}: {err}", path.display()))?
        .permissions();
    permissions.set_mode(0o755);
    fs::set_permissions(path, permissions)
        .map_err(|err| format!("failed to set permissions on {}: {err}", path.display()))
}

#[cfg(not(unix))]
fn make_executable(_path: &Path) -> Result<(), String> {
    Ok(())
}