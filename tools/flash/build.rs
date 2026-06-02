use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=OPENPUCK_EMBED_FIRMWARE");

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR not set"));
    let generated = out_dir.join("embedded_firmware.rs");

    let firmware_path = env::var_os("OPENPUCK_EMBED_FIRMWARE").map(PathBuf::from);

    let firmware_rust = generate_embedded("EMBEDDED_FIRMWARE", "EMBEDDED_FIRMWARE_NAME", firmware_path.as_deref(), &out_dir);

    fs::write(&generated, firmware_rust).expect("failed to write generated source");
}

fn generate_embedded(bytes_name: &str, file_name_const: &str, source: Option<&Path>, out_dir: &Path) -> String {
    match source {
        Some(path) => {
            println!("cargo:rerun-if-changed={}", path.display());

            let bytes = fs::read(path)
                .unwrap_or_else(|err| panic!("failed to read {}: {err}", path.display()));

            let extension = path.extension().and_then(|ext| ext.to_str()).unwrap_or("bin");
            let copied = out_dir.join(format!("{}.{}", bytes_name.to_ascii_lowercase(), extension));

            fs::write(&copied, bytes).unwrap_or_else(|err| {
                panic!("failed to copy {} to {}: {err}", path.display(), copied.display())
            });

            let file_name = path
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or("embedded.bin");

            format!(
                "const {bytes_name}: Option<&[u8]> = Some(include_bytes!(r#\"{}\"#));\nconst {file_name_const}: Option<&str> = Some({:?});\n",
                copied.display(),
                file_name,
            )
        }
        None => format!(
            "const {bytes_name}: Option<&[u8]> = None;\nconst {file_name_const}: Option<&str> = None;\n"
        ),
    }
}