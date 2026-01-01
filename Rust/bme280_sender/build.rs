use std::env;
use std::fs;
use std::path::Path;

fn main() {
    embuild::espidf::sysenv::output();

    // Read .env file from repo root (3 levels up from this project)
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let env_path = Path::new(&manifest_dir).join("../..").join(".env");

    if env_path.exists() {
        let contents = fs::read_to_string(&env_path).expect("Failed to read .env file");
        for line in contents.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if let Some((key, value)) = line.split_once('=') {
                println!("cargo:rustc-env={}={}", key.trim(), value.trim());
            }
        }
    } else {
        panic!("Missing .env file at {:?}. Create it with WIFI_SSID, WIFI_PASSWORD, API_ENDPOINT, and DEVICE_ID.", env_path);
    }

    // Compile protobuf
    let proto_path = Path::new(&manifest_dir).join("proto/sensor_reading.proto");
    if proto_path.exists() {
        prost_build::compile_protos(&[&proto_path], &[Path::new(&manifest_dir).join("proto")])
            .expect("Failed to compile protos");
    }

    println!("cargo:rerun-if-changed=proto/sensor_reading.proto");
    println!("cargo:rerun-if-changed=../../../.env");
}
