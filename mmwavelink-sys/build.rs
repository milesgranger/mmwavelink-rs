use bindgen;
use cc;
use std::env;
use std::path::PathBuf;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn main() -> Result<()> {
    let lib_name = compile()?;

    let includes = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let header = includes.join("ti/control/mmwavelink/mmwavelink.h");
    let out_dir = env::var_os("OUT_DIR").unwrap();

    println!("cargo:rustc-link-search={}", out_dir.to_str().unwrap());
    println!("cargo:rustc-link-lib=static={}", lib_name);
    //println!("cargo:rerun-if-changed={}", header.display());

    let bindings = bindgen::Builder::default()
        .header(header.to_str().unwrap())
        .clang_arg(format!("-I{}", &includes.display()))
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .use_core()
        .ctypes_prefix("cty")
        .derive_default(true)
        .derive_copy(true)
        .derive_debug(true)
        .generate()
        .expect("Failed to build bindings");

    let out_path = PathBuf::from(&format!("{}/src/ffi.rs", env!("CARGO_MANIFEST_DIR")));
    bindings.write_to_file(out_path)?;

    Ok(())
}

// Compile mmwavelink into static lib
fn compile() -> Result<&'static str> {
    let src = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("ti/control/mmwavelink/src");
    let includes = PathBuf::from(env!("CARGO_MANIFEST_DIR"));

    let rl_sensor = src.join("rl_sensor.c");
    let rl_device = src.join("rl_device.c");
    let rl_driver = src.join("rl_driver.c");
    let rl_monitoring = src.join("rl_monitoring.c");
    let rl_controller = src.join("rl_controller.c");

    cc::Build::new()
        .files(&[
            rl_sensor,
            rl_device,
            rl_driver,
            rl_monitoring,
            rl_controller,
        ])
        .include(includes)
        .static_flag(true)
        .compile("mmwavelink");

    Ok("mmwavelink")
}
