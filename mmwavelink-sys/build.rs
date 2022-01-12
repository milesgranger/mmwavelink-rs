use bindgen;
use std::env;
use std::path::PathBuf;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn main() -> Result<()> {

    let _name = "libmmwavelink";
    let header = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("ti/control/mmwavelink/mmwavelink.h");
    let includes = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    
    //println!("cargo:rustc-link-search={}", &lib.display());
    //println!("cargo:rustc-link-lib=static={}", name);
    println!("cargo:rerun-if-changed={}", header.display());
    
    let bindings = bindgen::Builder::default()
        .header(header.to_str().unwrap())
        .clang_arg(format!("-I{}", &includes.display()))
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .use_core()
        .ctypes_prefix("cty")
        .generate()
        .expect("Failed to build bindings");

    let out_path = PathBuf::from(&format!("{}/src/ffi.rs", env!("CARGO_MANIFEST_DIR")));
    bindings.write_to_file(out_path)?;
    Ok(())
}
