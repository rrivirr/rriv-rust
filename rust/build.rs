//! This copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn macos_copy_memory_x() {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");
}

fn main() {
    println!("build.rs");
    macos_copy_memory_x();
    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR")
        .expect("CARGO_MANIFEST_DIR env var is not defined"));
    println!("CARGO_MANIFEST_DIR: {:?}", crate_dir);

    let out_dir = PathBuf::from(env::var("OUT_DIR")
        .expect("OUT_DIR env var is not defined"));
    println!("OUT_DIR: {:?}", out_dir);

    let config = cbindgen::Config::from_file("cbindgen.toml")
        .expect("Unable to find cbindgen.toml configuration file");

    // OUT_DIR doesn't appear to be configurable, so prolly not a good destination
    // cargo +nightly build --out-dir `pwd` -Z unstable-options
    // added question to this issue: https://github.com/rust-lang/cargo/issues/6790
    // for now, CARGO_MANIFEST_DIR (crate_dir) seems reasonable

    cbindgen::generate_with_config(&crate_dir, config)
        .unwrap()
        .write_to_file(crate_dir.join("rust_serial.h"));
}