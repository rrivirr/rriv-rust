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
use std::process::Command;


fn main() {
    println!("running datalogger build.rs");

    let branch = Command::new("git")
        .arg("rev-parse")
        .arg("--abbrev-ref")
        .arg("HEAD")
        .output()
        .expect("Failed to get git branch");
    let branch_name = String::from_utf8_lossy(&branch.stdout).trim().to_string();


    let gitref = Command::new("git")
        .arg("rev-parse")
        .arg("--short")
        .arg("HEAD")
        .output()
        .expect("Failed to get git branch");
    let ref_name = String::from_utf8_lossy(&gitref.stdout).trim().to_string();

    println!("cargo:rustc-env=GIT_BRANCH={}", branch_name);
    println!("cargo:rustc-env=GIT_REF={}", ref_name);


}
