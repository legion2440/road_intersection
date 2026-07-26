use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn main() {
    let target = env::var("TARGET").unwrap_or_default();
    // vendor/sdl2 contains the MSVC import library and runtime, not a MinGW
    // import library. Reject that ABI explicitly so the build fails with the
    // supported-platform policy instead of an unrelated linker error.
    if target.ends_with("-pc-windows-gnu") {
        panic!("Windows MSVC is supported; Windows GNU/MinGW is intentionally unsupported");
    }
    // Other supported targets resolve SDL2 through their platform toolchain;
    // only Windows x64 MSVC consumes the vendored import library and DLL.
    if target != "x86_64-pc-windows-msvc" {
        return;
    }

    println!("cargo:rerun-if-changed=vendor/sdl2/lib/x64/SDL2.lib");
    println!("cargo:rerun-if-changed=vendor/sdl2/lib/x64/SDL2.dll");

    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let library_dir = manifest.join("vendor/sdl2/lib/x64");
    println!("cargo:rustc-link-search=native={}", library_dir.display());

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let profile_dir = out_dir
        .ancestors()
        .nth(3)
        .expect("Cargo OUT_DIR must be inside target/<profile>/build");
    copy_dll(&library_dir.join("SDL2.dll"), profile_dir);
    copy_dll(&library_dir.join("SDL2.dll"), &profile_dir.join("deps"));
}

fn copy_dll(source: &Path, destination_dir: &Path) {
    fs::create_dir_all(destination_dir).expect("failed to create target directory");
    let destination = destination_dir.join("SDL2.dll");
    if fs::read(source).ok() == fs::read(&destination).ok() {
        return;
    }
    fs::copy(source, destination).expect("failed to copy vendored SDL2.dll");
}
