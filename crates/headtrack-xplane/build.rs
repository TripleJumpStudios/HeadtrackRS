fn main() {
    // XPLM symbols are provided by X-Plane at plugin load time — not at
    // compile time.  Allow unresolved references in the cdylib output.
    println!("cargo:rustc-link-arg=-Wl,--allow-shlib-undefined");
    println!("cargo:rerun-if-env-changed=XPLANE_SDK_PATH");
}
