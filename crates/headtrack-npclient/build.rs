fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    // Pass the .def file to the mingw linker so exports get correct ordinals
    // and DllMain is NOT exported (it's an entry point, not a public export).
    println!("cargo:rustc-cdylib-link-arg={manifest_dir}/NPClient64.def");
}
