//! This library acts as a small extension to the `nannou_laser` library, providing a small set of
//! SMODE-specific items as they are required during development of the plugin.
//!
//! We use this library within our C++ SMODE plugin by compiling it to a static library, generating
//! a C header using `cbindgen` and linking to the static library using the traditional SMODE
//! approach. Please see the `README.md` for a detailed guide.
//!
//! This library re-exports the `nannou_laser` library in its entirety.

#[doc(inline)]
pub use nannou_laser::*;

// TODO: Add SMODE specific items here.
#[no_mangle]
pub extern "C" fn rust_test_func() -> u32 {
    27302
}
