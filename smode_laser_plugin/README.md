# smode_laser_plugin (Rust library)

A small library extending the `nannou_laser` library with some SMODE-specific
requirements. We use this library within our C++ SMODE plugin by compiling it to
a static library, generating a C header using `cbindgen` and linking to the
static library using the traditional SMODE approach. These steps are explained
in more detail below.

## Building the library

We can build this library from scratch by following these steps:

1. Install Rust. Follow the instructions [here][1]. If you are new to Rust, you
   can learn more about the language [here][2], however this is not necessary
   for the following steps.

2. Clone the repository:
   ```
   git clone https://github.com/MindBuffer/smode_laser_plugin.git
   ```

3. Change to this library's directory:
   ```
   cd smode_laser_plugin/smode_laser_plugin
   ```

4. Build the library with optimisations enabled:
   ```
   cargo build --release
   ```
   Note that the first `cargo build` might take a while as cargo fetches the
   registry, the necessary dependencies and builds the library from scratch.

If all went well, we should now have our static library at
`target/release/libsmode_laser_plugin.<ext>` where `<ext>` is the static library
extension for your platform.

## Generating the C header

This library along with the `nannou_laser` and `ether-dream` dependencies expose
a C-ABI compatible API using Rust's foreign function interface. In order to call
into these functions from C or C++ code, we require a header file with a set of
extern function declarations that exactly match those exposed by our Rust code.
Rather than writing this header manually, we can generate it using a tool called
[cbindgen][3]. This tool creates C/C++11 headers for Rust libraries which expose
a public C API. Let's generate our header:

1. Install cbindgen:
   ```
   cargo install cbindgen
   ```

2. Generate the bindings:
   ```
   cbindgen --crate smode_laser_plugin --output smode_laser_plugin.h
   ```

We should now have a `smode_laser_plugin.h` file containing all the necessary
extern declarations for calling into our static library! We are now ready to
include our static library and header in our plugin.

[1]: https://www.rust-lang.org/tools/install
[2]: https://www.rust-lang.org/learn
[3]: https://github.com/eqrion/cbindgen
