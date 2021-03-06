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
   cd smode_laser_plugin/smode_laser
   ```

4. Build the library with optimisations enabled:
   ```
   cargo build --release
   ```
   Note that the first `cargo build` might take a while as cargo fetches the
   registry, the necessary dependencies and builds the library from scratch.

If all went well, we should now have our static library at
`target/release/libsmode_laser.<ext>` where `<ext>` is the static library
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
   cbindgen --crate smode_laser --output smode_laser.h
   ```

We should now have a `smode_laser.h` file containing all the necessary
extern declarations for calling into our static library! We are now ready to
include our static library and header in our plugin.

*Please note that the `smode_laser.h` file may already exist within the
repository. However, we should still run the commands above to ensure that it is
up-to-date with the latest version of `smode_laser` and its dependencies.*

## Generating Documentation

You can generate the API reference documentation for the library with the
following:

```
cargo doc --open
```

This will compile the documentation for the library and open it with the
system's default web browser.

## Updating dependencies

Occasionally, new versions of upstream dependencies are released with various
fixes, improvements and/or optimisations that we might want to take advantage
of. To update to the most recent, non-breaking patch release for each
dependency, we can run the following:

```
cargo update
```

For example, if we are currently using `nannou_laser` version 0.14.0, but
version 0.14.2 was recently released and we want to use it, we can do so by
running `cargo update`.

We can find out if a new version of a dependency is released by using `cargo
search`. For example, to search for the latest version of `nannou_laser`, we can
run the following:

```
cargo search nannou_laser
```

In the case that `nannou_laser` is updated to version `0.15.0`, we must also
update the dependency version within the `Cargo.toml` file under the
`dependencies` section. However, note that a jump in the major version like this
indicates a breaking change. Breaking changes may require updating some of the
code within `smode_laser` and in turn the plugin that depends on it.

Details about changes to the `nannou_laser` dependency between different
versions can be found at [the nannou changelog][4]. This log can be invaluable
for determining how to update `smode_laser` for a new breaking change. This
changelog also includes changes to other libraries within the nannou ecosystem,
so it may be worth searching the page for "laser" to find relevant entries.

Once dependencies have been updated, be sure to follow the steps above to
rebuild the library and re-generate the C header.

[1]: https://www.rust-lang.org/tools/install
[2]: https://www.rust-lang.org/learn
[3]: https://github.com/eqrion/cbindgen
[4]: https://guide.nannou.cc/changelog.html
