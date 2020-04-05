//! This program checks to see if there are any valid release or debug builds of the plugin
//! If successful, it copies the .dll files into the Plugins directory of SmodeSDK/Plugins
//!
//! The program takes an optional command line argument which specifies the Path to the SmodeSDK
//! folder located on you system. If no argument is supplied, it assumes your SmodeSDK folder
//! is located at C:\Program Files (x86)\SmodeTech\Smode SDK
use std::env;
use std::fs;
use std::path::Path;

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // The path where the users Smode SDK folder is installed
    let smode_sdk_dir = if args.len() > 1 {
        Path::new(&args[1])
    } else {
        Path::new("C:\\Program Files (x86)\\SmodeTech\\Smode SDK")
    };

    if !smode_sdk_dir.join("Plugins").exists() {
        panic!("couldn't find Smode SDK/Plugins folder");
    }

    let smode_plugins_dir = smode_sdk_dir.join("Plugins");

    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    // This is the folder where our .h file lives that we need to copy
    let plugin_build_dir = Path::new(manifest_dir)
        .parent()
        .unwrap()
        .join("plugin")
        .join("build")
        .join("Visual Studio 15 2017");

    let release_build = Path::new(&plugin_build_dir)
        .join("Release")
        .join("Laser.dll");
    if !release_build.exists() {
        println!("Can not find a release build to copy!");
    } else {
        fs::copy(release_build, smode_plugins_dir.join("Laser.dll")).unwrap();
    }

    let debug_build = Path::new(&plugin_build_dir)
        .join("Debug")
        .join("Laser-debug.dll");
    if !debug_build.exists() {
        println!("Can not find a debug build to copy!");
    } else {
        fs::copy(debug_build, smode_plugins_dir.join("Laser-debug.dll")).unwrap();
    }
}
