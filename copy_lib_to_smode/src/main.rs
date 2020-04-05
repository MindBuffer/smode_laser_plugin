//! This program copies the necessary header and lib files into 
//! a folder within the Smode SDK folder. The folder created will be
//! at SmodeSDK\include\exern\nannou_laser
//! 
//! The program takes an optional command line argument which specifies the Path to the SmodeSDK
//! folder located on you system. If no argument is supplied, it assumes your SmodeSDK folder
//! is located at C:\Program Files (x86)\SmodeTech\Smode SDK
use std::path::Path;
use std::env;
use std::fs;

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // The path where the users Smode SDK folder is installed
    let smode_sdk_dir = if args.len() > 1 {
        Path::new(&args[1])
    } else {
        Path::new("C:\\Program Files (x86)\\SmodeTech\\Smode SDK")
    };

    if !smode_sdk_dir.join("include").join("extern").exists() {
        panic!("couldn't find Smode SDK/include/extern folder");
    }

    let smode_nannou_laser_dir = smode_sdk_dir.join("include").join("extern").join("nannou_laser");

    // Create a nannou_laser folder inside of /include/extern/ if it doesn't exsit
    if !smode_nannou_laser_dir.exists() {
        fs::create_dir(&smode_nannou_laser_dir).unwrap();
    }
    
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    // This is the folder where our .h file lives that we need to copy
    let plugin_dir = Path::new(manifest_dir).parent().unwrap().join("smode_laser_plugin");
    // Here is the folder for our .lib file
    let lib_dir = plugin_dir.join("target").join("release");
    
    fs::copy(plugin_dir.join("smode_laser_plugin.h"), smode_nannou_laser_dir.join("smode_laser_plugin.h")).unwrap();
    fs::copy(lib_dir.join("smode_laser_plugin.lib"), smode_nannou_laser_dir.join("smode_laser_plugin.lib")).unwrap();

    println!("successfully copied header and library to Smode SDK")
}
