# smode_laser_plugin [![Build Status](https://github.com/mindbuffer/smode_laser_plugin/workflows/smode_laser_plugin/badge.svg)](https://github.com/mindbuffer/smode_laser_plugin/actions)

Smode plugin for sending streams to LASER projectors.

## Building the plugin

1. Build a static library and C header from the Rust library. See the Rust
   `smode_laser` library README.md [here][1] for more details on how to achieve
   this.
2. Build the plugin project file using the batch script. There are 3 included 
   .bat files inside the plugin/ folder. Double click on the .bat file that 
   matches the version of Visual Studio you have installed on your system. 
   This will create a build/ folder which contains a visual studio solution. 
3. Build the plugin itself using the generated visual studio solution. To do so, 
   navigate to build/Visual Studio xx/ and open the Laser.sln file. Next, in the 
   Solution Configuration drop down menu, select to build in either Debug or Release 
   mode. Finally, right click on the SMODE PLUGIN Laser in the Solution Explorer 
   and click Build.
4. Install the plugin within the plugins directory of your Smode SDK. There will
   be either a new folder call Debug or Release created that contains the plugin.
   For debug, copy the file Laser-debug.dll ... For release, copy the file
   Laser.dll. Navigate to the SmodeSDK folder and paste the .dll file within the
   plugins/ folder. 

## The Ether Dream DAC Emulator

Nannou's [`ether-dream` repository][2] provides a small [DAC emulation
library][3]. The examples provided with this library are quite useful for
testing DAC streaming in the case that you do not have an Ether Dream available.

To run the DAC emulator examples, follow these steps:

1. Make sure you have installed Rust as described in the `smode_laser` Rust
   library steps [here][1].
2. Clone the ether-dream repository somewhere and `cd` into it:
   ```
   git clone https://github.com/nannou-org/ether-dream
   cd ether-dream
   ```
3. Build and run one of the examples:
   ```
   cargo run --release --example dac_emulator_default
   ```
   or
   ```
   cargo run --release --example dac_emulator_visualiser
   ```

The examples may take a while to build as the visualisation requires the
`nannou` library which cargo will build from scratch. If you run into any build
issues, you may wish to check you have the necessary system requirements as
mentioned within the relevant section of [the nannou guide][4].

Please keep in mind that the visualisation itself is currently very basic and
may not accurately reflect the behaviour of a LASER projector controlled by an
Ether Dream DAC. For example, there is no inertia simulation, luminance
attenuation based on scanner speed, or individual point accentuation. On top of
this, the behaviour of each projector will differ based on their own hardware
limitations.

### Running multiple emulators

Running multiple emulators can be useful for testing communication between more
than one DAC in the case that multiple physical DACs are unavailable. However,
doing so will require writing a little extra code in the example. Within the
`ether-dream/dac-emulator/examples/dac_emulator_default.rs` example, we can see
that the first line within the main function creates a default DAC description:
```rust
    let dac_description = Default::default();
```
If we wish to run a second DAC emulator on the same network, we'll need to
change its MAC address by making the `dac_description` mutable and assigning the
field like so:
```rust
    let mut dac_description = Default::default();
    dac_description.mac_address = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55].into();
```

The default MAC address can be found [here][5].

[1]: https://github.com/MindBuffer/smode_laser_plugin/tree/master/smode_laser
[2]: https://github.com/nannou-org/ether-dream
[3]: https://github.com/nannou-org/ether-dream/tree/master/dac-emulator
[4]: https://guide.nannou.cc/getting_started/platform-specific_setup.html
[5]: https://github.com/nannou-org/ether-dream/blob/0153de15c82fe351384f5b5f55a41e5006aa404c/dac-emulator/src/lib.rs#L27
