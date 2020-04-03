# smode_laser_plugin [![Build Status](https://github.com/mindbuffer/smode_laser_plugin/workflows/smode_laser_plugin/badge.svg)](https://github.com/mindbuffer/smode_laser_plugin/actions)

Smode plugin for sending streams to LASER projectors.

## Building the plugin

1. Build a static library and C header from the Rust library. See the Rust
   library README.md [here][1] for more details on how to achieve this.
2. Include the generated header within the plugin XML.
3. Link to the built static library.
4. Build the plugin.
5. Install the plugin.

*TODO: Let Josh update the steps above with more details.*

*TODO: See if we can automate the static library and header generation and
updating the XML.*

[1]: https://github.com/MindBuffer/smode_laser_plugin/tree/master/smode_laser_plugin
