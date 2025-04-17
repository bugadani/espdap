espdap
======

A CMSIS-DAP probe running on ESP32-S2 and ESP32-S3 devkits, built using [dap-rs].

The probe implements both SWD and JTAG interfaces.

The firmware uses the [bitbang-dap] crate that acts as an
adapter between dap-rs and the hardware. The hardware then only needs to implement a bidirectional
GPIO driver, and a cycle-resolution delay method.

Originally based on https://github.com/embassy-rs/eprobe/

[bitbang-dap]: https://github.com/bugadani/bitbang-dap
[dap-rs]: https://crates.io/crates/dap-rs
