# Dev environment setup

## required tools

- `sudo pacman -S --needed gcc git make flex bison gperf python cmake ninja ccache dfu-util 
libusb python-virtualenv libxml2-legacy`
- `cargo install espup`
- `cargo install ldproxy`
- `cargo install espflash`
- `cargo install cargo-espflash`
- `espup install`

## config

- `cp config.rs.example config.rs`

## build and deploy

- `. ~/export-esp.sh` (once per terminal)
- `cargo +esp check`
- `cargo +esp espflash flash --no-stub --monitor` or `cargo run` as toolchain is set in `rust-toolchain.toml` and runner
  is set in `.cargo/config.toml`

## Tips

if espflash fails with `assertion failed: termios2.c_ospeed == termios2.c_ispeed`, try setting the baudrate manually
before running `cargo run` with  `stty -F /dev/ttyUSB0 115200`