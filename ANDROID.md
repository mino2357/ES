# Android Build Notes

The Android target runs the same GUI simulator as desktop.
There is no separate CLI path and there is no audio backend to configure.

## Related Documents

- [README.md](README.md): repository overview and documentation map
- [docs/USER_MANUAL.md](docs/USER_MANUAL.md): GUI operation and configuration usage
- [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md): mathematical model and implementation mapping

## Requirements

- Rust target for Android, for example `aarch64-linux-android`
- Android SDK / NDK
- `cargo-apk`

## Typical Setup

```bash
rustup target add aarch64-linux-android
cargo install cargo-apk
```

Set the usual Android environment variables for your SDK and NDK before building.

## Run

```bash
cargo apk run --lib --release --target aarch64-linux-android
```

## Config

Android currently boots from the checked-in [config/sim.yaml](config/sim.yaml) embedded in the app package.
That keeps the mobile path aligned with the desktop GUI solver.
