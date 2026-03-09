# Android Build Notes

## Current Status

`es_sim` can now be built as an Android shared library and launched through `cargo apk`.
The Android path uses the same `egui` dashboard and reduced-order simulator as desktop.

## Terms Used Here

This note is written to stand on its own.

- `shared library`: the Android-loadable native library produced from the Rust crate through `cdylib`.
- `cargo apk`: the Rust mobile packaging tool that builds, signs, installs, and can launch the Android APK from Cargo metadata.
- `APK`: the Android application package installed on the device.
- `embedded config`: `config/sim.yaml` compiled into the binary with `include_str!` instead of being loaded from device storage at runtime.
- `Android target`: the Rust compilation target triple, currently `aarch64-linux-android` in this repository.

What changed in the code:

- `src/lib.rs`: adds the shared-library entry point and `android_main`
- `src/dashboard.rs`: adds the Android `eframe` launcher
- `src/config.rs`: loads an embedded copy of `config/sim.yaml` on Android
- `Cargo.toml`: adds `cdylib`, Android metadata, and the Android `winit` dependency

## What Works

- same realtime dashboard as desktop
- same simulator core
- same checked-in `config/sim.yaml` values, embedded at build time

## Current Limits

- runtime config editing from device storage is not implemented
- `sim.yaml` beside the executable is a desktop-only flow
- audio may fall back to OFF if the device stream cannot be opened
- this repository currently prepares `aarch64-linux-android` only

## Host Setup

Install these on the build machine:

1. Rust Android target
2. Android SDK
3. Android NDK
4. JDK
5. `cargo-apk`

Typical commands:

```bash
rustup target add aarch64-linux-android
cargo install cargo-apk
```

Set the Android toolchain paths in your shell:

```bash
export ANDROID_SDK_ROOT=/path/to/Android/Sdk
export ANDROID_NDK_ROOT=/path/to/Android/Sdk/ndk/<version>
```

On Windows PowerShell:

```powershell
$env:ANDROID_SDK_ROOT = "C:\Users\<you>\AppData\Local\Android\Sdk"
$env:ANDROID_NDK_ROOT = "C:\Users\<you>\AppData\Local\Android\Sdk\ndk\<version>"
```

## Run On Device

USB debugging must be enabled on the phone and `adb devices` must show the device.

Debug build and install:

```bash
cargo apk run --lib --target aarch64-linux-android
```

Release build and install:

```bash
cargo apk run --lib --release --target aarch64-linux-android
```

Build the APK without installing:

```bash
cargo apk build --lib --release --target aarch64-linux-android
```

## Config Behavior On Android

Android does not currently look for an external `sim.yaml`.
Instead, `src/config.rs` parses the checked-in `config/sim.yaml` through `include_str!`, so the APK always boots with the same baseline calibration that is committed in the repository.

If you want to change calibration on Android today, edit `config/sim.yaml` and rebuild the APK.

## Recommended Next Step

If Android becomes a real deployment target, the next practical improvement is a small import/export path for YAML and bench CSV files using app storage instead of desktop-style relative paths.
