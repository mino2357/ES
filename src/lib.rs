#[cfg(feature = "gui")]
mod audio;
#[cfg(feature = "gui")]
mod config;
#[cfg(feature = "gui")]
mod constants;
#[cfg(feature = "gui")]
mod dashboard;
#[cfg(feature = "gui")]
mod simulator;

#[cfg(feature = "gui")]
use config::load_config;
#[cfg(feature = "gui")]
use constants::DEFAULT_CONFIG_PATH;

#[cfg(feature = "gui")]
pub fn launch_desktop() -> eframe::Result {
    let config = load_config(DEFAULT_CONFIG_PATH);
    dashboard::run_app(config)
}

#[cfg(all(feature = "gui", target_os = "android"))]
#[unsafe(no_mangle)]
fn android_main(app: winit::platform::android::activity::AndroidApp) {
    // Android APK assets are not regular desktop files, so we boot from the embedded YAML.
    let config = load_config(DEFAULT_CONFIG_PATH);
    if let Err(err) = dashboard::run_android_app(config, app) {
        eprintln!("Android launch failed: {err}");
    }
}
