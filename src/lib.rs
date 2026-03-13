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
