mod audio;
mod config;
mod constants;
mod dashboard;
mod simulator;

use config::load_config;
use constants::DEFAULT_CONFIG_PATH;

fn main() -> eframe::Result {
    // Keep the binary entry point thin so configuration and UI own the app behavior.
    let config = load_config(DEFAULT_CONFIG_PATH);
    dashboard::run_app(config)
}
