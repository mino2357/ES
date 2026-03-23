mod cli;
mod config;
mod constants;
mod simulator;

pub fn run_cli() -> Result<(), String> {
    cli::run(std::env::args().collect())
}
