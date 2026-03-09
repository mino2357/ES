#[path = "../hp/mod.rs"]
mod hp;

fn main() {
    if let Err(err) = hp::cli::run_from_env("es_cli") {
        eprintln!("es_cli error: {err}");
        std::process::exit(1);
    }
}
