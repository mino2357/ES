#[path = "../hp/mod.rs"]
mod hp;

fn main() {
    if let Err(err) = hp::cli::run_from_env("es_hp") {
        eprintln!("es_hp error: {err}");
        std::process::exit(1);
    }
}
