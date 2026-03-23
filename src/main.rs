fn main() {
    if let Err(err) = es_sim::run_cli() {
        eprintln!("{err}");
        std::process::exit(1);
    }
}
