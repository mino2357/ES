pub(super) const APP_NAME: &str = "ES";

pub(super) fn window_title() -> String {
    format!("{} [{}]", APP_NAME, short_badge())
}

pub(super) fn header_badge() -> String {
    format!(
        "BUILD v{} | {} | {} | b{} | {}",
        env!("CARGO_PKG_VERSION"),
        profile_short(),
        git_sha(),
        build_id(),
        layout_revision(),
    )
}

pub(super) fn layout_revision() -> &'static str {
    option_env!("ES_LAYOUT_REV").unwrap_or("layout-unknown")
}

pub(super) fn cache_identity() -> String {
    format!(
        "v{}|{}|{}|b{}|{}",
        env!("CARGO_PKG_VERSION"),
        profile_short(),
        git_sha(),
        build_id(),
        layout_revision(),
    )
}

fn short_badge() -> String {
    format!(
        "v{} {} {} b{} {}",
        env!("CARGO_PKG_VERSION"),
        profile_short(),
        git_sha(),
        build_id(),
        layout_revision(),
    )
}

fn build_id() -> &'static str {
    option_env!("ES_BUILD_ID").unwrap_or("no-build-id")
}

fn git_sha() -> &'static str {
    option_env!("ES_GIT_SHA").unwrap_or("nogit")
}

fn profile_short() -> &'static str {
    match option_env!("ES_BUILD_PROFILE").unwrap_or("unknown") {
        "release" => "rel",
        "debug" => "dbg",
        other => other,
    }
}
