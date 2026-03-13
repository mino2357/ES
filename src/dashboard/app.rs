use std::time::Duration;

use eframe::egui;

use super::panels;
use super::state::DashboardState;
use super::theme::DashboardTheme;
use crate::config::{AppConfig, UiConfig};

pub(super) struct DashboardApp {
    theme: DashboardTheme,
    ui_config: UiConfig,
    state: DashboardState,
}

impl DashboardApp {
    fn new(config: AppConfig) -> Self {
        let theme = DashboardTheme::default();
        let ui_config = config.ui.clone();
        let state = DashboardState::new(config);
        Self {
            theme,
            ui_config,
            state,
        }
    }
}

impl eframe::App for DashboardApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.state.apply_shortcuts(ctx, &self.ui_config);
        self.state.advance_simulation(&self.ui_config);

        panels::render_simple_dashboard(ctx, self.theme, &self.ui_config, &mut self.state);

        let frame_ms = 1000u64 / self.ui_config.repaint_hz.max(1) as u64;
        ctx.request_repaint_after(Duration::from_millis(frame_ms.max(1)));
    }
}

fn run_app_with_options(config: AppConfig, options: eframe::NativeOptions) -> eframe::Result {
    eframe::run_native(
        "ES Simulator Dashboard",
        options,
        Box::new(move |cc| {
            DashboardTheme::default().apply(&cc.egui_ctx);
            Ok(Box::new(DashboardApp::new(config.clone())))
        }),
    )
}

pub(crate) fn run_app(config: AppConfig) -> eframe::Result {
    // Window size comes from YAML so different machine setups can keep their preferred layout.
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([config.ui.window_width_px, config.ui.window_height_px]),
        ..Default::default()
    };

    run_app_with_options(config, options)
}
