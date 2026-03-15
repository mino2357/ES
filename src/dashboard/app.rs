use std::time::Duration;

use eframe::egui;

use super::build_info;
use super::panels;
use super::state::{DashboardState, DashboardUiLoadHint};
use super::theme::DashboardTheme;
use crate::config::{LoadedAppConfig, UiConfig};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum VisualizationTab {
    Overview,
    Map,
    Fit,
    Pressure,
    Cycle,
}

impl VisualizationTab {
    pub(super) const ALL: [Self; 5] = [
        Self::Overview,
        Self::Map,
        Self::Fit,
        Self::Pressure,
        Self::Cycle,
    ];

    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Overview => "Overview",
            Self::Map => "Map",
            Self::Fit => "Fit",
            Self::Pressure => "Pressure",
            Self::Cycle => "Cycle",
        }
    }

    pub(super) fn description(self) -> &'static str {
        match self {
            Self::Overview => {
                "Live gauges, torque cards, and actuator meters for the current operating point"
            }
            Self::Map => {
                "Operating-point table showing where the fitted release point and the live point sit"
            }
            Self::Fit => {
                "Startup-fit torque curve across throttle bins, with selected, verified, and live markers"
            }
            Self::Pressure => {
                "Cylinder p-V, log-log p-V, and p-theta plots for combustion diagnostics"
            }
            Self::Cycle => "Recent-cycle RPM, torque, trapped-air, and valve-lift histories",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum OperatorTab {
    Fit,
    Status,
    Runtime,
}

impl OperatorTab {
    pub(super) const ALL: [Self; 3] = [Self::Fit, Self::Status, Self::Runtime];

    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Fit => "Fit",
            Self::Status => "Status",
            Self::Runtime => "Runtime",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PressureSubview {
    Pv,
    LogPv,
    Ptheta,
}

impl PressureSubview {
    pub(super) const ALL: [Self; 3] = [Self::Pv, Self::LogPv, Self::Ptheta];

    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Pv => "p-V",
            Self::LogPv => "log p-V",
            Self::Ptheta => "p-theta",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CycleSubview {
    Rpm,
    Torque,
    Air,
    Valve,
}

impl CycleSubview {
    pub(super) const ALL: [Self; 4] = [Self::Rpm, Self::Torque, Self::Air, Self::Valve];

    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Rpm => "RPM",
            Self::Torque => "Torque",
            Self::Air => "Air",
            Self::Valve => "Valve",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum MapFillMode {
    BsfcLimit,
    FullBsfc,
}

impl MapFillMode {
    pub(super) const ALL: [Self; 2] = [Self::BsfcLimit, Self::FullBsfc];

    pub(super) fn label(self) -> &'static str {
        match self {
            Self::BsfcLimit => "BSFC + limit",
            Self::FullBsfc => "Full BSFC",
        }
    }

    pub(super) fn detail(self) -> &'static str {
        match self {
            Self::BsfcLimit => {
                "Reachable cells are colored by BSFC, and cells above local WOT stay red."
            }
            Self::FullBsfc => {
                "Fill the whole speed x torque table with the BSFC surrogate and keep over-WOT cells marked by a red border."
            }
        }
    }
}

pub(super) struct DashboardApp {
    theme: DashboardTheme,
    ui_config: UiConfig,
    state: DashboardState,
    visualization_tab: VisualizationTab,
    operator_tab: OperatorTab,
    map_fill_mode: MapFillMode,
    pressure_subview: PressureSubview,
    cycle_subview: CycleSubview,
    header_badge: String,
}

impl DashboardApp {
    fn new(loaded: LoadedAppConfig) -> Self {
        let theme = DashboardTheme::default();
        let ui_config = loaded.config.ui.clone();
        let state = DashboardState::from_loaded_config(loaded);
        Self {
            theme,
            ui_config,
            state,
            visualization_tab: VisualizationTab::Map,
            operator_tab: OperatorTab::Fit,
            map_fill_mode: MapFillMode::FullBsfc,
            pressure_subview: PressureSubview::Pv,
            cycle_subview: CycleSubview::Rpm,
            header_badge: build_info::header_badge(),
        }
    }
}

impl eframe::App for DashboardApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.state.apply_shortcuts(ctx, &self.ui_config);
        self.state.set_ui_load_hint(DashboardUiLoadHint {
            pressure_tab_visible: self.visualization_tab == VisualizationTab::Pressure,
            interaction_active: ui_interaction_active(ctx),
        });
        self.state.advance_simulation(&self.ui_config);

        panels::render_simple_dashboard(
            ctx,
            self.theme,
            &self.ui_config,
            &mut self.state,
            &mut self.visualization_tab,
            &mut self.operator_tab,
            &mut self.map_fill_mode,
            &mut self.pressure_subview,
            &mut self.cycle_subview,
            &self.header_badge,
        );

        let frame_ms = 1000u64 / self.ui_config.repaint_hz.max(1) as u64;
        ctx.request_repaint_after(Duration::from_millis(frame_ms.max(1)));
    }
}

fn ui_interaction_active(ctx: &egui::Context) -> bool {
    ctx.input(|i| {
        i.pointer.any_down()
            || i.raw_scroll_delta != egui::Vec2::ZERO
            || !i.keys_down.is_empty()
            || i.events.iter().any(|event| {
                matches!(
                    event,
                    egui::Event::PointerButton { .. }
                        | egui::Event::Text(_)
                        | egui::Event::Paste(_)
                        | egui::Event::Cut
                        | egui::Event::Copy
                        | egui::Event::Key { pressed: true, .. }
                )
            })
    })
}

fn run_app_with_options(
    loaded: LoadedAppConfig,
    app_title: &str,
    options: eframe::NativeOptions,
) -> eframe::Result {
    eframe::run_native(
        app_title,
        options,
        Box::new(move |cc| {
            DashboardTheme::default().apply(&cc.egui_ctx);
            Ok(Box::new(DashboardApp::new(loaded.clone())))
        }),
    )
}

pub(crate) fn run_app(loaded: LoadedAppConfig) -> eframe::Result {
    let app_title = build_info::window_title();
    // Window size comes from YAML so different machine setups can keep their preferred layout.
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(app_title.clone())
            .with_inner_size([
                loaded.config.ui.window_width_px,
                loaded.config.ui.window_height_px,
            ]),
        ..Default::default()
    };

    run_app_with_options(loaded, &app_title, options)
}
