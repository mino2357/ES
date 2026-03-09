use eframe::egui;

#[derive(Debug, Clone, Copy)]
pub(super) struct DashboardTheme {
    pub(super) shell_bg: egui::Color32,
    pub(super) panel_bg: egui::Color32,
    pub(super) panel_alt_bg: egui::Color32,
    pub(super) bezel: egui::Color32,
    pub(super) chrome: egui::Color32,
    pub(super) text_main: egui::Color32,
    pub(super) text_soft: egui::Color32,
    pub(super) amber: egui::Color32,
    pub(super) cyan: egui::Color32,
    pub(super) green: egui::Color32,
    pub(super) red: egui::Color32,
    pub(super) lamp_off: egui::Color32,
}

impl Default for DashboardTheme {
    fn default() -> Self {
        Self {
            shell_bg: egui::Color32::from_rgb(14, 18, 20),
            panel_bg: egui::Color32::from_rgb(24, 29, 31),
            panel_alt_bg: egui::Color32::from_rgb(18, 22, 24),
            bezel: egui::Color32::from_rgb(44, 52, 56),
            chrome: egui::Color32::from_rgb(118, 127, 123),
            text_main: egui::Color32::from_rgb(229, 233, 224),
            text_soft: egui::Color32::from_rgb(152, 166, 157),
            amber: egui::Color32::from_rgb(255, 184, 58),
            cyan: egui::Color32::from_rgb(93, 214, 245),
            green: egui::Color32::from_rgb(121, 228, 140),
            red: egui::Color32::from_rgb(255, 108, 92),
            lamp_off: egui::Color32::from_rgb(44, 48, 49),
        }
    }
}

impl DashboardTheme {
    pub(super) fn apply(self, ctx: &egui::Context) {
        let mut visuals = egui::Visuals::dark();
        visuals.override_text_color = Some(self.text_main);
        visuals.panel_fill = self.shell_bg;
        visuals.extreme_bg_color = self.panel_alt_bg;
        visuals.faint_bg_color = self.panel_bg;
        visuals.window_fill = self.panel_bg;
        visuals.window_stroke = egui::Stroke::new(1.0, self.bezel);
        visuals.widgets.noninteractive.bg_fill = self.panel_bg;
        visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, self.bezel);
        visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0, self.text_soft);
        visuals.widgets.inactive.bg_fill = self.panel_alt_bg;
        visuals.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, self.bezel);
        visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.0, self.text_main);
        visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(31, 40, 43);
        visuals.widgets.hovered.bg_stroke = egui::Stroke::new(1.0, self.amber);
        visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.2, self.text_main);
        visuals.widgets.active.bg_fill = egui::Color32::from_rgb(34, 44, 47);
        visuals.widgets.active.bg_stroke = egui::Stroke::new(1.4, self.amber);
        visuals.widgets.active.fg_stroke = egui::Stroke::new(1.2, self.text_main);
        visuals.selection.bg_fill = self.amber.gamma_multiply(0.24);
        visuals.selection.stroke = egui::Stroke::new(1.0, self.amber);
        visuals.hyperlink_color = self.cyan;
        ctx.set_visuals(visuals);

        let mut style = (*ctx.style()).clone();
        style.spacing.item_spacing = egui::vec2(10.0, 8.0);
        style.spacing.button_padding = egui::vec2(10.0, 8.0);
        style.spacing.indent = 18.0;
        style
            .text_styles
            .insert(egui::TextStyle::Heading, egui::FontId::proportional(24.0));
        style
            .text_styles
            .insert(egui::TextStyle::Body, egui::FontId::proportional(15.0));
        style
            .text_styles
            .insert(egui::TextStyle::Monospace, egui::FontId::monospace(14.0));
        style
            .text_styles
            .insert(egui::TextStyle::Button, egui::FontId::proportional(14.0));
        style
            .text_styles
            .insert(egui::TextStyle::Small, egui::FontId::monospace(11.0));
        ctx.set_style(style);
    }

    pub(super) fn header_frame(self) -> egui::Frame {
        egui::Frame::new()
            .fill(self.panel_alt_bg)
            .inner_margin(egui::Margin::symmetric(16, 12))
            .stroke(egui::Stroke::new(1.0, self.bezel))
    }

    pub(super) fn rack_frame(self) -> egui::Frame {
        egui::Frame::new()
            .fill(self.panel_bg)
            .inner_margin(egui::Margin::symmetric(14, 12))
            .stroke(egui::Stroke::new(1.0, self.bezel))
            .corner_radius(10)
    }

    pub(super) fn monitor_frame(self) -> egui::Frame {
        egui::Frame::new()
            .fill(self.panel_bg)
            .inner_margin(egui::Margin::symmetric(12, 10))
            .stroke(egui::Stroke::new(1.0, self.chrome))
            .corner_radius(12)
    }

    pub(super) fn instrument_frame(self, accent: egui::Color32) -> egui::Frame {
        egui::Frame::new()
            .fill(self.panel_alt_bg)
            .inner_margin(egui::Margin::symmetric(10, 10))
            .stroke(egui::Stroke::new(1.0, accent.gamma_multiply(0.6)))
            .corner_radius(12)
    }
}
