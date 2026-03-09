use std::f64::consts::PI;

pub(crate) const R_AIR: f64 = 287.0;
pub(crate) const GAMMA_MIX: f64 = 1.35;

pub(crate) fn cp_air() -> f64 {
    GAMMA_MIX * R_AIR / (GAMMA_MIX - 1.0)
}

pub(crate) fn rpm_to_rad_s(rpm: f64) -> f64 {
    rpm * 2.0 * PI / 60.0
}

pub(crate) fn shaft_power_kw(rpm: f64, torque_nm: f64) -> f64 {
    torque_nm * rpm_to_rad_s(rpm) * 1.0e-3
}

pub(crate) fn wrap_deg(angle_deg: f64) -> f64 {
    angle_deg.rem_euclid(720.0)
}

pub(crate) fn crossed_angle(prev_deg: f64, next_deg: f64, target_deg: f64) -> bool {
    let prev = wrap_deg(prev_deg);
    let mut next = wrap_deg(next_deg);
    let target = wrap_deg(target_deg);
    if next < prev {
        next += 720.0;
    }
    let mut target_unwrapped = target;
    if target_unwrapped < prev {
        target_unwrapped += 720.0;
    }
    prev < target_unwrapped && target_unwrapped <= next
}

pub(crate) fn compressible_orifice_mass_flow(
    cda_m2: f64,
    upstream_pa: f64,
    downstream_pa: f64,
    upstream_temp_k: f64,
) -> f64 {
    if cda_m2 <= 0.0 || upstream_pa <= 0.0 || upstream_temp_k <= 0.0 || upstream_pa <= downstream_pa
    {
        return 0.0;
    }

    let pressure_ratio = (downstream_pa / upstream_pa).clamp(0.0, 1.0);
    let critical_ratio = (2.0 / (GAMMA_MIX + 1.0)).powf(GAMMA_MIX / (GAMMA_MIX - 1.0));
    let prefactor = cda_m2 * upstream_pa / upstream_temp_k.sqrt();

    if pressure_ratio <= critical_ratio {
        prefactor
            * (GAMMA_MIX / R_AIR).sqrt()
            * (2.0 / (GAMMA_MIX + 1.0)).powf((GAMMA_MIX + 1.0) / (2.0 * (GAMMA_MIX - 1.0)))
    } else {
        let inner = (2.0 * GAMMA_MIX / (R_AIR * (GAMMA_MIX - 1.0)))
            * (pressure_ratio.powf(2.0 / GAMMA_MIX)
                - pressure_ratio.powf((GAMMA_MIX + 1.0) / GAMMA_MIX));
        if inner <= 0.0 {
            0.0
        } else {
            prefactor * inner.sqrt()
        }
    }
}

pub(crate) fn bidirectional_orifice_mass_flow(
    cda_m2: f64,
    p_a_pa: f64,
    t_a_k: f64,
    p_b_pa: f64,
    t_b_k: f64,
) -> f64 {
    if p_a_pa >= p_b_pa {
        compressible_orifice_mass_flow(cda_m2, p_a_pa, p_b_pa, t_a_k)
    } else {
        -compressible_orifice_mass_flow(cda_m2, p_b_pa, p_a_pa, t_b_k)
    }
}

pub(crate) fn runner_pressure_loss_pa(
    m_dot_kg_s: f64,
    density_kg_m3: f64,
    area_m2: f64,
    loss_coeff: f64,
) -> f64 {
    if area_m2 <= 0.0 || density_kg_m3 <= 0.0 || m_dot_kg_s.abs() <= f64::EPSILON {
        return 0.0;
    }
    m_dot_kg_s.signum() * loss_coeff.max(0.0) * m_dot_kg_s.powi(2)
        / (2.0 * density_kg_m3 * area_m2.powi(2))
}

#[cfg(test)]
pub(crate) fn rk4_scalar_step(y: f64, dt: f64, rhs: impl Fn(f64) -> f64) -> f64 {
    let k1 = rhs(y);
    let k2 = rhs(y + 0.5 * dt * k1);
    let k3 = rhs(y + 0.5 * dt * k2);
    let k4 = rhs(y + dt * k3);
    y + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
}

#[cfg(test)]
mod tests {
    use super::{bidirectional_orifice_mass_flow, crossed_angle, rk4_scalar_step};

    #[test]
    fn rk4_scalar_matches_exact_exponential_decay() {
        let lambda = 7.0;
        let y0 = 1.25;
        let dt = 0.001;
        let steps = 100usize;
        let mut y = y0;
        for _ in 0..steps {
            y = rk4_scalar_step(y, dt, |state| -lambda * state);
        }
        let exact = y0 * (-(lambda * dt * steps as f64)).exp();
        assert!((y - exact).abs() < 1.0e-10);
    }

    #[test]
    fn crossed_angle_detects_wraparound_crossing() {
        assert!(crossed_angle(710.0, 735.0, 5.0));
        assert!(!crossed_angle(620.0, 650.0, 5.0));
    }

    #[test]
    fn bidirectional_orifice_flips_sign_with_pressure_order() {
        let forward = bidirectional_orifice_mass_flow(1.0e-4, 101_000.0, 300.0, 80_000.0, 320.0);
        let reverse = bidirectional_orifice_mass_flow(1.0e-4, 80_000.0, 320.0, 101_000.0, 300.0);
        assert!(forward > 0.0);
        assert!(reverse < 0.0);
    }
}
