# Torque Curve Review

## 1. Change policy summary

- Keep the documented ODE state as the only runtime state:
  `x = [omega, theta, p_im, p_ir, p_em, p_er, m_dot_ir, m_dot_er, alpha_th]^T`.
- Keep the operator inputs explicit:
  `u = [alpha_cmd, u_load, theta_ign, VVT_i, VVT_e, u_spark, u_fuel]^T`.
- Improve torque shape only by changing algebraic closures used by `Simulator::eval()` and by tightening the speed-hold settle logic used by the CLI sweep.
- Do not add a hidden helper, surrogate runtime, ad hoc correction layer, or post-hoc torque remap.

## 2. Physical gaps identified in the previous representative result

The prior fixed-input representative sweep had two dominant deficiencies.

1. **High-rpm cylinder filling collapsed too early.**
   The previous `VE_base` used a symmetric Gaussian-style rpm term, so the high-rpm side fell away as fast as the low-rpm side rose.
   In addition, the cylinder-boundary pressure reconstruction used a fixed runner/plenum blend, which under-weighted the runner at high rpm and therefore muted beneficial runner pressure recovery.

2. **Low-speed overlap / residual penalties leaked too far into the mid/high-speed range.**
   The previous `M_ov` and internal-EGR closures penalized backflow based only on overlap, pressure difference, and reverse flow, with no explicit rpm attenuation.
   That made low-speed residual-style penalties remain too influential even after available overlap time per cycle had become short at high rpm.

## 3. Main equation changes

### 3.1 Effective volumetric efficiency

The torque-producing air path remains

```math
VE_{eff}=VE_{base}\,M_{ov}\,M_{wave}.
```

The changed pieces are:

```math
VE_{base} =
\operatorname{clip}\left[
\left(VE_0 + \Delta VE \exp\left(-\frac{(N-N_c)^2}{W(N)}\right)\right) M_{VVT} M_{th},
VE_{min},VE_{max}
\right]
```

with

```math
W(N)=
\begin{cases}
W_{lo}, & N < N_c \\
W_{lo}\,k_{hi}, & N \ge N_c
\end{cases}
```

so the high-rpm side can decay more slowly than the low-rpm side.

### 3.2 Overlap multiplier

```math
M_{ov} =
\operatorname{clip}\left[
1+\phi_{ov}S_{ov}(N)\left(
c_{p,ov}\frac{p_{ir}-p_{er}}{p_a}
c_{\dot m,ov}\frac{\dot m_{er}}{\dot m_{ov,ref}}
\right),
M_{ov,min},M_{ov,max}
\right]
```

with rpm attenuation

```math
S_{ov}(N)=\frac{1}{1+\exp\left(\frac{N-N_{ov}}{W_{ov}}\right)}.
```

### 3.3 Internal EGR closure

```math
EGR_i=
\operatorname{clip}\left[
\phi_{ov}S_{egr}(N)\left(
f_0
+k_p\Pi_{back}
+k_w\Pi_{wave}
+k_r\Pi_{rev}
\right),
EGR_{min},EGR_{max}
\right]
```

with

```math
S_{egr}(N)=\frac{1}{1+\exp\left(\frac{N-N_{egr}}{W_{egr}}\right)}.
```

### 3.4 Boundary pressure reconstruction

```math
p_{ic}=(1-w_i(N))p_{im}+w_i(N)p_{ir}+p_{wave,i}^{close}
```

```math
p_{ec}=(1-w_e(N))p_{em}+w_e(N)p_{er}+p_{wave,e}^{ov}
```

where `w_i(N)` and `w_e(N)` are logistic blends that increase runner weighting at high rpm.

## 4. Torque-shape decomposition by rpm band

### Low-speed region

- `VE_base` sets the baseline trapped-mass level.
- `M_ov` and internal `EGR_i` have their strongest authority here because overlap backflow has more time to occur.
- `Q_loss` and charge-temperature rise penalize indicated efficiency at low piston speed.
- Pumping torque remains material because throttle / manifold pressure mismatch dominates quickly when charge motion is weak.

### Mid-speed region

- `VE_base` and `M_wave` together determine where the main torque shoulder develops.
- The new asymmetric `VE_base` keeps the build-up broad instead of peaking sharply and collapsing immediately.
- `eta_th_base`, combustion phasing efficiency, and residual dilution decide whether increased filling actually becomes shaft torque.

### High-speed region

- The boundary-pressure reconstruction and `M_wave` dominate shape, because a small change in effective intake boundary pressure produces a large trapped-mass change at short fill time.
- `tau_fric = c0 + c1 omega + c2 omega^2` and pumping torque increasingly subtract from the available combustion torque.
- Dyno load coupling should cancel out in the exported brake torque; only residual settling error should appear in `tau_net`.

## 5. Representative fixed-input metrics

Current regression metrics saved in `config/reference_na_i4_metrics_golden.tsv`:

| Metric | Value |
|---|---:|
| Peak brake torque | 187.479901 Nm |
| Peak torque location | 1990.464 rpm |
| Peak brake power | 87.180618 kW |
| Peak power location | 6000.158 rpm |
| Low-to-peak gain | 143.005086 Nm |
| Peak-to-high drop | 147.854405 Nm |
| High-rpm retention ratio `r_hi` | 0.323428 |
| Monotonicity before peak | 1.000000 |
| Monotonicity after peak | 1.000000 |

## 6. Before / after comparison

The previous short representative sweep had `r_hi ≈ 0.147`.
The updated closure set raises that to `r_hi ≈ 0.323`, so the 8000 rpm torque fraction is materially improved even though the curve still peaks too early for a high-rev reference case.

## 7. Remaining unresolved points

- The full-load fixed-input peak still occurs too early.
- Absolute high-rpm power remains below the intended high-rev naturally aspirated reference envelope.
- Further work is still needed in combustion-phase efficiency, fueling / lambda treatment, and possibly a more explicit runner-pressure-loss / in-cylinder charge-temperature coupling.

## 8. Primary references used

1. Heywood, *Internal Combustion Engine Fundamentals* (2nd ed.).
2. Stone, *Introduction to Internal Combustion Engines* (4th ed.).
3. Standard Woschni-style heat-transfer and single-zone combustion references.
4. Public naturally aspirated 2.0 L SI-engine brochure / catalog torque and power envelopes used as trend anchors, not as a hidden torque remap.
