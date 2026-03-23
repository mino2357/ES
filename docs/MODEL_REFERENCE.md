# ES Model Reference

This document is self-contained.
It describes the current reduced-order mathematical model, distinguishes differential states from algebraic closures, maps each part to the implementation, and lists the literature used as physical anchors.

## Terms And Symbols

- `ODE`: ordinary differential equation
- `SI engine`: spark-ignition engine
- `degCA`: crank-angle degrees
- `IMEP`: indicated mean effective pressure
- `BMEP`: brake mean effective pressure
- `EGR`: exhaust gas recirculation
- `internal EGR`: residual gas trapped internally by valve overlap and backflow
- `gamma`: ratio of specific heats
- `c_p`: constant-pressure specific heat
- `alpha_th`: effective throttle opening
- `omega`: crank angular speed
- `theta`: crank angle
- `tau`: torque
- `p_im`, `p_ir`, `p_em`, `p_er`: intake plenum, intake runner, exhaust manifold, and exhaust runner pressures
- `dot m_ir`, `dot m_er`: reduced-order intake and exhaust runner flow states
- `eta_d`: driveline efficiency
- `r_w`: effective wheel radius
- `i_tot`: total drivetrain ratio

## Scope

The present solver is a reduced-order transient model.
It integrates a compact engine-periphery state and reconstructs cylinder-display quantities from that state.

It is therefore:

- more physically grounded than a pure lookup-table dashboard
- much cheaper than CFD or a full 1D gas-dynamics solver
- still dependent on semi-empirical closures for volumetric efficiency, combustion, heat transfer, and internal residual effects

## Related Documents

- [../README.md](../README.md): repository overview and document map
- [USER_MANUAL.md](USER_MANUAL.md): CLI operation, exported artifacts, and reference sweep interpretation
- [MODEL_REFERENCE.ja.md](MODEL_REFERENCE.ja.md): Japanese version of this model reference
- [../ENGINE_MODEL_WORKLOG.md](../ENGINE_MODEL_WORKLOG.md): chronological implementation log


## Documentation And Code Traceability

The maintained CLI workflow uses the following implementation split:

- `src/simulator.rs`: integrates the ODE state, evaluates algebraic closures, reconstructs `p-V` / `p-theta` / `T-S`, and keeps torque bookkeeping consistent
- `src/cli.rs`: selects the WOT operating-point policy, holds target speed with the absorber model, and exports brake-torque-centered artifacts
- `src/config.rs`: defines default parameter values and YAML schema
- `config/reference_na_i4.yaml`: high-rev naturally aspirated 2.0 L reference calibration used by the manuals

For the CLI artifacts, the exported dyno-style brake quantity is

```math
\tau_{brake} = \tau_{net} + \tau_{load}
```

which matches a speed-held dynamometer interpretation better than reporting `tau_net` alone.

## Literature Anchors

The current reference calibration and documentation are anchored to the following source families:

1. public brochure / catalog specifications for high-rev naturally aspirated 2.0 L SI engines.
2. Heywood, *Internal Combustion Engine Fundamentals* (2nd ed.), for filling-and-emptying, throttling, and indicated work interpretation.
3. Stone, *Introduction to Internal Combustion Engines* (4th ed.), for plausible naturally aspirated SI trend ranges.
4. Woschni-style cylinder heat-transfer literature and standard single-zone heat-release references for the display-side pressure reconstruction assumptions.

## State Partition

### Differential State

The integrated state vector is:

```math
\mathbf{x} =
\begin{bmatrix}
\omega &
\theta &
p_{im} &
p_{ir} &
p_{em} &
p_{er} &
\dot m_{ir} &
\dot m_{er} &
\alpha_{th}
\end{bmatrix}^{\mathsf T}
```

In the code, this corresponds to `EngineState` in `src/simulator.rs`.

When the computed curve departs from reality, the preferred response in this repository is to revisit the mathematical model itself rather than adding a surrogate correction or an MVEM side path, identify the missing physical phenomena, extend the closures or state equations accordingly, and solve the updated ODE system again.

### Algebraic And Reconstructed Quantities

The following are not independent ODE states.
They are evaluated as algebraic closures from `x` and operator inputs:

- throttle mass flow
- cylinder boundary pressures
- volumetric efficiency
- trapped air and fuel per cycle
- `internal EGR` fraction
- charge temperature
- effective `c_p` and `gamma`
- combustion phasing and burn duration
- combustion, friction, pumping, and load torques
- `p-V` and `p-theta` display traces

## Complete ODE System Summary

The integrated system can be written compactly as

```math
\frac{d\mathbf{x}}{dt}
=
\mathbf{f}\!\left(\mathbf{x}, \mathbf{u}, \mathbf{z}(\mathbf{x},\mathbf{u})\right)
```

with operator input

```math
\mathbf{u} =
\begin{bmatrix}
\alpha_{cmd} &
u_{load} &
\theta_{ign} &
VVT_i &
VVT_e &
u_{spark} &
u_{fuel}
\end{bmatrix}^{\mathsf T}
```

and algebraic closure vector `z(x,u)` containing the throttle flow, cylinder boundary pressures, wave-action multipliers, trapped air, fueling, `internal EGR`, charge properties, and torque closures.

Written component-wise, the ODE system is

```math
\frac{d}{dt}
\begin{bmatrix}
\omega \\
\theta \\
p_{im} \\
p_{ir} \\
p_{em} \\
p_{er} \\
\dot m_{ir} \\
\dot m_{er} \\
\alpha_{th}
\end{bmatrix}
=
\begin{bmatrix}
\dfrac{\tau_{comb} - \tau_{fric} - \tau_{pump} - \tau_{load}}{J_{eff}} \\
\omega \\
\dfrac{R T_i}{V_{im}}\left(\dot m_{th} - \dot m_{ir}\right) \\
\dfrac{R T_i}{V_{ir}}\left(\dot m_{ir} - \dot m_{cyl}\right) \\
\dfrac{R T_e}{V_{em}}\left(\dot m_{er} - \dot m_{tp}\right) \\
\dfrac{R T_e}{V_{er}}\left(\dot m_{exh,in} - \dot m_{er}\right) \\
\dfrac{(p_{im} - p_{ir}) - \Delta p_{loss,ir}}{L_{ir}} - c_{ir}\dot m_{ir} \\
\dfrac{(p_{er} - p_{em}) - \Delta p_{loss,er}}{L_{er}} - c_{er}\dot m_{er} \\
\dfrac{\alpha_{cmd} - \alpha_{th}}{\tau_{th}}
\end{bmatrix}
```

This is the full differential system actually integrated by the realtime solver.
Everything else described later in this document enters through the algebraic closure vector `z(x,u)`.

## Governing Differential Equations

The solver uses a compact reduced-order state-space structure.
The nonlinear terms are first evaluated by `Simulator::eval()`, then injected into the ODE right-hand side in `Simulator::derivatives()`.

### Crankshaft Dynamics

```math
J_{eff}\frac{d\omega}{dt}
=
\tau_{comb}
- \tau_{fric}
- \tau_{pump}
- \tau_{load}
```

with

```math
J_{eff} = J_{engine} + J_{load,reflected}
```

and

```math
\frac{d\theta}{dt} = \omega
```

### Filling-And-Emptying Pressure States

For each lumped control volume:

```math
\frac{dp}{dt} = \frac{R T}{V}\left(\dot m_{in} - \dot m_{out}\right)
```

Specifically:

```math
\frac{dp_{im}}{dt}
=
\frac{R T_i}{V_{im}}
\left(\dot m_{th} - \dot m_{ir}\right)
```

```math
\frac{dp_{ir}}{dt}
=
\frac{R T_i}{V_{ir}}
\left(\dot m_{ir} - \dot m_{cyl}\right)
```

```math
\frac{dp_{em}}{dt}
=
\frac{R T_e}{V_{em}}
\left(\dot m_{er} - \dot m_{tp}\right)
```

```math
\frac{dp_{er}}{dt}
=
\frac{R T_e}{V_{er}}
\left(\dot m_{exh,in} - \dot m_{er}\right)
```

### Runner Flow States

```math
\frac{d\dot m_{ir}}{dt}
=
\frac{(p_{im} - p_{ir}) - \Delta p_{loss,ir}}{L_{ir}}
- c_{ir}\dot m_{ir}
```

```math
\frac{d\dot m_{er}}{dt}
=
\frac{(p_{er} - p_{em}) - \Delta p_{loss,er}}{L_{er}}
- c_{er}\dot m_{er}
```

with

```math
\Delta p_{loss}
=
\operatorname{sgn}(\dot m)
\left(
f\frac{L}{D} + K
\right)
\frac{\dot m^2}{2\rho A^2}
```

and

```math
\rho = \frac{\tfrac{1}{2}(p_{up}+p_{down})}{R T},
\qquad
A = N_{paths}\frac{\pi D^2}{4}
```

### Throttle Actuator

```math
\frac{d\alpha_{th}}{dt}
=
\frac{\alpha_{cmd} - \alpha_{th}}{\tau_{th}}
```

## Algebraic Closures

### Effective Throttle Area And Throttle Flow

```math
A_{th}
=
A_{th,max}
\operatorname{clamp}
\left(
a_0 + a_1 \alpha_{th}^{n_{th}},
a_{min},
1
\right)
```

```math
\dot m_{th}
=
C_d A_{th}
\frac{p_{up}}{\sqrt{T}}
\Phi\!\left(\frac{p_{down}}{p_{up}}\right)
```

with

```math
\Phi(pr) =
\begin{cases}
\sqrt{\frac{\gamma}{R}}
\left(\frac{2}{\gamma+1}\right)^{\frac{\gamma+1}{2(\gamma-1)}},
& pr \le \left(\frac{2}{\gamma+1}\right)^{\frac{\gamma}{\gamma-1}}
\\[6pt]
\sqrt{
\frac{2\gamma}{R(\gamma-1)}
\left(
pr^{2/\gamma} - pr^{(\gamma+1)/\gamma}
\right)
},
& \text{otherwise}
\end{cases}
```

### Volumetric Efficiency

```math
VE_{base}
=
\operatorname{clamp}
\left(
VE_{rpm}
M_{vvt,lin}
M_{vvt,opt}
VE_{th},
VE_{min},
VE_{max}
\right)
```

```math
M_{vvt,lin}
=
1 + c_{vi}VVT_i - c_{ve}VVT_e
```

```math
\beta_{vvt}
=
\operatorname{clamp}
\left(
\frac{rpm-rpm_{vvt,low}}{rpm_{vvt,high}-rpm_{vvt,low}},
0,
1
\right)
```

```math
VVT_{i,opt}
=
(1-\beta_{vvt})VVT_{i,low}
+
\beta_{vvt}VVT_{i,high}
```

```math
VVT_{e,opt}
=
(1-\beta_{vvt})VVT_{e,low}
+
\beta_{vvt}VVT_{e,high}
```

```math
M_{vvt,opt}
=
\max\left(
0.75,
\left(
1 - g_i\left(\frac{VVT_i - VVT_{i,opt}}{w_i}\right)^2
\right)
\left(
1 - g_e\left(\frac{VVT_e - VVT_{e,opt}}{w_e}\right)^2
\right)
\right)
```

```math
VE_{rpm}
=
\operatorname{clamp}
\left(
b_0 + b_1
\exp\left(
-\frac{(rpm-rpm_c)^2}{w_{rpm}}
\right),
VE_{rpm,min},
VE_{rpm,max}
\right)
```

```math
VE_{th}
=
\operatorname{clamp}
\left(
t_0 + t_1 \sqrt{\alpha_{th}},
VE_{th,min},
VE_{th,max}
\right)
```

```math
M_{ov}
=
1 + \lambda_{ov}
\left(
c_{p,ov}\frac{p_{ir}-p_{er}}{p_a}
-
c_{f,ov}\frac{\dot m_{er}}{\dot m_{ref}}
\right)
```

```math
VE_{eff}
=
\operatorname{clamp}
\left(
VE_{base} M_{ov} M_{wave},
VE_{overall,min},
VE_{overall,max}
\right)
```

### Trapped Air And Fueling

```math
m_{air,cyl}
=
\frac{V_d/N_{cyl}\; VE_{eff}\; p_{int,cyl}}{R T_{charge}}
```

```math
\dot m_{air}
=
\frac{rpm}{120}
\frac{V_d\; VE_{eff}\; p_{int,cyl}}{R T_{charge}}
```

```math
\lambda_{target}
=
\operatorname{clamp}
\left(
\lambda_0 - c_\lambda \alpha_{th},
\lambda_{min},
\lambda_{max}
\right)
```

```math
AFR = AFR_{stoich}\lambda_{target}
```

```math
m_{fuel,cyl}
=
\frac{m_{air,cyl}}{AFR}
```

### Charge Cooling, Internal EGR, And Mixture Properties

```math
T_{fresh}
=
\operatorname{clamp}
\left(
T_{int}
-
\frac{
m_{fuel} h_{fg}\eta_{evap}
}{
(m_{air}+m_{fuel}) c_{p,fresh}
},
T_{fresh,min},
T_{int}
\right)
```

```math
f_{EGR}
=
\operatorname{clamp}
\left[
\lambda_{ov}
\left(
c_0
+ c_p \max\left(\frac{p_{er}-p_{ir}}{p_a},0\right)
+ c_w \max(-H_{scav},0)
+ c_r \max\left(-\frac{\dot m_{er}}{\dot m_{rev,ref}},0\right)
\right)
\right],
f_{min},
f_{max}
\right]
```

```math
m_{res}
=
m_{air}
\frac{f_{EGR}}{1-f_{EGR}}
```

```math
c_{p,b}
=
\operatorname{clamp}
\left(
c_{p,b,ref}
+ k_T(T_{res}-T_{ref})
+ k_{EGR} f_{EGR},
c_{p,min},
c_{p,max}
\right)
```

```math
c_{p,mix}
=
\frac{m_{fresh}c_{p,fresh}+m_{res}c_{p,b}}{m_{fresh}+m_{res}}
```

```math
T_{charge}
=
\frac{
m_{fresh}c_{p,fresh}T_{fresh}
+ m_{res}c_{p,b}T_{res}
}{
m_{fresh}c_{p,fresh}+m_{res}c_{p,b}
}
```

```math
\gamma_{mix}
=
\frac{c_{p,mix}}{c_{p,mix}-R}
```

### Combustion Phasing, Burn Duration, And Wiebe Rate

```math
\theta_{SOC}
=
\theta_{soc,base}
- \theta_{ign}
+ c_{soc,vvt}VVT_i
```

```math
\Delta\theta_b
=
\operatorname{clamp}
\left(
\Delta\theta_{b,base}
- c_{b,th}\alpha_{th}
+ c_{b,EGR} f_{EGR},
\Delta\theta_{b,min},
\Delta\theta_{b,max}
\right)
```

```math
\eta_{phase,dil}
=
\operatorname{clamp}
\left(
1-k_{dil}f_{EGR},
\eta_{dil,min},
1
\right)
```

```math
\eta_{phase} = \eta_{phase,base}\eta_{phase,dil}
```

```math
\frac{dx_b}{d\theta}
=
\frac{a(m+1)}{\Delta\theta_b}
x^m
\exp\left(-a x^{m+1}\right),
\qquad
x=\frac{\theta-\theta_{SOC}}{\Delta\theta_b}
```

### Heat Transfer, Efficiency, And Torque

```math
T_{comp}
=
T_{charge} r_c^{\gamma_{mix}-1}
```

```math
\Delta T_{adiabatic}
=
\frac{m_{fuel}LHV}{m_{charge}c_{p,b}}
```

```math
T_g
=
\operatorname{clamp}
\left(
T_{comp}
+ k_{Tg}\eta_{phase}\Delta T_{adiabatic},
T_{int},
T_{g,max}
\right)
```

```math
h_w
=
h_{w,0}
\left(\frac{p_{cyl}}{p_{ref}}\right)^{n_p}
\left(\frac{T_g}{T_{ref}}\right)^{n_T}
\left(\frac{\bar U_p}{U_{p,ref}}\right)^{n_u}
```

```math
\bar U_p = \frac{2 S\; rpm}{60}
```

```math
t_{exp}
=
k_t\frac{\Delta\theta_b}{720}\frac{120}{rpm}
```

```math
Q_{loss}
=
\operatorname{clamp}
\left(
h_w A_{wall}(T_g-T_{wall}) t_{exp},
0,
\chi_{max} m_{fuel}LHV
\right)
```

```math
\eta_{th,base}
=
\operatorname{clamp}
\left(
\eta_0 + c_L \ell - c_r |rpm-rpm_{ref}|,
\eta_{base,min},
\eta_{base,max}
\right)
```

```math
\ell = \operatorname{clamp}\left(\frac{p_{int,cyl}}{p_a},\ell_{min},\ell_{max}\right)
```

```math
\eta_{th}
=
\operatorname{clamp}
\left(
\eta_{th,base}\eta_{phase}
- \frac{Q_{loss}}{m_{fuel}LHV},
\eta_{min},
\eta_{max}
\right)
```

```math
W_{cyc,cyl} = m_{fuel}LHV\eta_{th}
```

```math
\bar\tau_{comb}
=
\frac{W_{cyc,cyl}N_{cyl}}{4\pi}
```

```math
\tau_{fric} = c_0 + c_1\omega + c_2\omega^2
```

```math
\tau_{pump}
=
\operatorname{clamp}
\left(
\frac{(p_{exh,cyl}-p_{int,cyl})V_d}{4\pi},
\tau_{pump,min},
\tau_{pump,max}
\right)
```

## External Load Models

### Brake Dyno Mode

```math
s = u_{load}^{n_{load}}
```

```math
=
s
\tau_{avail}
```

with

```math
\tau_{ref}
=
\left(
a_0 + a_1\omega + a_2\omega^2
\right)
```

and

```math
\tau_{avail}
=
\min\left(
\tau_{ref},
\frac{P_{abs,max}}{\omega}
\right)
```

### Vehicle-Equivalent Mode

```math
v = \omega\frac{r_w}{i_{tot}}
```

```math
F_{roll} = C_{rr} m_{eq} g \cos\phi
```

```math
F_{grade} = m_{eq} g \sin\phi
```

```math
F_{drag} = \frac{1}{2}\rho C_d A_f v^2
```

```math
\tau_w = (F_{roll}+F_{grade}+F_{drag})r_w
```

```math
\tau_{load}
=
s
\min\left(
\frac{\tau_w}{i_{tot}\eta_d} + \tau_{acc}
\;,\;
\frac{P_{abs,max}}{\omega}
\right)
```

```math
J_{load,reflected}
=
J_{abs}
+
|s|\,m_{eq}\left(\frac{r_w}{i_{tot}}\right)^2
```

## Display-Oriented Single-Zone `p-V` And `p-theta`

```math
V(\theta)
=
V_c + \frac{V_s}{2}(1-\cos\theta_p)
```

```math
\frac{dp}{d\theta}
=
\frac{\gamma(\theta)-1}{V(\theta)}\frac{dQ_{net}}{d\theta}
- \gamma(\theta)\frac{p}{V(\theta)}\frac{dV}{d\theta}
```

```math
\frac{dQ_{net}}{d\theta}
=
m_f LHV \frac{dx_b}{d\theta}
- \frac{dQ_{loss}}{d\theta}
```

```math
x_b(\theta)
=
\frac{1-\exp\left[-a\left(\frac{\theta-\theta_{SOC}}{\Delta \theta_b}\right)^{m+1}\right]}
{1-\exp(-a)}
```

```math
W_i \approx \oint p\,dV
```

```math
IMEP = \frac{W_i}{V_{swept}}
```

```math
\eta_{i,pV} = \operatorname{clamp}\left(\frac{W_i}{m_{fuel}LHV},\eta_{i,min},\eta_{i,max}\right)
```

```math
\eta_{Otto} = 1 - \frac{1}{r_c^{\gamma_{air}-1}}
```

The display-side closed-cycle pressure solve uses the equation above over `180 degCA .. EVO`.
`gamma(theta)` transitions from the unburned-mixture `mixture_gamma` toward `gamma_expansion`,
and `dQ_loss/dtheta` distributes the cycle heat-loss estimate across the burn window plus a short post-burn tail.
The intake and exhaust strokes are then blended back to the intake and exhaust boundary pressures for display.

## Numerical Method

### RK3

This simulator uses the classical Kutta three-stage, third-order explicit Runge-Kutta formula, with Butcher tableau
`c = [0, 1/2, 1]`, `A = [[0], [1/2], [-1, 2]]`, `b = [1/6, 4/6, 1/6]`.

```math
k_1 = f(x_n)
```

```math
k_2 = f\left(x_n + \frac{\Delta t}{2}k_1\right)
```

```math
k_3 = f\left(x_n + \Delta t(-k_1 + 2k_2)\right)
```

```math
x_{n+1}
=
x_n + \Delta t
\left(
\frac{1}{6}k_1 + \frac{4}{6}k_2 + \frac{1}{6}k_3
\right)
```

### Accuracy-First Timestep Policy

```math
\Delta t
=
\operatorname{clamp}
\left(
\frac{\Delta\theta_{target}}{6\;rpm_{eff}},
\Delta t_{min},
\Delta t_{max}
\right)
```

## What Comes From Literature Versus What Is Implementation-Specific

Literature-anchored structures:

- compact reduced-order state selection and filling-and-emptying formulation
- residual-gas and EGR interpretation
- Woschni-style heat-transfer scaling
- Wiebe-function burn-rate shaping
- road-load force balance and reflected inertia

Implementation-specific surrogates:

- the exact volumetric-efficiency algebra
- the exact overlap multiplier and wave-action blending
- the exact `internal EGR` surrogate
- the exact efficiency-base surrogate
- the exact display-oriented single-zone `p-V` pressure solve

## Implementation Map

| Model element | Main implementation |
| --- | --- |
| state definition and ODE RHS | `src/simulator.rs` |
| algebraic closures | `src/simulator.rs` |
| configuration schema | `src/config.rs` |
| parameter plausibility audit | `src/config/audit.rs` |
| CLI sweep orchestration and output writing | `src/cli.rs` |
| `p-V` / `p-theta` / `T-S` artifact reconstruction | `src/simulator.rs`, `src/cli.rs` |

Useful entry points:

- `Simulator::eval()`
- `Simulator::derivatives()`
- `Simulator::step()`
- `accuracy_priority_dt()`
- `volumetric_efficiency()`
- `trapped_air_mass()`
- `wiebe_combustion_rate()`
- `instantaneous_pv_sample()`
- `external_load_torque_nm()`
- `external_load_reflected_inertia_kgm2()`

## Source Map

- Mean-value SI engine structure and filling-and-emptying viewpoint:
  Hendricks and Sorenson, 1990; Hendricks and Vesterholm, 1992
- Reduced-order EGR and residual-gas treatment:
  Fons et al., 1999
- Reduced-order dyno-load abstraction and operator-facing load instrumentation:
  HORIBA SPARC Engine; Froude Texcel V12 PRO; Froude InCell
- Single-zone combustion framing and Wiebe-style burn law usage:
  Grau et al., 2002
- Qualitative VVT direction used for the low-speed versus high-speed phasing surrogate:
  Ma et al., 2001; Lee and Min, 2010; Asmus, 1985
- Heat-release interpretation from pressure data:
  Gatowski et al., 1984
- Heat-transfer scaling:
  Woschni, 1967
- Longitudinal road load and reflected inertia:
  Rajamani, 2012; Hellgren and Larsson, 2024

## Validation Limits

What is currently defensible:

- qualitative transient response to throttle, load, ignition, and VVT changes
- qualitative low-speed versus high-speed VVT torque-direction trends
- transient load-response analysis
- physically interpretable vehicle-equivalent loading
- reduced-order visualization of manifold, runner, and reconstructed cylinder behavior

What still requires model refinement before claiming stronger realism:

- absolute torque level over the full map
- exact combustion phasing and burn duration
- exact internal residual fraction
- exact cylinder pressure trace
- emissions and aftertreatment behavior

## Literature Sources


2. Hendricks, E., and Vesterholm, T., "The Analysis of Mean Value SI Engine Models," SAE Technical Paper 920682, 1992. https://doi.org/10.4271/920682

3. Fons, M., Muller, M., Chevalier, A., Vigild, C., Hendricks, E., and Sorenson, S. C., "Mean Value Engine Modelling of an SI Engine with EGR," SAE Technical Paper 1999-01-0909, 1999. https://doi.org/10.4271/1999-01-0909

4. Woschni, G., "A Universally Applicable Equation for the Instantaneous Heat Transfer Coefficient in the Internal Combustion Engine," SAE Technical Paper 670931, 1967. https://doi.org/10.4271/670931

5. Gatowski, J. A., Balles, E. N., Chun, K. M., Nelson, F. E., Ekchian, J. A., and Heywood, J. B., "Heat Release Analysis of Engine Pressure Data," SAE Technical Paper 841359, 1984. https://doi.org/10.4271/841359

6. Grau, J., Garcia, J., Garcia, J., Robles, A., and Royo, R., "Modelling Methodology of a Spark-Ignition Engine and Experimental Validation: Part I: Single-Zone Combustion Model," SAE Technical Paper 2002-01-2193, 2002. https://doi.org/10.4271/2002-01-2193

7. Rajamani, R., *Vehicle Dynamics and Control*, Springer. Book page: https://link.springer.com/book/10.1007/978-1-4614-1433-9

8. Hellgren, J., and Larsson, E., "A Propulsion Energy Estimator for Road Vehicles," in *FISITA World Congress 2023*, Springer, 2024. Chapter page: https://link.springer.com/chapter/10.1007/978-3-031-70392-8_45

9. Ma, T., Kuo, T.-W., Lin, C.-A., and Huang, Z.-Y., "Optimization of the Variable Valve Timing and Lift Profile for a SI Engine," SAE Technical Paper 2001-01-0247, 2001. https://doi.org/10.4271/2001-01-0247

10. Lee, Y., and Min, K., "Effects of Variable Intake Valve Timing on Engine Performance in a Spark Ignition Engine," SAE Technical Paper 2010-01-1190, 2010. https://doi.org/10.4271/2010-01-1190

11. Asmus, T., "Variable Valve Timing: A Valuable Means for Improving the Overall Engine Performance," SAE Technical Paper 850507, 1985. https://doi.org/10.4271/850507

12. HORIBA, "SPARC Engine," official product page. https://www.horiba.com/usa/medical/products/detail/action/show/Product/sparc-95/

13. Froude, "Texcel V12 PRO Controller," official product page. https://www.froudedyno.com/products/control-systems/texcel-v12-controller

14. Froude, "InCell Control and Data Acquisition System," official product page. https://www.froudedyno.com/products/control-systems/incell-control-and-data-acquisition-system
