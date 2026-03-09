# ES

Language: English | [Japanese](README.ja.md)

Fixed-4-cylinder 4-stroke engine physics simulator in Rust + egui.

The realtime path integrates a 0D dynamic model with 3-stage RK3, and visualizes:

- RPM over the latest completed `N` cycles plus the currently accumulating cycle on a fixed `0..720 degCA` x-axis
- Net/load torque over the latest completed `N` cycles plus the currently accumulating cycle (`Nm`, fixed `0..720 degCA` x-axis)
- Gross combustion power and net brake power (`kW`, `HP`)
- Trapped air mass over the latest completed `N` cycles plus the currently accumulating cycle (`mg/cyl`, fixed `0..720 degCA` x-axis)
- Recent-cycle `p-V` diagram (fixed configured axis range with overflow headroom)
- Cam lift profiles (intake/exhaust + crank cursor)
- Thermal efficiency metrics

Audio is synthesized from crank-synchronous exhaust events, exhaust pressure / flow states, a simple tailpipe-reflection surrogate, and exhaust-pipe resonance (no real engine audio assets).

The engine layout is fixed to 4 cylinders throughout the simulator. Any `history_recent_cycles` / "cycles" wording below refers to plotted engine-cycle history, not to cylinder count.

`config/sim.yaml` is also checked by a dedicated plausibility audit in `src/config/audit.rs`. Physical or semi-physical quantities are validated against engine-plausible bands, while audio / UI / numerics are validated against runtime-practical bands.

## Terminology And Notation

This README is intended to be self-contained. The terms below are the local definitions used throughout the rest of the document.

- `realtime GUI path`: the interactive `es_sim` binary. It solves a reduced-order mean-value model while drawing plots and optionally synthesizing audio.
- `GUI`: graphical user interface.
- `headless CLI path`: the non-GUI `es_cli` / `es_hp` binaries. They run without GUI / audio / `serde_yaml` dependencies and emit files such as CSV or text reports.
- `CLI`: command-line interface.
- `compatibility alias`: a secondary binary name that intentionally calls the same implementation as the primary binary.
- `offline`: not constrained by interactive wall-clock timing. The headless solver is both headless and offline.
- `plausibility audit`: a range and sanity check pass applied to parsed configuration before the simulator accepts it.
- `reduced-order mean-value model`: a low-state-count surrogate that keeps aggregate manifold, runner, combustion, and torque behavior without claiming full CFD or full commercial 1D gas dynamics fidelity.
- `single-zone cylinder model`: a cylinder model that tracks one thermodynamic gas zone per cylinder rather than separate burned / unburned spatial zones.
- `ODE`: ordinary differential equation.
- `RK3` / `RK4`: explicit third-order / fourth-order Runge-Kutta time integration formulas.
- `PI controller`: proportional-integral controller.
- `bench`: the automatic torque-curve measurement workflow inside the GUI. It is a software test-bench mode, not a claim that the program is attached to physical dyno hardware.
- `dyno`: the absorber / load-holding surrogate used by the bench path to hold target RPM and report brake torque.
- `sweep`: an ordered series of operating points, usually across RPM, used to produce a torque or power curve.
- `lambda`: the air-fuel equivalence ratio relative to stoichiometric air-fuel ratio. `lambda < 1` is rich, `lambda = 1` is stoichiometric, and `lambda > 1` is lean.
- `YAML`: the human-readable configuration-file format used for checked-in inputs.
- `CSV`: comma-separated values output files written by the headless path and by bench export.
- `VVT`: variable valve timing.
- `EGR`: exhaust gas recirculation. `internal EGR` in this repo means residual gas retained by valve overlap and backflow rather than by an external recirculation loop.
- `SI`: spark ignition.
- `WOT`: wide-open throttle.
- `NA`: naturally aspirated.
- `OEM`: original equipment manufacturer. In this README it means manufacturer-published specifications used as external reference anchors.
- `plenum`: the shared intake air volume upstream of the intake runners.
- `runner`: an intake or exhaust duct between a common volume and the cylinder-side boundary.
- `collector`: the shared exhaust volume downstream of the exhaust runners / header primaries.
- `degCA`: crank-angle degrees over a `0 .. 720` four-stroke engine cycle.
- `TDC` / `BDC`: top dead center / bottom dead center.
- `BTDC`: before top dead center.
- `VE`: volumetric efficiency.
- `IMEP` / `BMEP`: indicated / brake mean effective pressure.
- `BSFC`: brake-specific fuel consumption.
- `APK`: Android application package.

Whenever a later section uses one of these abbreviations without re-expanding it, this section is the authoritative definition for this file.

## Windows 11 Binary Download

Prebuilt portable Windows x64 binaries are intended to be distributed from the GitHub Releases page of this repository as individual assets rather than as a zip archive.

Download these release assets:

- `es_sim.exe`
- `sim.yaml`
- `es_sim-readme-en.pdf`
- `es_sim-readme-ja.pdf`

Optional supporting assets:

- `LICENSE`

To run on Windows 11:

1. Create or choose any writable folder.
2. Put `es_sim.exe` and `sim.yaml` in the same folder.
3. Run `es_sim.exe`.

The binary still accepts the historical `config/sim.yaml` path if you prefer the old subfolder layout.

Each release also publishes a matching SHA-256 manifest for the Windows runtime assets:

```text
es_sim-windows-x64-v0.1.0.sha256
```

If you download from a successful CI run instead of a formal release, the Actions artifact names are:

- `es_sim-windows-x64-snapshot`
- `es_sim-docs-pdf-snapshot`

## Run From Source

```bash
cargo run --release
```

## Dependency-Free CLI

The repository also ships a std-only headless CLI path. `es_cli` is the primary name and `es_hp` remains as a compatibility alias.

```bash
cargo run --no-default-features --bin es_cli -- validate config/high_precision.yaml
cargo run --no-default-features --bin es_cli -- sweep config/high_precision.yaml
cargo run --no-default-features --bin es_cli -- point config/high_precision.yaml --rpm 3500 --write-pv
```

Supported commands:

- `validate`: parse YAML, run strict range validation, and print derived engine metrics
- `sweep`: run the configured torque sweep, write `torque_curve.csv`, `torque_summary.csv`, optional `pv_<rpm>rpm.csv`, and `sweep_report.txt`
- `point`: solve one operating point and optionally write `pv_<rpm>rpm.csv` plus `point_<rpm>rpm_summary.txt`
- `list-presets`: list YAML presets under `config/` and `config/presets/`

## Android (Experimental)

The GUI path can now be built as an Android shared library and launched with `cargo apk`.
The Android entry point lives in `src/lib.rs`, the mobile `eframe` launcher is in `src/dashboard.rs`, and Android currently boots from an embedded copy of `config/sim.yaml` instead of reading an external file from device storage.

Quick start:

```bash
cargo apk run --lib --release --target aarch64-linux-android
```

See [ANDROID.md](ANDROID.md) for setup, current limitations, and the exact build flow.

## Stage Local Windows Release Assets

Build the release binary first, then stage the flat Windows runtime assets published to GitHub Releases:

```powershell
cargo build --release
powershell -ExecutionPolicy Bypass -File .\scripts\prepare-release-assets.ps1 -Tag v0.1.0
```

The generated files are written to `dist/release-assets/`.
README PDFs are still produced separately by `scripts/build-doc-pdfs.sh` or by the `release-windows` workflow.

If you still want a local portable zip, `scripts/package-windows.ps1` remains available.

For documented headless calibration targets and their current fit quality against OEM specs, see [ENGINE_REFERENCE_TARGETS.md](ENGINE_REFERENCE_TARGETS.md).

## GitHub Automation

This repository includes two GitHub Actions workflows relevant to distribution:

- `ci`: runs on push / pull request, builds/tests the release binary, uploads a portable Windows snapshot zip, and uploads English/Japanese README PDFs as Actions artifacts
- `release-windows`: runs on tag push `v*` or manual dispatch, publishes `es_sim.exe`, `sim.yaml`, `LICENSE`, a `.sha256` manifest, and English/Japanese README PDFs to GitHub Releases

Maintainer release flow:

1. Update `Cargo.toml` version if needed.
2. Push a release tag such as `v0.1.0`.

```bash
git tag v0.1.0
git push origin v0.1.0
```

3. Wait for the `release-windows` workflow to finish.
4. Download the generated release assets from the GitHub Release page.

If you prefer manual release creation from the Actions tab, run `release-windows` with a `tag_name` input. The workflow will create the GitHub release if it does not already exist.

## Keyboard Shortcuts

- `A`: Auto Start + Idle ON/OFF
- `O`: Auto WOT Efficiency Search ON/OFF
- `B`: Run/stop the selected automatic bench sweep
- `L`: Run/stop the dedicated `lambda=1` automatic bench sweep
- `S`: Starter ON/OFF
- `I`: Spark ON/OFF
- `F`: Fuel ON/OFF
- `W` / `X`: Throttle up/down
- `Q`: Quit

## Configuration (YAML)

Configuration is loaded from `config/sim.yaml`.
The release binary also falls back to `sim.yaml` placed beside `es_sim.exe` so flat GitHub Release downloads work without creating a `config/` subfolder.

Tuning constants that were previously hard-coded are now centralized in YAML sections:

- `auto_control.*`: auto start/idle PI gains and thresholds
- `bench.*`: automatic continuous-sweep bench settings, point-sample bench settings, bench dyno speed-hold / absorber settings, and bench mixture targets
- `model.*`: combustion, flow, p-V, torque smoothing, state-limit coefficients
- `numerics.*`: RPM-linked timestep and realtime-floor estimator coefficients
- `plot.*`: history-buffer length and fixed plot ranges
- `ui.*`: GUI cadence, plot scaling, slider/key-step behavior
- `audio.model.*`: firing-frequency / exhaust-pressure synth coefficients

Use `config/sim.yaml` as the authoritative full parameter list.

```yaml
environment:
  ambient_pressure_pa: 101325.0
  intake_temp_k: 305.0
  exhaust_temp_k: 880.0
  dt: 0.001

engine:
  compression_ratio: 13.0
  bore_m: 0.0805
  stroke_m: 0.0976
  inertia_kgm2: 0.16
  friction_c0_nm: 5.1
  friction_c1_nms: 0.0040
  friction_c2_nms2: 0.000020
  intake_volume_m3: 0.0032
  intake_runner_volume_m3: 0.00040
  exhaust_volume_m3: 0.0050
  exhaust_runner_volume_m3: 0.00055
  throttle_area_max_m2: 0.0030
  tailpipe_area_m2: 0.0025
  idle_target_rpm: 850.0
  max_rpm: 7000.0

cam:
  intake_centerline_deg: 470.0
  exhaust_centerline_deg: 250.0
  intake_duration_deg: 248.0
  exhaust_duration_deg: 244.0
  intake_max_lift_mm: 10.5
  exhaust_max_lift_mm: 9.8
  display_y_max_mm: 14.0

audio:
  output_gain: 0.0
  sample_rate_hz: 48000

control_defaults:
  throttle_cmd: 0.05
  load_cmd: 0.0
  starter_cmd: false
  spark_cmd: false
  fuel_cmd: false
  ignition_timing_deg: 12.0
  vvt_intake_deg: 0.0
  vvt_exhaust_deg: 0.0

plot:
  rpm_history_capacity: 200
  pv_recent_cycles: 4
  pv_subsamples_per_step: 120
  pv_x_min: 0.0
  pv_x_max: 1.15
  pv_y_min_kpa: 0.0
  pv_y_max_kpa: 2500.0
```

## Visualization Scope

The current dashboard is best interpreted as a mean-value engine / drivability / controls view, not as a full combustion-analysis workstation.

- RPM, net-torque, external-load-torque, and trapped-air history plots use crank angle on the x-axis, fixed to `0..720 degCA`. The dashboard keeps the most recent completed `N` cycles plus the current in-progress cycle so a full trace remains visible when the crank angle wraps back to zero.
- The `p-V` plot uses normalized cylinder volume and a reconstructed cylinder-pressure model driven by the same fueling and burn-phasing inputs as the torque model. It is intended for qualitative cycle-shape inspection and indicated-work estimation, not as a direct substitute for measured in-cylinder pressure traces.
- The cam plot is a valve-lift / phasing visualization. It is not yet a curtain-area or event-timing analysis view.
- The automatic bench view now has two layers: an instant preview curve computed from a coarse locked-cycle surrogate so a dyno-like torque/power envelope appears immediately, and the budgeted continuous WOT sweep that refines the same RPM bins afterward. The default bench mode is `rich_charge_cooling`, and a dedicated `lambda=1` bench is also available. The measured sweep now uses a bench-only speed-hold dyno controller that applies absorber load automatically instead of hard-clamping crank speed, so the displayed bench torque is the dyno brake output rather than the residual shaft-acceleration torque. Unlike the live dashboard path, the measured bench solver is not constrained by the realtime floor: it uses a bench-specific crank-angle target timestep plus step-doubling error checks before each accepted step.
- Completed GUI bench runs are exported automatically to `dist/bench/bench-rich_charge_cooling-latest.csv` or `dist/bench/bench-lambda_one-latest.csv`.
- The GUI shows gross combustion power, net brake power, brake BMEP, and commanded external load separately. At unloaded idle or free-rev steady states, net brake torque can settle near zero because combustion torque is mostly consumed by friction and pumping, so near-zero brake power there is expected.
- The GUI already shows lambda target, charge temperature, displayed VE, power, and brake BMEP. Metrics still missing for a full combustion-analysis workflow include `p-theta`, `PCP`, `CA10/50/90`, apparent heat-release, and `BSFC`.
- The Auto WOT Efficiency Search still uses a reduced-order model. Ignition is searched locally around the current best point, but intake and exhaust VVT are now scanned across the configured full range. Any high-RPM overlap trend should therefore be read as an indicator from the grouped-runner / grouped-exhaust model, not as a calibrated 1D gas-dynamics result.

## Implementation-Exact Mathematical Model

This section maps directly to `src/simulator.rs`, `src/dashboard.rs`, and `src/audio.rs`.

The engine model is a 0D mean-value formulation, not a full CFD or multi-zone cycle solver [S1].
Its physics split is:

- first-principles states: crank dynamics and intake/exhaust manifold pressure dynamics
- first-principles closures: ideal-gas relations, compressible throttle/tailpipe orifice flow, Otto-efficiency reference [S2, S3]
- semi-physical thermochemistry: fresh-charge evaporative cooling, burned-gas `c_p / \gamma` variation, and overlap-backflow internal-EGR mixing
- reduced empirical closures: volumetric efficiency, combustion phasing loss, friction, starter map
- reconstructed visualization: the plotted `p-V` loop is a thermodynamically informed display model driven by the same fueling and burn-phasing inputs, but it is not itself a solved cylinder state ODE

For GitHub rendering robustness, block equations in this README use fenced `math` blocks instead of bare `$$...$$` [S8].
If you publish the same Markdown through GitHub Pages, prefer the `GFM` Markdown processor so the Pages output stays aligned with the repository view [S8].

For bench mode only, the time integration policy differs from the realtime dashboard path. The GUI bench first draws an instant preview from a coarse locked-cycle average and then warms at the start RPM before sweeping continuously according to

```math
\mathrm{rpm}_{target}(t)=
\mathrm{rpm}_{start}
\;+\;
\left(\mathrm{rpm}_{end}-\mathrm{rpm}_{start}\right)
\frac{t}{t_{sweep}},
\qquad 0 \le t \le t_{sweep}
```

and samples are accumulated into the nearest configured RPM bin. The accepted timestep still starts from a crank-angle target

```math
\Delta t_{bench,nom}=
\mathrm{clamp}\left(
\frac{\Delta \theta_{bench}}{6\,\mathrm{rpm}},
\Delta t_{bench,min},
\Delta t_{bench,max}
\right)
```

and uses RK2 step-doubling on the state vector

```math
\varepsilon_{bench}
=
\left\lVert
\mathbf{x}_{RK2}(\Delta t)-\mathbf{x}_{RK2}\!\left(\tfrac{\Delta t}{2}\right)^{(2)}
\right\rVert_{norm}
```

to halve the trial timestep until $\varepsilon_{bench}\le \varepsilon_{tol}$ or the configured refinement limit is reached.

### 1. State, inputs, outputs

State vector:

```math
\mathbf{x}
=
\begin{bmatrix}
\omega & \theta & p_{im} & p_{ir} & p_{em} & p_{er} & \dot m_{ir} & \dot m_{er} & \alpha_{th}
\end{bmatrix}^{\mathsf T}
```

- $\omega$: crank angular speed $[\mathrm{rad/s}]$
- $\theta$: crank angle $[\mathrm{rad}]$ (wrapped in $[0,4\pi)$)
- $p_{im}$: intake plenum pressure upstream of the runner $[\mathrm{Pa}]$
- $p_{ir}$: intake runner / port pressure at the cylinder side $[\mathrm{Pa}]$
- $p_{em}$: exhaust collector pressure downstream of the runner $[\mathrm{Pa}]$
- $p_{er}$: exhaust runner / header-primary pressure at the cylinder side $[\mathrm{Pa}]$
- $\dot m_{ir}$: intake runner flow state, positive from plenum to runner $[\mathrm{kg/s}]$
- $\dot m_{er}$: exhaust runner flow state, positive from runner to collector $[\mathrm{kg/s}]$
- $\alpha_{th}$: effective throttle opening $[-]$

Control input:

```math
\mathbf{u}
=
\begin{bmatrix}
\alpha_{cmd} & u_{load} & u_{st} & u_{spk} & u_f & \Delta\theta_{ign} & \Delta\theta_{VVT,I} & \Delta\theta_{VVT,E}
\end{bmatrix}^{\mathsf T}
```

- $\alpha_{cmd}$: throttle command
- $u_{load}$: normalized external load command
- $u_{st}$: starter command (bool)
- $u_{spk}$: spark command (bool)
- $u_f$: fuel command (bool)
- $\Delta\theta_{ign}$: ignition timing command [deg BTDC]
- $\Delta\theta_{VVT,I}, \Delta\theta_{VVT,E}$: intake/exhaust VVT offsets [degCA]

### 2. Continuous-time ODEs

```math
\dot \omega = \frac{\tau_{comb}+\tau_{start}-\tau_{fric}-\tau_{pump}-\tau_{load}}{J}
```

```math
\dot \theta = \omega
```

```math
\dot p_{im} = \frac{R_{air}T_{im}}{V_{im}}\left(\dot m_{th}-\dot m_{ir}\right)
```

```math
\dot p_{ir} = \frac{R_{air}T_{im}}{V_{ir}}\left(\dot m_{ir}-\dot m_{cyl}\right)
```

```math
\dot p_{em} = \frac{R_{air}T_{exh,eff}}{V_{em}}\left(\dot m_{er}-\dot m_{tail}\right)
```

```math
\dot p_{er} = \frac{R_{air}T_{exh,eff}}{V_{er}}\left(\dot m_{exh,in}-\dot m_{er}\right)
```

```math
\frac{d\dot m_{ir}}{dt}
=
\frac{\left(p_{im}-p_{ir}\right)-\Delta p_{loss,ir}}{L_{ir}}
- d_{ir}\dot m_{ir}
```

```math
\frac{d\dot m_{er}}{dt}
=
\frac{\left(p_{er}-p_{em}\right)-\Delta p_{loss,er}}{L_{er}}
- d_{er}\dot m_{er}
```

```math
\dot \alpha_{th} = \frac{\alpha_{cmd}-\alpha_{th}}{\tau_{th}},\quad \tau_{th}=0.060\ \mathrm{s}
```

The runner-loss terms used in the solved ODE are

```math
\Delta p_{loss}
=
\operatorname{sgn}\!\left(\dot m\right)
\left(f\frac{L}{D}+K\right)
\frac{\dot m^2}{2\rho A_{eff}^2}
```

with density estimated from the mean upstream/downstream pressure and the corresponding gas temperature.

The mass-flow closures inserted into the pressure states are

```math
\dot m_{cyl}
=
\dot m_{cyl,mean}\,
\phi_I(\theta)\,
\psi_{I,wave}(\theta)
```

```math
\dot m_{exh,in}
=
\left(\dot m_{cyl,mean}+\dot m_f\right)\,
\phi_E(\theta)\,
\psi_{E,wave}(\theta)
```

where `\phi_I,\phi_E` are the normalized valve-event pulse factors and `\psi_{I,wave},\psi_{E,wave}` are the instantaneous wave-action multipliers.

The fired-path charge-state closures that feed combustion, heat loss, and exhaust temperature are

```math
x_{egr}
=
\operatorname{clamp}\!\left[
\chi_{ov}
\left(
x_{0}
+k_p\Delta p_{back}^{+}
+k_w s_{scav}^{-}
+k_r \dot m_{rev}^{+}
\right)
\right]
```

```math
c_{p,burn}
=
\operatorname{clamp}\!\left(
c_{p,ref}
+k_T\left(T_{res}-T_{ref}\right)
+k_{egr}x_{egr}
\right)
```

```math
\gamma_{mix}
=
\frac{c_{p,mix}}{c_{p,mix}-R_{air}}
```

```math
T_{charge}
=
\frac{m_{fresh} c_{p,f}T_f + m_r c_{p,burn}T_{res}}
{m_{fresh} c_{p,f}+m_r c_{p,burn}}
```

with `x_{egr}` derived from overlap lift `\chi_{ov}`, exhaust-over-intake backflow head `\Delta p_{back}^{+}`, negative scavenging head `s_{scav}^{-}`, and reverse exhaust-runner flow `\dot m_{rev}^{+}`.

#### Running steady-state interpretation

A fired engine that is "steady" in the usual calibration sense is not a static equilibrium of the full state vector, because

```math
\dot \theta = \omega
```

and the gas-exchange / combustion closures are periodic functions of crank angle.  
For this model, the correct equilibrium notion is therefore a periodic orbit, or equivalently a fixed point of a Poincare map on a fixed crank-angle section.

Define the reduced running-state vector without the wrapped phase coordinate:

```math
\mathbf{x}_r
=
\begin{bmatrix}
\omega & p_{im} & p_{ir} & p_{em} & p_{er} & \dot m_{ir} & \dot m_{er} & \alpha_{th}
\end{bmatrix}^{\mathsf T}
```

At a chosen phase section $\theta=\theta_s$ sampled once every $720\ \mathrm{degCA}$, the simulator induces the cycle map

```math
\mathbf{x}_{r,k+1} = \mathcal{P}\!\left(\mathbf{x}_{r,k}\right)
```

and steady running means

```math
\mathbf{x}_r^* = \mathcal{P}\!\left(\mathbf{x}_r^*\right)
```

An equivalent cycle-averaged statement is

```math
\frac{\mathbf{x}_r(t+T_c)-\mathbf{x}_r(t)}{T_c}
=
\frac{1}{T_c}\int_t^{t+T_c}\dot{\mathbf{x}}_r(\tau)\,d\tau
=
\mathbf{0},
\qquad
T_c=\frac{4\pi}{\omega}
```

The regression test `representative_operating_points_are_periodic_steady_states` evaluates this exact idea in code: after settling fixed controls, it samples the reduced state on the same $\theta_s=360\ \mathrm{degCA}$ section across successive cycles and checks that the cycle-to-cycle drift is small.

### 3. Auto start + idle controller (discrete logic)

Enabled only when auto mode is ON:

```math
u_{spk}\leftarrow 1,\quad u_f\leftarrow 1
```

Cranking condition:

```math
\text{cranking} = (\neg running)\ \lor\ (\mathrm{rpm}<600)
```

```math
u_{st} \leftarrow \text{cranking} \land (\mathrm{rpm}<760)
```

Target RPM:

```math
\mathrm{rpm}_{target}=
\begin{cases}
620, & u_{st}=1\\
\mathrm{rpm}_{idle}, & u_{st}=0
\end{cases}
```

Error:

```math
e = \mathrm{rpm}_{target}-\mathrm{rpm}
```

Controller gains:

```math
(k_p,k_i,b)=
\begin{cases}
(2.2\times 10^{-4},\ 8.0\times 10^{-5},\ 0.14), & u_{st}=1\\
(1.2\times 10^{-4},\ 5.0\times 10^{-5},\ 0.040), & u_{st}=0
\end{cases}
```

Integral state:

```math
I \leftarrow \mathrm{clamp}\left(I + e\Delta t,\ -600,\ 600\right)
```

Throttle command:

```math
\alpha_{cmd}\leftarrow \mathrm{clamp}\left(b+k_pe+k_iI,\ 0.02,\ 0.42\right)
```

Anti-windup decay:

```math
\mathrm{if}\ \mathrm{rpm}>\mathrm{rpm}_{target}+120,\quad I\leftarrow 0.85I
```

### 4. Gas exchange and compressible flow

Effective throttle area map:

```math
A_{th}=A_{th,max}\cdot\mathrm{clamp}\left(0.005+0.98\,\alpha_{th}^{1.8},\ 0.005,\ 1.0\right)
```

The code uses discharge-area factors:

```math
(C_dA)_{th}=0.82\,A_{th},\qquad (C_dA)_{tail}=0.87\,A_{tail}
```

Compressible orifice model (`orifice_mass_flow`; quasi-1D isentropic nozzle relation [S2]):

```math
\Pi=\mathrm{clamp}\!\left(\frac{p_d}{p_u},0,1\right),\quad
\Pi_*=\left(\frac{2}{\gamma+1}\right)^{\frac{\gamma}{\gamma-1}}
```

```math
\dot m=
\begin{cases}
C_dA\frac{p_u}{\sqrt{T_u}}
\left(\frac{\gamma}{R}\right)^{1/2}
\left(\frac{2}{\gamma+1}\right)^{\frac{\gamma+1}{2(\gamma-1)}},
& \Pi\le \Pi_*\\[6pt]
C_dA\frac{p_u}{\sqrt{T_u}}
\left[
\frac{2\gamma}{R(\gamma-1)}
\left(\Pi^{2/\gamma}-\Pi^{(\gamma+1)/\gamma}\right)
\right]^{1/2},
& \Pi>\Pi_*
\end{cases}
```

```math
\dot m \leftarrow \max(\dot m,0)
```

Applied in this simulator:

```math
\dot m_{th}=\dot m_{orifice}\!\left((C_dA)_{th},\ p_{amb},\ p_{im},\ T_{im}\right)
```

```math
\dot m_{tail}=\dot m_{orifice}\!\left((C_dA)_{tail},\ p_{em},\ p_{amb},\ T_{exh,eff}\right)
```

Volumetric efficiency:

```math
\eta_v=\mathrm{clamp}\left(\eta_{rpm}\eta_{vvt}\eta_{th}g_{ov}g_{wave},\ 0.30,\ 1.08\right)
```

```math
\eta_{rpm}=\mathrm{clamp}\left(0.78+0.22\exp\!\left(-\frac{(\mathrm{rpm}-4800)^2}{8.0\times 10^6}\right),\ 0.58,\ 1.00\right)
```

```math
\eta_{vvt}=1+0.0022\,\Delta\theta_{VVT,I}-0.0016\,\Delta\theta_{VVT,E}
```

```math
\eta_{th}=\mathrm{clamp}\left(0.40+0.62\sqrt{\alpha_{th}},\ 0.38,\ 1.03\right)
```

Overlap / exhaust-interference correction:

```math
\ell_{ov}=\min\!\left(\tilde l_I(360^\circ),\tilde l_E(360^\circ)\right)
```

```math
g_{ov}=
\mathrm{clamp}\left(
1+\ell_{ov}
\left[
k_p\frac{p_{ir}-p_{er}}{p_{amb}}
+
k_f\frac{\dot m_{er}}{\dot m_{ref}}
\right],
g_{min},g_{max}
\right)
```

with
$k_p=0.22$,
$k_f=0.08$,
$\dot m_{ref}=0.08\ \mathrm{kg/s}$,
$g_{min}=0.88$,
$g_{max}=1.12$.

Grouped intake / exhaust wave-action closure:

```math
G_I=\max\left\{d\in\mathbb{N}\mid d\le G_I^*,\ d\mid N_{cyl}\right\},\qquad
G_E=\max\left\{d\in\mathbb{N}\mid d\le G_E^*,\ d\mid N_{cyl}\right\}
```

```math
a_I=\sqrt{\gamma R T_{im}},\qquad
a_E=\sqrt{\gamma R T_{exh,eff}}
```

```math
f_{q,I}=\frac{a_I}{4L_I},\qquad
f_{q,E}=\frac{a_E}{4L_E}
```

```math
\tau_{d,I}=s_I\frac{L_I}{a_I},\qquad
\tau_{d,E}=s_E\frac{L_E}{a_E}
```

```math
K(t;\tau_d,\tau_{dec},f_q)=
\begin{cases}
0, & t<\tau_d\\[4pt]
\exp\!\left[-\frac{t-\tau_d}{\tau_{dec}}\right]
\cos\!\left(2\pi f_q(t-\tau_d)\right), & t\ge \tau_d
\end{cases}
```

For each intake or exhaust branch group $g$, the code sums recent valve-event impulses from the cylinders bundled into that branch:

```math
\Delta p_{I,g}(\theta)=
\mathrm{clamp}\left(
k_{p,I}p_{amb}\frac{\dot m_{I,src}}{\dot m_{I,ref}}
\frac{1}{n_{g,I}}
\sum_{c\in g}\sum_{n=0}^{N_{mem}-1}
K_I\!\left(\frac{\Delta\theta_{c}(\theta)+720n}{6\,\mathrm{rpm}}\right),
-\Delta p_{I,max},
\Delta p_{I,max}
\right)
```

```math
\Delta p_{E,g}(\theta)=
\mathrm{clamp}\left(
-k_{p,E}p_{amb}\frac{\dot m_{E,src}}{\dot m_{E,ref}}
\frac{1}{n_{g,E}}
\sum_{c\in g}\sum_{n=0}^{N_{mem}-1}
K_E\!\left(\frac{\Delta\theta_{c}(\theta)+720n}{6\,\mathrm{rpm}}\right),
-\Delta p_{E,max},
\Delta p_{E,max}
\right)
```

where $n_{g,\bullet}$ is the number of cylinders inside the branch group and $\Delta\theta_c(\theta)$ is the positive crank-angle distance from the most recent source event of cylinder $c$ to the current evaluation angle $\theta$.

Current valve-window weighted wave pressures:

```math
\bar{\Delta p}_{I,cur}=
\frac{\sum_g w_{I,g}(\theta)\Delta p_{I,g}(\theta)}
{\sum_g w_{I,g}(\theta)},\qquad
\bar{\Delta p}_{E,cur}=
\frac{\sum_g w_{E,g}(\theta)\Delta p_{E,g}(\theta)}
{\sum_g w_{E,g}(\theta)}
```

where the weights are sums of normalized valve lifts for the cylinders connected to group $g$.

Event-averaged wave pressures used in VE:

```math
\bar{\Delta p}_{I,IVC}=
\frac{1}{N_{cyl}}\sum_{c=1}^{N_{cyl}}\Delta p_{I,g(c)}(\theta_{IVC,c})
```

```math
\bar{\Delta p}_{I,ov}=
\frac{1}{N_{cyl}}\sum_{c=1}^{N_{cyl}}\Delta p_{I,g(c)}(\theta_{TDC,ov,c}),\qquad
\bar{\Delta p}_{E,ov}=
\frac{1}{N_{cyl}}\sum_{c=1}^{N_{cyl}}\Delta p_{E,g(c)}(\theta_{TDC,ov,c})
```

```math
g_{ram}=1+k_{ram}\frac{\bar{\Delta p}_{I,IVC}}{p_{amb}}
```

```math
g_{scav}=1+k_{scav}\frac{\bar{\Delta p}_{I,ov}-\bar{\Delta p}_{E,ov}}{p_{amb}}
```

```math
g_{wave}=\mathrm{clamp}\left(g_{ram}g_{scav},\ g_{wave,min},\ g_{wave,max}\right)
```

Valve-event pulse factors:

```math
\phi_I = 1+\beta_I(\tilde l_I-1),\qquad
\phi_E = 1+\beta_E(\tilde l_E-1)
```

where $\tilde l_I,\tilde l_E$ are normalized aggregate lift sums over all cylinders, with cycle mean equal to 1, and
$\beta_I=0.32$,
$\beta_E=0.40$.

Effective cylinder-side boundary pressures:

```math
p_{I,cyl} = (1-w_I)p_{im}+w_Ip_{ir}+\bar{\Delta p}_{I,IVC},\qquad
p_{E,cyl} = (1-w_E)p_{em}+w_Ep_{er}+\bar{\Delta p}_{E,ov}
```

with
$w_I=0.35$ and
$w_E=0.45$.

Cylinder air consumption:

```math
f_{cyc}=\frac{\mathrm{rpm}}{120},\qquad
m_{air,cycle}= \frac{V_d\eta_v p_{I,cyl}}{R_{air}T_{charge}}
```

```math
\dot m_{cyl,mean}=\max\left(m_{air,cycle}f_{cyc},0\right),\qquad
\psi_I=\max\left(1+k_{\psi,I}\frac{\bar{\Delta p}_{I,cur}}{p_{amb}},0\right)
```

```math
\dot m_{cyl}=\phi_I\psi_I\dot m_{cyl,mean}
```

Exhaust delivery into the runner state:

```math
\psi_E=\max\left(1-k_{\psi,E}\frac{\bar{\Delta p}_{E,cur}}{p_{amb}},0\right)
```

```math
\dot m_{exh,in}=\phi_E\psi_E\dot m_{exh,mean}
```

Trapped air per cylinder:

```math
m_{air,cyl}=\frac{V_d/N_{cyl}\cdot\eta_v p_{I,cyl}}{R_{air}T_{charge}}
```

### 5. Combustion and torque

Spark timing and burn duration:

```math
\Delta\theta_{spark}=\mathrm{clamp}\left(\Delta\theta_{ign},\ -5,\ 45\right)
```

```math
\theta_{start}=360-\Delta\theta_{spark}+0.08\,\Delta\theta_{VVT,I}
```

```math
\Delta\theta_b=\mathrm{clamp}\left(66-8\alpha_{th},\ 38,\ 75\right)
```

MBT reference and timing error:

```math
\Delta\theta_{MBT}=\mathrm{clamp}\left(12+0.0018(\mathrm{rpm}-900)+3(1-load),\ 8,\ 26\right)
```

```math
e_{ign}=\Delta\theta_{spark}-\Delta\theta_{MBT}
```

Combustion phasing efficiency:

```math
\eta_{phase}=
\begin{cases}
\exp\!\left[-\left(\frac{e_{ign}}{6.5}\right)^2\right], & e_{ign}\ge 0\\[4pt]
\exp\!\left[-\left(\frac{e_{ign}}{15}\right)^2\right], & e_{ign}<0
\end{cases}
```

Stability gate (very advanced/retarded timing -> no combustion):

```math
phase\_stable = (e_{ign}\le 14)\land (e_{ign}\ge -32)
```

Multi-cylinder Wiebe burn-rate proxy:

```math
\theta_c = \left(\theta_{deg}-c\frac{720}{N_{cyl}}\right)\bmod 720,\quad
x_c=\frac{\theta_c-\theta_{start}}{\Delta\theta_b}
```

For each cylinder $c\in\{0,\dots,N_{cyl}-1\}$:

```math
\frac{dx_{b,c}}{d\theta}=
\begin{cases}
\dfrac{a(m+1)}{\Delta\theta_b}x_c^m e^{-ax_c^{m+1}}, & 0\le x_c\le 1\\
0, & \text{otherwise}
\end{cases}
```

with $a=5.2$, $m=2.0$.

Summed and normalized:

```math
r_{burn}=
\frac{\sum_c dx_{b,c}/d\theta}{N_{cyl}/720}
```

Fueling:

```math
\lambda=
\begin{cases}
\lambda_{override}, & \text{if a bench mode forces lambda}\\
\mathrm{clamp}(1.15-0.35\alpha_{th},\ 0.82,\ 1.12), & \text{otherwise}
\end{cases}
```

The bench-mode override is:

```math
\lambda_{override}=
\begin{cases}
\lambda_{bench,\ rich}=0.88, & \text{rich charge-cooling bench}\\
\lambda_{bench,\ 1}=1.0, & \text{lambda=1 bench}
\end{cases}
```

```math
AFR=14.7\lambda
```

```math
m_{f,cycle,cyl}=\frac{m_{air,cyl}}{AFR}
```

Charge cooling from fuel evaporation (bench modes only):

```math
Q_{evap}=m_{f,guess}h_{fg}\chi_{evap}
```

```math
T_{charge}=\mathrm{clamp}\left(
T_{im}-\frac{Q_{evap}}{(m_{air,guess}+m_{f,guess})c_p},
T_{charge,min},
T_{im}
\right)
```

The implementation evaluates a first-pass trapped air and fuel mass at $T_{im}$, computes $T_{charge}$, and then recomputes $m_{air,cyl}$ and $\dot m_{cyl,mean}$ with that cooled charge temperature.
Because $m_{f,guess}$ is larger in the rich bench mode than in the `lambda=1` bench mode, the rich bench gets a larger evaporative-cooling effect and therefore a slightly denser trapped charge.

Empirical thermal efficiency:

```math
load=\mathrm{clamp}\left(\frac{p_{I,cyl}}{p_{amb}},\ 0.20,\ 1.20\right)
```

```math
\eta_{th,base}=\mathrm{clamp}\left(0.17+0.22\,load-1.2\times 10^{-5}\left|\mathrm{rpm}-2500\right|,\ 0.08,\ 0.40\right)
```

Woschni-inspired single-zone wall heat loss:

```math
T_{comp}=T_{charge}r_c^{\gamma-1},\qquad
c_p=\frac{\gamma R}{\gamma-1}
```

```math
T_g=\mathrm{clamp}\left(T_{comp}+k_{\Delta T}\eta_{phase}\frac{m_{f,cycle,cyl}LHV}{(m_{air,cyl}+m_{f,cycle,cyl})c_p},\ T_{im},\ T_{g,max}\right)
```

```math
h=h_0\left(\frac{p_c}{p_{ref}}\right)^{a_p}
\left(\frac{T_g}{T_{ref}}\right)^{a_T}
\left(\frac{U_p}{U_{ref}}\right)^{a_u}
```

```math
Q_{loss}=
\mathrm{clamp}\left(
hA_w\max(T_g-T_w,0)\tau_{exp},
0,
\chi_{max}m_{f,cycle,cyl}LHV
\right)
```

```math
\eta_{th,emp}=
\mathrm{clamp}\left(
\eta_{th,base}\eta_{phase}-\frac{Q_{loss}}{m_{f,cycle,cyl}LHV},
0.02,
0.42
\right)
```

Cycle work and mean combustion torque:

```math
W_{cyc,cyl}=m_{f,cycle,cyl}\,LHV\,\eta_{th,emp}
```

```math
\tau_{comb,mean}=\frac{W_{cyc,cyl}\,N_{cyl}}{4\pi}
```

Combustion enable condition:

```math
enabled=
u_f\land u_{spk}
\land (\mathrm{rpm}>120)
\land (p_{I,cyl}>25000)
\land phase\_stable
\land (u_{st}\lor running\lor \mathrm{rpm}>450)
```

Exhaust-temperature submodel (retard raises $T_{exh}$, advance lowers it):

```math
s_T=\mathrm{clamp}\left(1+0.024\max(-e_{ign},0)-0.010\max(e_{ign},0),\ 0.75,\ 1.70\right)
```

```math
T_{exh,phase}=\mathrm{clamp}(T_{exh,base}\,s_T,\ 500,\ 1900)\ \mathrm{K}
```

```math
T_{exh,eff}=\mathrm{clamp}\left(T_{exh,phase}\left[1-k_Q\frac{Q_{loss}}{m_{f,cycle,cyl}LHV}\right],\ 500,\ 1900\right)\ \mathrm{K}
```

Instantaneous combustion torque:

```math
\tau_{comb}=
\begin{cases}
\tau_{comb,mean}\cdot\mathrm{clamp}(r_{burn},0,5.5), & enabled\\
0, & \neg enabled
\end{cases}
```

Fuel and exhaust inflow rates:

```math
\dot m_f=
\begin{cases}
m_{f,cycle,cyl}\,f_{cyc}\,N_{cyl}, & enabled\\
0, & \neg enabled
\end{cases}
```

```math
\dot m_{exh,in}=\phi_E\left(\dot m_{cyl,mean}+\dot m_f\right)
```

### 6. Mechanical torques and mode switching

Friction torque:

```math
\tau_{fric}=c_0+c_1\omega+c_2\omega^2
```

Pumping torque:

```math
\tau_{pump}=\mathrm{clamp}\left(\frac{(p_{E,cyl}-p_{I,cyl})V_d}{4\pi},\ -12,\ 25\right)
```

Starter torque map:

```math
\tau_{start}=
\begin{cases}
62, & u_{st}=1\ \land\ \mathrm{rpm}<220\\
36, & u_{st}=1\ \land\ 220\le \mathrm{rpm}<450\\
0, & \text{otherwise}
\end{cases}
```

External load torque:

```math
\tau_{load,ref}
=
\mathrm{clamp}\left(
\tau_{L,0}+c_{L,1}\omega+c_{L,2}\omega^2,
\tau_{L,min},
\tau_{L,max}
\right)
```

```math
\tau_{load}
=
\begin{cases}
0, & u_{load}\le 0\\
u_{load}^{\gamma_L}\tau_{load,ref}, & u_{load}>0
\end{cases}
```

with default coefficients
$\gamma_L=1.20$,
$\tau_{L,0}=14\ \mathrm{Nm}$,
$c_{L,1}=0.012\ \mathrm{N\,m\,s/rad}$,
$c_{L,2}=1.8\times 10^{-4}\ \mathrm{N\,m\,s^2/rad^2}$,
$\tau_{L,min}=0$,
$\tau_{L,max}=220\ \mathrm{Nm}$.

Net torque:

```math
\tau_{net}=\tau_{comb}+\tau_{start}-\tau_{fric}-\tau_{pump}-\tau_{load}
```

Brake mean effective pressure shown in the GUI:

```math
\mathrm{BMEP}_{br}=\frac{4\pi\tau_{net}}{V_d}
```

State clamps after each RK2 step:

```math
\omega\leftarrow \max(\omega,0),\quad
\theta\leftarrow \theta \bmod 4\pi
```

```math
p_{im}\leftarrow \mathrm{clamp}(p_{im},18000,130000)
```

```math
p_{ir},p_{er}\leftarrow \mathrm{clamp}\left(p,18000,320000\right),\qquad
p_{em}\leftarrow \mathrm{clamp}\left(p_{em},0.78p_{amb},p_{amb}+120000\right)
```

```math
\dot m_{ir},\dot m_{er}\leftarrow \mathrm{clamp}\left(\dot m,\ -0.45,\ 0.45\right)\ \mathrm{kg/s}
```

```math
\alpha_{th}\leftarrow \mathrm{clamp}(\alpha_{th},0,1)
```

Running flag hysteresis:

```math
\mathrm{if}\ (u_{spk}\land u_f\land \mathrm{rpm}>520)\ \Rightarrow\ running\leftarrow 1
```

```math
\mathrm{if}\ (\mathrm{rpm}<260\land \neg u_{st})\ \Rightarrow\ running\leftarrow 0
```

### 7. Time integration and RPM-linked variable timestep

RK2 midpoint integration:

```math
\mathbf{k}_1=f(\mathbf{x}_n,\mathbf{u}),\quad
\mathbf{x}_{mid}=\mathbf{x}_n+\frac{\Delta t}{2}\mathbf{k}_1
```

```math
\mathbf{k}_2=f(\mathbf{x}_{mid},\mathbf{u}),\quad
\mathbf{x}_{n+1}=\mathbf{x}_n+\Delta t\,\mathbf{k}_2
```

Nominal RPM-linked timestep (`rpm_linked_dt`):

```math
\Delta t_{base}'=\max(\Delta t_{base},5\times 10^{-5}),\quad
\mathrm{rpm}_{idle}'=\max(\mathrm{rpm}_{idle},200)
```

```math
\Delta\theta_{target}=\mathrm{clamp}(6\Delta t_{base}'\mathrm{rpm}_{idle}',1,12)\ \ [\deg/\text{step}]
```

```math
\mathrm{rpm}_{eff}=\max(\mathrm{rpm},80),\quad
\Delta t_{raw}=\frac{\Delta\theta_{target}}{6\,\mathrm{rpm}_{eff}}
```

```math
\Delta t_{min,local}=\max(0.08\Delta t_{base}',5\times 10^{-5}),\quad
\Delta t_{max,local}=\min(2.5\Delta t_{base}',0.012)
```

```math
\Delta t_{nom}=\mathrm{clamp}(\Delta t_{raw},\Delta t_{min,local},\Delta t_{max,local})
```

Real-time floor from benchmark (`estimate_realtime_dt_floor`):

```math
\Delta t_{rt,min}=\mathrm{clamp}(1.25\,t_{wall/step},1\times 10^{-5},6\Delta t_{probe})
```

Global bounds in dashboard:

```math
\Delta t_{min}=\max(\Delta t_{rt,min},0.05\Delta t_{base}),\quad
\Delta t_{max}=\max(3\Delta t_{base},1.5\Delta t_{min})
```

Runtime step target:

```math
\Delta t_{target}=\mathrm{clamp}(\Delta t_{nom},\Delta t_{min},\Delta t_{max})
```

To avoid jitter, dashboard applies first-order smoothing to timestep:

```math
\Delta t_{next}\leftarrow \Delta t_{next}+0.25(\Delta t_{target}-\Delta t_{next})
```

Then each simulation substep uses:

```math
\Delta t_{step}=\max\left(\min(\Delta t_{next},\ t_{remaining}),10^{-6}\right)
```

The real-time frame budget is consumed by repeatedly applying RK2 with $\Delta t_{step}$ until simulated time catches elapsed wall time.

### 8. p-V model, subsampling, and smoothing

Per-cylinder normalized volume model:

```math
v_{min}=\frac{1}{r-1},\quad v_{max}=v_{min}+1
```

```math
v(\theta)=v_{min}+\frac{1}{2}(v_{max}-v_{min})\left(1-\cos(\theta\bmod 2\pi)\right)
```

Compression and expansion exponents are fixed:

```math
\gamma_c=1.34,\quad \gamma_e=1.24
```

Combustion strength from instantaneous combustion torque:

```math
s_{comb}=
\begin{cases}
1, & m_{f,cycle,cyl}>10^{-9}\\
0, & \text{otherwise}
\end{cases}
```

Peak-pressure helper terms:

```math
p_{comp,end}=p_{im}\left(\frac{v_{max}}{v_{min}}\right)^{\gamma_c}
```

```math
p_{comp,pk}=\mathrm{clamp}(p_{im}r^{1.32},\ p_{im},\ 6.0\times10^6)
```

```math
V_{d,cyl}=\frac{V_d}{N_{cyl}},\quad
V_{cyl,clr}=\max\left(\frac{V_{d,cyl}}{\max(r-1,1)},10^{-6}\right)
```

```math
load=\mathrm{clamp}\left(\frac{p_{im}}{p_{amb}},0.20,1.20\right),\quad
G_p=0.38\,load^{1.35}
```

```math
Q_{f,cycle}=m_{f,cycle,cyl}LHV,\quad
\Delta p_{comb}=G_p\,\eta_{phase}\frac{Q_{f,cycle}}{V_{cyl,clr}}
```

```math
p_{pk}=\mathrm{clamp}\left(p_{comp,pk}+\Delta p_{comb},\ p_{comp,pk},\ 8.0\times10^6\right)
```

In `instantaneous_pv_sample`, effective peak is:

```math
p_{pk}'=\max(p_{pk},p_{comp,end})
```

The p-V burn window now follows the same combustion phasing model used for torque generation:

```math
\theta_{SOC}=\theta_{b,start}=360-\Delta\theta_{ign}+0.08\Delta\theta_{VVT,I}
```

```math
\Delta\theta_b=
\mathrm{clamp}\left(66-8\alpha_{th},38,75\right)
```

```math
\theta_{EOC}=\theta_{SOC}+\Delta\theta_b
```

Blowdown timing remains fixed in the display model:

```math
\theta_{EVO}=565,\qquad \theta_{BD,end}=620\ \ [\deg]
```

Smoothstep:

```math
\mathrm{smoothstep}(x)=t^2(3-2t),\quad t=\mathrm{clamp}(x,0,1)
```

Burn progress:

```math
x_b=\mathrm{smoothstep}\left(\frac{\theta_{deg}-\theta_{SOC}}{\theta_{EOC}-\theta_{SOC}}\right)
```

```math
p_{tdc,fired}=p_{comp,end}+s_{comb}(p_{pk}'-p_{comp,end})x_b
```

Pressure piecewise model:

```math
p(\theta)=
\begin{cases}
p_{im}\left(1-0.02\sin^2\!\left(\pi\theta_{deg}/180\right)\right), & 0\le\theta_{deg}<180\\[4pt]
p_{im}\left(\dfrac{v_{max}}{v}\right)^{\gamma_c}, & 180\le\theta_{deg}<360\\[8pt]
\max\!\left(p_{tdc,fired}\left(\dfrac{v_{min}}{v}\right)^{\gamma_e},\ 1.05p_{exh}\right), & 360\le\theta_{deg}<\theta_{EVO}\\[8pt]
p_{pow,EVO}+(p_{exh}-p_{pow,EVO})\mathrm{smoothstep}\!\left(\dfrac{\theta_{deg}-\theta_{EVO}}{\theta_{BD,end}-\theta_{EVO}}\right), & \theta_{EVO}\le\theta_{deg}<\theta_{BD,end}\\[8pt]
p_{exh}+(p_{im}-p_{exh})\mathrm{smoothstep}\!\left(\dfrac{\theta_{deg}-\theta_{BD,end}}{720-\theta_{BD,end}}\right), & \theta_{BD,end}\le\theta_{deg}<720
\end{cases}
```

where

```math
p_{pow,EVO}=p_{tdc,fired}\left(\frac{v_{min}}{v(\theta_{EVO})}\right)^{\gamma_e}
```

Final clamp:

```math
p\leftarrow \max(p,20000\ \mathrm{Pa})
```

Subsampling of each integration step (`pv_subsamples_per_step = N_{sub}`):

```math
\theta_i=\theta_{start}+(\theta_{end}-\theta_{start})\frac{i}{N_{sub}},\quad i=1,\dots,N_{sub}
```

with linear interpolation of state variables between previous and current state for each subsample.

Recent-cycle retention:

```math
cycle_{min}=cycle_{current}-(N_{pv}-1)
```

and old samples with `cycle < cycle_min` are removed.

Display smoothing by crank-angle bins:

```math
k=\left\lfloor \frac{\theta_{deg}}{720}\cdot 1440\right\rfloor,\quad
\bar v_k=\frac{1}{n_k}\sum v_j,\quad
\bar p_k=\frac{1}{n_k}\sum p_j
```

If enough bins are populated, plotted points are $\{(\bar v_k,\bar p_k)\}$ plus first-point closure.

### 9. Displayed observables and efficiencies

Net torque smoothing (first-order filter):

```math
\tau_{net,cycle\ mean}=\frac{1}{N_s}\sum_{j=1}^{N_s}\tau_{net,j}
```

```math
\tau_{net,target}=
\begin{cases}
\dfrac{1}{N_r}\sum_{k=1}^{N_r}\tau_{net,cycle\ mean,k},\ \ N_r\le 4, & \text{at least one complete cycle exists}\\
\tau_{net}, & \text{startup (before first complete cycle)}
\end{cases}
```

```math
\alpha_\tau=\mathrm{clamp}\left(\frac{\Delta t}{0.180+\Delta t},0,1\right)
```

```math
\tau_{net,filt}\leftarrow \tau_{net,filt}+\alpha_\tau(\tau_{net,target}-\tau_{net,filt})
```

Combustion-cycle mean torque:

```math
\tau_{comb,cycle\ mean}=\frac{1}{N_s}\sum_{j=1}^{N_s}\tau_{comb,j}
```

(updated when a 720 deg cycle boundary is crossed).

Theoretical Otto efficiency:

```math
\eta_{th,Otto}=\mathrm{clamp}\left(1-\frac{1}{r^{\gamma-1}},0,1\right)
```

Indicated cycle work from sampled p-V loop:

```math
V_i=v_i\cdot V_{d,cyl}
```

```math
W_i\approx \sum_{i=1}^{n-1}\frac{p_i+p_{i+1}}{2}(V_{i+1}-V_i)
+\frac{p_n+p_1}{2}(V_1-V_n)
```

(`n<4` samples -> no valid cycle work).

Mean indicated work over recent complete cycles:

```math
\bar W_i=\frac{1}{N}\sum_{c=1}^{N}W_{i,c},\quad N\le 3
```

IMEP and indicated thermal efficiency:

```math
IMEP=\frac{\bar W_i}{V_{d,cyl}},\qquad
IMEP_{bar}=IMEP\times 10^{-5}
```

```math
Q_{in,cycle}=m_{f,cycle,cyl}LHV
```

For numerical stability under throttle transitions, the implementation uses recent-cycle average fuel:

```math
\bar m_{f,cycle}=\frac{1}{N}\sum_{c=1}^{N}m_{f,cycle,c},\quad N\le 3
```

```math
Q_{in,cycle}=\bar m_{f,cycle}LHV
```

```math
\eta_{th,ind}=
\begin{cases}
\mathrm{clamp}\left(\dfrac{\bar W_i}{Q_{in,cycle}},-0.20,0.44\right), & Q_{in,cycle}>10^{-9}\\
0, & \text{otherwise}
\end{cases}
```

Idle-stable indicator in GUI:

```math
stable\_idle=
running
\land |\mathrm{rpm}-\mathrm{rpm}_{idle}|<90
\land \alpha_{cmd}<0.20
```

GUI redraw cadence is capped to approximately:

```math
f_{GUI}\approx 30\ \mathrm{Hz}
```

## Steady-State Efficiency Calibration (Default Config)

The table below is an implementation-exact regression anchor for the current default configuration.
It is not a claim that the present efficiency calibration is final. Under the current fixed-spark default setup, the loaded sweep reaches the configured `eta_indicated_max` ceiling over much of the throttle range, so this section should be read as "what the code presently does at steady state" rather than as an external validation claim by itself.

Reference sweep (test `steady_state_efficiency_sweep_matches_gasoline_si_range`):

| Throttle [-] | Steady RPM [rpm] | Indicated efficiency from p-V [%] |
|---:|---:|---:|
| 0.10 | 2083 | 44.0 |
| 0.16 | 3149 | 44.0 |
| 0.24 | 4231 | 44.0 |
| 0.34 | 4910 | 44.0 |
| 0.50 | 5704 | 44.0 |
| 0.70 | 6445 | 44.0 |
| 1.00 | 6665 | 44.0 |

Target assertion band in test:

```math
0.08 \le \eta_{th,ind} \le 0.44
```

#### Representative periodic steady-state check

The same fixed-control operating points can also be checked directly against the governing ODE by verifying that the reduced state returns to the same phase section on successive engine cycles. The table below is produced from test `representative_operating_points_are_periodic_steady_states`.

| Case | Throttle [-] | Load cmd [-] | Mean RPM [rpm] | Same-section state-error norm [-] | Cycle-to-cycle `Δrpm` [rpm] | Max cycle-to-cycle `Δp` [kPa] | Max cycle-to-cycle `Δm_runner` [g/s] |
|---|---:|---:|---:|---:|---:|---:|---:|
| Light throttle | 0.10 | 0.80 | 2075 | 0.135 | 0.170 | 0.159 | 0.022 |
| Mid throttle | 0.34 | 0.80 | 5060 | 0.709 | 2.488 | 0.852 | 0.380 |
| WOT | 1.00 | 0.80 | 6976 | 2.524 | 0.091 | 3.222 | 3.812 |

These points are therefore treated as periodic steady states of the reduced ODE: even at WOT, the same-phase cycle-to-cycle speed drift is still below `0.1%` of mean RPM, pressure drift stays within about `3.3 kPa`, and runner-flow drift stays within about `3.9 g/s`.

### 10. Cam profile visualization

Centerlines with VVT:

```math
\theta_{I,center}=\theta_{I,cl}-\Delta\theta_{VVT,I},\qquad
\theta_{E,center}=\theta_{E,cl}+\Delta\theta_{VVT,E}
```

For generic lobe (center $\theta_c$, duration $\Delta\theta$, max lift $L_{max}$):

```math
h=\max\left(\frac{\Delta\theta}{2},1\right),\quad
\delta=((\theta-\theta_c+360)\bmod 720)-360
```

```math
L(\theta)=
\begin{cases}
0, & |\delta|\ge h\\
L_{max}\left[\frac{1+\cos\left(\pi\,\mathrm{clamp}(\delta/h,-1,1)\right)}{2}\right]^{1.2}, & |\delta|<h
\end{cases}
```

### 11. Exhaust audio synthesis model

The implemented audio model does not use recorded assets. It uses:

- simulated crank-angle phase to keep exhaust events synchronized
- exhaust pressure and runner flow for pulse and turbulence strength
- exhaust-gas temperature and pipe length for resonant coloring and round-trip reflection timing

#### 11.1 Pressure normalization, RPM gate, and envelope states

The GUI observation supplies exhaust pressure in `kPa`:

```math
p_{exh,kPa}=10^{-3}p_{exh}
```

Normalized exhaust overpressure:

```math
p_n=\mathrm{clamp}\left(\frac{p_{exh,kPa}-p_{amb,kPa}}{\Delta p_{span}},0,1\right)
```

RPM gate:

```math
g_{rpm}=
\mathrm{clamp}\left(
\frac{\mathrm{rpm}-\mathrm{rpm}_{gate,0}}
{\mathrm{rpm}_{gate,1}-\mathrm{rpm}_{gate,0}},
0,1
\right)^{\beta_{rpm}}
```

Smoothed pressure state and pressure rise:

```math
\tilde p_n=g_{rpm}p_n
```

```math
p_s\leftarrow p_s+\alpha_p(\tilde p_n-p_s),\qquad
\Delta p=p_s-p_{s,prev}
```

Pulse envelope:

```math
env\leftarrow
\mathrm{clamp}\left(
d_{env}\,env+k_{env,\Delta p}\max(\Delta p,0),
env_{min},env_{max}
\right)
```

At zero RPM, `g_{rpm}=0`, so the synthesized output is driven to silence.

#### 11.2 Firing-frequency pitch model

For a 4-stroke engine, the aggregate firing frequency is:

```math
f_{fire}=\frac{\mathrm{rpm}\,N_{cyl}}{120}
```

The phase state is advanced from firing frequency and then nudged toward the simulated crank phase:

```math
\phi_{obs}=\left(\frac{\theta_{cycle}}{180}\right)\bmod 1
```

```math
\phi\leftarrow
\left(
\phi+\frac{f_{fire}}{f_s}+k_{\phi}\,\mathrm{wrap}(\phi_{obs}-\phi)
\right)\bmod 1
```

This keeps the pulse train phase-locked to the simulated `0..720 degCA` state instead of letting
the audio oscillator drift freely from RPM alone. A 4-cylinder engine gives
$f_{fire}=33.3\ \mathrm{Hz}$ at `1000 rpm` and $100\ \mathrm{Hz}$ at `3000 rpm`.

The per-event deterministic pulse shape is:

```math
p_{train}=\max\left(e^{-a_f\phi}-e^{-a_s\phi},0\right)
```

#### 11.3 Exhaust-pipe resonance from gas temperature

The model uses ideal-gas sound speed in the hot exhaust stream:

```math
c_{exh}=\sqrt{\gamma_{air}R_{air}T_{exh}}
```

with clamped temperature state:

```math
T_{exh}\leftarrow
\mathrm{clamp}(T_{exh},T_{exh,min},T_{exh,max})
```

Quarter-wave exhaust-pipe base frequency:

```math
f_q=\frac{c_{exh}}{4L_{pipe}}
```

Three resonator targets are placed at configurable odd-like modes:

```math
f_{1,target}=m_1f_q,\qquad
f_{2,target}=m_2f_q,\qquad
f_{3,target}=m_3f_q
```

Every `audio.model.resonator_retarget_interval` samples, each target is mapped to the realizable
digital band-pass frequency:

```math
f_i=\mathrm{clamp}\left(f_{i,target},f_{min},\rho_{Nyq}f_s\right)
```

```math
Q_i=\max(Q_{i,target},Q_{min})
```

#### 11.4 Excitation, pipe reflection, turbulence, and output

Pressure-rise click term:

```math
pulse_{\Delta p}=\mathrm{clamp}(k_{\Delta p}\Delta p,pulse_{min},pulse_{max})
```

Exhaust-pulse term:

```math
pulse_{exh}=p_{train}(b_{exh}+k_{exh}p_s)g_{rpm}
```

Sinusoidal support term:

```math
pulse_{sin}=k_{sin}\,env\,g_{rpm}\sin(2\pi\phi)
```

Total excitation:

```math
x=pulse_{\Delta p}+pulse_{exh}+pulse_{sin}
```

Simple tailpipe round-trip reflection surrogate:

```math
N_{ref}\approx \mathrm{round}\left(\frac{2L_{pipe}}{c_{exh}}f_s\right)
```

```math
x_{pipe}=x-k_{ref}x[n-N_{ref}]
```

This is not a full 1D waveguide, but it inserts the correct sign of open-end pressure reflection and
ties the delay to pipe length and hot-gas sound speed.

Flow-driven broadband term:

```math
x_{jet}\propto \dot m_{er,n}^{1.15}\,p_{drive,n}\,p_{train}
```

The implementation uses a deterministic high-passed wideband source so the sound picks up some
jet-like edge as exhaust flow rises, without relying on sampled audio assets.

Three-resonator weighted sum:

```math
y_{res}=w_1y_1+w_2y_2+w_3y_3
```

Direct pulse and low rumble additions:

```math
y_{dir}=w_{dir}x
```

```math
y_{rum}=k_{rum}\,env\,\sin(2\pi h_{rum}\phi)
```

```math
raw=y_{res}+y_{dir}+y_{jet}+y_{rum}
```

DC removal:

```math
dc\leftarrow d_{dc}\,dc+k_{dc}\,raw,\qquad raw\leftarrow raw-dc
```

Loudness model:

```math
G=
\left(
g_0+g_p p_s+g_{env}\min(env,1)
\right)
\max(G_{out},G_{out,min})g_{rpm}
```

Final output:

```math
audio=g_{lim}\tanh(G\,raw)
```

#### 11.5 Per-resonator digital filter equations

For each resonator, the implemented RBJ-style band-pass coefficients are:

```math
\omega_0=\frac{2\pi f_i}{f_s},\qquad
\alpha=\frac{\sin\omega_0}{2Q_i}
```

```math
b_0=\alpha,\quad b_1=0,\quad b_2=-\alpha
```

```math
a_0=1+\alpha,\quad a_1=-2\cos\omega_0,\quad a_2=1-\alpha
```

```math
\tilde b_k=\frac{b_k}{a_0},\qquad
\tilde a_1=\frac{a_1}{a_0},\qquad
\tilde a_2=\frac{a_2}{a_0}
```

Difference equation:

```math
y_i[n]=
\tilde b_0x[n]+\tilde b_1x[n-1]+\tilde b_2x[n-2]
-\tilde a_1y_i[n-1]-\tilde a_2y_i[n-2]
```

## Thermal Efficiency Metrics (as shown in GUI)

- Theoretical Otto efficiency: $\eta_{th,Otto}$
- Indicated efficiency from p-V: $\eta_{th,ind}$
- Indicated work per cycle (kJ/cyl/cycle): $\bar W_i \times 10^{-3}$
- IMEP from $\bar W_i / V_{d,cyl}$

The GUI shows both indicated-side quantities from the reconstructed `p-V` loop and brake-side quantities such as smoothed brake power and brake BMEP. It still does not show `BSFC`.

## Parameter List (Exhaustive, with Units)

Sample values below are taken from the checked-in `config/sim.yaml`.
For the calibration-heavy sections, symbols are omitted where they would hurt readability, but every YAML key loaded by `src/config.rs` is listed.

### Environment

| YAML key | Symbol | Unit | Sample value | Description |
|---|---:|---:|---:|---|
| `environment.ambient_pressure_pa` | $p_{amb}$ | Pa | 101325.0 | Ambient pressure |
| `environment.intake_temp_k` | $T_{im}$ | K | 305.0 | Intake gas temperature |
| `environment.exhaust_temp_k` | $T_{exh}$ | K | 880.0 | Exhaust gas temperature |
| `environment.dt` | $\Delta t_{base}$ | s | 0.0010 | Base timestep for adaptive integration |

### Engine

Total displacement is not tuned independently in YAML. It is derived from geometry as

```math
V_d = N_{cyl}\frac{\pi}{4}B^2S
```

| YAML key | Symbol | Unit | Sample value | Description |
|---|---:|---:|---:|---|
| `engine.compression_ratio` | $r$ | - | 13.0 | Compression ratio |
| `engine.bore_m` | $B$ | m | 0.0805 | Cylinder bore used in piston-speed and wall-area estimates |
| `engine.stroke_m` | $S$ | m | 0.0976 | Crank stroke used in piston-speed and wall-area estimates |
| `engine.inertia_kgm2` | $J$ | kg m^2 | 0.16 | Lumped rotational inertia |
| `engine.friction_c0_nm` | $c_0$ | N m | 5.1 | Friction offset |
| `engine.friction_c1_nms` | $c_1$ | N m s/rad | 0.0040 | Linear friction coefficient |
| `engine.friction_c2_nms2` | $c_2$ | N m s^2/rad^2 | 0.000020 | Quadratic friction coefficient |
| `engine.intake_volume_m3` | $V_{im}$ | m^3 | 0.0032 | Intake manifold control volume |
| `engine.intake_runner_volume_m3` | $V_{ir}$ | m^3 | 0.00040 | Intake runner / port control volume |
| `engine.exhaust_volume_m3` | $V_{exh}$ | m^3 | 0.0050 | Exhaust control volume |
| `engine.exhaust_runner_volume_m3` | $V_{er}$ | m^3 | 0.00055 | Exhaust runner / header-primary control volume |
| `engine.throttle_area_max_m2` | $A_{th,max}$ | m^2 | 0.0030 | Max throttle area |
| `engine.tailpipe_area_m2` | $A_{tail}$ | m^2 | 0.0025 | Tailpipe effective area |
| `engine.idle_target_rpm` | $\mathrm{rpm}_{idle}$ | rpm | 850.0 | Idle target |
| `engine.max_rpm` | $\mathrm{rpm}_{max}$ | rpm | 7000.0 | RPM plot fixed top |

### Cam

| YAML key | Symbol | Unit | Sample value | Description |
|---|---:|---:|---:|---|
| `cam.intake_centerline_deg` | $\theta_{I,cl}$ | degCA | 470.0 | Intake lobe centerline |
| `cam.exhaust_centerline_deg` | $\theta_{E,cl}$ | degCA | 250.0 | Exhaust lobe centerline |
| `cam.intake_duration_deg` | $\Delta\theta_I$ | degCA | 248.0 | Intake duration |
| `cam.exhaust_duration_deg` | $\Delta\theta_E$ | degCA | 244.0 | Exhaust duration |
| `cam.intake_max_lift_mm` | $L_{I,max}$ | mm | 10.5 | Intake max lift |
| `cam.exhaust_max_lift_mm` | $L_{E,max}$ | mm | 9.8 | Exhaust max lift |
| `cam.display_y_max_mm` | - | mm | 14.0 | Cam plot y max |

### Control defaults

| YAML key | Symbol | Unit | Sample value | Description |
|---|---:|---:|---:|---|
| `control_defaults.throttle_cmd` | $\alpha_{cmd}$ | - | 0.05 | Initial throttle command |
| `control_defaults.load_cmd` | $u_{load}$ | - | 0.0 | Initial external-load command applied to the dyno / vehicle-load surrogate |
| `control_defaults.starter_cmd` | $u_{st}$ | bool | false | Initial starter command |
| `control_defaults.spark_cmd` | $u_{spk}$ | bool | false | Initial spark command |
| `control_defaults.fuel_cmd` | $u_f$ | bool | false | Initial fuel command |
| `control_defaults.ignition_timing_deg` | $\Delta\theta_{ign}$ | deg BTDC | 12.0 | Ignition timing command referenced to TDC |
| `control_defaults.vvt_intake_deg` | $\Delta\theta_{VVT,I}$ | degCA | 0.0 | Intake VVT offset |
| `control_defaults.vvt_exhaust_deg` | $\Delta\theta_{VVT,E}$ | degCA | 0.0 | Exhaust VVT offset |

### Auto control

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `auto_control.cranking_rpm_threshold` | rpm | 600.0 | RPM below which auto mode treats the engine as cranking |
| `auto_control.starter_cutoff_rpm` | rpm | 760.0 | RPM above which the starter is forcibly cut |
| `auto_control.starter_target_rpm` | rpm | 620.0 | Cranking target RPM while the starter remains engaged |
| `auto_control.starter_kp` | - | 0.00022 | Proportional gain in cranking mode |
| `auto_control.starter_ki` | - | 0.00008 | Integral gain in cranking mode |
| `auto_control.starter_base_throttle` | - | 0.14 | Feedforward throttle bias during cranking |
| `auto_control.run_kp` | - | 0.00012 | Proportional gain in running / idle mode |
| `auto_control.run_ki` | - | 0.00005 | Integral gain in running / idle mode |
| `auto_control.run_base_throttle` | - | 0.040 | Feedforward throttle bias during idle control |
| `auto_control.integral_min` | - | -600.0 | Lower clamp on the controller integral state |
| `auto_control.integral_max` | - | 600.0 | Upper clamp on the controller integral state |
| `auto_control.throttle_min` | - | 0.005 | Minimum throttle command the auto controller may issue |
| `auto_control.throttle_max` | - | 0.42 | Maximum throttle command the auto controller may issue |
| `auto_control.integral_relief_rpm_band` | rpm | 120.0 | Overspeed band used to bleed integrator state |
| `auto_control.integral_relief_factor` | - | 0.85 | Integral multiplier applied during overspeed relief |

### Model

The `model.*` section contains most reduced-order calibration coefficients, guards, and display-model parameters, so it is grouped by function here.

#### State initialization and history buffers

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.initial_intake_pressure_pa` | Pa | 38000.0 | Intake-manifold pressure used to initialize the state |
| `model.initial_throttle_eff` | - | 0.05 | Initial effective throttle state before actuator lag evolves it |
| `model.max_rpm_floor` | rpm | 1000.0 | Minimum fallback max-RPM reference if engine config is too small |
| `model.history_capacity_floor` | samples | 256 | Minimum GUI history capacity |
| `model.cycle_metric_history_capacity` | cycles | 16 | Retained completed-cycle metrics for rolling averages |
| `model.pv_capacity_scale` | - | 1600 | Scale factor from p-V subsampling density to p-V history capacity |
| `model.pv_capacity_min` | samples | 2000 | Absolute minimum p-V history capacity |
| `model.pv_display_raw_min_points` | samples | 128 | Minimum raw p-V points before angle-bin smoothing is attempted |
| `model.pv_display_bins` | bins | 1440 | Number of crank-angle bins used for p-V display smoothing across 720 deg |
| `model.pv_display_min_populated_bins` | bins | 180 | Minimum populated bins required before the smoothed p-V loop is used |

#### Ignition and burn phasing

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.ignition_timing_min_deg` | deg BTDC | -5.0 | Lower clamp on commanded ignition timing |
| `model.ignition_timing_max_deg` | deg BTDC | 45.0 | Upper clamp on commanded ignition timing |
| `model.mbt_base_deg` | deg BTDC | 12.0 | Base MBT timing estimate near the reference operating point |
| `model.mbt_rpm_slope_deg_per_rpm` | deg/rpm | 0.0018 | RPM sensitivity of the MBT timing estimate |
| `model.mbt_rpm_reference` | rpm | 900.0 | Reference RPM for the MBT timing estimate |
| `model.mbt_load_coeff` | deg | 3.0 | MBT retard amount applied as load falls from full load |
| `model.mbt_min_deg` | deg BTDC | 8.0 | Lower clamp on the MBT timing estimate |
| `model.mbt_max_deg` | deg BTDC | 26.0 | Upper clamp on the MBT timing estimate |
| `model.phase_sigma_advanced_deg` | deg | 6.5 | Width of the advanced-side efficiency penalty around MBT |
| `model.phase_sigma_retarded_deg` | deg | 15.0 | Width of the retarded-side efficiency penalty around MBT |
| `model.phase_stable_advanced_limit_deg` | deg | 14.0 | Largest advance error before combustion is treated as unstable |
| `model.phase_stable_retarded_limit_deg` | deg | 32.0 | Largest retard error before combustion is treated as unstable |
| `model.burn_start_base_deg` | degCA | 360.0 | Base start-of-combustion angle before ignition and VVT offsets |
| `model.burn_start_vvt_intake_coeff` | degCA/deg | 0.08 | Intake-VVT sensitivity of displayed burn start |
| `model.burn_duration_base_deg` | degCA | 66.0 | Base burn duration used in the Wiebe burn-rate model |
| `model.burn_duration_throttle_coeff` | degCA/throttle | 8.0 | Burn-duration shortening with increasing throttle |
| `model.burn_duration_min_deg` | degCA | 38.0 | Lower clamp on burn duration |
| `model.burn_duration_max_deg` | degCA | 75.0 | Upper clamp on burn duration |
| `model.wiebe_a` | - | 5.2 | Wiebe shape coefficient `a` |
| `model.wiebe_m` | - | 2.0 | Wiebe shape coefficient `m` |

#### Temperature, flow, and load closures

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.exhaust_temp_retard_gain` | 1/deg | 0.024 | Exhaust-temperature gain when ignition is retarded from MBT |
| `model.exhaust_temp_advance_gain` | 1/deg | 0.010 | Exhaust-temperature reduction gain when ignition is advanced from MBT |
| `model.exhaust_temp_scale_min` | - | 0.75 | Lower clamp on timing-based exhaust-temperature scaling |
| `model.exhaust_temp_scale_max` | - | 1.70 | Upper clamp on timing-based exhaust-temperature scaling |
| `model.exhaust_temp_min_k` | K | 500.0 | Lower clamp on effective exhaust temperature |
| `model.exhaust_temp_max_k` | K | 1900.0 | Upper clamp on effective exhaust temperature |
| `model.throttle_area_offset` | - | 0.005 | Offset term in the nonlinear throttle-area map |
| `model.throttle_area_gain` | - | 0.98 | Main gain in the nonlinear throttle-area map |
| `model.throttle_area_exponent` | - | 1.8 | Exponent in the nonlinear throttle-area map |
| `model.throttle_area_min_scale` | - | 0.005 | Minimum fraction of maximum area allowed by the throttle map |
| `model.throttle_discharge_coeff` | - | 0.82 | Discharge coefficient of the throttle orifice |
| `model.tailpipe_discharge_coeff` | - | 0.87 | Discharge coefficient of the tailpipe orifice |
| `model.load_min` | - | 0.20 | Lower clamp on normalized load `p_im / p_amb` |
| `model.load_max` | - | 1.20 | Upper clamp on normalized load `p_im / p_amb` |

#### Mixture, efficiency, and combustion enable

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.lambda_base` | - | 1.15 | Base lambda command near closed throttle |
| `model.lambda_throttle_coeff` | - | 0.35 | Lambda enrichment gain with throttle opening |
| `model.lambda_min` | - | 0.82 | Lower clamp on commanded lambda |
| `model.lambda_max` | - | 1.12 | Upper clamp on commanded lambda |
| `model.stoich_afr` | kg air/kg fuel | 14.7 | Stoichiometric air-fuel ratio by mass |
| `model.eta_base_offset` | - | 0.172 | Offset term in the reduced-order thermal-efficiency map |
| `model.eta_load_coeff` | - | 0.215 | Load gain in the reduced-order thermal-efficiency map |
| `model.eta_rpm_abs_coeff` | 1/rpm | 0.000011 | Absolute RPM penalty coefficient in the efficiency map |
| `model.eta_rpm_reference` | rpm | 2500.0 | Reference RPM for the efficiency map |
| `model.eta_base_min` | - | 0.08 | Lower clamp on the base efficiency-map output |
| `model.eta_base_max` | - | 0.40 | Upper clamp on the base efficiency-map output |
| `model.eta_min` | - | 0.02 | Lower clamp on final thermal efficiency after timing correction |
| `model.eta_max` | - | 0.44 | Upper clamp on final thermal efficiency after timing correction |
| `model.combustion_enable_rpm_min` | rpm | 120.0 | Minimum RPM required before combustion may start |
| `model.combustion_enable_intake_pressure_min_pa` | Pa | 25000.0 | Minimum intake pressure required before combustion may start |
| `model.combustion_enable_running_rpm_min` | rpm | 450.0 | Minimum running RPM above which combustion may continue without starter support |
| `model.combustion_rate_max` | - | 5.5 | Upper clamp on normalized Wiebe burn-rate output |

#### Torque, state guards, and display post-processing

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.pumping_torque_min_nm` | Nm | -12.0 | Lower clamp on pumping torque |
| `model.pumping_torque_max_nm` | Nm | 25.0 | Upper clamp on pumping torque |
| `model.throttle_time_constant_s` | s | 0.060 | Throttle-actuator first-order time constant |
| `model.intake_pressure_min_pa` | Pa | 18000.0 | Lower clamp on the intake-pressure state |
| `model.intake_pressure_max_pa` | Pa | 130000.0 | Upper clamp on the intake-pressure state |
| `model.exhaust_pressure_min_ambient_ratio` | - | 0.78 | Minimum exhaust pressure as a multiple of ambient pressure |
| `model.exhaust_pressure_max_over_ambient_pa` | Pa | 120000.0 | Maximum exhaust overpressure above ambient |
| `model.running_set_rpm` | rpm | 520.0 | RPM at which the running flag latches true |
| `model.running_clear_rpm` | rpm | 260.0 | RPM below which the running flag clears if the starter is off |
| `model.net_torque_smoothing_cycles` | cycles | 4 | Completed cycles used to smooth displayed net torque |
| `model.net_torque_filter_time_constant_s` | s | 0.180 | Low-pass time constant used on displayed net torque |
| `model.fuel_mass_presence_threshold_kg` | kg | 1.0e-9 | Fuel-per-cycle threshold below which combustion is treated as absent |
| `model.compression_peak_gamma` | - | 1.32 | Exponent used in the compression-only peak-pressure estimate |
| `model.compression_peak_max_pa` | Pa | 6.0e6 | Upper clamp on the compression-only peak-pressure estimate |
| `model.combustion_pressure_gain` | - | 0.38 | Gain from fuel heat into displayed p-V combustion pressure rise |
| `model.combustion_pressure_load_exponent` | - | 1.35 | Load exponent used in displayed p-V combustion pressure rise |
| `model.peak_pressure_max_pa` | Pa | 8.0e6 | Absolute upper clamp on displayed cylinder pressure |
| `model.swept_volume_floor_sampling_m3` | m^3 | 1.0e-6 | Lower swept-volume guard used during p-V work sampling |
| `model.clearance_volume_floor_m3` | m^3 | 1.0e-6 | Lower clearance-volume guard used in p-V reconstruction |
| `model.compression_ratio_guard` | - | 1.0 | Minimum denominator guard used in `r - 1` style formulas |
| `model.eta_indicated_average_cycles` | cycles | 3 | Number of recent cycles used for indicated-efficiency averaging |
| `model.eta_indicated_min` | - | -0.20 | Lower clamp on displayed indicated efficiency from the p-V loop |
| `model.eta_indicated_max` | - | 0.44 | Upper clamp on displayed indicated efficiency from the p-V loop |
| `model.imep_swept_volume_floor_m3` | m^3 | 1.0e-9 | Swept-volume guard used when computing IMEP |
| `model.stable_idle_rpm_band` | rpm | 90.0 | RPM band around target used to declare stable idle |
| `model.stable_idle_throttle_max` | - | 0.20 | Maximum throttle allowed while declaring stable idle |
| `model.theoretical_efficiency_compression_floor` | - | 1.001 | Compression-ratio guard used in Otto-efficiency evaluation |
| `model.cam_profile_samples` | samples | 721 | Number of points generated for the cam-profile plot |
| `model.cam_half_duration_min_deg` | degCA | 1.0 | Minimum half-duration allowed in the synthetic cam-lobe shape |
| `model.cam_shape_exponent` | - | 1.2 | Exponent shaping the synthetic cosine cam-lobe profile |

#### Model.volumetric_efficiency

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.volumetric_efficiency.rpm_base` | - | 0.82 | Baseline RPM-dependent VE factor away from the tuned center |
| `model.volumetric_efficiency.rpm_gain` | - | 0.24 | Gaussian bump magnitude around the tuned RPM center |
| `model.volumetric_efficiency.rpm_center` | rpm | 5100.0 | Center RPM of the VE Gaussian bump |
| `model.volumetric_efficiency.rpm_width` | rpm^2 | 10500000.0 | Width parameter of the VE Gaussian bump |
| `model.volumetric_efficiency.rpm_min` | - | 0.62 | Lower clamp on the RPM-dependent VE factor |
| `model.volumetric_efficiency.rpm_max` | - | 1.04 | Upper clamp on the RPM-dependent VE factor |
| `model.volumetric_efficiency.vvt_intake_coeff` | 1/deg | 0.0022 | Intake-VVT sensitivity of VE |
| `model.volumetric_efficiency.vvt_exhaust_coeff` | 1/deg | 0.0016 | Exhaust-VVT sensitivity of VE |
| `model.volumetric_efficiency.throttle_base` | - | 0.38 | Baseline throttle contribution to VE at very low opening |
| `model.volumetric_efficiency.throttle_gain` | - | 0.62 | Additional VE contribution as throttle opens |
| `model.volumetric_efficiency.throttle_min` | - | 0.38 | Lower clamp on the throttle-dependent VE factor |
| `model.volumetric_efficiency.throttle_max` | - | 1.03 | Upper clamp on the throttle-dependent VE factor |
| `model.volumetric_efficiency.overall_min` | - | 0.30 | Lower clamp on total VE |
| `model.volumetric_efficiency.overall_max` | - | 1.10 | Upper clamp on total VE |

#### Model.starter_torque

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.starter_torque.low_rpm_threshold` | rpm | 220.0 | RPM threshold separating the low-speed starter plateau |
| `model.starter_torque.high_rpm_threshold` | rpm | 450.0 | RPM threshold above which starter torque falls to zero |
| `model.starter_torque.low_torque_nm` | Nm | 62.0 | Starter torque below the low-RPM threshold |
| `model.starter_torque.high_torque_nm` | Nm | 36.0 | Starter torque on the mid-RPM plateau before cutout |

#### Model.pv_model

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.pv_model.gamma_compression` | - | 1.34 | Polytropic exponent used in displayed compression |
| `model.pv_model.gamma_expansion` | - | 1.24 | Polytropic exponent used in displayed expansion |
| `model.pv_model.evo_deg` | degCA | 565.0 | Exhaust-valve opening angle used in displayed blowdown |
| `model.pv_model.blowdown_end_deg` | degCA | 620.0 | End angle of the displayed blowdown transition |
| `model.pv_model.intake_pulsation_amplitude` | - | 0.02 | Small intake-stroke pressure pulsation amplitude in the displayed p-V loop |
| `model.pv_model.expansion_floor_exhaust_ratio` | - | 1.05 | Minimum expansion pressure as a multiple of exhaust pressure |
| `model.pv_model.pressure_floor_pa` | Pa | 20000.0 | Absolute minimum displayed cylinder pressure |

#### Model.gas_path

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.gas_path.intake_runner_inertance_pa_s2_per_kg` | Pa s^2/kg | 18000.0 | Intake runner inertance in the plenum-to-runner flow ODE |
| `model.gas_path.exhaust_runner_inertance_pa_s2_per_kg` | Pa s^2/kg | 14000.0 | Exhaust runner inertance in the runner-to-collector flow ODE |
| `model.gas_path.intake_runner_damping_per_s` | 1/s | 6.0 | Linear damping on the intake runner flow state |
| `model.gas_path.exhaust_runner_damping_per_s` | 1/s | 6.0 | Linear damping on the exhaust runner flow state |
| `model.gas_path.intake_pulse_blend` | - | 0.32 | Blend from mean intake flow to normalized valve-event pulsation |
| `model.gas_path.exhaust_pulse_blend` | - | 0.40 | Blend from mean exhaust flow to normalized valve-event pulsation |
| `model.gas_path.intake_boundary_runner_weight` | - | 0.35 | Intake runner weight in the effective cylinder-side intake boundary pressure |
| `model.gas_path.exhaust_boundary_runner_weight` | - | 0.45 | Exhaust runner weight in the effective cylinder-side exhaust boundary pressure |
| `model.gas_path.runner_flow_limit_kg_s` | kg/s | 0.45 | Symmetric clamp on the runner flow states |
| `model.gas_path.runner_pressure_min_pa` | Pa | 18000.0 | Lower clamp on runner pressures |
| `model.gas_path.runner_pressure_max_pa` | Pa | 320000.0 | Upper clamp on runner pressures |
| `model.gas_path.overlap_pressure_coeff` | - | 0.22 | Overlap VE gain from runner pressure difference |
| `model.gas_path.overlap_flow_coeff` | - | 0.08 | Overlap VE gain from exhaust-runner flow |
| `model.gas_path.overlap_flow_reference_kg_s` | kg/s | 0.08 | Reference exhaust-runner flow used in the overlap correction |
| `model.gas_path.overlap_effect_min` | - | 0.88 | Lower clamp on the overlap VE multiplier |
| `model.gas_path.overlap_effect_max` | - | 1.12 | Upper clamp on the overlap VE multiplier |

#### Model.wave_action

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.wave_action.intake_group_count` | groups | 4 | Requested number of independently tuned intake runner groups; runtime snaps this to a divisor of the fixed 4 cylinders |
| `model.wave_action.exhaust_group_count` | groups | 2 | Requested number of independently tuned exhaust header groups; runtime snaps this to a divisor of the fixed 4 cylinders |
| `model.wave_action.event_memory` | cycles | 3 | Number of past 720 deg cycles retained in the damped wave superposition |
| `model.wave_action.intake_runner_length_m` | m | 0.36 | Effective acoustic length of the intake runner |
| `model.wave_action.exhaust_primary_length_m` | m | 0.74 | Effective acoustic length of the exhaust primary/header branch |
| `model.wave_action.intake_delay_scale` | - | 2.0 | Intake travel-time scale multiplying runner length / sound speed |
| `model.wave_action.exhaust_delay_scale` | - | 2.0 | Exhaust travel-time scale multiplying primary length / sound speed |
| `model.wave_action.intake_decay_time_s` | s | 0.012 | Intake wave-packet decay time after arrival at the valve |
| `model.wave_action.exhaust_decay_time_s` | s | 0.010 | Exhaust wave-packet decay time after arrival at the valve |
| `model.wave_action.intake_pressure_gain` | - | 0.06 | Gain from normalized intake source flow into intake-side wave pressure |
| `model.wave_action.exhaust_pressure_gain` | - | 0.08 | Gain from normalized exhaust source flow into exhaust-side wave pressure |
| `model.wave_action.intake_pressure_limit_pa` | Pa | 6000.0 | Symmetric clamp on intake-side wave pressure correction |
| `model.wave_action.exhaust_pressure_limit_pa` | Pa | 9000.0 | Symmetric clamp on exhaust-side wave pressure correction |
| `model.wave_action.intake_flow_reference_kg_s` | kg/s | 0.055 | Intake source-flow magnitude corresponding to unit wave excitation |
| `model.wave_action.exhaust_flow_reference_kg_s` | kg/s | 0.075 | Exhaust source-flow magnitude corresponding to unit wave excitation |
| `model.wave_action.intake_flow_wave_gain` | - | 0.0 | Gain from current intake wave pressure into the intake flow multiplier |
| `model.wave_action.exhaust_flow_wave_gain` | - | 0.0 | Gain from current exhaust wave pressure into the exhaust flow multiplier |
| `model.wave_action.intake_ram_gain` | - | 0.22 | Gain from IVC-side pressure recovery into the intake ram multiplier |
| `model.wave_action.exhaust_scavenge_gain` | - | 0.18 | Gain from overlap pressure head into the scavenging multiplier |
| `model.wave_action.ve_pulse_min` | - | 0.94 | Lower clamp on the combined wave-action VE multiplier |
| `model.wave_action.ve_pulse_max` | - | 1.08 | Upper clamp on the combined wave-action VE multiplier |

#### Model.heat_transfer

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.heat_transfer.wall_temp_k` | K | 470.0 | Representative combustion-chamber wall temperature |
| `model.heat_transfer.base_h_w_m2k` | W/m^2/K | 120.0 | Base heat-transfer coefficient before scaling |
| `model.heat_transfer.pressure_exponent` | - | 0.8 | Pressure exponent in the Woschni-inspired heat-transfer correlation |
| `model.heat_transfer.temperature_exponent` | - | -0.45 | Gas-temperature exponent in the heat-transfer correlation |
| `model.heat_transfer.piston_speed_exponent` | - | 0.8 | Mean-piston-speed exponent in the heat-transfer correlation |
| `model.heat_transfer.reference_pressure_pa` | Pa | 1000000.0 | Reference cylinder pressure used in the heat-transfer correlation |
| `model.heat_transfer.reference_temp_k` | K | 1000.0 | Reference gas temperature used in the heat-transfer correlation |
| `model.heat_transfer.reference_piston_speed_mps` | m/s | 10.0 | Reference mean piston speed used in the heat-transfer correlation |
| `model.heat_transfer.gas_temp_rise_gain` | - | 0.30 | Fraction of adiabatic fuel-energy temperature rise used in the gas-temperature estimate |
| `model.heat_transfer.gas_temp_max_k` | K | 2800.0 | Upper clamp on the single-zone gas-temperature estimate |
| `model.heat_transfer.wall_area_scale` | - | 1.0 | Scale factor applied to the simple chamber surface-area estimate |
| `model.heat_transfer.duration_scale` | - | 0.85 | Scale factor converting burn duration into heat-transfer exposure time |
| `model.heat_transfer.heat_loss_fraction_max` | - | 0.08 | Upper clamp on per-cycle heat loss as a fraction of fuel heat |
| `model.heat_transfer.exhaust_temp_cooling_gain` | - | 0.35 | Exhaust-temperature reduction gain from heat loss |

#### Model.fuel_evaporation

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.fuel_evaporation.latent_heat_j_per_kg` | J/kg | 350000.0 | Effective fuel latent heat used in the bench charge-cooling estimate |
| `model.fuel_evaporation.charge_cooling_effectiveness` | - | 0.72 | Fraction of the ideal latent-heat cooling that is applied to the intake charge |
| `model.fuel_evaporation.intake_charge_temp_min_k` | K | 255.0 | Lower clamp on charge temperature after evaporative cooling |

#### Model.gas_thermo

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.gas_thermo.fresh_cp_j_per_kgk` | J/kg/K | 1005.0 | Representative constant-pressure specific heat of the inducted fresh charge |
| `model.gas_thermo.burned_cp_ref_j_per_kgk` | J/kg/K | 1150.0 | Burned-gas `c_p` at the reference temperature before temperature / EGR corrections |
| `model.gas_thermo.burned_cp_reference_temp_k` | K | 900.0 | Reference temperature used by the burned-gas `c_p` correlation |
| `model.gas_thermo.burned_cp_temp_coeff_j_per_kgk2` | J/kg/K^2 | 0.12 | Slope from burned-gas temperature into `c_p` |
| `model.gas_thermo.burned_cp_egr_coeff_j_per_kgk` | J/kg/K | 120.0 | Additional burned-gas `c_p` gain as internal EGR fraction rises |
| `model.gas_thermo.cp_min_j_per_kgk` | J/kg/K | 950.0 | Lower clamp on any gas specific heat used by the realtime model |
| `model.gas_thermo.cp_max_j_per_kgk` | J/kg/K | 1450.0 | Upper clamp on any gas specific heat used by the realtime model |

#### Model.internal_egr

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.internal_egr.overlap_base_fraction` | - | 0.010 | Residual-gas fraction generated by overlap alone before backflow terms |
| `model.internal_egr.pressure_backflow_gain` | - | 0.12 | Gain from exhaust-over-intake runner pressure head into internal EGR fraction |
| `model.internal_egr.wave_backflow_gain` | - | 0.10 | Gain from negative overlap scavenging head into internal EGR fraction |
| `model.internal_egr.reverse_flow_gain` | - | 0.16 | Gain from reverse exhaust-runner flow into internal EGR fraction |
| `model.internal_egr.reverse_flow_reference_kg_s` | kg/s | 0.06 | Reverse-flow magnitude corresponding to unit internal-EGR backflow excitation |
| `model.internal_egr.fraction_min` | - | 0.0 | Lower clamp on the modeled internal EGR fraction |
| `model.internal_egr.fraction_max` | - | 0.18 | Upper clamp on the modeled internal EGR fraction |
| `model.internal_egr.burn_duration_gain_deg_per_fraction` | degCA/fraction | 22.0 | Burn-duration increase applied per unit internal EGR fraction |
| `model.internal_egr.phase_dilution_gain` | - | 0.30 | Gain reducing ignition-phase effectiveness as internal EGR rises |
| `model.internal_egr.phase_dilution_min` | - | 0.86 | Lower clamp on the internal-EGR phase-dilution multiplier |

#### Model.external_load

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `model.external_load.command_exponent` | - | 1.20 | Exponent shaping the normalized load command into brake demand |
| `model.external_load.base_torque_nm` | Nm | 14.0 | Base resisting torque at full load command before speed-dependent terms |
| `model.external_load.speed_linear_nms` | N m s/rad | 0.012 | Linear speed-dependent load-torque coefficient |
| `model.external_load.speed_quadratic_nms2` | N m s^2/rad^2 | 0.00018 | Quadratic speed-dependent load-torque coefficient |
| `model.external_load.torque_min_nm` | Nm | 0.0 | Lower clamp on the external load torque |
| `model.external_load.torque_max_nm` | Nm | 220.0 | Upper clamp on the external load torque |

### Audio

| YAML key | Symbol | Unit | Sample value | Description |
|---|---:|---:|---:|---|
| `audio.output_gain` | $G_{out}$ | - | 0.0 | Configured master gain before the GUI floor is applied |
| `audio.sample_rate_hz` | $f_s$ | Hz | 48000 | Audio sample rate |
| `audio.model.ambient_pressure_kpa` | $p_{amb,kPa}$ | kPa | 101.325 | Ambient pressure used in audio pressure normalization |
| `audio.model.pressure_span_kpa` | $\Delta p_{span}$ | kPa | 80.0 | Pressure span used for audio normalization |
| `audio.model.rpm_gate_floor_rpm` | $\mathrm{rpm}_{gate,0}$ | rpm | 0.0 | RPM where audio gate starts |
| `audio.model.rpm_gate_full_rpm` | $\mathrm{rpm}_{gate,1}$ | rpm | 450.0 | RPM where audio gate reaches full scale |
| `audio.model.rpm_gate_exponent` | $\beta_{rpm}$ | - | 1.6 | RPM gate curvature |
| `audio.model.exhaust_temp_min_k` | $T_{exh,min}$ | K | 450.0 | Lower clamp for exhaust temperature used in acoustic resonance |
| `audio.model.exhaust_temp_max_k` | $T_{exh,max}$ | K | 1900.0 | Upper clamp for exhaust temperature used in acoustic resonance |
| `audio.model.exhaust_pipe_length_m` | $L_{pipe}$ | m | 1.70 | Effective quarter-wave exhaust pipe length |
| `audio.model.resonator_mode_1` | $m_1$ | - | 1.0 | First resonator mode multiplier |
| `audio.model.resonator_mode_2` | $m_2$ | - | 3.0 | Second resonator mode multiplier |
| `audio.model.resonator_mode_3` | $m_3$ | - | 5.0 | Third resonator mode multiplier |
| `audio.model.resonator_freq_min_hz` | $f_{min}$ | Hz | 20.0 | Lower clamp for digital resonator center frequency |
| `audio.model.resonator_freq_max_nyquist_ratio` | $\rho_{Nyq}$ | - | 0.45 | Upper resonator frequency as a fraction of Nyquist |
| `audio.model.resonator_q_min` | $Q_{min}$ | - | 0.2 | Lower bound for resonator Q |
| `audio.model.resonator_retarget_interval` | $N_{retarget}$ | samples | 256 | Samples between resonator retuning passes |
| `audio.model.resonator_1_q` | $Q_{1,target}$ | - | 2.4 | First resonator Q |
| `audio.model.resonator_2_q` | $Q_{2,target}$ | - | 3.6 | Second resonator Q |
| `audio.model.resonator_3_q` | $Q_{3,target}$ | - | 5.0 | Third resonator Q |
| `audio.model.pressure_smoothing_alpha` | $\alpha_p$ | - | 0.0018 | First-order smoothing gain for normalized pressure |
| `audio.model.runner_pressure_mix` | $w_{runner}$ | - | 1.05 | Exhaust-runner pressure weight in the excitation model |
| `audio.model.collector_pressure_mix` | $w_{collector}$ | - | 0.55 | Exhaust-collector pressure weight in the excitation model |
| `audio.model.intake_pressure_mix` | $w_{intake}$ | - | 0.18 | Intake-runner vacuum weight in the excitation model |
| `audio.model.flow_span_gps` | $\dot m_{span}$ | g/s | 75.0 | Runner-flow magnitude mapped to normalized flow excitation 1.0 |
| `audio.model.flow_pulse_gain` | $k_{\dot m}$ | - | 0.42 | Gain from exhaust runner flow into the pulse envelope |
| `audio.model.pulse_env_decay` | $d_{env}$ | - | 0.991 | Envelope decay factor |
| `audio.model.pulse_env_dp_gain` | $k_{env,\Delta p}$ | - | 30.0 | Pressure-rise gain into the pulse envelope |
| `audio.model.pulse_env_min` | $env_{min}$ | - | 0.0 | Envelope lower clamp |
| `audio.model.pulse_env_max` | $env_{max}$ | - | 2.0 | Envelope upper clamp |
| `audio.model.pulse_shape_decay_fast` | $a_f$ | - | 28.0 | Fast exponential in the exhaust pulse kernel |
| `audio.model.pulse_shape_decay_slow` | $a_s$ | - | 140.0 | Slow exponential in the exhaust pulse kernel |
| `audio.model.pressure_pulse_gain` | $k_{\Delta p}$ | - | 60.0 | Pressure-rise click gain |
| `audio.model.pressure_pulse_min` | $pulse_{min}$ | - | -2.2 | Lower clamp for pressure-rise click term |
| `audio.model.pressure_pulse_max` | $pulse_{max}$ | - | 2.2 | Upper clamp for pressure-rise click term |
| `audio.model.exhaust_pulse_base` | $b_{exh}$ | - | 0.10 | Base exhaust pulse amplitude |
| `audio.model.exhaust_pulse_gain` | $k_{exh}$ | - | 1.15 | Pressure-scaled exhaust pulse amplitude gain |
| `audio.model.pulse_sine_gain` | $k_{sin}$ | - | 0.018 | Sinusoidal support mixed into the excitation |
| `audio.model.direct_pulse_mix` | $w_{dir}$ | - | 0.18 | Direct excitation mixed around the resonators |
| `audio.model.resonator_mix_1` | $w_1$ | - | 0.95 | Weight of resonator 1 |
| `audio.model.resonator_mix_2` | $w_2$ | - | 0.55 | Weight of resonator 2 |
| `audio.model.resonator_mix_3` | $w_3$ | - | 0.24 | Weight of resonator 3 |
| `audio.model.rumble_gain` | $k_{rum}$ | - | 0.06 | Low-frequency rumble gain |
| `audio.model.rumble_harmonic` | $h_{rum}$ | - | 0.5 | Rumble frequency as a fraction of firing phase |
| `audio.model.dc_filter_decay` | $d_{dc}$ | - | 0.996 | DC blocker state decay |
| `audio.model.dc_filter_input_gain` | $k_{dc}$ | - | 0.004 | DC blocker input gain |
| `audio.model.loudness_base` | $g_0$ | - | 0.22 | Base loudness term |
| `audio.model.loudness_pressure_gain` | $g_p$ | - | 0.90 | Loudness gain from normalized exhaust pressure |
| `audio.model.loudness_env_gain` | $g_{env}$ | - | 0.08 | Loudness gain from pulse envelope |
| `audio.model.loudness_normalize_mix` | - | - | 0.80 | Blend from raw loudness to automatic normalization |
| `audio.model.output_gain_floor` | $G_{out,min}$ | - | 0.1 | Lower bound for top-level output gain |
| `audio.model.limiter_out_gain` | $g_{lim}$ | - | 0.95 | Final limiter output scale |

### Numerics

#### RPM-linked timestep rule and state-error normalization

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `numerics.rpm_link_base_dt_min_s` | s | 0.00005 | Minimum base timestep admitted by the RPM-linked timestep rule |
| `numerics.rpm_link_idle_rpm_min` | rpm | 200.0 | Minimum idle-speed reference admitted by the RPM-linked timestep rule |
| `numerics.rpm_link_deg_per_step_min` | deg/step | 1.0 | Lower clamp on target crank-angle advance per integration step |
| `numerics.rpm_link_deg_per_step_max` | deg/step | 12.0 | Upper clamp on target crank-angle advance per integration step |
| `numerics.rpm_link_rpm_floor` | rpm | 80.0 | RPM floor used when converting crank-angle target to timestep |
| `numerics.rpm_link_dt_min_factor` | - | 0.08 | Minimum local timestep as a multiple of base dt |
| `numerics.rpm_link_dt_min_floor_s` | s | 0.00005 | Absolute minimum local timestep |
| `numerics.rpm_link_dt_max_factor` | - | 2.5 | Maximum local timestep as a multiple of base dt |
| `numerics.rpm_link_dt_max_cap_s` | s | 0.012 | Absolute maximum local timestep |
| `numerics.state_error_omega_bias` | rad/s | 10.0 | Bias term in normalized angular-speed error |
| `numerics.state_error_theta_scale_rad` | rad | 0.008726646259971648 | Scale used to normalize crank-angle error |
| `numerics.state_error_p_intake_bias_pa` | Pa | 150.0 | Bias term in normalized intake-pressure error |
| `numerics.state_error_p_intake_rel` | - | 0.01 | Relative term in normalized intake-pressure error |
| `numerics.state_error_p_intake_runner_bias_pa` | Pa | 150.0 | Bias term in normalized intake-runner-pressure error |
| `numerics.state_error_p_intake_runner_rel` | - | 0.01 | Relative term in normalized intake-runner-pressure error |
| `numerics.state_error_p_exhaust_bias_pa` | Pa | 180.0 | Bias term in normalized exhaust-pressure error |
| `numerics.state_error_p_exhaust_rel` | - | 0.01 | Relative term in normalized exhaust-pressure error |
| `numerics.state_error_p_exhaust_runner_bias_pa` | Pa | 180.0 | Bias term in normalized exhaust-runner-pressure error |
| `numerics.state_error_p_exhaust_runner_rel` | - | 0.01 | Relative term in normalized exhaust-runner-pressure error |
| `numerics.state_error_m_dot_intake_runner_scale_kg_s` | kg/s | 0.03 | Scale used to normalize intake-runner-flow error |
| `numerics.state_error_m_dot_exhaust_runner_scale_kg_s` | kg/s | 0.03 | Scale used to normalize exhaust-runner-flow error |
| `numerics.state_error_throttle_scale` | - | 0.01 | Scale used to normalize throttle-state error |

#### Real-time throughput floor estimation

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `numerics.realtime_probe_dt_min_s` | s | 1.0e-5 | Minimum probe timestep used while benchmarking real-time throughput |
| `numerics.realtime_warmup_steps` | steps | 48 | Warmup steps discarded before throughput timing starts |
| `numerics.realtime_sample_steps` | steps | 240 | Timed steps used to estimate wall-clock cost per simulation step |
| `numerics.realtime_margin_factor` | - | 1.25 | Safety margin applied to measured wall time |
| `numerics.realtime_floor_min_s` | s | 1.0e-5 | Absolute minimum real-time timestep floor |
| `numerics.realtime_floor_probe_factor_max` | - | 6.0 | Maximum real-time floor expressed as a multiple of the probe timestep |

### UI

#### Realtime cadence and controls

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `ui.min_base_dt_s` | s | 0.0002 | Minimum base dt the dashboard will accept |
| `ui.realtime_dt_min_factor` | - | 0.05 | Lower bound of dashboard dt relative to base dt |
| `ui.realtime_dt_max_factor` | - | 3.0 | Upper bound of dashboard dt relative to base dt |
| `ui.realtime_dt_max_over_min_factor` | - | 1.5 | Extra constraint enforcing `max_dt >= factor * min_dt` |
| `ui.throttle_key_step` | - | 0.01 | Throttle increment used by the `W` / `X` shortcuts |
| `ui.max_steps_per_frame` | steps/frame | 600 | Maximum simulation substeps performed per GUI frame |
| `ui.dt_smoothing_factor` | - | 0.25 | First-order smoothing factor applied to dt target changes |
| `ui.dt_epsilon_s` | s | 1.0e-6 | Smallest per-step dt allowed once frame remainders are split |
| `ui.vvt_slider_min_deg` | degCA | -40.0 | Minimum intake-VVT slider value shown in the GUI |
| `ui.vvt_slider_max_deg` | degCA | 40.0 | Maximum intake-VVT slider value shown in the GUI |
| `ui.ignition_slider_min_deg` | deg BTDC | -5.0 | Minimum ignition-timing slider value shown in the GUI |
| `ui.ignition_slider_max_deg` | deg BTDC | 45.0 | Maximum ignition-timing slider value shown in the GUI |

#### Plot geometry and window settings

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `ui.plot_height_px` | px | 145.0 | Standard height used for each plot widget |
| `ui.pv_plot_height_px` | px | 290.0 | Height of the main p-V plot placed at the top of the central panel |
| `ui.line_width_px` | px | 2.0 | Standard line width for plot traces |
| `ui.crank_line_width_px` | px | 1.0 | Line width used for the cam-plot crank-angle cursor |
| `ui.torque_min_span_nm` | Nm | 10.0 | Minimum vertical span enforced on the torque plot |
| `ui.torque_margin_ratio` | - | 0.15 | Extra headroom ratio added around the torque span |
| `ui.torque_floor_abs_nm` | Nm | 5.0 | Absolute minimum torque floor span to avoid a flat plot |
| `ui.trapped_air_min_y_max_mg` | mg/cyl | 100.0 | Minimum y-axis maximum for the trapped-air plot |
| `ui.trapped_air_headroom_ratio` | - | 1.15 | Headroom ratio applied above the trapped-air history maximum |
| `ui.pv_headroom_ratio` | - | 1.08 | Reserved p-V headroom ratio used by UI scaling logic |
| `ui.pv_min_headroom_kpa` | kPa | 100.0 | Minimum extra pressure headroom for p-V display calculations |
| `ui.repaint_hz` | Hz | 20 | Requested GUI repaint rate |
| `ui.window_width_px` | px | 1600.0 | Initial application-window width sized for the full dashboard on a typical FHD desktop |
| `ui.window_height_px` | px | 1000.0 | Initial application-window height sized to fit the default graph layout |
| `ui.audio_gain_floor` | - | 0.1 | Minimum audio gain forwarded from the GUI into the synthesizer |

### Bench

| YAML key | Unit | Sample value | Description |
|---|---:|---:|---|
| `bench.default_mode` | enum | `rich_charge_cooling` | Bench mode selected by default in the GUI and on the `B` shortcut |
| `bench.display_rpm_min` | rpm | 0.0 | Left x-axis limit of the displayed bench torque curve |
| `bench.include_zero_rpm_anchor` | - | `true` | Draw a synthetic `0 rpm / 0 Nm` anchor for visual reference |
| `bench.rpm_start_rpm` | rpm | 1000.0 | First RPM point of the automatic torque-curve sweep |
| `bench.rpm_end_rpm` | rpm | 6500.0 | Last RPM point of the automatic torque-curve sweep |
| `bench.rpm_step_rpm` | rpm | 250.0 | RPM increment between successive bench points |
| `bench.sweep_warmup_time_s` | s | 0.35 | Warmup time held at the sweep start RPM before the continuous ramp |
| `bench.sweep_duration_s` | s | 4.0 | Simulated duration of the continuous RPM sweep |
| `bench.sweep_integration_deg_per_step` | deg/step | 2.4 | Nominal crank-angle advance per accepted step during the continuous sweep |
| `bench.settle_time_s` | s | 0.45 | Stabilization time before averaging at each locked-RPM point |
| `bench.average_time_s` | s | 0.25 | Averaging time used by the single-point locked-RPM bench helper |
| `bench.locked_cycle_samples` | samples | 240 | Crank-angle samples used by the single-point locked-cycle averaging helper |
| `bench.initial_map_ratio` | - | 0.95 | Initial intake-pressure guess as a fraction of ambient when a bench point starts |
| `bench.initial_exhaust_over_ambient_pa` | Pa | 7000.0 | Initial exhaust overpressure guess when a bench point starts |
| `bench.lambda_one_target` | - | 1.0 | Lambda target used by the stoichiometric bench mode |
| `bench.rich_charge_cooling_lambda` | - | 0.88 | Lambda target used by the rich bench mode with stronger charge cooling |
| `bench.integration_deg_per_step` | deg/step | 0.8 | Nominal crank-angle advance per accepted step in the locked-RPM bench helper |
| `bench.integration_dt_min_s` | s | 5.0e-6 | Lower timestep clamp for the adaptive bench integrator |
| `bench.integration_dt_max_s` | s | 2.0e-4 | Upper timestep clamp for the adaptive bench integrator |
| `bench.integration_error_tolerance` | - | 0.03 | Normalized local state-error tolerance used by bench step-doubling |
| `bench.integration_dt_growth` | - | 1.25 | Growth factor applied to the next bench timestep after a comfortably accepted step |
| `bench.integration_refine_limit` | steps | 8 | Maximum number of timestep halvings attempted for one bench step |
| `bench.steps_per_frame` | accepted steps/frame | 8000 | Maximum accepted bench integration steps processed per GUI frame |
| `bench.frame_time_budget_ms` | ms | 18.0 | Wall-clock budget spent on bench integration inside one GUI frame |

### Plot

| YAML key | Symbol | Unit | Sample value | Description |
|---|---:|---:|---:|---|
| `plot.rpm_history_capacity` | - | samples | 200 | RPM and history buffer length |
| `plot.history_recent_cycles` | $N_{hist}$ | cycles | 1 | Number of completed history cycles retained behind the current partial cycle in the RPM / torque / trapped-air crank-angle plots; unrelated to cylinder count |
| `plot.pv_recent_cycles` | $N_{pv}$ | cycles | 4 | Number of recent cycles shown in p-V |
| `plot.pv_subsamples_per_step` | $N_{sub}$ | samples/step | 120 | p-V subsamples per integration step |
| `plot.pv_x_min` | $v_{plot,min}$ | - | 0.0 | p-V x min |
| `plot.pv_x_max` | $v_{plot,max}$ | - | 1.15 | p-V x max |
| `plot.pv_y_min_kpa` | $p_{plot,min}$ | kPa | 0.0 | p-V y min |
| `plot.pv_y_max_kpa` | $p_{plot,max}$ | kPa | 2500.0 | p-V y max |

## Hard-Coded Physical Constants

| Name | Value | Unit | Use |
|---|---:|---:|---|
| `R_AIR` | 287.0 | J/(kg K) | Ideal-gas relation and compressible flow |
| `GAMMA_AIR` | 1.4 | - | Compressible flow and Otto efficiency |
| `FUEL_LHV_J_PER_KG` | 43.0e6 | J/kg | Fuel-to-work conversion |

## Code Organization

- `src/main.rs`: entry point
- `src/config.rs`: YAML config schema and loader
- `src/constants.rs`: global constants
- `src/simulator.rs`: ODE model, submodels, RK2, p-V sampling, model tests
- `src/dashboard.rs`: GUI loop, adaptive-step controller, plotting
- `src/audio.rs`: firing-frequency and exhaust-resonance synthesis engine

## External Validation

The default calibration is now intentionally centered on the modern high-efficiency `2.0 L` naturally aspirated gasoline family represented by Toyota Dynamic Force and Mazda SKYACTIV-G, while Toyota 86 is used only as a sporty upper-envelope check [S4, S5, S6, S7].

### Official reference anchors used for calibration

| Engine family | Geometry / compression ratio | Official torque | Official power | Role in calibration |
|---|---|---:|---:|---|
| Toyota Corolla Hatchback 2.0 Dynamic Force | `80.5 x 97.6 mm`, `r=13.0` | `205 Nm @ 4800 rpm` | `126 kW (169 hp) @ 6600 rpm` | Main production torque / power anchor |
| Mazda SKYACTIV-G 2.0 | `83.5 x 91.2 mm`, `r=12.0` | `194 Nm @ 4100 rpm` | `113 kW (154 PS) @ 6000 rpm` | Secondary production anchor for geometry and peak-output band |
| Honda Civic 2.0 i-VTEC | `86.0 x 85.9 mm`, `r=10.8` | `187 Nm (138 lb-ft) @ 4200 rpm` | `117.8 kW (158 hp) @ 6500 rpm` | Lower-specific-output naturally aspirated production anchor |
| Ford EcoSport 2.0 Ti-VCT | `87.5 x 83.1 mm`, `r=12.0` | `202 Nm (149 lb-ft) @ 4500 rpm` | `123.8 kW (166 hp) @ 6500 rpm` | Broader mainstream 2.0L NA envelope anchor |
| Toyota 86 2.0 | `86.0 x 86.0 mm`, `r=12.5` | `205 Nm @ 6600 rpm` | `147 kW @ 7000 rpm` | Sports-engine upper envelope; not the default target |

### Resulting default simulator calibration

The checked-in default geometry is therefore long-stroke and high-compression rather than square-bore / low-compression:

| Quantity | Default config |
|---|---:|
| Displacement | `1.99 L` |
| Bore x stroke | `80.5 x 97.6 mm` |
| Compression ratio | `13.0` |
| Peak brake torque in reference bench | `198.9 Nm @ 4500 rpm` |
| Peak brake power in reference bench | `129.0 kW @ 6500 rpm` |
| Peak brake BMEP | `12.6 bar` |
| Peak specific power | `64.9 kW/L` |

This places the default bench curve close to Corolla / Mazda production `2.0 L NA` engines, above the lower-output Honda Civic 2.0 tune, near the Ford EcoSport 2.0 mainstream band, and still below the Toyota 86 sports-engine power band.

### Reference rich-WOT bench curve

The table below is emitted by test `rich_wot_bench_curve_resembles_modern_2l_na_shape` after the present calibration:

| RPM [rpm] | Net torque [Nm] | Brake power [kW] |
|---:|---:|---:|
| 1500 | 187.6 | 29.5 |
| 2500 | 197.3 | 51.6 |
| 3500 | 196.5 | 72.0 |
| 4500 | 198.9 | 93.7 |
| 5500 | 174.4 | 100.5 |
| 6500 | 189.5 | 129.0 |

The important qualitative behavior is preserved:

- torque climbs quickly from low speed into a broad mid-speed plateau
- peak power arrives later than peak torque
- specific power stays in the same rough class as current efficient-production `2.0 L` engines, without drifting into the higher-output sports-engine envelope

The most important calibration changes that moved the model into that band were:

- switching the default geometry to the official Dynamic Force-like `80.5 x 97.6 mm`, `r=13.0` set
- reducing friction coefficients modestly to match modern low-friction production engines
- broadening the VE peak so high-rpm power does not collapse too early
- easing heat-loss clipping so full-load BMEP is not suppressed unrealistically

The present default calibration is therefore intentionally a "high-efficiency modern 2.0L NA inline-4" baseline, not a fit to every naturally aspirated 2.0L engine on the market. Honda Civic 2.0 and Ford EcoSport 2.0 are used as broader envelope checks, while Toyota 86 remains the sporty upper bound.

## Engineering Interpretation of Displayed Values

For a `2.0 L` 4-cylinder engine, each cylinder displaces about `0.50 L`. At roughly `101 kPa`, `305 K`, and `VE=1`, the ideal-gas trapped-air estimate is about `0.58 g/cyl = 580 mg/cyl`. That keeps the dashboard's `100-150 mg/cyl` idle-like region and `450-500 mg/cyl` high-load region in a believable naturally aspirated SI range.

The fixed-spark steady-state indicated-efficiency sweep should still be read as an internal model regression anchor, not as a final dyno-facing claim by itself. Under the present default coefficients it clips the configured `eta_indicated_max = 0.44` ceiling over much of the loaded sweep, so brake torque / power comparison against production engines is presently the more meaningful external validation metric.

## Tests

```bash
cargo test --release -- --nocapture
```

Current tests cover:

- checked-in commented YAML parsing
- auto mode reaching idle from stop
- dedicated `lambda=1` WOT bench points hitting stoichiometric lambda
- rich charge-cooling WOT bench points outperforming the stoichiometric bench at the same RPM
- physical bounds under start sequence
- zero-RPM audio silence
- audio RMS increase with exhaust pressure
- audio periodicity tracking the 4-stroke firing frequency as RPM rises
- recent-cycle p-V retention
- p-V combustion phasing moving with the burn window / ignition timing model
- p-V work integration sanity against a rectangular loop
- finite thermal observables
- RPM-linked dt monotonic behavior
- exact RK2 throttle-lag solution matching at high RPM
- exact RK2 linear spin-decay matching at high RPM
- nonlinear high-RPM state convergence under timestep refinement
- near-redline fired operation staying physical
- state error norm sanity
- ignition timing command mapping sanity
- retarded ignition raises modeled exhaust temperature (and lowers indicated efficiency)
- overly advanced ignition can stall engine
- higher compression ratio -> higher Otto efficiency
- no-fuel -> near-zero indicated efficiency and combustion torque
- friction-only coastdown
- fired pressure not collapsing immediately after combustion
- IMEP not alternating zero at WOT fired operation
- net torque consistency with component torques
- representative fixed-control operating points returning to the same `360 degCA` phase section with small cycle-to-cycle drift
- steady-state throttle sweep remains finite and inside the configured indicated-efficiency bounds
- locked-speed full-load brake torque stays in a realistic modern 2.0 L NA gasoline band
- locked-speed full-load air consumption stays in a realistic 2.0 L NA gasoline band
- default geometry stays inside the Toyota Dynamic Force / Mazda SKYACTIV-G `2.0 L` production family
- rich WOT bench samples stay inside the Corolla / Mazda production torque-power envelope while remaining below the Toyota 86 sports upper envelope
- rich WOT bench peak torque / power / peak-speed locations also stay inside a broader modern 2.0 L NA envelope that includes Honda Civic 2.0 and Ford EcoSport 2.0

## Sources and Reference Anchors

- `[S1]` Elbert Hendricks, *A Generic Mean Value Engine Model for Spark Ignition Engines*, Proceedings of the 41st Simulation Conference, SIMS 2000. DTU Orbit landing page: <https://orbit.dtu.dk/en/publications/a-generic-mean-value-engine-model-for-spark-ignition-engines/>
- `[S2]` NASA Glenn Research Center, *Mass Flow Choking* and related compressible-nozzle notes used as the reference form for the choked / unchoked orifice relations in this README: <https://www.grc.nasa.gov/www/k-12/BGP/mflchk.html>
- `[S3]` NASA Glenn Research Center, *Ideal Otto Cycle*, used as the reference ideal-cycle efficiency baseline for the theoretical Otto-efficiency discussion: <https://www.grc.nasa.gov/www/k-12/airplane/otto.html>
- `[S4]` Toyota Corolla Hatchback eBrochure / specs page, used as the main official production torque / power anchor for the default calibration (`169 hp @ 6600 rpm`, `151 lb-ft @ 4800 rpm`): <https://www.toyota.com/corollahatchback/ebrochure/>
- `[S5]` Toyota, *TNGA Powertrain 2018, Engine*, used as the official Dynamic Force geometry / compression-ratio / efficiency anchor (`80.5 x 97.6 mm`, `r=13.0`, thermal efficiency around `40%`): <https://global.toyota/en/mobility/tnga/powertrain2018/engine/>
- `[S6]` Mazda, *Major Specifications (Japan) for Mazda Axela / Mazda3 with SKYACTIV-G 2.0*, used as the official SKYACTIV-G geometry / torque / power anchor (`83.5 x 91.2 mm`, `r=12.0`, `194 Nm`, `154 PS`): <https://newsroom.mazda.com/en/publicity/release/2011/201109/110927a.html>
- `[S7]` Toyota, *Toyota 86* official detail page, used as the sporty `2.0 L NA` upper-envelope reference (`147 kW @ 7000 rpm`, `205 Nm @ 6600 rpm`, `86.0 x 86.0 mm`, `r=12.5`): <https://global.toyota/en/detail/173965>
- `[S8]` GitHub Docs, *Writing mathematical expressions* and *Setting a Markdown processor for your GitHub Pages site using Jekyll*, used for the README math-rendering guidance: <https://docs.github.com/en/enterprise-cloud%40latest/get-started/writing-on-github/working-with-advanced-formatting/writing-mathematical-expressions>, <https://docs.github.com/en/pages/setting-up-a-github-pages-site-with-jekyll/setting-a-markdown-processor-for-your-github-pages-site-using-jekyll>
- `[S9]` Honda, *2024 Civic Sedan Features & Specs*, used as the official Civic 2.0 naturally aspirated anchor (`158 hp @ 6500 rpm`, `138 lb-ft @ 4200 rpm`, `86.0 mm x 85.9 mm`, `r=10.8`): <https://automobiles.honda.com/2024/civic-sedan/specs-features-trim-comparison>
- `[S10]` Ford, *2022 EcoSport* brochure, used as the official EcoSport 2.0 Ti-VCT anchor (`166 hp @ 6500 rpm`, `149 lb-ft @ 4500 rpm`): <https://www.ford.com.pr/content/dam/Ford/website-assets/latam/pr/en-ebrochure/ford-pr-ecosport-2022-brochure-descargable-eng.pdf>
