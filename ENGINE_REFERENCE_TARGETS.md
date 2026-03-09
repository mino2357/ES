# Engine Reference Targets

## Scope

This repository currently has two practical calibration classes:

- fixed inline-4 naturally aspirated targets that can be calibrated today
- clearly documented future targets that require architecture work first

The present solver is still fixed to 4 cylinders and does not model turbocharging or supercharging.
Because of that, V8/V10/V12 supercar engines are useful reference targets, but not honest calibration
targets for the current implementation.

The reference targets below are intentionally anchored to primary sources with bore, stroke, compression ratio,
and rated power / torque points. I did not lock the current presets to secondary database sites.

## Terminology Used In This Document

This file is intended to stand on its own.

- `calibration target`: an engine whose published data are detailed enough to tune the simulator against in a technically honest way.
- `preset`: a checked-in YAML file under `config/presets/` that encodes one reference engine setup for the headless solver.
- `spec anchor`: a published quantity such as bore, stroke, compression ratio, rated power, or rated torque that the preset should match or approach.
- `dyno fit`: a calibration judged against measured or published torque / power behavior, not just against internal numerical consistency.
- `NA`: naturally aspirated.
- `OEM`: original equipment manufacturer. In this file it refers to the manufacturer-issued specification page used as the primary source.

## Calibrated / Approximated Today

### 1. Nissan Tiida HR16DE from Nissan official specs

Source:

- Nissan official Tiida specification page:
  - inline-4 HR16DE, `1598 cc`
  - `78.0 mm x 83.6 mm`
  - compression ratio `9.8`
  - max power `80 kW @ 6000 rpm`
  - max torque `153 Nm @ 4400 rpm`
  - https://www.nissan-sd.com/models/tiidasdsl/Specifications/12151804385257987_1.htm

Preset:

- `config/presets/nissan_tiida_hr16de.yaml`

Current headless result:

- `168.7 Nm @ 4400 rpm`
- `84.5 kW @ 6000 rpm`

Read of the mismatch:

- geometry is anchored to the published engine
- rated-power point is close enough to use as a practical anchor
- torque at `4400 rpm` is still about `+10%` high
- this economy-oriented 1.6 L engine remains a useful small-displacement NA reference, but it is not yet a finished dyno fit

Run:

```bash
cargo run --no-default-features --bin es_hp -- config/presets/nissan_tiida_hr16de.yaml
```

### 2. Honda S2000 F20C from Honda official specs

Source:

- Honda official S2000 launch / F20C announcement:
  - `1997 cm3`
  - `87.0 mm x 84.0 mm`
  - compression ratio `11.7`
  - `250 PS @ 8300 rpm` (`184 kW`)
  - `22.2 kgm @ 7500 rpm` (`217.7 Nm`)
  - https://global.honda/jp/news/1999/4990415.html
  - https://global.honda/en/newsroom/worldnews/1999/4990223b.html

Preset:

- `config/presets/honda_s2000_f20c.yaml`

Current headless result:

- `239.5 Nm @ 7500 rpm`
- `190.7 kW @ 8300 rpm`

Read of the mismatch:

- high-rpm power is close enough to be a useful reference anchor
- torque peak is still about 10% high
- the preset is therefore a reasonable high-specific-output NA-4 target, but not yet a finished dyno fit

Run:

```bash
cargo run --no-default-features --bin es_hp -- config/presets/honda_s2000_f20c.yaml
```

## Future Clear-Spec Target

### Ferrari 458 Italia

Official Ferrari specs:

- V8, `4497 cc`
- `419 kW (570 CV) @ 9000 rpm`
- `540 Nm @ 6000 rpm`
- compression ratio `12.5:1`
- https://www.ferrari.com/en-EN/auto/458-italia

Why it is not calibrated yet:

- current solver is fixed to 4 cylinders
- no flat-plane V8 firing-order model exists yet
- the exhaust / audio path is still built around the fixed 4-cylinder assumption

This is therefore a good future benchmark once variable cylinder count and engine-bank / firing-order support are added.

## Notes For Next Calibration Pass

If the next goal is tighter spec matching rather than broader engine coverage, the most cost-effective
order is:

1. finish the F20C fit at `7500 / 8300 rpm`
2. improve trapped-mass / residual-gas shaping for the small-displacement HR16DE case
3. add variable cylinder count to the headless solver
4. then attack a true supercar engine such as the 458 Italia
