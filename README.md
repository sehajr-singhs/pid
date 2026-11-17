# Spool

**CAD in, tuned robot out.** Physics-based starting gains (kS, kV, kA, kG, PIDF)
for every mechanism, computed before the robot is built, exported into Pedro
Pathing (FTC) and WPILib / MoSim (FRC), then finished on the real robot from one
short logged test run.

```
pip install -e .
pytest
spool fatman        # -> out/fatman/{robot.json,gains.json,plots,exports}
spool web           # http://127.0.0.1:8000/  local UI
docs/demo/index.html  # static demo page, opens from file, no server
```

New to this? `docs/QUICKSTART.md` walks an FTC team from CAD export to measured
gains in six steps.

## What it actually computes

Every gain comes from a formula or from a simulation you can re-run. Nothing is
an LLM guess, and nothing is hand-waved: if a number could not be derived from
the CAD/datasheet it is tagged `estimated`, and if it cannot be modelled at all
(friction, roller slip) it is flagged for measurement.

| Quantity | Formula | Source of inputs |
|---|---|---|
| kV | `G * k_e` (rotary), `G * k_e / r` (linear) | datasheet |
| kA | `J_eff * R / (n * eta * G * k_t)` | CAD inertia + datasheet |
| kG | `m * g * r * R / (n * eta * G * k_t)` | CAD mass + geometry |
| kS | friction model, `tau_f * R / (n * eta * G * k_t)` | **flagged: measure it** |

`J_eff = J_load + n * G^2 * J_rotor`. Forgetting the `G^2` term is the single most
common way to get kA wrong, so it is explicit everywhere.

## Pipeline

```
CAD export ──> robot.json ──> gains ──> step-response sim + tuning ──> exports
   (local FAT MAN export, URDF, STEP, or optional Onshape API)
```

1. **Ingest** (`spool/ingest/`) — FAT MAN from the local Onshape export + BOM
   (an xlsx reader with no extra dependencies), plus generic URDF / STEP paths.
2. **Physics** (`spool/physics/`) — motor model, feedforward, mechanism models, a
   1 kHz headless simulator, and a deterministic differential-evolution tuner.
3. **Exports** (`spool/export/`) — Pedro Pathing constants, an FTC Dashboard /
   Panels config class, WPILib `Constants.java` + SysId log field map, and a MoSim
   mod skeleton.
4. **Runtime** (`spool/runtime/`) — `SpoolCalibrate` for FTC (OpMode) and FRC
   (Command), and `fit.py` which recovers kS/kV/kA from that log.

## Honest status

* MoSim is an **FRC** simulator; the FAT MAN export is the FRC-equivalent
  mechanism set, not the two-servo FTC robot. See `out/fatman/exports/mosim/`.
* The FTC turret, hood and flower arm are **position servos**. PIDF is not the
  right tool there, so Spool exports ratio/speed/torque/limits instead.
* `docs/VERIFIED_APIS.md` lists every third-party class and field used, with the
  URL it was read from.

See `REPORT.md` for the full write-up: what is computed, what is estimated, every
formula and every open question.
