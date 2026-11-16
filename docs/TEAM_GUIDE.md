# Spool Team Guide — FAT MAN Worked Example

_Get tuned gains from CAD to the real robot in ~15 minutes_.

## 1. Install

```bash
cd spool
pip install -e .
pytest
```

## 2. Run Spool — generate gains from CAD

```bash
spool fatman
```

This reads your local CAD export and BOM, computes physics-based gains (kS, kV, kA, kG), and writes everything to `out/fatman/`.

**What you get:**
| File | What it is |
|---|---|
| `robot.json` | Robot description; every number tagged by source (cad/bom/estimated) |
| `gains.json` | kS/kV/kA/kG per mechanism, tuned PID, servo axes, ball recovery |
| `plots/*.png` | Step-response plots: flywheel, turret, hood, arm, drive |
| `exports/pedro/Constants.java` | Pedro Pathing constants (verified field names) |
| `exports/pedro/PEDRO_TUNING_CHECKLIST.md` | AutoTune procedures in order |
| `exports/ftc_dashboard/SpoolGains.java` | Live-editable `@Config` class for Dashboard Panels |
| `exports/wpilib/` | FRC-equivalent `Constants.java` + SysId field map |
| `exports/mosim/` | FRC-equivalent mod skeleton (hierarchy + params + C#) |

## 3. Drop Pedro Constants into your TeamCode

1. Copy `out/fatman/exports/pedro/Constants.java` into `TeamCode/src/.../pedro/`
2. Fix motor names (`lf`, `lr`, `rf`, `rr`) and Pinpoint name to match your DS config
3. Invert directions until "forward" is actually forward
4. Set `xPodOffset`/`yPodOffset` to your real pod offsets (Spool copies example placeholders)
5. Upload the `@Tuner` from `PEDRO_TUNING_CHECKLIST.md`, open `http://192.168.43.1:10158`
6. Run checklist procedures in order: Forward Velocity ID, Strafe Velocity ID, Forward Decel ID, Strafe Decel ID

**Spool-derived numbers for FAT MAN:** 100.66 in/s forward, 78.52 in/s strafe, 30.89 in/s² forward decel, 35.52 in/s² strafe decel.

## 4. Calibrate on the Real Robot

```bash
# Copy SpoolCalibrate.java to TeamCode, set MOTOR_NAME and TICKS_PER_REV
# Clear the mechanism so it spins freely (no stops during run)
# On DS: select SpoolCalibrate, press start
# It runs three blocks: 0→2 V ramp (kS), steady 2–7 V (kV), 3/6/9/6/3 V steps (kA)
# Log lands at: /sdcard/FIRST/spool/<MOTOR_NAME>.csv
```

## 5. Fit the Log & Compare

```bash
adb pull /sdcard/FIRST/spool/flywheel.csv
spool calibrate flywheel.csv --predicted out/fatman/gains.json --mechanism flywheel
```

**Good fit:** R² > 0.99. The first block gives measured kS/kV/kA; the second compares against CAD:
```json
"kS": { "cad": 1.1167, "fitted": 1.1189, "abs_diff": 0.0022, "rel_diff": 0.002 }
```

## 6. Paste Measured Gains Back

Update `gains.json` (or `robot.json` with `kS_measured`), re-run `spool fatman`, and downloads update.

**Rules of thumb:**
- **kS will move most:** It is a friction model, not CAD. Expect flywheel to be furthest off.
- **kV should land within a few percent** of datasheet. If not, check gear ratio or ticks/rev.
- **kA is noisiest:** Needs a clean step and known inertia; ±5 % is good.
- **kG (arm):** Hold arm horizontal, raise kS/kV/kA to zero, the voltage that just holds it is kG.
- **Two-servo axes:** Export ratio/speed/torque/limits, not PIDF. Position servo closes its own loop. Clamp commanded angles 5° inside mechanical stops.

## 7. What's Still Estimated

| Number | Tag | What replaces it |
|---|---|---|
| robot mass 12.70 kg | estimated | weigh the robot |
| flywheel inertia | estimated | spin-up log gives kA, hence J |
| turret/hood/arm inertia | estimated | slew log, or CAD with densities |
| ball mass 0.06 kg, exit 9.4 m/s | estimated | chronograph + scale |
| rolling μ 0.08, strafe 0.78 | estimated | Pedro's Forward/Strafe Velocity ID |
| every kS | estimated | `SpoolCalibrate` + `spool calibrate` |

---

**Troubleshooting:**
- `spool: command not found` — `pip install -e .` didn't finish, or scripts dir not on PATH; use `python -m spool.cli fatman`
- Web UI `/api/run` takes minutes — that's the full optimizer; use `spool fatman --no-tune` for fast export only
- Pedro constants look like old examples — Spool emits current `ForesightConfig` API, not the old one

---
*FAT MAN: FTC robot with mecanum drive, 2-motor flywheel, 2-servo turret, 1-servo hood, 2-servo flower arm. All gains derived from physics formulas or measured on robot.*