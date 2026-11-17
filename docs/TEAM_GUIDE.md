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

Open `docs/demo/index.html` in any browser for a read-only page with the same
gains, plots and before/after table. It needs no server and no network, and every
`spool fatman` rebuilds it (`spool demo` rebuilds it from an existing `out/`).

### Turret travel: one number Spool will not guess

The FAT MAN CAD encodes two different turret travels: soft stops 270 deg apart
(-135 to +135 deg) and hard stops 280 deg apart. The 25T to 60T servo gearing can
only produce 320 deg / 2.40 = 133.3 deg at the axis, which cannot reach either
reading. `out/fatman/SUMMARY.md` opens with a WARNING when the CAD holds two
numbers like this, and `robot.json` carries the same text under `turret_travel`.

Measure the real travel between the hard stops on the built robot, then:

```bash
spool fatman --turret-travel-deg 132
```

That sets the hard stops to +-66 deg and the soft stops 5 deg inside (+-61 deg),
tags the limits `measured` in `robot.json`, and replaces the warning with the
measured values. Without the flag Spool keeps the CAD encoding exactly as written
and keeps warning; it does not pick a value. If the measured travel is still more
than the gearing can reach (133.3 deg at 2.4:1), the warning stays: that means
the gear ratio or the measurement is wrong, and only the team can say which.

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

### Log format accepted by `spool calibrate`

The canonical header is:

```
time_s,voltage_v,velocity,position,current
```

Only `time_s`, `voltage_v` and `velocity` are required. `position` is used by
`--gravity`; `current` is read but the fit does not need it. Header names are
matched loosely, so `t`, `time`, `voltage`, `volts`, `speed`, and units written
into the name (`velocity (ticks/s)`) all load.

Velocity is rad/s unless you say otherwise. Encoder counts are common on a real
FTC log, so:

| What the log records | Command |
|---|---|
| rad/s (SpoolCalibrate) | `spool calibrate flywheel.csv` |
| ticks/s, bare `velocity` column | `spool calibrate flywheel.csv --units ticks --ticks-per-rev 28` |
| a `velocity_ticks_s` column | `spool calibrate flywheel.csv --ticks-per-rev 28` |
| old `t,voltage,velocity` header | `spool calibrate flywheel.csv` |

`--ticks-per-rev` is encoder counts per revolution of the shaft your velocity is
measured on. On FAT MAN that is 28 on the flywheel (goBILDA 5203 at 1:1), 384.5
for the 435 rpm gearmotor and 537.7 for the 312 rpm one. Position is converted
with the same factor, so `--gravity` works on a ticks log too.

A minimal rad/s log looks like this:

```csv
time_s,voltage_v,velocity
0.000,1.00,0.0
0.010,1.05,0.42
0.020,1.10,0.97
```

If a required column is missing, a row is not numeric or a ticks log has no
`--ticks-per-rev`, the command stops with the column names it expected instead of
fitting zeros. The fitted gains are always in rad/s, whatever the input header.

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
- `a ticks/s log needs --ticks-per-rev` — add the encoder's counts per revolution, or pass `--units rad` if the column really is rad/s
- `SUMMARY.md` says the turret travel is ambiguous — measure the travel between hard stops and pass `--turret-travel-deg <total deg>`
- Web UI `/api/run` takes minutes — that's the full optimizer; use `spool fatman --no-tune` for fast export only
- Pedro constants look like old examples — Spool emits current `ForesightConfig` API, not the old one

---
*FAT MAN: FTC robot with mecanum drive, 2-motor flywheel, 2-servo turret, 1-servo hood, 2-servo flower arm. All gains derived from physics formulas or measured on robot.*