# Piπd quickstart: FTC team

You have a CAD model and a pile of parts. This is the shortest path from that to
gains on the real robot. Every number Piπd prints is either computed from a
formula (and re-derivable by re-running) or tagged `estimated` with the assumption
stated; nothing is a model guess.

Piπd does **not** need the Onshape API, a cloud account, or a GPU. One laptop,
CPU only, a few minutes.

---

## 0. Install (once)

```bash
cd pid
pip install -e .          # numpy, scipy, matplotlib
pytest                    # the physics + fit tests
```

Optional extras: `pip install -e ".[step]"` for STEP ingest, `.[onshape]` for the
off-by-default Onshape path.

## 1. Get the robot into `robot.json`

**FAT MAN (the bundled robot):**

```bash
pid fatman
```

It reads the local CAD export and BOM from `C:/Users/sehaj/ftc-cad/robots/fatman`
(parts JSONL, assembly bbox, Onshape mass properties, `FAT_MAN_BOM.xlsx`) and
writes everything to `out/fatman/`. Use `--root` / `--out` for another checkout.

**Any other robot:** export a URDF (or a STEP file) from your CAD and run

```bash
pid urdf robot.urdf  --out out/myrobot
pid step robot.step  --out out/myrobot --motor gobilda-5203-2402-0014 --ratio 13.73
```

Either way you get `out/<robot>/robot.json`: mass, envelope, and one block per
mechanism with `gearing.ratio`, motor/servo SKU, inertia, limits, and a
`provenance` tag on every input (`cad` / `bom` / `datasheet` / `estimated`).

Open it before you trust it. If a gear ratio or a mass is wrong, every gain is
wrong in the same direction.

## 2. Piπd the gains

```bash
pid fatman          # prints the before/after step-response table at the end
pid web             # http://127.0.0.1:8000/, mechanisms, tagged gains, plots, downloads
```

`out/fatman/` then holds:

| File | What it is |
|---|---|
| `robot.json` | the input document, every number tagged by source |
| `gains.json` | kS/kV/kA/kG per mechanism, tuned PID, servo axes, ball recovery, source tags |
| `plots/*.png` | flywheel spin-up + recovery, turret, hood, arm, drive forward/strafe |
| `exports/pedro/Constants.java` | Pedro Pathing constants (verified field names) |
| `exports/pedro/PEDRO_TUNING_CHECKLIST.md` | the AutoTune procedures, in order |
| `exports/ftc_dashboard/PidGains.java` | live-editable `@Config` class for Panels |
| `exports/wpilib/` | FRC-equivalent `Constants.java` + SysId field map |
| `exports/mosim/` | FRC-equivalent hierarchy/params + C# mod skeleton |

The web UI's **Downloads** section serves exactly those files.

## 3. Drop Constants into Pedro

1. Copy `out/fatman/exports/pedro/Constants.java` into your TeamCode under
   `org/firstinspires/ftc/teamcode/pedro/` (keep the package line, or rename the
   package to match your project).
2. Fix the four motor names (`lf`, `lr`, `rf`, `rr`) and the Pinpoint name to
   match your Driver Station configuration. Invert directions in the config until
   "forward" is actually forward.
3. Set `xPodOffset` / `yPodOffset` to your real pod offsets. Piπd copies the
   Pedro docs' example values (2.187 / -4.572 in) and marks them as placeholders -
   they are not your robot.
4. Upload the `@Tuner` from `PEDRO_TUNING_CHECKLIST.md`, open
   `http://192.168.43.1:10158`, and run the procedures in the checklist order.
   Piπd fills in the four quantities it can derive (max forward/strafe speed,
   natural forward/strafe deceleration); braking coefficients are placeholders
   until AutoTune measures them.
5. Confirm on the first run that `maxAchievableForwardVelocity` is being read as
   **inches/second** (Piπd emits ~100 for FAT MAN). The docs never state the unit
   in words; it is inferred from the docs example.

The four numbers Piπd supplies for FAT MAN: 100.66 in/s forward, 78.52 in/s
strafe, 30.89 in/s² forward deceleration, 35.52 in/s² strafe deceleration.

## 4. Run PidCalibrate on the robot

`pidtool/runtime/ftc/PidCalibrate.java` is a plain OpMode with no dependencies
beyond the FTC SDK.

1. Copy it to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pidtool/`.
2. Set `MOTOR_NAME` to the name in your robot configuration and `TICKS_PER_REV`
   for that motor (28 at the output shaft of a goBILDA 5203 1:1, 384.5 for the
   435 rpm gearmotor, 537.7 for 312 rpm).
3. **Clear the mechanism so it can spin freely.** The arm/flywheel must not hit a
   stop during the run.
4. Select `PidCalibrate` on the Driver Station and press start. It runs three
   blocks, a 0→2 V ramp in 0.1 V steps (kS), steady 2–7 V (kV), and 3/6/9/6/3 V
   steps (kA), logging at 100 Hz.

The log lands on the Robot Controller at:

```
/sdcard/FIRST/pidtool/<MOTOR_NAME>.csv
```

with columns `t,voltage,velocity,position,current` in SI units (s, V, rad/s, rad, A).
Battery voltage is measured every loop, so the logged volts are the volts actually
applied even as the pack sags.

## 5. Fit the log and compare with CAD

Pull the CSV and fit it:

```bash
adb pull /sdcard/FIRST/pidtool/flywheel.csv
pid calibrate flywheel.csv --predicted out/fatman/gains.json --mechanism flywheel
```

The first JSON block is the measured kS/kV/kA (plus R² and per-parameter standard
errors). The second is the comparison against the CAD prediction:

```json
"kS": { "cad": 1.1167, "fitted": 1.1189, "abs_diff": 0.0022, "rel_diff": 0.002 }
```

A good log fits with R² > 0.99. If R² is below 0.9, the mechanism hit a stop, the
speed gate excluded too much data, or the motor's encoder scale is wrong, fix
that before trusting the gains.

`--gravity` adds the `kG·cos(theta)` regressor for an arm, and needs a `position`
column (PidCalibrate always writes one).

## 6. Paste the measured gains back

Update `gains.json` (or `robot.json` with `kS_measured`), re-run `pid fatman`,
and the exports regenerate. Then:

```bash
pid web        # plots move to the measured values; download the new exports
```

Rules of thumb from the FAT MAN model:

* **kS will move most.** It is a friction model, not CAD. On FAT MAN expect the
  flywheel to be the furthest off (the belt-preload term is a guess).
* **kV should land within a few percent** of the datasheet value. If it does not,
  the gear ratio or the encoder's ticks-per-rev is wrong.
* **kA is the noisiest.** It needs a clean step and a known inertia; ±5 % is a
  good fit.
* **kG** (arm) can be checked statically: hold the arm horizontal, raise kS/kV/kA
  to zero, and the voltage that just holds it is kG.

## 7. What is still estimated

Piπd tags every input. On FAT MAN the ones that matter:

| Number | Tag | What replaces it |
|---|---|---|
| robot mass 12.70 kg | estimated | weigh the robot |
| flywheel inertia 1.04e-4 kg·m² | estimated | a spin-up log gives kA, hence J |
| turret/hood/arm inertia | estimated | a slew log, or CAD with densities |
| ball mass 0.06 kg, exit 9.4 m/s | estimated | chronograph + scale |
| rolling μ 0.08, strafe 0.78 | estimated | Pedro's Forward/Strafe Velocity ID |
| every kS | estimated | `PidCalibrate` + `pid calibrate` |

The two-servo axes export ratio/speed/torque/limits, not PIDF: a position servo
closes its own loop, so there is nothing on the hub to tune. Clamp commanded
angles 5° inside the mechanical stops.

## Troubleshooting

* **`pid: command not found`**, `pip install -e .` did not finish, or the
  scripts directory is not on `PATH`; use `python -m pidtool.cli fatman`.
* **`/api/run` takes minutes in the web UI**, that is the full optimizer, same as
  `pid fatman`. Use `pid fatman --no-tune` for a fast export-only pass.
* **Pedro constants look like older examples**, Piπd emits the current
  `ForesightConfig` API, not the old `FollowerConstants` one. See
  `docs/VERIFIED_APIS.md` for the exact source of every name.
* **MoSim** simulates **FRC** robots, so its export is the FRC-equivalent
  mechanism set (Kraken/NEO), not the servo-driven FTC turret. It also needs the
  Unity editor to build the `.bundle`; Piπd stops at hierarchy + params + C#.
