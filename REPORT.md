# Spool — FAT MAN

CAD in, tuned robot out: physics-based starting gains for every mechanism of
**FAT MAN** (FTC, Star Strikers), exported to Pedro Pathing, FTC Dashboard/Panels,
WPILib and MoSim, with one short logged test run left to finish the job.

Every number below is either **computed** from a formula (and re-derivable by
re-running `spool fatman`), **estimated** from geometry with the assumption
stated, or **unverified** and marked as such. Nothing is an LLM guess: there is no
model in the product loop.

```
pip install -e .
pytest            # 46 passed
spool fatman      # ~4.9 min on this laptop, CPU only -> out/fatman/
spool web         # local UI at 127.0.0.1:8000
```

---

## 1. What came out

### Robot

| | Value | Source |
|---|---|---|
| Mass | **12.70 kg** | estimated: Onshape reports 11.55 kg with 39 bodies missing density (+10%) |
| Envelope | 0.457 × 0.543 × 0.339 m | cad (`src/asm_bbox.json`) |
| Controllers | Control Hub + Expansion Hub | bom |

The 48 part bounding boxes in `build/parts_final.json` total 1.19 L, which is
~1.5–3 kg of structure. The rest of the CAD mass is hardware, motors, battery and
electronics that are not in that export — so the Onshape mass properties, not the
bbox reconstruction, are the anchor. This is the number every gain is proportional
to, and it is the weakest one in the model.

### Gains

| Mechanism | kS (V) | kV | kA | kG (V) | Units |
|---|---|---|---|---|---|
| drivetrain forward | 0.4873 | 4.4288 | 1.2596 | — | V per m/s, V per m/s² |
| drivetrain strafe | 0.4873 | 5.6780 | 1.6149 | — | V per m/s, V per m/s² |
| flywheel (2 motors) | 1.1167 | 0.018580 | 0.005173 | — | V per rad/s, V per rad/s² |
| FRC arm | 0.1563 | 0.4859 | 0.03447 | **1.4537** | V per rad/s, V per rad/s² |

Derived speeds:

* drivetrain free speed **2.636 m/s = 103.8 in/s** (435 rpm × 24/20 = 522 rpm on a
  96 mm wheel), strafe 80.9 in/s.
* flywheel 6000 rpm no-load; the shot target is **4987 rpm** (522 rad/s).

### Before / after (simulated step responses, 1 kHz)

| Scenario | Rise before | Rise after | Overshoot before % | Overshoot after % | Settle before | Settle after |
|---|---|---|---|---|---|---|
| flywheel spin-up | 0.544 | 0.425 | -6.0 | 0.0 | 2.000 | 0.575 |
| turret slew | 0.264 | 0.264 | -0.4 | -0.4 | 0.327 | 0.327 |
| hood slew | 0.216 | 0.216 | -0.9 | -0.9 | 0.266 | 0.266 |
| arm hold (20°) | 0.069 | 0.070 | -4.9 | 0.1 | 0.141 | 0.109 |
| drive forward | 0.706 | 0.413 | -10.3 | -0.0 | 2.000 | 0.545 |
| drive strafe | 0.718 | 0.341 | -8.9 | 1.0 | 2.000 | 0.420 |

Times in seconds; settle bands 2% (speed) and 1° (angle). Plots in
`out/fatman/plots/`.

Negative overshoot means the axis came to rest **below** target — a steady-state
error, not a dip. "before" is a PID with no feedforward (for the arm: the same PID
with the kG term removed), "after" is Spool's feedforward plus a tuned PID and a
trapezoidal reference.

**The turret and hood rows are identical on purpose.** They are position servos:
the servo closes its own loop, so there is no PIDF for Spool to tune, and the move
is purely rate-limited (90° ÷ 272°/s = 0.33 s, which is what the table shows).
Profiling the command can only add time, so the optimizer correctly chooses no
profile. That is the honest result, and it is why the servo export is a table of
ratio / speed / torque / limits rather than a PIDF block.

### Ball recovery

A shot steals `dE = ½·m·v²/η` = **5.30 J**, dropping the wheel **18.1 %**
(522 → 429 rad/s), and it is back inside 2 % in **0.214 s** (requirement: 0.5 s).

### Servo axes — no PIDF, and why

| Axis | Servo × n | Ratio | Output speed | Torque out | Required | Margin |
|---|---|---|---|---|---|---|
| turret | SWYFT Balance × 2 | 2.40 | 272 °/s | 11.57 N·m | — | — |
| hood | SWYFT Balance × 1 | 3.95 | 165 °/s | 9.51 N·m | 0.02 N·m | 476× |
| flower arm | SWYFT Balance × 2 | 1.00 | 652 °/s | 5.09 N·m | 0.851 N·m | 6.0× |

A cross-check that fell out of the geometry: the servo has 320° of usable travel,
and 320° ÷ 2.40 = **133°** of turret travel, which matches the 135° soft / 140°
hard stops in the CAD. That independently supports the 25T→60T ratio.

### Synthetic calibration (proves `fit.py`)

`sim.py` generates a log from a *known* plant; `fit.py` recovers the gains:

| | Truth | Fitted | Error |
|---|---|---|---|
| kS | 1.116656 V | 1.116878 V | **+0.02 %** |
| kV | 0.0185796 | 0.0185809 | **+0.007 %** |
| kA | 0.0051726 | 0.0051296 | **−0.83 %** |

R² 0.996, equivalent voltage error 0.00023 V. Artifacts:
`out/fatman/synthetic_log.csv`, `out/fatman/synthetic_fit.json`.

---

## 2. Every formula

Motor (datasheet identities):

```
V = I·R + k_e·ω          τ = k_t·I
R   = V_nom / I_stall
k_t = τ_stall / I_stall
k_e = (V_nom − I_free·R) / ω_free
τ_out = η·G·τ_motor      ω_motor = G·ω_out        (G = motor turns per output turn)
```

`k_t` and `k_e` disagree by 17.3 % for the goBILDA RS-555 because the datasheet's
no-load speed includes friction. Spool keeps both — `k_e` sets volts-per-speed,
`k_t` sets volts-per-torque — and reports the mismatch instead of hiding it.

Feedforward:

```
kV = G · k_e                       [V per output rad/s]
   = G · k_e / r                   [V per m/s]
kA = J_eff · R / (n·η·G·k_t)       [V per output rad/s²]
   = m · r · R / (n·η·G·k_t)       [V per m/s²]
kG = m·g·r_com · R / (n·η·G·k_t)   [V at horizontal]
kS = τ_friction · R / (n·η·G·k_t)  [V]  ← model, flagged
J_eff = J_load + n·G²·J_rotor
```

Why `R/(η·G·k_t)`: acceleration torque is `J·α`; the current that makes it is
`τ/(η·G·k_t)`; current needs `I·R` of voltage headroom.

`kS` includes the motor's own free-run friction, `k_t·I_free`, which alone is
`I_free·R` = 0.326 V for this motor — matching the unstick voltage teams measure on
a bare goBILDA.

Friction model (starting kS only):

```
τ_bearing = μ_b · preload · n_bearings · r_bearing      μ_b=0.0015, preload=25 N, r=7 mm
τ_belt    = μ_belt · preload · r_pulley · n_belts        μ_belt=0.02, preload=40 N
τ_friction = τ_bearing·G + τ_belt + τ_seal (2 mN·m)
```

Mechanism-specific:

```
flywheel   J = Σ ½·m·r²   (Rhino 72 mm wheel + 2 stainless discs from 3/4" rod)
drivetrain kA uses m (whole robot) and r (wheel), n = 4 motors
arm        τ_gravity(θ) = m·g·r_com·cos θ ; kG is the value at θ = 0
servo      ω_out = (π/3)/t_60° / ratio ;  τ_out = n·τ_servo·ratio·η_gear
ball       dE = ½·m_ball·v_exit²/η_transfer ;  ω' = √(ω² − 2·dE/J)
           applied as an angular-momentum impulse J·(ω−ω') over 5 ms of contact
```

Plant integrated at 1 kHz (`dt = 1 ms`, explicit Euler):

```
I  = (V_app − k_e·G·ω)/R     (current-limited)
V_t = V_app − I·n·R_batt     (battery sag)
J·α = n·η·G·k_t·I − τ_c·tanh(ω/0.05) − τ_g(θ) − τ_ext
```

Simulator validation: at 12 V with zero extra friction the model lands on the
datasheet free speed to within 1 % (unit-tested).

---

## 3. Computed vs estimated vs unverified

**Computed** (closed form from CAD + datasheet, reproducible):

* kS, kV, kA, kG for the flywheel and the FRC arm.
* kV, kA for the drivetrain (forward and strafe).
* All servo ratios, output speeds, output torques and travel ranges.
* Free speeds, stall torques, ball-recovery energy and speed drop.
* Tuned kP/kI/kD for the flywheel and drivetrain; kP/kD for the FRC arm.

**Estimated** (stated assumption, tagged `estimated` in `robot.json`):

| Quantity | Value | Why |
|---|---|---|
| robot mass | 12.70 kg | 39 CAD bodies have no density; +10 % |
| flywheel inertia | 1.04e-4 kg·m² (+1.4e-5 reflected) | component masses estimated from geometry |
| turret inertia | 8.3e-3 kg·m² | 1.15 kg at 85 mm radius of gyration |
| arm mass / r_com | 0.62 kg / 0.14 m | printed tray + letters + 6 mm links |
| ball mass | 0.06 kg | not in the CAD or BOM |
| shot exit speed | 9.4 m/s | from the game requirement, not the CAD |
| transfer efficiency | 0.5 | ball leaves at ~half rim speed (typical single-wheel launcher) |
| rolling resistance μ | 0.08 | mecanum rollers scrubbing on foam; Pedro's Forward Deceleration identification replaces it |
| strafe efficiency | 0.78 | roller slip; Pedro's Strafe Velocity identification replaces it |
| rotor inertia | 7e-6 (RS-555), 1.3e-5–1.9e-5 (FRC) | vendors do not publish it; from rotor geometry |
| battery R | 0.03 Ω FTC, 0.018 Ω FRC | pack + wiring |
| current limit | none FTC, 60 A/motor FRC | controller configuration |

**Unverified / explicitly not modelled:**

* `kS` for every axis — it is a friction model, *by definition* not derivable from
  CAD. Every export says so; `SpoolCalibrate` measures it.
* MoSim's ability to represent an FTC robot — it is an FRC simulator, so Spool
  exports the FRC-equivalent set (see §5).
* The exact unit of Pedro's `maxAchievable*Velocity` (inches/second by magnitude;
  the docs do not spell it out).
* Pedro's `coast` / `brake` / brake-coefficient semantics — those values are
  copied from the docs example and marked placeholder.
* The servo's internal loop: modelled from outside as rate-limit + deadband.

---

## 4. Assumptions that would change the answer

1. **Mass.** Every kA and kG scales with it. ±10 % mass → ±10 % kA/kG.
2. **Gear ratio.** The drivetrain is `13.73 × 20/24 = 11.44`. Getting the belt
   direction backwards (24/20 instead of 20/24) would cap the robot at 43 in/s
   instead of 104 — the single most consequential number in the file.
3. **Turret ratio 2.4.** The BOM has *both* a printed 25T spur gear (in
   `turret > gear`) and a bought 20T pinion. 2.4 was chosen because the servo is
   H25T-spline and the 320°÷2.4 = 133° travel matches the CAD stops; a 20T pinion
   would give 3.0 and a slightly slower, stronger turret.
4. **Flywheel inertia.** The gain table is insensitive to it (kA is tiny); the
   ball recovery is not (±50 % J → roughly ±50 % speed drop).
5. **Friction model constants** only set the *starting* kS. They do not affect kV,
   kA or kG.

---

## 5. Exports, and what each one really is

| File | Status |
|---|---|
| `exports/pedro/Constants.java` | current Pedro API (`ForesightConfig`, `MecanumConfig`, `PinpointConfig`), verified field-by-field against the live docs. Four quantities filled from CAD; brake/coast fields are documented-example placeholders |
| `exports/pedro/PEDRO_TUNING_CHECKLIST.md` | the exact AutoTune procedures, in order, with Spool's starting value next to each |
| `exports/pedro/pedro_values.json` | the same numbers as data, with the assumptions |
| `exports/ftc_dashboard/SpoolGains.java` | `@Config` class, live-editable kS/kV/kA/kG/kP/kI/kD |
| `exports/wpilib/Constants.java` | FRC-equivalent mechanisms with `SimpleMotorFeedforward` / `ArmFeedforward` |
| `exports/wpilib/sysid_fields.json` | the log entry map a SysId-readable `.wpilog` must satisfy |
| `exports/mosim/` | hierarchy + params + C# mod skeleton + README |
| `runtime/ftc/SpoolCalibrate.java` | compiles (against SDK stubs, see §7) |
| `runtime/frc/SpoolCalibrate.java` | written, **not compiled** (no WPILib here) |

Pedro numbers actually emitted: forward 100.66 in/s, strafe 78.52 in/s, forward
deceleration 30.89 in/s², strafe 35.52 in/s², heading kP 4.52, translational
kP 0.128 V/in.

MoSim: mods *do* carry C# code (the `.dll` is "the robot's code" — verified), and
the mod folder is `.bundle` + `.dll` + `catalog.json`. But MoSim simulates **FRC**
robots, so the FAT MAN export is the FRC-equivalent mechanism set (Kraken drive,
Kraken flywheel, NEO turret, 2×NEO arm), not the two-servo FTC robot. No `.bundle`
is produced: that needs the Unity editor and the team's meshes.

---

## 6. Open questions for a real test run

1. **Measure kS** on every axis with `SpoolCalibrate`. It is the one gain that
   cannot come from CAD. Expect the flywheel to be the furthest off (the belt
   preload model is a guess).
2. **Weigh the robot.** 12.7 kg is an estimate over a CAD mass with 39 massless
   bodies.
3. **Turret ratio**: confirm the 25T horn gear vs the 20T pinion, and measure the
   actual travel between hard stops (Spool predicts 133° usable).
4. **Ball mass and exit speed**: the shot target (4987 rpm → 9.4 m/s) rests on
   0.06 kg and η_transfer = 0.5. One chronograph session settles both.
5. **Mecanum slip**: run Pedro's Forward and Strafe Velocity Identification; the
   0.78 strafe factor and 0.08 rolling friction are the two numbers most likely to
   move.
6. **Pedro units**: confirm `maxAchievableForwardVelocity` is in/s on first run.
7. **Flywheel inertia**: a spin-up log gives kA directly, which is J_eff.

---

## 7. Verification performed (and what it does not cover)

| Check | Result |
|---|---|
| `pip install -e .` | success |
| `pytest` | **46 passed** |
| `spool fatman` | success, 287 s, 24 artifacts |
| Motor model identities | unit-tested exactly (R, k_t, k_e, round-trips) |
| Feedforward formulas | hand-checked case with literals in the test |
| Simulator vs datasheet | free speed within 1 % |
| `fit.py` on synthetic log | kS +0.02 %, kV +0.007 %, kA −0.83 % |
| `SpoolCalibrate.java` | compiles with `javac` against hand-written SDK stubs |
| Pedro/WPILib Java | **not compiled** — no Pedro or WPILib vendordep on this laptop |
| MoSim C# | **not compiled** — no Unity assemblies |

`tools/compile_check.sh` reproduces the Java check. The stubs are marked as stubs:
they prove syntax and member arity, not that the real SDK matches them.

Bugs the verification caught (fixed, and now covered by tests):

* the PID's unfiltered derivative was numerically unstable at 1 ms (commands
  alternated ±12 V) — fixed with a standard N-factor derivative filter;
* battery sag was applied without a controller current limit, which collapsed an
  FRC arm to 1.7 V — fixed with a 60 A/motor limit;
* the synthetic plant took the friction sign from the *command*, so a 0 V block
  spun the wheel up and the identified kS came out 30–45 % high — fixed with
  proper static/Coulomb friction;
* `fit.py` could start integrating inside the stall region if one noisy sample
  cleared the speed gate — fixed with a sustained-motion window.

That last pair is why the fit table in §1 is trustworthy: it was wrong in exactly
the way the brief warned about, and the regression test named the failure.
