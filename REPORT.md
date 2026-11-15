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
spool fatman      # ~5.1 min on this laptop, CPU only -> out/fatman/
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

* drivetrain free speed **2.636 m/s = 103.8 in/s** (6000 rpm ÷ 11.4417 = 524.4 rpm
  at the wheel on a 96 mm wheel; the datasheet's measured 435 rpm × 24/20 = 522 rpm
  gives 103.3 in/s, 0.5 % lower), strafe 80.9 in/s.
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
and 320° ÷ 2.40 = **133°** of turret travel. That matches the CAD's 135° soft /
140° hard stops **if those are total travel** — and it is why the hand check in
§1b flags that `robot.json` currently encodes them as ±135° (a 270° span).

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

## 1b. Hand check against physics

Every headline number recomputed independently from the datasheet and the geometry,
with a calculator rather than through Spool's own code path. Inputs: goBILDA
5203-2402-0001 (12 V, 6000 rpm, 0.25 A free, 9.2 A stall, 1.47 kg·cm) and the REV
NEO (12 V, 5820 rpm, 2.065 A free, 160 A stall, 3.00 N·m), plus the ratios in
`robot.json`.

### Flywheel: 6000 rpm motor, 20T -> 20T GT2 belt (G = 1)

| Check | Arithmetic | Result |
|---|---|---|
| Free speed | 6000 x 2pi/60 | 628.32 rad/s (basis: 628.3185) |
| k_e from datasheet | (12 - 0.25 x 12/9.2) / 628.32 | 0.018580 V/(rad/s) = printed kV (G = 1) |
| kA from inertia | 1.181e-4 x (12/9.2) / (2 x 0.95 x 0.015674) | 0.005173 V/(rad/s^2) (printed 0.0051726) |
| kS from friction | (0.01805 + 2 x 0.95 x 0.015674 x 0.25) / 0.02283 | 1.117 V (printed 1.1167) |
| Target 4987 rpm | 1.1167 + 0.018580 x 522.22 | 10.82 V = 90 % of 12 V, 1.18 V headroom |
| Steady speed at 12 V | (12 - 1.1167) / 0.018580 | 585.8 rad/s = 5594 rpm (93 % of free speed; belt + bearing drag) |
| Energy per ball | 0.5 x 0.06 x 9.4^2 / 0.5 | 5.30 J |
| Speed after one ball | sqrt(522.22^2 - 2 x 5.30 / 1.181e-4) | 427.7 rad/s, **-18.1 %** (gains.json: 18.10 %) |
| Stored energy | 0.5 x 1.181e-4 x 522.22^2 | 16.1 J, so one shot takes 33 % of it — which is why 5.3 J costs 18 % of speed, not 33 % |

Consistent. 6000 rpm is the *motor's* free speed; the wheel cannot reach it with
the modelled drag, and 5594 rpm is the right steady state. The old `belt 158T`
note in `robot.json` was a belt *length* and could be misread as a ratio; it now
says 1:1 explicitly.

### Drivetrain: 6000 rpm motor, 13.73:1 planetary x 24T -> 20T belt

G = 13.73 x 20/24 = **11.4417 motor turns per wheel turn** — the belt *speeds the
wheel up* (24T on the motor, 20T on the wheel).

| Check | Arithmetic | Result |
|---|---|---|
| Wheel free speed | 6000 / 11.4417 | 524.4 rpm |
| Top speed | 524.4 x 2pi/60 x 0.048 | 2.636 m/s = **103.8 in/s** (printed 2.6359 / 103.78) |
| Datasheet cross-check | 435 x 24/20 = 522 rpm on the same wheel | 2.624 m/s = 103.3 in/s (+0.5 %: ingest uses the bare 6000 rpm motor behind the exact 13.73:1, not the measured 435 rpm) |
| Steady speed at 12 V | (12 - 0.4873) / 4.4288 | 2.600 m/s = 102.4 in/s |
| kA | 12.7 x 0.048 x 1.304 / (4 x 0.88 x 11.4417 x 0.015674) | 1.260 V/(m/s^2) (printed 1.2596) |
| Stall force | 4 x 0.88 x 11.4417 x 0.1442 / 0.048 | 121.0 N vs 124.5 N weight = **0.97 g** |
| Strafe | 4.4288/0.78, 1.2596/0.78 | 5.678, 1.615 (printed exactly) |

The 104 in/s top speed depends on the belt direction. If the 24T/20T were the
other way round the ratio would be 13.73 x 1.2 = 16.48, capping the robot at
6000/16.48 = 364 rpm = **72 in/s**, not 104 — still a fast FTC robot, but a very
different tune. Stall force is ~1 g, so the robot is traction-limited at launch;
the simulated 0.413 s 10-90 % rise to 2.11 m/s (~4-5 m/s^2 effective) is what
battery sag, kS and the 12 V ceiling leave, which is plausible rather than
optimistic.

### Arm kG (FRC-equivalent: 2x NEO, 25:1, 4.5 kg at r_com = 0.35 m)

| Check | Arithmetic | Result |
|---|---|---|
| k_t | 3.00 / 160 | 0.01875 N·m/A |
| Torque per volt | 2 x 0.85 x 25 x 0.01875 / (12/160) | 10.625 N·m/V |
| Gravity torque at horizontal | 4.5 x 9.80665 x 0.35 | 15.44 N·m |
| kG | 15.44 / 10.625 | **1.454 V** (printed 1.4537) |
| Fraction of stall torque | 1.454 / 12 | 12.1 % (8.2x torque margin) |

**Sign.** Positive kG is correct for a load that gravity pulls down. The plant
torque is `-m g r cos(theta)`; the simulator adds `+kG cos(theta)`. The independent
test is what happens without it: the "before" arm settles **below** the 20° target
by 0.01718 rad, and the missing gravity voltage divided by the tuned kP is
`1.4537 x cos(20°) / 80 = 0.01707 rad`. Those agree to 0.6 %, which confirms both
the magnitude and the sign of kG from a second direction.

`ArmFeedforward.calculate(position, ...)` takes radians measured from horizontal,
which is Spool's convention. If the robot defines 0 as vertical, subtract pi/2
before calling it.

### Servo geometry: one real conflict, one small one

| Axis | Servo travel / ratio | Axis travel | CAD soft-limit span | Verdict |
|---|---|---|---|---|
| turret | 320° / 2.40 | 133.3° | 270° (-135°..+135°) | **conflict** |
| hood | 320° / 3.947 | 81.1° | 85.9° (-28.6°..+57.3°) | conflict (4.9° short) |
| flower arm | 320° / 1.00 | 320° | 91.7° (-11.5°..+80.2°) | fine |

The turret is the important one. The `320/2.4 = 133°` cross-check only supports
135° if that figure is the **total** travel. As encoded in `robot.json` it is
+-135° (a 270° span), which would need 648° of servo rotation with the 25T->60T
(672° for the +-140° hard stops). So exactly one of these is true:

1. **135° is total travel:** the soft limits belong at +-67.5° (+-1.178 rad), not
   +-2.356 rad. The 0->90° slew scenario then runs past the physical stop and is
   an unconstrained slew test, not a move the robot can make.
2. **135° is half-travel (270° sweep):** the 25T->60T (2.4:1) cannot be the real
   reduction — it would need about 1.19:1, or a continuous-rotation servo with an
   external encoder.

Spool keeps the CAD encoding and flags the conflict instead of silently picking
one; §6 carries it as an open question for the team. The hood is the same class
of issue but only 4.9° short, which is inside the ±5° clamp the servo export
recommends.

Torque checks out independently: the flower arm needs 0.62 x 9.80665 x 0.14 =
0.851 N·m against 2 x 2.677 x 0.95 = 5.087 N·m (6.0x, as printed). The turret's
"required" column is 0 because a yaw axis has no gravity load; its inertia torque
at full slew (J·alpha ~ 0.8 N·m for 8.3e-3 kg·m² at ~95 rad/s²) is still only 7 %
of the 11.6 N·m available.

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
3. **Turret ratio 2.4 and the stop angles.** The BOM has *both* a printed 25T spur
   gear (in `turret > gear`) and a bought 20T pinion. 2.4 was chosen because the
   servo is H25T-spline and 320°÷2.4 = 133° travel; a 20T pinion would give 3.0
   and a slightly slower, stronger turret. But §1b shows 133° total cannot reach
   the ±135° stops as encoded: either the stops are ±67.5°, or the ratio is
   ~1.19:1 (continuous-rotation servo). Measure the actual travel between the
   hard stops; that single number settles both the ratio and the limits.
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
| `spool fatman` | success, 305 s, 22 files in `out/fatman/` |
| `spool web` endpoints | `/`, `/api/robot`, `/api/gains`, `/api/exports`, `/exports/…`, `/plots/…` all 200; 11 download files listed |
| `spool calibrate … --mechanism flywheel` | the documented CLI path recovers the synthetic log (row below) |
| `docs/VERIFIED_APIS.md` re-check | Pedro Constants page re-fetched; WPILib `PIDController` and MoSim `JointAxis` / `withAxis` / `SetAngularVelocity` added with sources |
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

**Found, but deliberately not "fixed":** the hand check in §1b shows that the
turret's ±135° soft stops cannot be reached by 320°/2.4 = 133° of servo travel,
and the hood is 4.9° short. Both readings are written down; one measurement of the
real travel between hard stops decides which is right. Inventing a new ratio or
new limits would have made the model look self-consistent while hiding the
question.
