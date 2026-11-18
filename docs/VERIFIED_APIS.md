# Verified APIs

Every third-party class, field and method Piπd emits, where it was read from, and
what could **not** be verified. Rule: if it is not in this file, Piπd does not
emit it.

All URLs fetched 2026-11-15.

---

## 1. Pedro Pathing (FTC)

### Pages read

| Page | URL | What it establishes |
|---|---|---|
| Constants | https://pedropathing.com/docs/pathing/tuning/constants | the whole `Constants.java` shape, `ForesightConfig`, `MecanumConfig`, `PinpointConfig`, `Follower` constructor, `Controller.*` calls |
| Foresight tuning | https://pedropathing.com/docs/pathing/tuning/foresight | `ForesightTuner`, `@Tuner`, the AutoTune URL, every procedure name |
| Mecanum drivetrain | https://pedropathing.com/docs/pathing/tuning/drivetrain/mecanum | `MecanumTuner`, AutoTune at `192.168.43.1:10158` |

### Classes and members used

| Symbol | Where it appears | Emitted by |
|---|---|---|
| `com.pedropathing.follower.Follower` | `new Follower(localizer, drivetrain, algorithm)` | `export/pedro.py` |
| `com.pedropathing.algorithm.Foresight` | `new Foresight(foresightConfig)` | `export/pedro.py` |
| `com.pedropathing.algorithm.ForesightConfig` | lambda config, `c -> {...}` | `export/pedro.py` |
| `ForesightConfig.forwardTranslational` / `.strafeTranslational` | `set(Controller.piecewise(...).put(2.5, ...))` | `export/pedro.py` |
| `ForesightConfig.coast` / `.brake` | `set(Controller.proportionalFeedforward(x))` | `export/pedro.py` (placeholder values) |
| `ForesightConfig.headingFeedback` | `set(Controller.proportional(x))` | `export/pedro.py` |
| `ForesightConfig.headingBrakeCoefficients` | `set(Vector2D.cartesian(x, y))` | `export/pedro.py` (placeholder) |
| `ForesightConfig.linearBrakeCoefficients` / `.quadraticBrakeCoefficients` | `set(Matrix.diag(a, b))` | `export/pedro.py` (placeholder) |
| `ForesightConfig.maxAchievableForwardVelocity` | `set(in/s)` | `export/pedro.py` (from CAD) |
| `ForesightConfig.maxAchievableStrafeVelocity` | `set(in/s)` | `export/pedro.py` (from CAD) |
| `ForesightConfig.naturalForwardDeceleration` | `set(in/s^2)` | `export/pedro.py` (from friction model) |
| `ForesightConfig.naturalStrafeDeceleration` | `set(in/s^2)` | `export/pedro.py` (from friction model) |
| `com.pedropathing.controllers.Controller` | `.proportional`, `.piecewise(...).put`, `.proportionalFeedforward` | `export/pedro.py` |
| `com.pedropathing.math.Matrix` | `Matrix.diag(a, b)` | `export/pedro.py` |
| `com.pedropathing.math.Vector2D` | `Vector2D.cartesian(x, y)` | `export/pedro.py` |
| `com.pedropathing.revhub.drivetrains.Mecanum` | `new Mecanum(h, drivetrainConfig)` | `export/pedro.py` |
| `com.pedropathing.revhub.drivetrains.MecanumConfig` | `c.frontLeftName.set("lf")`, `backLeftName`, `frontRightName`, `backRightName`, `frontLeftDirection`, `backLeftDirection`, `frontRightDirection`, `backRightDirection`, `manualBrakeMode` | `export/pedro.py` |
| `com.pedropathing.revhub.localizers.PinpointLocalizer` | `new PinpointLocalizer(h, localizerConfig)` | `export/pedro.py` |
| `com.pedropathing.revhub.localizers.PinpointConfig` | `c.name`, `c.xPodOffset`, `c.yPodOffset`, `c.xPodDirection`, `c.yPodDirection` | `export/pedro.py` |
| `com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection` | `EncoderDirection.FORWARD` | `export/pedro.py` |
| `ForesightTuner`, `@Tuner`, `Procedure` | tuner OpMode skeleton | `PEDRO_TUNING_CHECKLIST.md` |
| `MecanumTuner` | drivetrain AutoTuner | `PEDRO_TUNING_CHECKLIST.md` |
| AutoTune URL `http://192.168.43.1:10158` | browser address | checklist, report |

### Procedure names used in the checklist (all from the Foresight page)

Forward Velocity Identification, Strafe Velocity Identification, Forward
Deceleration Identification, Strafe Deceleration Identification, Heading Braking
Identification, Heading Tuner, Forward Braking Identification, Strafe Braking
Identification, Forward Translational Tuner, Strafe Translational Tuner, Tests.

### Two corrections to the brief's earlier notes

1. **`Foresight`, `ForesightConfig` and the port-10158 AutoTune page are real.**
   The brief flagged them as possibly invented. They are on the live docs pages
   listed above, including the exact `ForesightConfig` field names.
2. **`FollowerConstants` / `MecanumConstants` / `PIDFCoefficients(p,i,d,f)` are the
   previous API.** `https://pedro-pathing.github.io/Docs/` now redirects to
   `https://pedropathing.com/`, and the pages that showed that API
   (`/docs/pathing/tuning/setup`, `/docs/pathing/examples/constants`) return
   **404**. Piπd therefore does not emit them. If a team is pinned to an older
   Pedro release, the numbers transfer (max velocity, natural deceleration,
   d-pad POD offsets) but the class names must be moved by hand.

### Units

`maxAchievable*Velocity` are **inches/second** and `natural*Deceleration` are
**inches/second^2**. The docs show `72.72923108818539` and `85.01144677379789`
for a working FTC robot, which is consistent with in/s; the pages do not state the
unit in words. This is called out in the generated file so nobody discovers it on
the field.

---

## 2. MoSimulator (FRC)

### Pages read

| Page | URL | What it establishes |
|---|---|---|
| Modding hub | https://mosimulator.com/modding/ | mod = `.bundle` + `.dll` + `catalog.json`; install folder; "Mods include DLL code"; community mods are FRC robots |
| Docs intro | https://docs.mosimulator.com/modding/intro | robot mods only; fields, game pieces, rules, physics, UI, audio are **not** moddable |
| Code walkthrough | https://docs.mosimulator.com/modding/lynk-walkthrough/code | `GenericJoint`, `GenericElevator`, `PidConstants`, `SetPid`, `UpdatePid`, `SetTargetAngle`, `SetTarget(...).withAxis(...)`, `JointAxis.X`, base class `ReefscapeRobotBase`, `CurrentSetpoint` |
| Mod building | https://raw.githubusercontent.com/MoSimulator/MoSim-Docs/main/Site/modding/lynk-walkthrough/mod-building.md | Addressables group + `modpack_metadata` label, `catalog.hash` / `settings.json` from `Library/com.unity.addressables/aa/Windows`, `.dll` from `Library/ScriptAssemblies` |
| `GenericJoint.cs` source | https://github.com/Tuff-Modding/MoSim-Modding/blob/main/Assets/Scripts/RobotFramework/Components/GenericJoint.cs | the `JointAxis` enum (`X`, `Y`, `Z`), `PIDAngle.withAxis`, `PIDLinear.withAxis`, `Velocity.WithAxis`, `SetAngularVelocity` |

### Verified facts

* Each mod is a folder: `<Mod>.bundle`, `<Mod>.dll`, `catalog.json` (plus
  `catalog.hash` and `settings.json` copied from the Addressables build).
* Install path: `%userprofile%\AppData\LocalLow\CascadeStudios\MoSimulator\Mods`.
* **Mods can carry custom C# code**, the `.dll` is described as "the robot's
  code". The brief's "UNVERIFIED" question is answered: yes.
### Symbols used, and where each was read

| Symbol | Where it appears | Source |
|---|---|---|
| `ReefscapeRobotBase`, `CurrentSetpoint` | blank robot template, `switch (CurrentSetpoint)` | docs "Code" page |
| `ReefscapeSetpoints.Stow` / `.Place` / `.L4` | the same switch | docs "Code" page |
| `GenericJoint`, `GenericElevator`, `PidConstants` | serialized fields and PID setup | docs "Code" page |
| `SetPid(pid)` / `UpdatePid(pid)` | `Start()` / `LateUpdate()` | docs "Code" page |
| `SetTargetAngle(rad).withAxis(JointAxis.X)` | `UpdateSetpoints()` | docs "Code" page; `PIDAngle.withAxis` in `GenericJoint.cs` |
| `SetAngularVelocity(deg/s).WithAxis(axis)` | velocity command | `GenericJoint.cs` (`Velocity.WithAxis`, `SetAngularVelocity`) |
| `JointAxis.X` / `.Y` / `.Z` | `switch (linearAxis)` maps X/Y/Z to right/up/forward | `GenericJoint.cs` |

All of these are emitted by `export/mosim.py`. `ReefscapeRobotBase` and
`ReefscapeSetpoints` are **game-specific** (Reefscape); another game renames them,
which the generated file says in its header.

### Consequence for FAT MAN

MoSim simulates **FRC** robots; FAT MAN is FTC. Piπd therefore exports the
FRC-equivalent mechanism set (Kraken/NEO) and says so in
`out/fatman/exports/mosim/README.md`. The two-servo FTC turret is not simulated.

### Not done

No `.bundle` is produced (needs the Unity editor and the team's meshes), and the
C# file is not compiled (no Unity assemblies on this laptop). The hierarchy,
masses, inertias and gains are real; the bundle is a Unity-side step.

`SetTarget(value)` semantics are joint-configuration dependent (the walkthrough
uses it for an elevator height in inches), so the flywheel export uses the
verified `SetAngularVelocity` in degrees/second instead of guessing what a
`SetTarget` on a rotational joint means.

---

## 3. WPILib (FRC)

| Symbol | Signature / behaviour | Source |
|---|---|---|
| `edu.wpi.first.math.controller.SimpleMotorFeedforward` | `new SimpleMotorFeedforward(kS, kV, kA)`, `.calculate(velocity, acceleration)` | https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html |
| `edu.wpi.first.math.controller.ArmFeedforward` | `new ArmFeedforward(kS, kG, kV, kA)`, `.calculate(position, velocity, acceleration)`, **position in radians** | same page |
| `edu.wpi.first.math.controller.PIDController` | `new PIDController(kP, kI, kD)`, `.calculate(measurement, setpoint)` | https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html |
| `edu.wpi.first.math.controller.ElevatorFeedforward` | `new ElevatorFeedforward(kS, kG, kV, kA)`, `.calculate(velocity, acceleration)` | same page |
| `setVoltage()` | recommended way to apply feedforward (compensates sag) | same page |
| Gain units | kS volts, kV volts·s/unit, kA volts·s²/unit; C++ throws on unit mismatch | same page |
| SysId input | SysId reads a **`.wpilog`**, not JSON: drag `sysid-test-state-<mechanism>` into Test State, then Position/Velocity/Voltage | https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/loading-data.html |

### SysId "compatible JSON"

The brief asks for SysId-compatible JSON. SysId's documented input is a WPILog
file; no public JSON *dataset* schema is documented, so Piπd does not invent one.
Instead it emits `sysid_fields.json`, the field map that `PidCalibrate`'s
wpilog satisfies, plus `SYSID.md` with the click-by-click. Fabricating a JSON
schema would have been exactly the kind of guessed API the brief forbids.

---

## 4. FTC SDK (for `runtime/ftc/PidCalibrate.java`)

Symbols used: `LinearOpMode`, `@TeleOp`, `DcMotorEx`, `DcMotor.RunMode`
(`STOP_AND_RESET_ENCODER`, `RUN_WITHOUT_ENCODER`),
`DcMotor.ZeroPowerBehavior.FLOAT`, `hardwareMap.get(Class, name)`,
`hardwareMap.voltageSensor`, `motor.getVelocity()`,
`motor.getCurrentPosition()`, `motor.setPower()`,
`motor.getCurrent(CurrentUnit.AMPS)`, `ElapsedTime`, `sleep`,
`opModeIsActive`, `telemetry`.

**Verification method:** the FTC SDK is not installed on this laptop, so the file
was compiled with `javac` against small hand-written stubs of exactly those
`com.qualcomm.robotcore.*` / `org.firstinspires.ftc.robotcore.*` types
(`tools/ftc-sdk-stubs/`). That proves the file is syntactically valid and uses
those members with the right arity and types; it does **not** prove the stubs
match the real SDK field-for-field. The stub files say so in their headers.

`getCurrent(CurrentUnit.AMPS)` is wrapped in a `try/catch (RuntimeException)`:
the goBILDA 5203 does not report current, and the fit does not need the column.

The Pedro exporter also uses `com.qualcomm.robotcore.hardware.DcMotorSimple`
(`Direction.FORWARD` / `REVERSE`) and `HardwareMap`; both appear in the Pedro
Constants example linked in §1.

---

## 5. FTC Dashboard / Panels

| Symbol | Source |
|---|---|
| `com.acmerobotics.dashboard.config.Config` (annotation on a class; public static **non-final** fields become live-editable) | https://acmerobotics.github.io/ftc-dashboard/javadoc/com/acmerobotics/dashboard/config/Config.html and https://acmerobotics.github.io/ftc-dashboard/features |

Panels reads the same configuration protocol, so one `@Config` class serves both.

---

## 6. Hardware datasheets

| Part | Numbers used | Source |
|---|---|---|
| goBILDA 5203-2402-0001 (1:1, 6000 rpm) | 12 V, 6000 rpm, 0.25 A free, 9.2 A stall, 1.47 kg·cm stall, 28 PPR, 391 g | https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/ |
| goBILDA 5203-2402-0014 (13.7:1, 435 rpm) | 12 V, 435 rpm, 0.25 A free, 9.2 A stall, 18.7 kg·cm stall, 384.5 PPR, 438 g | https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/ |
| goBILDA 5203 1620 / 1150 / 312 / 223 rpm | free speed, stall torque, stall current | https://github.com/crossbotics/ftc-helper-lib/blob/main/docs/guides/parts-catalog.md (seller's own pages) |
| REV Core Hex (REV-41-1300) | 125 rpm, 3.2 N·m, 4.4 A stall; 0.2 A free from gm0 | https://www.revrobotics.com/rev-41-1300 |
| REV HD Hex 40:1 (REV-41-1301) | 150 rpm, 4.2 N·m | https://www.revrobotics.com/rev-41-1301 |
| SWYFT Balance Servo (SR-Servo-02) | 6 V, 0.092 s/60°, 27.3 kg·cm, 2.7 A stall, 7.53 W | https://ftc-resources.firstinspires.org/ftc/event/inspection-reference (FTC Inspection Quick Reference) |
| REV Smart Servo (REV-41-1097) | 6 V, 0.14 s/60°, 13.5 kg·cm | same FIRST table |
| Kraken X60 / Falcon 500 / NEO / NEO Vortex / CIM | free speed, free current, stall torque, stall current | https://www.reca.lc/motors and https://www.revrobotics.com/content/docs/FRC-Brushless-Motor-Power-Comparison.pdf |

Rotor inertia is **estimated from rotor geometry** for every motor (the vendors do
not publish it); each database entry says so and marks it in
`rotor_inertia_source`.

---

## 7. Not verified, and therefore not used

| Thing | Status |
|---|---|
| `FollowerConstants`, `MecanumConstants`, `PIDFCoefficients` | pages 404 on the live docs; not emitted |
| Pedro AutoTune port 10158 contents | the page is documented, but the generated config's *semantics* for `coast`/`brake` are not; those values are copied from the docs example and marked placeholder |
| MoSim `.bundle` format | not attempted; Unity-only |
| SysId JSON dataset format | not documented publicly; not emitted |
| goBILDA 96 mm mecanum roller geometry | not needed; strafe efficiency is a measured/estimated scalar |
| MoSim `SetTarget` semantics | configuration-dependent; the flywheel uses verified `SetAngularVelocity` instead |
| MoSim `.bundle` contents / build | Unity-only; not attempted |

---

## 8. SP4 additions: no new third-party symbols

The static demo page (`docs/demo/index.html`), the wider `pid calibrate` CSV
reader and the `--turret-travel-deg` flag add no Pedro Pathing, MoSim, WPILib, FTC
SDK or Dashboard calls. The demo is plain HTML/CSS/JS built from `out/fatman`; the
CSV reader uses Python's standard `csv` module. Nothing in section 7 changed
status.
