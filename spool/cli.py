"""`spool` command line.

    spool fatman          full pipeline: CAD -> robot.json -> gains -> plots -> exports
    spool tune <file>     retune an existing robot.json
    spool calibrate ...   fit kS/kV/kA from a SpoolCalibrate CSV
    spool web             local web UI
    spool urdf / step     generic ingest paths
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from . import __version__
from .physics.mechanisms import build_robot
from .physics.sim import ServoAxis
from .physics.tune import ball_recovery, hold_arm, tune_drive, tune_rotary, tune_servo
from .motors import get_servo, motors as load_motors
from .ingest import fatman_local
from . import report

DEFAULT_ROOT = "C:/Users/sehaj/ftc-cad/robots/fatman"
DEFAULT_OUT = "out/fatman"

IN_PER_M = 39.3701


def _say(msg: str) -> None:
    print(f"[spooling...] {msg}", flush=True)


def _write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def _servo_axis(mech: dict) -> ServoAxis:
    servo = get_servo(mech["servo"])
    ratio = float(mech.get("gearing", {}).get("ratio", 1.0))
    limits = mech.get("limits") or {}
    lim = None
    if limits.get("soft_min_rad") is not None and limits.get("soft_max_rad") is not None:
        lim = (float(limits["soft_min_rad"]), float(limits["soft_max_rad"]))
    return ServoAxis(
        name=mech["name"],
        servo=servo,
        ratio=ratio,
        servo_count=int(mech.get("servo_count", 1)),
        inertia_kgm2=float(mech.get("inertia_kgm2", 1e-4)),
        holding_torque_nm=float(mech.get("holding_torque_nm", 0.0)),
        gear_efficiency=float(mech.get("gearing", {}).get("efficiency", 0.9)),
        limits_rad=lim,
    )


def run_fatman(root: str = DEFAULT_ROOT, out: str = DEFAULT_OUT, seed: int = 0,
               do_tune: bool = True) -> dict:
    t0 = time.time()
    outdir = Path(out)
    (outdir / "plots").mkdir(parents=True, exist_ok=True)
    (outdir / "exports").mkdir(parents=True, exist_ok=True)

    _say(f"reading CAD + BOM from {root}")
    robot_doc = fatman_local.build_robot(root)

    _say("building mechanism models")
    motors_db = load_motors()
    model = build_robot(robot_doc, motors_db)
    frc_doc = {"name": robot_doc["name"] + " (FRC equivalent)",
               "mass_kg": 54.0, "mechanisms": robot_doc["frc_equivalent"]}
    frc_model = build_robot(frc_doc, motors_db)

    drive = model.drive
    flywheel = model.rotary["flywheel"]
    target_rpm = float(flywheel.params["target_speed_rad_s"]) * 60 / (2 * math.pi)

    _say("spooling the drivetrain (forward + strafe)")
    scn_fwd = tune_drive(drive, target_speed=drive.free_speed_forward * 0.8, axis="forward", seed=seed) \
        if do_tune else None
    scn_str = tune_drive(drive, target_speed=drive.free_speed_strafe * 0.8, axis="strafe", seed=seed) \
        if do_tune else None

    _say("spooling the flywheel (spin-up + ball recovery)")
    scn_fly = tune_rotary(
        flywheel,
        target=float(flywheel.params["target_speed_rad_s"]),
        mode="velocity",
        max_overshoot=3.0,
        max_settle=0.6,
        tolerance=float(flywheel.params["target_speed_rad_s"]) * 0.02,
        t_end=2.0,
        seed=seed,
    )
    shot = ball_recovery(
        flywheel,
        exit_speed=float(flywheel.params.get("target_exit_speed_m_s", 9.4)),
        ball_mass=float(flywheel.params.get("ball_mass_kg", 0.06)),
        transfer_efficiency=float(flywheel.params.get("transfer_efficiency", 0.5)),
        kp=scn_fly.tuned["kp"],
        kd=scn_fly.tuned["kd"],
    )

    _say("spooling the turret (servo slew, hard stops are constraints)")
    turret_mech = next(m for m in robot_doc["mechanisms"] if m["name"] == "turret")
    turret = _servo_axis(turret_mech)
    scn_turret = tune_servo(turret, target=math.radians(90.0), tolerance=math.radians(1.0))

    _say("spooling the hood (servo)")
    hood_mech = next(m for m in robot_doc["mechanisms"] if m["name"] == "hood")
    hood = _servo_axis(hood_mech)
    scn_hood = tune_servo(hood, target=math.radians(45.0), tolerance=math.radians(1.0))

    _say("spooling the arm (FRC-equivalent motorized 4-bar holds with kG)")
    arm = frc_model.rotary["flower_arm"]
    arm_hold = hold_arm(arm, target=math.radians(20.0), tolerance=math.radians(1.0))

    scenarios = {
        "flywheel spin-up": scn_fly,
        "turret slew": scn_turret,
        "hood slew": scn_hood,
        "arm hold": _arm_scenario(arm_hold),
        "drive forward": scn_fwd,
        "drive strafe": scn_str,
    }

    _say("drawing step responses")
    plots = []
    plots.append(report.fig_flywheel(scn_fly, shot, target_rpm, outdir / "plots" / "flywheel.png"))
    plots.append(report.fig_step(scn_turret, "turret angle (deg)", "Turret slew 0 -> 90 deg",
                                 outdir / "plots" / "turret_slew.png", scale=180 / math.pi))
    plots.append(report.fig_step(scn_hood, "hood angle (deg)", "Hood slew 0 -> 45 deg",
                                 outdir / "plots" / "hood_slew.png", scale=180 / math.pi))
    plots.append(report.fig_arm(arm_hold, outdir / "plots" / "arm_hold.png"))
    plots.append(report.fig_drive(scn_fwd, outdir / "plots" / "drive_forward.png", "forward"))
    plots.append(report.fig_drive(scn_str, outdir / "plots" / "drive_strafe.png", "strafe"))

    _say("computing servo-side settings")
    servo_table = [_servo_settings(a) for a in (turret, hood, _servo_axis(arm_mech(robot_doc)))]

    _say("writing robot.json and gains.json")
    _write_json(outdir / "robot.json", robot_doc)
    gains_doc = _gains_document(robot_doc, model, frc_model, scenarios, shot, servo_table)
    _write_json(outdir / "gains.json", gains_doc)

    _say("fitting a synthetic calibration log (fit.py check)")
    from .runtime import fit as fitmod
    from .physics.sim import synthetic_calibration_log

    log = synthetic_calibration_log(
        flywheel, ks_true=flywheel.gains.kS, kv_true=flywheel.gains.kV,
        ka_true=flywheel.gains.kA, rng_seed=seed or 7,
    )
    log_path = fitmod.write_csv(outdir / "synthetic_log.csv", log)
    fitted = fitmod.fit_feedforward(log)
    _write_json(outdir / "synthetic_fit.json", {
        "truth": log["truth"],
        "fitted": fitted.as_dict(),
        "comparison": fitmod.compare(fitted, log["truth"]),
    })

    _say("exporting Pedro / FTC Dashboard / WPILib / MoSim")
    from .export import pedro, ftc_dashboard, wpilib, mosim

    exports = {}
    robo_for_pedro = dict(robot_doc)
    exports["pedro"] = {k: str(v) for k, v in pedro.build_pedro(
        robo_for_pedro, model, outdir / "exports" / "pedro").items()}

    dashboard_rows = []
    for name in ("flywheel",):
        axis = model.rotary[name]
        dashboard_rows.append({
            "name": name,
            "label": f"Shooter flywheel - {axis.params.get('notes', [''])[0]}",
            "gains": axis.gains.as_dict(),
            "tuned": scn_fly.tuned,
            "target": float(axis.params.get("target_speed_rad_s", 0.0)),
        })
    exports["ftc_dashboard"] = {"gains": str(ftc_dashboard.build_dashboard(
        robot_doc, dashboard_rows, outdir / "exports" / "ftc_dashboard"))}

    wp_rows = [{
        "name": name,
        "label": f"FRC-equivalent {name}",
        "kind": "arm" if name == "flower_arm" else "rotary",
        "gains": axis.gains.as_dict(),
        "tuned": (scn_fly.tuned if name == "flywheel" else
                  (scn_turret.tuned if name == "turret" else
                   {"kp": 20.0, "ki": 0.0, "kd": 2.0})),
    } for name, axis in frc_model.rotary.items()]
    wp = wpilib.build_wpilib(frc_model, robot_doc, wp_rows, outdir / "exports" / "wpilib")
    wp["notes"] = str(wpilib.build_sysid_routine_notes(outdir / "exports" / "wpilib"))
    exports["wpilib"] = {k: str(v) for k, v in wp.items()}

    exports["mosim"] = {k: str(v) for k, v in mosim.build_mosim(
        robot_doc, frc_model, outdir / "exports" / "mosim").items()}

    _say("writing SUMMARY.md")
    summary = _summary(robot_doc, model, scenarios, shot, servo_table, plots, exports, outdir)
    (outdir / "SUMMARY.md").write_text(summary, encoding="utf-8")

    elapsed = time.time() - t0
    _say(f"done in {elapsed:.1f} s -> {outdir}")
    return {
        "robot": robot_doc,
        "gains": gains_doc,
        "scenarios": scenarios,
        "shot": shot,
        "plots": [str(p) for p in plots],
        "exports": exports,
        "elapsed_s": elapsed,
        "out": str(outdir),
    }


def arm_mech(robot_doc: dict) -> dict:
    return next(m for m in robot_doc["mechanisms"] if m["name"] == "flower_arm")


def _servo_settings(axis: ServoAxis) -> dict:
    """What you actually set on a position servo instead of PIDF."""
    speed_deg = math.degrees(axis.output_speed_rad_s)
    return {
        "name": axis.name,
        "servo": axis.servo["sku"],
        "servo_count": axis.servo_count,
        "gear_ratio": axis.ratio,
        "output_speed_deg_s": speed_deg,
        "stall_torque_out_nm": axis.output_torque_nm,
        "required_torque_nm": axis.holding_torque_nm,
        "torque_margin": (axis.output_torque_nm / axis.holding_torque_nm
                          if axis.holding_torque_nm else None),
        "servo_travel_deg": axis.servo["usable_range_deg"],
        "axis_travel_deg": axis.servo["usable_range_deg"] / axis.ratio,
        "settings": {
            "control": "position (setPosition); the servo closes its own loop",
            "pidf": "not applicable: no PIDF runs on the hub for a servo",
            "range_mapping": (
                f"servo {axis.servo['usable_range_deg']:.0f} deg -> axis "
                f"{axis.servo['usable_range_deg'] / axis.ratio:.0f} deg; "
                "scale setPosition(0..1) accordingly and never command the ends"
            ),
            "slew_limit": f"<= {speed_deg:.0f} deg/s output; ramp the setpoint in software",
            "soft_limits": "clamp the commanded angle 5 deg inside the mechanical stop",
        },
    }


def _arm_scenario(arm_hold: dict):
    class _S:
        pass
    s = _S()
    s.name = "arm hold"
    s.before = arm_hold["before"]
    s.after = arm_hold["after"]
    s.tuned = {"kG": round(float(arm_hold.get("kG", 0.0)), 4),
               "kp": arm_hold.get("kp"), "kd": arm_hold.get("kd")}
    s.traces = {"before": arm_hold["trace_before"], "after": arm_hold["trace_after"]}
    return s


def _gains_document(robot_doc, model, frc_model, scenarios, shot, servo_table) -> dict:
    mechanisms = {}
    if model.drive:
        d = model.drive
        mechanisms["drivetrain"] = {
            "kind": "mecanum_drive",
            "forward": d.gains_forward.as_dict(),
            "strafe": d.gains_strafe.as_dict(),
            "max_forward_m_s": d.free_speed_forward,
            "max_forward_in_s": d.free_speed_forward * IN_PER_M,
            "max_strafe_m_s": d.free_speed_strafe,
            "max_strafe_in_s": d.free_speed_strafe * IN_PER_M,
            "tuned": {
                "forward": scenarios["drive forward"].tuned if scenarios.get("drive forward") else {},
                "strafe": scenarios["drive strafe"].tuned if scenarios.get("drive strafe") else {},
            },
        }
    for name, axis in model.rotary.items():
        scn = next((s for key, s in scenarios.items()
                    if s is not None and s.name == name), None)
        mechanisms[name] = {
            "kind": axis.params.get("type", "rotary"),
            "gains": axis.gains.as_dict(),
            "motor": axis.motor.key,
            "motor_count": axis.n_motors,
            "gear_ratio": axis.gearbox.ratio,
            "efficiency": axis.gearbox.efficiency,
            "inertia_kgm2": axis.j_eff,
            "free_speed_rad_s": axis.free_speed,
            "stall_torque_nm": axis.max_torque,
            "tuned": dict(scn.tuned) if scn is not None else {},
        }
    return {
        "robot": robot_doc["name"],
        "spool_version": __version__,
        "units": {
            "kS": "V",
            "kV": "V per (rad/s) rotary, V per (m/s) linear",
            "kA": "V per (rad/s^2) rotary, V per (m/s^2) linear",
            "kG": "V at the gravity-worst pose (arm horizontal)",
        },
        "mechanisms": mechanisms,
        "servo_axes": servo_table,
        "ball_recovery": {k: v for k, v in shot.items() if k != "trace"},
        "scenarios": {
            name: {"before": s.before, "after": s.after, "tuned": s.tuned}
            for name, s in scenarios.items() if s is not None
        },
        "provenance": robot_doc.get("provenance", {}),
        "provenance_notes": robot_doc.get("provenance_notes", {}),
    }


def _summary(robot_doc, model, scenarios, shot, servo_table, plots, exports, outdir) -> str:
    rows = report.table_rows({k: v for k, v in scenarios.items() if v is not None})
    table = report.markdown_table(rows)
    lines = [
        f"# {robot_doc['name']} - Spool summary",
        "",
        f"Generated by spool {__version__}. Mass {robot_doc['mass_kg']} kg "
        f"({robot_doc['provenance'].get('mass_kg', 'estimated')}).",
        "",
        "## Gains",
        "",
        "| Mechanism | kS (V) | kV | kA | kG (V) |",
        "|---|---|---|---|---|",
    ]
    if model.drive:
        for label, g in (("drivetrain forward", model.drive.gains_forward),
                         ("drivetrain strafe", model.drive.gains_strafe)):
            lines.append(f"| {label} | {g.kS:.4f} | {g.kV:.5f} | {g.kA:.5f} | - |")
    for name, axis in model.rotary.items():
        g = axis.gains
        lines.append(f"| {name} | {g.kS:.4f} | {g.kV:.5f} | {g.kA:.5f} | {g.kG:.4f} |")
    lines += [
        "",
        "## Before / after",
        "",
        table,
        "",
        f"Ball recovery: drop {shot['dip_pct']:.1f}%, "
        f"recovery {shot['recovery_time_s']:.3f} s "
        f"({'PASS' if shot['recovery_ok'] else 'FAIL'} at 2% in 0.5 s).",
        "",
        "## Servo axes (no PIDF)",
        "",
        "| Axis | Servo | Ratio | Speed (deg/s) | Torque out (N m) | Required (N m) | Margin |",
        "|---|---|---|---|---|---|---|",
    ]
    for s in servo_table:
        margin = f"{s['torque_margin']:.1f}x" if s["torque_margin"] else "-"
        lines.append(
            f"| {s['name']} | {s['servo']} | {s['gear_ratio']:.2f} | {s['output_speed_deg_s']:.0f} | "
            f"{s['stall_torque_out_nm']:.2f} | {s['required_torque_nm']:.3f} | {margin} |"
        )
    lines += ["", "## Files", ""]
    for p in plots:
        lines.append(f"- plot: `{Path(p).name}`")
    for group, files in exports.items():
        for kind, path in files.items():
            lines.append(f"- {group}/{kind}: `{Path(path).name}`")
    return "\n".join(lines) + "\n"


def cmd_calibrate(args) -> int:
    from .runtime import fit as fitmod

    log = fitmod.load_csv(args.log)
    result = fitmod.fit_feedforward(log, include_gravity=args.gravity)
    print(json.dumps(result.as_dict(), indent=2))
    if args.predicted:
        predicted = json.loads(Path(args.predicted).read_text(encoding="utf-8"))
        print(json.dumps(fitmod.compare(result, predicted), indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="spool", description="CAD in, tuned robot out.")
    parser.add_argument("--version", action="version", version=f"spool {__version__}")
    sub = parser.add_subparsers(dest="cmd")

    p_fat = sub.add_parser("fatman", help="run the whole FAT MAN pipeline")
    p_fat.add_argument("--root", default=DEFAULT_ROOT)
    p_fat.add_argument("--out", default=DEFAULT_OUT)
    p_fat.add_argument("--seed", type=int, default=0)
    p_fat.add_argument("--no-tune", action="store_true", help="skip the optimizer")

    p_cal = sub.add_parser("calibrate", help="fit gains from a SpoolCalibrate CSV")
    p_cal.add_argument("log")
    p_cal.add_argument("--gravity", action="store_true")
    p_cal.add_argument("--predicted", help="JSON with kS/kV/kA from the CAD model")

    p_web = sub.add_parser("web", help="serve the local web UI")
    p_web.add_argument("--port", type=int, default=8000)
    p_web.add_argument("--dir", default=DEFAULT_OUT)

    p_urdf = sub.add_parser("urdf", help="ingest a URDF")
    p_urdf.add_argument("file")
    p_urdf.add_argument("--out", default="out/urdf")
    p_urdf.add_argument("--motor", default="gobilda-5203-2402-0014")

    p_step = sub.add_parser("step", help="ingest a STEP file (needs the step extra)")
    p_step.add_argument("file")
    p_step.add_argument("--out", default="out/step")
    p_step.add_argument("--motor", default="gobilda-5203-2402-0014")
    p_step.add_argument("--ratio", type=float, default=1.0)

    args = parser.parse_args(argv)

    if args.cmd == "fatman":
        result = run_fatman(args.root, args.out, args.seed, do_tune=not args.no_tune)
        rows = report.table_rows({k: v for k, v in result["scenarios"].items() if v is not None})
        print()
        print(report.markdown_table(rows))
        return 0

    if args.cmd == "calibrate":
        return cmd_calibrate(args)

    if args.cmd == "web":
        from .web.server import serve
        serve(args.dir, args.port)
        return 0

    if args.cmd == "urdf":
        from .ingest.urdf import robot_from_urdf
        doc = robot_from_urdf(args.file, default_motor=args.motor)
        out = Path(args.out) / "robot.json"
        _write_json(out, doc)
        print(f"wrote {out}")
        return 0

    if args.cmd == "step":
        from .ingest.step import robot_from_step
        doc = robot_from_step(args.file, motor=args.motor, ratio=args.ratio)
        out = Path(args.out) / "robot.json"
        _write_json(out, doc)
        print(f"wrote {out}")
        return 0

    parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
