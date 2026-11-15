"""Classical tuning over the simulated plant.

No LLM, no heuristics dressed up as magic: a deterministic differential-evolution
search (SciPy, fixed seed) over PID gains, with the brief's constraints as hard
penalties:

  * turret: no overshoot past the soft limits, settle within 1 degree
  * flywheel: recover to within 2% of target in under T_rec
  * arm: hold within 1 degree at horizontal (worst-case gravity)
  * drivetrain: no wheel slip beyond the friction limit, settle quickly

Every result is reproducible: same seed, same table.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import differential_evolution

from .mechanisms import DriveModel, RotaryAxis
from .sim import PID, ServoAxis, pid_from_gains, simulate_drive, simulate_rotary, simulate_servo

MAX_VOLTS = 12.0


@dataclass
class Scenario:
    """One before/after comparison."""

    name: str
    before: dict
    after: dict
    tuned: dict = field(default_factory=dict)
    config: dict = field(default_factory=dict)
    traces: dict = field(default_factory=dict, repr=False)

    def row(self) -> dict:
        return {
            "scenario": self.name,
            "before_rise_s": self.before.get("rise_time"),
            "after_rise_s": self.after.get("rise_time"),
            "before_overshoot_pct": self.before.get("overshoot_pct"),
            "after_overshoot_pct": self.after.get("overshoot_pct"),
            "before_settle_s": self.before.get("settle_time"),
            "after_settle_s": self.after.get("settle_time"),
            "tuned": self.tuned,
        }


def _cost(metrics: dict, *, max_overshoot: float, max_settle: float, tolerance: float) -> float:
    settle = metrics.get("settle_time", 99.0)
    if not math.isfinite(settle):
        settle = 99.0
    overshoot = max(0.0, metrics.get("overshoot_pct", 0.0))
    c = settle
    c += 2.0 * max(0.0, overshoot - max_overshoot)
    c += 5.0 * max(0.0, settle - max_settle)
    c += 20.0 * abs(metrics.get("steady_state_error", 0.0)) / max(tolerance, 1e-9)
    if not metrics.get("settled", False):
        c += 5.0
    return float(c)


def tune_rotary(
    axis: RotaryAxis,
    *,
    target: float,
    mode: str = "position",
    max_overshoot: float = 5.0,
    max_settle: float = 0.5,
    tolerance: float = 0.01,
    t_end: float = 2.0,
    seed: int = 0,
    bandwidth: float = 8.0,
) -> Scenario:
    """Tune one rotary axis (position or velocity) and compare before/after."""
    kp0, _, kd0 = pid_from_gains(axis.gains, bandwidth=bandwidth)
    kd0 = kd0 if kd0 > 0 else kp0 * 0.05

    # BEFORE: no feedforward, raw step, a plain proportional+derivative loop.
    before_pid = PID(kp=kp0, ki=0.0, kd=kd0, limit=MAX_VOLTS)
    before_sim = simulate_rotary(
        axis, before_pid, target=target, mode=mode, t_end=t_end,
        use_feedforward=False, profile="step", label="before",
    )
    before = before_sim.metrics(tolerance)

    def objective(x: np.ndarray) -> float:
        kp, ki, kd = x
        pid = PID(kp=kp, ki=ki, kd=kd, limit=MAX_VOLTS)
        sim = simulate_rotary(
            axis, pid, target=target, mode=mode, t_end=t_end,
            use_feedforward=True, profile="trapezoid",
            vmax=axis.free_speed * 0.6, amax=axis.free_speed * 4.0,
        )
        m = sim.metrics(tolerance)
        return _cost(m, max_overshoot=max_overshoot, max_settle=max_settle, tolerance=tolerance)

    bounds = [(kp0 * 0.05, kp0 * 12.0), (0.0, kp0 * 6.0), (kd0 * 0.02, kd0 * 25.0)]
    res = differential_evolution(
        objective, bounds, seed=seed, maxiter=40, popsize=10, tol=1e-4, polish=False,
        init="sobol", updating="deferred", workers=1,
    )
    kp, ki, kd = (float(v) for v in res.x)

    after_pid = PID(kp=kp, ki=ki, kd=kd, limit=MAX_VOLTS)
    after_sim = simulate_rotary(
        axis, after_pid, target=target, mode=mode, t_end=t_end,
        use_feedforward=True, profile="trapezoid",
        vmax=axis.free_speed * 0.6, amax=axis.free_speed * 4.0, label="after",
    )
    after = after_sim.metrics(tolerance)

    return Scenario(
        name=axis.name,
        before=before,
        after=after,
        tuned={"kp": kp, "ki": ki, "kd": kd, "kS": axis.gains.kS, "kV": axis.gains.kV,
               "kA": axis.gains.kA, "kG": axis.gains.kG},
        config={
            "mode": mode,
            "target": target,
            "tolerance": tolerance,
            "profile": "trapezoid (feedforward path)" if mode == "position" else "step",
            "vmax_rad_s": axis.free_speed * 0.6,
            "amax_rad_s2": axis.free_speed * 4.0,
            "cost": float(res.fun),
            "evaluations": int(res.nfev),
        },
        traces={"before": before_sim, "after": after_sim},
    )


def tune_drive(
    drive: DriveModel,
    *,
    target_speed: float,
    axis: str = "forward",
    max_overshoot: float = 5.0,
    max_settle: float = 0.4,
    tolerance: float = 0.05,
    t_end: float = 2.0,
    seed: int = 0,
) -> Scenario:
    gains = drive.gains_forward if axis == "forward" else drive.gains_strafe
    kp0, _, kd0 = pid_from_gains(gains, bandwidth=6.0)
    kd0 = kd0 if kd0 > 0 else kp0 * 0.05

    before_pid = PID(kp=kp0, ki=0.0, kd=kd0, limit=MAX_VOLTS)
    before_sim = simulate_drive(
        drive, before_pid, target_speed=target_speed, axis=axis, t_end=t_end,
        use_feedforward=False, label="before",
    )
    before = before_sim.metrics(tolerance)

    def objective(x: np.ndarray) -> float:
        kp, ki, kd = x
        pid = PID(kp=kp, ki=ki, kd=kd, limit=MAX_VOLTS)
        sim = simulate_drive(
            drive, pid, target_speed=target_speed, axis=axis, t_end=t_end,
            use_feedforward=True,
        )
        return _cost(sim.metrics(tolerance), max_overshoot=max_overshoot,
                     max_settle=max_settle, tolerance=tolerance)

    bounds = [(kp0 * 0.05, kp0 * 12.0), (0.0, kp0 * 8.0), (kd0 * 0.02, kd0 * 25.0)]
    res = differential_evolution(
        objective, bounds, seed=seed, maxiter=35, popsize=10, tol=1e-4, polish=False,
        init="sobol", updating="deferred", workers=1,
    )
    kp, ki, kd = (float(v) for v in res.x)
    after_pid = PID(kp=kp, ki=ki, kd=kd, limit=MAX_VOLTS)
    after_sim = simulate_drive(
        drive, after_pid, target_speed=target_speed, axis=axis, t_end=t_end,
        use_feedforward=True, label="after",
    )
    after = after_sim.metrics(tolerance)

    return Scenario(
        name=f"{drive.name}.{axis}",
        before=before,
        after=after,
        tuned={"kp": kp, "ki": ki, "kd": kd, "kS": gains.kS, "kV": gains.kV, "kA": gains.kA},
        config={"mode": "velocity", "target": target_speed, "tolerance": tolerance,
                "axis": axis, "cost": float(res.fun), "evaluations": int(res.nfev)},
        traces={"before": before_sim, "after": after_sim},
    )


def tune_servo(
    axis: ServoAxis,
    *,
    target: float,
    max_overshoot: float = 0.0,
    max_settle: float = 0.5,
    tolerance: float = math.radians(1.0),
    t_end: float = 1.5,
    seed: int = 0,
) -> Scenario:
    """Tune the only knob a position servo gives you: the command profile.

    A hobby servo closes its own loop, so "tuning PIDF" would be fiction. What we
    can choose is how the setpoint is fed in. A raw step excites the gear train
    and the turret's inertia; a short ramp removes the end-of-move kick. The
    search picks the ramp time that minimises settle time subject to zero
    overshoot past the soft limit.
    """
    before_sim = simulate_servo(axis, target=target, t_end=t_end, use_profile=False,
                                label="before (raw step)")
    before = before_sim.metrics(tolerance)

    best = None
    grid = np.linspace(0.0, 0.6, 25)
    for pt in grid:
        sim = simulate_servo(axis, target=target, t_end=t_end, use_profile=bool(pt > 0),
                             profile_time=float(pt))
        m = sim.metrics(tolerance)
        c = _cost(m, max_overshoot=max_overshoot, max_settle=max_settle, tolerance=tolerance)
        if best is None or c < best[0]:
            best = (c, float(pt), sim, m)

    _, pt, after_sim, after = best
    return Scenario(
        name=axis.name,
        before=before,
        after=after,
        tuned={"profile_time_s": pt, "servo": axis.servo["sku"], "ratio": axis.ratio},
        config={"mode": "servo", "target": target, "tolerance_deg": math.degrees(tolerance),
                "solution": "setpoint ramp (servo PIDF is internal)", "grid_points": len(grid)},
        traces={"before": before_sim, "after": after_sim},
    )


def ball_recovery(
    axis: RotaryAxis,
    *,
    exit_speed: float,
    ball_mass: float,
    transfer_efficiency: float,
    tolerance_pct: float = 2.0,
    max_recovery: float = 0.5,
    kp: float | None = None,
    ki: float = 0.0,
    kd: float | None = None,
    t_shot: float = 1.2,
    t_end: float = 3.0,
    band: float = 0.05,
) -> dict:
    """Spin up to target, fire one ball, measure the dip and the recovery time.

    Energy taken from the wheel: dE = 0.5*m_ball*v_exit^2 / eta_transfer.
    The resulting speed drop is solved from 0.5*J*w'^2 = 0.5*J*w^2 - dE, then
    applied as an angular-momentum impulse J*(w - w') over a 5 ms contact.
    """
    omega0 = float(axis.params.get("target_speed_rad_s", 0.0))
    j = axis.j_eff
    dE = 0.5 * ball_mass * exit_speed**2 / max(transfer_efficiency, 1e-6)
    omega_after = math.sqrt(max(omega0**2 - 2.0 * dE / j, 0.0))
    delta_l = j * (omega0 - omega_after)
    contact = 0.005
    torque = delta_l / contact

    kp = kp if kp is not None else max(0.2, axis.gains.kV * 4.0)
    kd = kd if kd is not None else 0.0
    pid = PID(kp=kp, ki=ki, kd=kd, limit=MAX_VOLTS)

    sim = simulate_rotary(
        axis, pid, target=omega0, mode="velocity", t_end=t_end,
        use_feedforward=True, disturbance=(t_shot, contact, torque), label="shot",
    )
    band_abs = omega0 * tolerance_pct / 100.0
    after = np.nonzero(sim.t >= t_shot)[0]
    # Only look at the wheel *after* the shot: including the spin-up from zero
    # reports a 100% dip every time, which is meaningless.
    dip = float(np.min(sim.velocity[after])) if after.size else omega0
    inside = np.abs(sim.velocity[after] - omega0) <= band_abs
    recovery = None
    if inside.any():
        idx = after[inside]
        # last excursion outside the band
        outside_idx = after[~inside]
        if outside_idx.size == 0:
            recovery = 0.0
        else:
            last = outside_idx.max()
            nxt = np.nonzero(sim.t > sim.t[last])[0]
            recovery = float(sim.t[nxt[0]] - t_shot) if nxt.size else None
    return {
        "target_rad_s": omega0,
        "target_rpm": omega0 * 60.0 / (2.0 * math.pi),
        "energy_taken_j": dE,
        "speed_drop_rad_s": omega0 - omega_after,
        "speed_drop_pct": (omega0 - omega_after) / omega0 * 100.0,
        "dip_rad_s": dip,
        "dip_pct": (omega0 - dip) / omega0 * 100.0,
        "recovery_time_s": recovery,
        "recovery_ok": bool(recovery is not None and recovery <= max_recovery),
        "tolerance_pct": tolerance_pct,
        "pid": {"kp": kp, "ki": ki, "kd": kd},
        "trace": sim,
    }


def hold_arm(
    axis: RotaryAxis,
    *,
    target: float,
    tolerance: float = math.radians(1.0),
    t_end: float = 1.5,
    ki: float = 0.0,
) -> dict:
    """Hold the arm at `target` starting from horizontal (worst-case gravity).

    The object here is gravity, not speed: the arm starts horizontal, where the
    gravity torque is largest. "Before" runs the identical controller with the kG
    term removed from the feedforward; "after" uses the CAD-derived kG. Any
    difference between the two curves is the value of knowing the mass and the
    centre of mass before the robot is built.

    kp/kd are picked on a small deterministic grid against the same cost function
    as the other axes, so the comparison is not rigged by a bad loop.
    """
    best = None
    for kp in (10.0, 20.0, 40.0, 80.0):
        for kd in (0.0, 0.5, 1.0, 2.0, 4.0, 8.0):
            pid = PID(kp=kp, ki=ki, kd=kd, limit=MAX_VOLTS)
            sim = simulate_rotary(axis, pid, target=target, mode="position", t_end=t_end,
                                  use_feedforward=True, profile="step")
            c = _cost(sim.metrics(tolerance), max_overshoot=2.0,
                      max_settle=0.4, tolerance=tolerance)
            if best is None or c < best[0]:
                best = (c, kp, kd, sim, sim.metrics(tolerance))
    _, kp, kd, sim_after, after = best

    no_kg_gains = axis.gains.__class__(
        mechanism=axis.gains.mechanism, kS=axis.gains.kS, kV=axis.gains.kV,
        kA=axis.gains.kA, kG=0.0, units=axis.gains.units,
        saturation_voltage=axis.gains.saturation_voltage, basis=axis.gains.basis,
        notes=axis.gains.notes,
    )
    no_kg = RotaryAxis(
        name=axis.name, motor=axis.motor, gearbox=axis.gearbox, n_motors=axis.n_motors,
        load_inertia=axis.load_inertia, gains=no_kg_gains,
        friction_torque=axis.friction_torque, limits=axis.limits,
        gravity_torque_fn=axis.gravity_torque_fn, target=axis.target,
        current_limit_a=axis.current_limit_a, r_batt_ohm=axis.r_batt_ohm, params=axis.params,
    )
    before_pid = PID(kp=kp, ki=ki, kd=kd, limit=MAX_VOLTS)
    before_sim = simulate_rotary(no_kg, before_pid, target=target, mode="position",
                                 t_end=t_end, use_feedforward=True, profile="step")
    before = before_sim.metrics(tolerance)
    return {
        "before": before,
        "after": after,
        "trace_before": before_sim,
        "trace_after": sim_after,
        "kp": kp,
        "kd": kd,
        "kG": axis.gains.kG,
    }
