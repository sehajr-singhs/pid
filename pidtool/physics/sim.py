"""Headless simulation: per-mechanism ODEs at 1 kHz, CPU only.

Plant (rotary), integrated explicitly with dt = 1 ms:

    I_m   = (V_app - k_e*G*w) / R                    per motor
    tau_m = k_t*I_m
    J_eff*a = n*eta*G*tau_m - tau_c*sign(w) - tau_g(theta) - tau_ext(t)

    J_eff = J_load + n*G^2*J_rotor
    tau_c = n*eta*G*k_t*I_free + friction model torque   (see feedforward.ks_from_friction)

Battery sag: the pack is modelled as V_oc in series with R_batt, so a hard
acceleration really does cost voltage. R_batt and R_wiring are assumptions.

Drivetrain: 2-D rigid body, four motors sharing the load, mecanum strafe costing
speed through roller slip. Rolling resistance is mu_roll*m*g (velocity-opposing).

Nothing here is random: same inputs give the same trace, which is what makes the
tuning table reproducible.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .mechanisms import DriveModel, RotaryAxis
from .motor import Motor

G = 9.80665


# ------------------------------------------------------------------ controller

@dataclass
class PID:
    """PID with derivative-on-measurement, a derivative filter and output clamping.

    The derivative is low-pass filtered with the standard N-factor form
    (Tf = kd / (N*kp)). Without it, a kd sized for the plant times 1/dt at a 1 kHz
    step is unstable: the command oscillates between the supply rails even though
    the gains are sensible, which is a numerical artefact, not robot behaviour.
    A real controller sees a filtered encoder velocity for the same reason.

    Anti-windup: conditional integration (the integrator stops moving when the
    output is saturated in the direction the error is pushing).
    """

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    limit: float = 12.0
    integral: float = 0.0
    prev_measurement: float | None = None
    _last_output: float = 0.0
    n_filter: float = 10.0
    _d_state: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_measurement = None
        self._last_output = 0.0
        self._d_state = 0.0

    def update(self, error: float, measurement: float, dt: float, feedforward: float = 0.0) -> float:
        raw_derivative = 0.0
        if self.prev_measurement is not None and dt > 0:
            raw_derivative = -(measurement - self.prev_measurement) / dt
        self.prev_measurement = measurement

        if self.kd > 0.0 and self.kp > 1e-9:
            tf = self.kd / (self.n_filter * self.kp)
            alpha = dt / (tf + dt) if tf > 0 else 1.0
            self._d_state += alpha * (raw_derivative - self._d_state)
            derivative = self._d_state
        else:
            derivative = raw_derivative

        p = self.kp * error
        d = self.kd * derivative
        candidate = p + self.ki * self.integral + d + feedforward
        saturated = abs(candidate) > self.limit and (candidate * error) > 0
        if not saturated:
            self.integral += error * dt
        u = self.kp * error + self.ki * self.integral + d + feedforward

        if u > self.limit:
            u = self.limit
        elif u < -self.limit:
            u = -self.limit
        self._last_output = u
        return u


def pid_from_gains(gains, bandwidth: float = 8.0, damping: float = 0.9) -> list[float]:
    """Starting PID from the computed inertia gain.

    kA is exactly the plant gain: kA * acceleration = volts, so the plant is
    K = 1/kA (rad/s^2 per volt, or m/s^2 per volt for a drive). Pole placement on
    that double integrator gives kp = w^2*K^-1 = w^2*kA and kd = 2*zeta*w*kA.
    Integral starts at zero; the optimizer decides whether it earns its place.
    """
    ka = float(gains.kA)
    if ka <= 0 or not math.isfinite(ka):
        return [1.0, 0.0, 0.0]
    w = bandwidth
    return [w * w * ka, 0.0, 2.0 * damping * w * ka]


# ---------------------------------------------------------------- result types

@dataclass
class SimResult:
    t: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    command: np.ndarray
    voltage: np.ndarray
    current: np.ndarray
    target: float
    label: str = ""

    def metrics(self, tolerance: float, settle_window: float = 0.05) -> dict:
        """Rise time, overshoot and settle time for a step response.

        tolerance is an absolute band around the target, in the units of
        `position` for position control and `velocity` for speed control.
        """
        y = self.position if abs(self.target) > 0 and self._is_position else self.velocity
        final = float(y[-1])
        target = self.target
        y0 = float(y[0])
        out = {"final": final, "target": target}

        # Rise time is measured against the value the axis actually reached, so a
        # steady-state error (e.g. friction the feedforward did not cancel) does
        # not turn the rise time into a NaN. Overshoot is still measured against
        # the target, because that is the number that matters for a hard stop.
        rise_ref = final if abs(final - y0) > 1e-12 else target
        if abs(rise_ref - y0) > 1e-12:
            lo, hi = sorted((y0, rise_ref))
            band = 0.1 * abs(rise_ref - y0)
            t10 = t90 = None
            for ti, yi in zip(self.t, y):
                if t10 is None and yi >= lo + band:
                    t10 = ti
                if t90 is None and yi >= hi - band:
                    t90 = ti
                if t10 is not None and t90 is not None:
                    break
            if t10 is None:
                t10 = 0.0
            if t90 is None:
                t90 = float(self.t[-1])
            out["rise_time"] = float(t90 - t10)
        else:
            out["rise_time"] = 0.0

        peak = float(np.max(y)) if target >= 0 else float(np.min(y))
        out["peak"] = peak
        out["overshoot_pct"] = (peak - target) / abs(target) * 100.0 if abs(target) > 1e-12 else 0.0

        outside = np.abs(y - target) > tolerance
        if np.any(outside):
            last = int(np.max(np.nonzero(outside)[0]))
            out["settle_time"] = float(self.t[min(last + 1, len(self.t) - 1)])
            # check the samples *after* the last excursion, not including it
            out["settled"] = bool(np.all(np.abs(y[last + 1:] - target) <= tolerance))
        else:
            out["settle_time"] = 0.0
            out["settled"] = True
        out["steady_state_error"] = target - final
        return out

    _is_position: bool = field(default=False, repr=False)


# --------------------------------------------------------------------- rotary

def _plant_friction_torque(axis: RotaryAxis) -> float:
    """Coulomb friction at the output: motor free-run + mechanism model."""
    free_run = axis.n_motors * axis.gearbox.efficiency * axis.gearbox.ratio * axis.motor.kt * axis.motor.free_current_a
    return free_run + axis.friction_torque


def _rotary_derivative(
    axis: RotaryAxis,
    theta: float,
    omega: float,
    voltage: float,
    extra_torque: float,
    r_batt: float,
    v_oc: float,
    current_limit: float | None = None,
) -> tuple[float, float, float]:
    motor = axis.motor
    G_ratio = axis.gearbox.ratio
    n = axis.n_motors
    eta = axis.gearbox.efficiency

    v = float(np.clip(voltage, -v_oc, v_oc))
    back_emf = motor.ke * G_ratio * omega
    current = (v - back_emf) / motor.resistance
    # A motor controller limits current before the pack sees it. Modelling the
    # limit matters: without it an FRC arm commanding 8 V at stall computes 112 A
    # per motor, drops most of the battery voltage across R_batt, and looks too
    # weak to lift itself.
    if current_limit is not None:
        current = float(np.clip(current, -current_limit, current_limit))
    v_terminal = v - current * n * r_batt

    tau = n * eta * G_ratio * motor.kt * current
    tau_c = _plant_friction_torque(axis) * math.tanh(omega / 0.05)
    tau_g = axis.gravity_torque(theta)
    j = axis.j_eff
    alpha = (tau - tau_c - tau_g - extra_torque) / j
    return alpha, current, v_terminal


def simulate_rotary(
    axis: RotaryAxis,
    controller: PID,
    *,
    target: float,
    mode: str = "position",          # "position" | "velocity"
    t_end: float = 3.0,
    dt: float = 1e-3,
    use_feedforward: bool = True,
    voltage_limit: float | None = None,
    r_batt: float | None = None,
    current_limit: float | None = None,
    v_oc: float = 12.4,
    profile: str = "step",           # "step" | "trapezoid"
    vmax: float | None = None,
    amax: float | None = None,
    disturbance: tuple[float, float, float] | None = None,   # (t_start, duration, torque)
    label: str = "",
) -> SimResult:
    """Integrate one rotary axis.

    mode="position" runs a move to `target` (rad); mode="velocity" runs to a
    target speed (rad/s). `profile="trapezoid"` shapes the position reference
    with vmax/amax so the axis never asks for more than the motor can give.
    """
    controller.reset()
    limit = voltage_limit if voltage_limit is not None else axis.motor.v_nom
    controller.limit = limit
    if r_batt is None:
        r_batt = getattr(axis, "r_batt_ohm", 0.03)
    if current_limit is None:
        current_limit = getattr(axis, "current_limit_a", None)

    n_steps = int(t_end / dt) + 1
    t = np.arange(n_steps) * dt
    theta = 0.0
    omega = 0.0
    pos_hist = np.empty(n_steps)
    vel_hist = np.empty(n_steps)
    cmd_hist = np.empty(n_steps)
    volt_hist = np.empty(n_steps)
    cur_hist = np.empty(n_steps)

    gains = axis.gains
    _vmax = vmax if vmax is not None else axis.free_speed * 0.5
    _amax = amax if amax is not None else axis.free_speed * 3.0

    for i, ti in enumerate(t):
        if mode == "velocity":
            ref_v = target
            ref_a = 0.0
            error = target - omega
            measurement = omega
        else:
            ref_pos, ref_v, ref_a = _reference(ti, target, profile, _vmax, _amax)
            error = ref_pos - theta
            measurement = theta

        ff = 0.0
        if use_feedforward:
            ff = gains.kV * ref_v + gains.kA * ref_a
            if axis.gravity_torque_fn is not None:
                ff += gains.kG * math.cos(theta)
            if abs(ref_v) > 1e-6:
                ff += gains.kS * (1.0 if ref_v > 0 else -1.0)
            elif mode == "position" and abs(error) > 1e-6:
                ff += gains.kS * (1.0 if error > 0 else -1.0) * 0.0  # kS only with motion
        cmd = controller.update(error, measurement, dt, ff)

        extra = 0.0
        if disturbance is not None:
            t0, dur, torque = disturbance
            if t0 <= ti < t0 + dur:
                extra = torque

        alpha, current, applied = _rotary_derivative(
            axis, theta, omega, cmd, extra, r_batt, v_oc, current_limit
        )

        pos_hist[i] = theta
        vel_hist[i] = omega
        cmd_hist[i] = cmd
        volt_hist[i] = applied
        cur_hist[i] = current

        omega += alpha * dt
        theta += omega * dt

        if axis.limits is not None:
            lo, hi = axis.limits
            if theta < lo:
                theta, omega = lo, 0.0
            elif theta > hi:
                theta, omega = hi, 0.0

    res = SimResult(t, pos_hist, vel_hist, cmd_hist, volt_hist, cur_hist, target, label)
    res._is_position = mode == "position"
    return res


def _reference(t: float, target: float, profile: str, vmax: float, amax: float) -> tuple[float, float, float]:
    """Position, velocity and acceleration reference at time t."""
    if profile == "step":
        return target, 0.0, 0.0
    # Trapezoidal (or triangular when it never reaches vmax).
    sign = 1.0 if target >= 0 else -1.0
    d = abs(target)
    t_acc = vmax / amax
    d_acc = 0.5 * amax * t_acc**2
    if 2 * d_acc >= d:
        # triangular
        t_acc = math.sqrt(d / amax)
        t_total = 2 * t_acc
        peak = amax * t_acc
        vmax_eff = peak
    else:
        vmax_eff = vmax
        t_total = 2 * t_acc + (d - 2 * d_acc) / vmax
    if t <= 0:
        return 0.0, 0.0, 0.0
    if t < t_acc:
        return sign * 0.5 * amax * t**2, sign * amax * t, sign * amax
    if t < t_total - t_acc:
        d1 = 0.5 * amax * t_acc**2
        return sign * (d1 + vmax_eff * (t - t_acc)), sign * vmax_eff, 0.0
    if t < t_total:
        tr = t - (t_total - t_acc)
        d_before = d - 0.5 * amax * t_acc**2
        return sign * (d_before + vmax_eff * tr - 0.5 * amax * tr**2), sign * (vmax_eff - amax * tr), -sign * amax
    return sign * d, 0.0, 0.0


# ----------------------------------------------------------------------- drive

def simulate_drive(
    drive: DriveModel,
    controller: PID,
    *,
    target_speed: float,
    axis: str = "forward",           # "forward" | "strafe"
    t_end: float = 2.5,
    dt: float = 1e-3,
    use_feedforward: bool = True,
    r_batt: float | None = None,
    current_limit: float | None = None,
    v_oc: float = 12.4,
    label: str = "",
) -> SimResult:
    """Velocity step for the chassis, one axis at a time.

    The drivetrain is a mass on four motors. Strafe is the same plant with the
    roller-slip factor applied to the force available per unit of chassis speed.
    """
    controller.reset()
    limit = drive.motor.v_nom
    controller.limit = limit
    if r_batt is None:
        r_batt = getattr(drive, "r_batt_ohm", 0.03)
    if current_limit is None:
        current_limit = getattr(drive, "current_limit_a", None)
    gains = drive.gains_forward if axis == "forward" else drive.gains_strafe
    slip = 1.0 if axis == "forward" else drive.strafe_efficiency

    n_steps = int(t_end / dt) + 1
    t = np.arange(n_steps) * dt
    x = 0.0
    v = 0.0
    pos = np.empty(n_steps)
    vel = np.empty(n_steps)
    cmd = np.empty(n_steps)
    volt = np.empty(n_steps)
    cur = np.empty(n_steps)

    motor = drive.motor
    n_motors = 4
    g_ratio = drive.gearbox.ratio
    eta = drive.gearbox.efficiency
    r = drive.wheel_radius

    for i, _ti in enumerate(t):
        error = target_speed - v
        ff = 0.0
        if use_feedforward:
            ff = gains.kV * target_speed + gains.kS * (1.0 if target_speed > 0 else -1.0)
        u = controller.update(error, v, dt, ff)

        v_app = float(np.clip(u, -v_oc, v_oc))
        omega_wheel = v / r
        back_emf = motor.ke * g_ratio * omega_wheel
        current = (v_app - back_emf) / motor.resistance
        if current_limit is not None:
            current = float(np.clip(current, -current_limit, current_limit))
        v_app = v_app - current * n_motors * r_batt

        tau_wheel_each = eta * g_ratio * motor.kt * current
        force = n_motors * tau_wheel_each / r * slip
        # drivetrain friction: motor free-run + gearbox model, plus rolling resistance
        tau_friction_each = eta * g_ratio * motor.kt * motor.free_current_a
        f_friction = n_motors * tau_friction_each / r * math.tanh(v / 0.02)
        f_roll = drive.mu_rolling * drive.mass * G * math.tanh(v / 0.02)
        a = (force - f_friction - f_roll) / drive.mass

        pos[i], vel[i], cmd[i], volt[i], cur[i] = x, v, u, v_app, current
        v += a * dt
        x += v * dt

    res = SimResult(t, pos, vel, cmd, volt, cur, target_speed, label)
    res._is_position = False
    return res


# ---------------------------------------------------------------------- servo

@dataclass
class ServoAxis:
    """Position servo axis: no PIDF, just a rate limit and a deadband."""

    name: str
    servo: dict
    ratio: float                       # servo turns per output turn (reduction)
    servo_count: int = 1
    inertia_kgm2: float = 1e-4
    holding_torque_nm: float = 0.0
    gear_efficiency: float = 0.9
    limits_rad: tuple[float, float] | None = None

    @property
    def output_speed_rad_s(self) -> float:
        s = self.servo["speed_s_per_60deg"]
        return (math.pi / 3.0) / s / self.ratio

    @property
    def output_torque_nm(self) -> float:
        return self.servo_count * self.servo["stall_torque_kgcm"] * 0.0980665 * self.ratio * self.gear_efficiency

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "servo": self.servo["sku"],
            "servo_count": self.servo_count,
            "ratio": self.ratio,
            "output_speed_deg_s": math.degrees(self.output_speed_rad_s),
            "output_torque_nm": self.output_torque_nm,
            "holding_torque_nm": self.holding_torque_nm,
            "torque_margin": (self.output_torque_nm / self.holding_torque_nm) if self.holding_torque_nm else None,
            "limits_rad": list(self.limits_rad) if self.limits_rad else None,
        }


def simulate_servo(
    axis: ServoAxis,
    *,
    target: float,
    t_end: float = 2.0,
    dt: float = 1e-3,
    deadband: float = math.radians(0.5),
    use_profile: bool = False,
    profile_time: float = 0.15,
    label: str = "",
) -> SimResult:
    """Servo move: constant-rate slew to the setpoint inside a deadband.

    A hobby servo has its own internal loop. From outside it behaves as a rate
    limit plus a small deadband, so that is what we simulate: no invented PIDF.
    With use_profile=True the command is ramped first, which is what removes the
    end-of-move jerk on a heavy turret.
    """
    n_steps = int(t_end / dt) + 1
    t = np.arange(n_steps) * dt
    theta = 0.0
    pos = np.empty(n_steps)
    vel = np.empty(n_steps)
    cmd = np.empty(n_steps)
    volt = np.zeros(n_steps)
    cur = np.zeros(n_steps)

    rate = axis.output_speed_rad_s
    for i, ti in enumerate(t):
        if use_profile:
            ref = target * min(1.0, ti / profile_time) if profile_time > 0 else target
        else:
            ref = target
        err = ref - theta
        step = rate * dt
        if abs(err) <= max(step, deadband):
            theta = ref if abs(err) > deadband else theta
            w = 0.0
        else:
            w = math.copysign(min(rate, abs(err) / dt), err)
            theta += w * dt
        cmd[i] = ref
        pos[i], vel[i] = theta, w
        if axis.limits_rad:
            lo, hi = axis.limits_rad
            if theta < lo or theta > hi:
                theta = min(max(theta, lo), hi)

    res = SimResult(t, pos, vel, cmd, volt, cur, target, label)
    res._is_position = True
    return res


# ------------------------------------------------------------- synthetic logs

def synthetic_calibration_log(
    axis: RotaryAxis,
    *,
    ks_true: float,
    kv_true: float,
    ka_true: float,
    kg_true: float = 0.0,
    rng_seed: int = 7,
    dt: float = 0.002,
    noise_volts: float = 0.01,
    noise_speed: float = 0.5,
    include_gravity: bool = False,
) -> dict:
    """Generate a PidCalibrate-style log from a known plant.

    The whole point is that ks_true/kv_true/ka_true are known exactly, so
    pidtool.runtime.fit can be scored on how well it recovers them. The plant used
    here is the same ODE as the simulator, so this is a closed-loop check of the
    identification code, not of the physics.
    """
    rng = np.random.default_rng(rng_seed)
    volts: list[float] = []
    speeds: list[float] = []
    accels: list[float] = []
    times: list[float] = []
    currents: list[float] = []
    positions: list[float] = []

    j = axis.j_eff
    force_per_volt = axis.n_motors * axis.gearbox.efficiency * axis.gearbox.ratio * axis.motor.kt / axis.motor.resistance

    state = {"omega": 0.0, "theta": 0.0}

    def run(v_cmd: float, seconds: float, t0: float) -> float:
        """One voltage block, integrated with the real motor equation.

        The plant is V = kS*sign(w) + kV*w + kA*alpha (+ kG*cos(theta)), solved for
        alpha, with proper Coulomb/static friction:

          * at rest nothing moves until |V| > kS   (static friction)
          * while moving, friction opposes the velocity and stops the wheel rather
            than reversing it

        Getting this wrong is not cosmetic. An earlier version took the friction
        sign from the commanded voltage, so a 0 V block spun the wheel *up*; the
        wheel was already turning before the ramp, and the identified kS came out
        30-45% high on a noiseless log.
        """
        for k in range(int(seconds / dt)):
            omega = state["omega"]
            theta = state["theta"]
            grav = kg_true * math.cos(theta) if include_gravity else 0.0

            if abs(omega) < 1e-9:
                if abs(v_cmd) <= ks_true:
                    alpha = 0.0                      # stalled: static friction wins
                else:
                    s = 1.0 if v_cmd > 0 else -1.0
                    alpha = (v_cmd - ks_true * s - grav) / ka_true
                    omega = alpha * dt
                    if omega * s < 0.0:
                        omega = 0.0
            else:
                s = 1.0 if omega > 0 else -1.0
                alpha = (v_cmd - ks_true * s - kv_true * omega - grav) / ka_true
                omega += alpha * dt
                if omega * s < 0.0:
                    omega = 0.0

            state["omega"] = omega
            state["theta"] = theta + omega * dt

            times.append(t0 + k * dt)
            volts.append(v_cmd + rng.normal(0.0, noise_volts))
            speeds.append(omega + rng.normal(0.0, noise_speed))
            accels.append(alpha)
            positions.append(state["theta"])
            currents.append(max(0.0, (v_cmd - ks_true) / max(axis.motor.resistance, 1e-6)))
        return t0 + int(seconds / dt) * dt

    # 1. slow voltage ramp for kS (quasi-static)
    t = 0.0
    for v in np.arange(0.0, 2.0, 0.1):
        t = run(float(v), 0.25, t)
    # 2. steady voltages for kV
    for v in (2.0, 3.0, 4.0, 5.0, 6.0, 7.0):
        t = run(float(v), 0.5, t)
    # 3. voltage steps for kA
    for v in (3.0, 6.0, 9.0, 6.0, 3.0):
        t = run(float(v), 0.3, t)

    return {
        "t": np.array(times),
        "voltage": np.array(volts),
        "velocity": np.array(speeds),
        "position": np.array(positions),
        "current": np.array(currents),
        "kg_true": kg_true,
        "truth": {"kS": ks_true, "kV": kv_true, "kA": ka_true, "kG": kg_true},
        "j_eff": j,
        "force_per_volt": force_per_volt,
    }
