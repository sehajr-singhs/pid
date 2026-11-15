"""Analytic feedforward: kS, kV, kA, kG for every mechanism type.

Formulas (brief, section "Physics"), written out with units:

    kV = G * k_e                        [V per output rad/s]
         = G * k_e / r                  [V per m/s]          (drivetrain)
    kA = J_eff * R / (n * eta * G * k_t)                    [V per output rad/s^2]
         = m * r * R / (n * eta * G * k_t)                  [V per m/s^2]
    kG = m * g * r_com * R / (n * eta * G * k_t)            [V]  (arm at horizontal)
         = m * g * r_spool * R / (n * eta * G * k_t)        [V]  (elevator)
    kS = tau_friction * R / (n * eta * G * k_t)             [V]  (model, FLAGGED)

`n` is the number of motors sharing the load. The brief writes the single-motor
form; n generalises it and reduces to the brief's formula at n = 1.

J_eff is the output-shaft inertia including reflected rotor inertia:

    J_eff = J_load + n * G^2 * J_rotor

Why kA has R/(eta*G*k_t): acceleration torque is (J_eff*alpha); the current that
produces it is tau/(eta*G*k_t); and current needs V = I*R of headroom.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .motor import Gearbox, Motor, reflected_inertia

GRAVITY = 9.80665

# ---------------------------------------------------------------- unit helpers

def v_per_rad_s_to_v_per_rpm(v: float) -> float:
    return v * (2.0 * math.pi / 60.0)


def v_per_rad_s_to_v_per_deg_s(v: float) -> float:
    return v * math.pi / 180.0


def v_per_m_s_to_v_per_in_s(v: float) -> float:
    return v * 0.0254


# ------------------------------------------------------------------- friction

@dataclass(frozen=True)
class FrictionModel:
    """Static-friction estimate used only to produce a STARTING kS.

    kS cannot be derived from CAD: it depends on bearing drag, belt preload and
    how tight the build is. This model is deliberately simple and every term is
    an explicit assumption. The result is always flagged as
    "to be measured by SpoolCalibrate".

    bearing_friction: mu_bearing * preload_n * n_bearings * bearing_radius
    belt_preload:     mu_belt * preload_n * pulley_radius         (per belt)
    seal_drag:        constant N*m at the output
    """

    mu_bearing: float = 0.0015          # deep-groove ball bearing, greased
    preload_n: float = 25.0             # axial preload from hubs/shaft collars
    n_bearings: int = 2
    bearing_radius_m: float = 0.007     # 8mm shaft -> ~7mm mean race radius
    mu_belt: float = 0.02
    belt_preload_n: float = 40.0
    pulley_radius_m: float = 0.0
    n_belts: int = 0
    seal_drag_nm: float = 0.002
    extra_nm: float = 0.0
    assumptions: tuple[str, ...] = (
        "mu_bearing=0.0015, preload 25 N per bearing pair",
        "belt preload 40 N with mu=0.02 when a belt is present",
        "seal/drag 2 mN*m constant",
    )

    def torque_at_output(self, gear_ratio: float = 1.0) -> float:
        """Friction torque referred to the output shaft [N*m].

        Bearing drag sits on the motor-side of the reduction for a gearmotor, so
        it is amplified by the gear ratio; belt drag already sits at the output.
        """
        bearing = self.mu_bearing * self.preload_n * self.n_bearings * self.bearing_radius_m
        belt = self.mu_belt * self.belt_preload_n * self.pulley_radius_m * self.n_belts
        return (bearing * gear_ratio) + belt + self.seal_drag_nm + self.extra_nm


DEFAULT_FRICTION = FrictionModel()


# ---------------------------------------------------------------------- gains

@dataclass(frozen=True)
class Gains:
    """One mechanism's feedforward gains, in SI-first units."""

    mechanism: str
    kS: float
    kV: float
    kA: float
    kG: float
    units: str                      # "V, V/(rad/s), V/(rad/s^2)" etc.
    saturation_voltage: float
    basis: dict = field(default_factory=dict)
    notes: tuple[str, ...] = ()

    @property
    def kS_measured(self) -> bool:
        return bool(self.basis.get("kS_measured", False))

    def as_dict(self) -> dict:
        out = {
            "mechanism": self.mechanism,
            "kS": self.kS,
            "kV": self.kV,
            "kA": self.kA,
            "kG": self.kG,
            "units": self.units,
            "saturation_voltage": self.saturation_voltage,
            "kS_source": "measured" if self.kS_measured else "model (flagged: measure with SpoolCalibrate)",
            "notes": list(self.notes),
            "basis": self.basis,
        }
        return out

    def describe(self) -> str:
        return (
            f"kS={self.kS:.4f} V  kV={self.kV:.6f}  kA={self.kA:.6f}  "
            f"kG={self.kG:.4f} V  [{self.units}]"
        )


def _validate(motor: Motor, gearbox: Gearbox, n_motors: int) -> None:
    if n_motors < 1:
        raise ValueError("n_motors must be >= 1")
    if gearbox.ratio <= 0:
        raise ValueError("gearbox ratio must be > 0")
    if not (0.0 < gearbox.efficiency <= 1.0):
        raise ValueError("gearbox efficiency must be in (0, 1]")
    if motor.stall_current_a <= 0 or motor.free_speed_rpm <= 0:
        raise ValueError("motor datasheet values must be positive")


def _torque_per_volt(motor: Motor, gearbox: Gearbox, n_motors: int) -> float:
    """Output torque produced per volt of extra applied voltage [N*m/V].

    tau_out = n * eta * G * k_t * I  and  I = V/R  ->  n*eta*G*k_t/R.
    """
    return n_motors * gearbox.efficiency * gearbox.ratio * motor.kt / motor.resistance


def ks_from_friction(
    motor: Motor,
    gearbox: Gearbox,
    friction_torque: float,
    n_motors: int = 1,
    include_motor_free_run: bool = True,
) -> float:
    """kS [V] = tau_friction * R / (n * eta * G * k_t).

    The motor's own free-run friction (tau = k_t * I_free at the motor shaft) is
    part of the friction the voltage has to overcome, so it is included by
    default. That alone contributes I_free*R volts (~0.33 V for a goBILDA RS-555),
    which matches the unstick voltage teams measure on a bare motor.
    """
    _validate(motor, gearbox, n_motors)
    tau_total = friction_torque
    if include_motor_free_run:
        tau_total += n_motors * gearbox.efficiency * gearbox.ratio * motor.kt * motor.free_current_a
    return tau_total / _torque_per_volt(motor, gearbox, n_motors)


def rotary_gains(
    name: str,
    motor: Motor,
    gearbox: Gearbox,
    load_inertia: float,
    *,
    n_motors: int = 1,
    friction: FrictionModel = DEFAULT_FRICTION,
    friction_torque: float | None = None,
    kS_measured: float | None = None,
    extra_notes: tuple[str, ...] = (),
) -> Gains:
    """Gains for a rotary mechanism: flywheel, turret, hood, arm (kG added later)."""
    _validate(motor, gearbox, n_motors)
    tau_f = friction.torque_at_output(gearbox.ratio) if friction_torque is None else friction_torque
    rotor = reflected_inertia(motor, gearbox, n_motors)
    j_eff = load_inertia + rotor
    kv = gearbox.ratio * motor.ke
    ka = j_eff * motor.resistance / (n_motors * gearbox.efficiency * gearbox.ratio * motor.kt)
    ks = ks_from_friction(motor, gearbox, tau_f, n_motors)
    notes = list(extra_notes)
    if kS_measured is not None:
        ks = kS_measured
    else:
        notes.append("kS is a friction model, not CAD: measure it with SpoolCalibrate")
    return Gains(
        mechanism=name,
        kS=ks,
        kV=kv,
        kA=ka,
        kG=0.0,
        units="V, V/(rad/s), V/(rad/s^2), V",
        saturation_voltage=motor.v_nom,
        basis={
            "motor": motor.key,
            "gear_ratio": gearbox.ratio,
            "efficiency": gearbox.efficiency,
            "n_motors": n_motors,
            "load_inertia": load_inertia,
            "reflected_rotor_inertia": rotor,
            "j_eff": j_eff,
            "friction_torque": tau_f,
            "kS_measured": kS_measured is not None,
            "torque_per_volt": _torque_per_volt(motor, gearbox, n_motors),
            "free_output_speed_rad_s": motor.free_speed_rad_s / gearbox.ratio,
        },
        notes=tuple(notes),
    )


def linear_gains(
    name: str,
    motor: Motor,
    gearbox: Gearbox,
    mass: float,
    wheel_radius: float,
    *,
    n_motors: int = 1,
    friction: FrictionModel = DEFAULT_FRICTION,
    friction_force: float | None = None,
    kS_measured: float | None = None,
    extra_notes: tuple[str, ...] = (),
) -> Gains:
    """Gains for a linear mechanism (drivetrain, elevator rack)."""
    _validate(motor, gearbox, n_motors)
    if wheel_radius <= 0:
        raise ValueError("wheel_radius must be > 0")
    if friction_force is None:
        tau_f = friction.torque_at_output(gearbox.ratio)
        friction_force = tau_f / wheel_radius
    kv = gearbox.ratio * motor.ke / wheel_radius
    ka = mass * wheel_radius * motor.resistance / (
        n_motors * gearbox.efficiency * gearbox.ratio * motor.kt
    )
    ks = ks_from_friction(motor, gearbox, friction_force * wheel_radius, n_motors)
    if kS_measured is not None:
        ks = kS_measured
    return Gains(
        mechanism=name,
        kS=ks,
        kV=kv,
        kA=ka,
        kG=0.0,
        units="V, V/(m/s), V/(m/s^2), V",
        saturation_voltage=motor.v_nom,
        basis={
            "motor": motor.key,
            "gear_ratio": gearbox.ratio,
            "efficiency": gearbox.efficiency,
            "n_motors": n_motors,
            "mass_kg": mass,
            "wheel_radius_m": wheel_radius,
            "friction_force_n": friction_force,
            "kS_measured": kS_measured is not None,
            "free_output_speed_m_s": motor.free_speed_rad_s / gearbox.ratio * wheel_radius,
        },
        notes=tuple(extra_notes),
    )


def gravity_gain(
    motor: Motor,
    gearbox: Gearbox,
    mass: float,
    moment_arm: float,
    n_motors: int = 1,
) -> float:
    """kG [V] = m*g*r*R / (n*eta*G*k_t). At the arm's horizontal pose."""
    _validate(motor, gearbox, n_motors)
    return mass * GRAVITY * moment_arm / _torque_per_volt(motor, gearbox, n_motors)


def arm_gains(
    name: str,
    motor: Motor,
    gearbox: Gearbox,
    mass: float,
    r_com: float,
    load_inertia: float,
    *,
    n_motors: int = 1,
    friction: FrictionModel = DEFAULT_FRICTION,
    kS_measured: float | None = None,
    extra_notes: tuple[str, ...] = (),
) -> Gains:
    """Rotary gains plus kG for an arm pivoting about a horizontal axis."""
    base = rotary_gains(
        name,
        motor,
        gearbox,
        load_inertia,
        n_motors=n_motors,
        friction=friction,
        kS_measured=kS_measured,
        extra_notes=extra_notes,
    )
    kg = gravity_gain(motor, gearbox, mass, r_com, n_motors)
    basis = dict(base.basis)
    basis.update({"mass_kg": mass, "r_com_m": r_com, "kG_pose": "horizontal (cos(theta) at runtime)"})
    return Gains(
        mechanism=base.mechanism,
        kS=base.kS,
        kV=base.kV,
        kA=base.kA,
        kG=kg,
        units=base.units,
        saturation_voltage=base.saturation_voltage,
        basis=basis,
        notes=base.notes,
    )


def elevator_gains(
    name: str,
    motor: Motor,
    gearbox: Gearbox,
    mass: float,
    spool_radius: float,
    *,
    n_motors: int = 1,
    friction: FrictionModel = DEFAULT_FRICTION,
    kS_measured: float | None = None,
) -> Gains:
    """Linear gains plus kG for an elevator/rope spool: kG = m g r R / (n eta G kt)."""
    base = linear_gains(
        name, motor, gearbox, mass, spool_radius, n_motors=n_motors, friction=friction,
        kS_measured=kS_measured,
    )
    kg = gravity_gain(motor, gearbox, mass, spool_radius, n_motors)
    basis = dict(base.basis)
    basis.update({"mass_kg": mass, "spool_radius_m": spool_radius})
    return Gains(
        mechanism=base.mechanism,
        kS=base.kS,
        kV=base.kV,
        kA=base.kA,
        kG=kg,
        units=base.units,
        saturation_voltage=base.saturation_voltage,
        basis=basis,
        notes=base.notes,
    )


def achievable_output_speed(motor: Motor, gearbox: Gearbox, fraction: float = 1.0) -> float:
    """Free output speed scaled by a fraction (e.g. roller-slip for strafing)."""
    return fraction * motor.free_speed_rad_s / gearbox.ratio


def voltage_at_state(gains: Gains, velocity: float, acceleration: float, gravity: float = 0.0) -> float:
    """Feedforward voltage for a (velocity, acceleration, gravity) operating point."""
    v = gains.kV * velocity + gains.kA * acceleration + gains.kG * gravity
    if velocity > 1e-9:
        v += gains.kS
    elif velocity < -1e-9:
        v -= gains.kS
    return v


def steady_state_speed(gains: Gains, voltage: float) -> float:
    """Speed reached at a constant applied voltage, ignoring acceleration and gravity."""
    if abs(voltage) <= gains.kS:
        return 0.0
    return (abs(voltage) - gains.kS) / abs(gains.kV) * (1.0 if voltage > 0 else -1.0)
