"""Mechanism models: turn a robot.json into numbers a simulator can integrate.

Each mechanism type knows which parameters it needs and how to build its inertia
and gravity load. Nothing here reads CAD directly: `pidtool.ingest` produces
robot.json, this module consumes it.

Every parameter in robot.json carries a `source` tag (cad | bom | datasheet |
estimated). Models keep that provenance so the report can say where each number
came from.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

from .feedforward import (
    DEFAULT_FRICTION,
    FrictionModel,
    Gains,
    arm_gains,
    linear_gains,
    rotary_gains,
)
from .motor import Gearbox, Motor, disc_inertia, plate_inertia, reflected_inertia, ring_inertia


# ----------------------------------------------------------------- data model

@dataclass
class RotaryAxis:
    """A single-DOF rotary mechanism: flywheel, turret, hood, arm."""

    name: str
    motor: Motor
    gearbox: Gearbox
    n_motors: int
    load_inertia: float
    gains: Gains
    friction_torque: float
    limits: tuple[float, float] | None = None      # hard stops [rad]
    gravity_torque_fn: Callable[[float], float] | None = None
    target: float = 0.0
    current_limit_a: float | None = None           # controller current limit, per motor
    r_batt_ohm: float = 0.03
    params: dict = field(default_factory=dict)

    @property
    def j_eff(self) -> float:
        return self.load_inertia + reflected_inertia(self.motor, self.gearbox, self.n_motors)

    @property
    def free_speed(self) -> float:
        return self.motor.free_speed_rad_s / self.gearbox.ratio

    @property
    def max_torque(self) -> float:
        return self.n_motors * self.gearbox.efficiency * self.gearbox.ratio * self.motor.stall_torque_at

    def gravity_torque(self, theta: float) -> float:
        if self.gravity_torque_fn is None:
            return 0.0
        return self.gravity_torque_fn(theta)


@dataclass
class DriveModel:
    """Mecanum drivetrain reduced to a 2-D rigid body with four motor pairs."""

    name: str
    motor: Motor
    gearbox: Gearbox
    mass: float
    wheel_radius: float
    track_width: float | None
    gains_forward: Gains
    gains_strafe: Gains
    mu_rolling: float
    strafe_efficiency: float
    current_limit_a: float | None = None
    r_batt_ohm: float = 0.03
    params: dict = field(default_factory=dict)

    @property
    def max_force(self) -> float:
        tau = self.gearbox.efficiency * self.gearbox.ratio * self.motor.stall_torque_at
        return 4.0 * tau / self.wheel_radius

    @property
    def rolling_force(self) -> float:
        return self.mu_rolling * self.mass * 9.80665

    @property
    def free_speed_forward(self) -> float:
        return self.motor.free_speed_rad_s / self.gearbox.ratio * self.wheel_radius

    @property
    def free_speed_strafe(self) -> float:
        return self.free_speed_forward * self.strafe_efficiency


@dataclass
class RobotModel:
    name: str
    mass: float
    drive: DriveModel | None
    rotary: dict[str, RotaryAxis]
    servo_axes: dict[str, dict]
    provenance: dict


# --------------------------------------------------------------- constructors

def _gearbox(params: dict) -> Gearbox:
    g = params["gearing"]
    return Gearbox(
        ratio=float(g["ratio"]),
        efficiency=float(g.get("efficiency", 0.85)),
        source=str(g.get("source", "assumed")),
    )


def _friction(params: dict) -> FrictionModel:
    f = params.get("friction", {})
    base = DEFAULT_FRICTION
    return FrictionModel(
        mu_bearing=float(f.get("mu_bearing", base.mu_bearing)),
        preload_n=float(f.get("preload_n", base.preload_n)),
        n_bearings=int(f.get("n_bearings", base.n_bearings)),
        bearing_radius_m=float(f.get("bearing_radius_m", base.bearing_radius_m)),
        mu_belt=float(f.get("mu_belt", base.mu_belt)),
        belt_preload_n=float(f.get("belt_preload_n", base.belt_preload_n)),
        pulley_radius_m=float(f.get("pulley_radius_m", 0.0)),
        n_belts=int(f.get("n_belts", 0)),
        seal_drag_nm=float(f.get("seal_drag_nm", base.seal_drag_nm)),
        extra_nm=float(f.get("extra_nm", 0.0)),
    )


def build_rotary(name: str, params: dict, motors: dict[str, Motor]) -> RotaryAxis:
    """Build a rotary axis from a robot.json mechanism block."""
    motor = motors[params["motor"]]
    gb = _gearbox(params)
    n = int(params.get("count", 1))
    inertia = float(params["inertia_kgm2"])
    fr = _friction(params)
    gains = rotary_gains(
        name, motor, gb, inertia,
        n_motors=n,
        friction=fr,
        extra_notes=tuple(params.get("notes", ())),
    )
    tau_f = fr.torque_at_output(gb.ratio)
    limits = params.get("limits_rad")
    return RotaryAxis(
        name=name,
        motor=motor,
        gearbox=gb,
        n_motors=n,
        load_inertia=inertia,
        gains=gains,
        friction_torque=tau_f,
        limits=tuple(limits) if limits else None,
        current_limit_a=params.get("current_limit_a"),
        r_batt_ohm=float(params.get("r_batt_ohm", 0.03)),
        params=params,
    )


def build_arm(name: str, params: dict, motors: dict[str, Motor]) -> RotaryAxis:
    """4-bar / pivot arm: rotary gains plus kG at the horizontal pose."""
    motor = motors[params["motor"]]
    gb = _gearbox(params)
    n = int(params.get("count", 1))
    inertia = float(params["inertia_kgm2"])
    mass = float(params["arm_mass_kg"])
    r_com = float(params["r_com_m"])
    fr = _friction(params)
    gains = arm_gains(
        name, motor, gb, mass, r_com, inertia,
        n_motors=n, friction=fr,
        extra_notes=tuple(params.get("notes", ())),
    )
    tau_f = fr.torque_at_output(gb.ratio)

    def gravity(theta: float) -> float:
        # Arm angle measured from horizontal: torque = m g r cos(theta).
        return mass * 9.80665 * r_com * math.cos(theta)

    limits = params.get("limits_rad")
    return RotaryAxis(
        name=name,
        motor=motor,
        gearbox=gb,
        n_motors=n,
        load_inertia=inertia,
        gains=gains,
        friction_torque=tau_f,
        limits=tuple(limits) if limits else None,
        gravity_torque_fn=gravity,
        current_limit_a=params.get("current_limit_a"),
        r_batt_ohm=float(params.get("r_batt_ohm", 0.03)),
        params=params,
    )


def build_drive(name: str, params: dict, motors: dict[str, Motor]) -> DriveModel:
    motor = motors[params["motor"]]
    gb = _gearbox(params)
    mass = float(params["robot_mass_kg"])
    r = float(params["wheel_radius_m"])
    fr = _friction(params)
    g_forward = linear_gains(
        f"{name}.forward", motor, gb, mass, r,
        n_motors=4, friction=fr,
        extra_notes=("kV/kA per m/s of chassis motion; all four motors share the load",),
    )

    # Strafe costs speed through roller slip: force is still available, but the
    # gearbox sees more current per unit of chassis velocity. Model it as an
    # efficiency on the achievable velocity, and carry the factor into kA/kV so
    # the gains stay consistent with what the robot actually does.
    strafe_eff = float(params.get("strafe_efficiency", 0.78))
    g_strafe = linear_gains(
        f"{name}.strafe", motor, gb, mass, r,
        n_motors=4, friction=fr,
        extra_notes=(f"strafe uses roller slip factor {strafe_eff:.2f} on achievable velocity",),
    )
    g_strafe = Gains(
        mechanism=g_strafe.mechanism,
        kS=g_strafe.kS,
        kV=g_strafe.kV / strafe_eff,
        kA=g_strafe.kA / strafe_eff,
        kG=0.0,
        units=g_strafe.units,
        saturation_voltage=g_strafe.saturation_voltage,
        basis=dict(g_strafe.basis, strafe_efficiency=strafe_eff),
        notes=g_strafe.notes,
    )
    return DriveModel(
        name=name,
        motor=motor,
        gearbox=gb,
        mass=mass,
        wheel_radius=r,
        track_width=params.get("track_width_m"),
        gains_forward=g_forward,
        gains_strafe=g_strafe,
        mu_rolling=float(params.get("mu_rolling", 0.035)),
        strafe_efficiency=strafe_eff,
        current_limit_a=params.get("current_limit_a"),
        r_batt_ohm=float(params.get("r_batt_ohm", 0.03)),
        params=params,
    )


# ------------------------------------------------------------ whole-robot glue

def build_robot(robot: dict, motors: dict[str, Motor]) -> RobotModel:
    """Build every mechanism in a robot.json document."""
    drive = None
    rotary: dict[str, RotaryAxis] = {}
    servo_axes: dict[str, dict] = {}
    for mech in robot.get("mechanisms", []):
        kind = mech.get("type")
        name = mech["name"]
        if kind == "mecanum_drive":
            drive = build_drive(name, mech, motors)
        elif kind == "servo_axis":
            servo_axes[name] = mech
        elif kind == "arm":
            rotary[name] = build_arm(name, mech, motors)
        elif kind in ("flywheel", "turret", "hood", "elevator", "rotary"):
            rotary[name] = build_rotary(name, mech, motors)
        else:
            raise ValueError(f"unknown mechanism type {kind!r} for {name!r}")
    return RobotModel(
        name=robot.get("name", "robot"),
        mass=float(robot.get("mass_kg", 0.0)),
        drive=drive,
        rotary=rotary,
        servo_axes=servo_axes,
        provenance=robot.get("provenance", {}),
    )


# --------------------------------------------------------- inertia estimators

def flywheel_inertia(mass: float, r_outer: float, r_inner: float = 0.0) -> float:
    """Inertia of a flywheel disc, solid when r_inner == 0."""
    if r_inner <= 0.0:
        return disc_inertia(mass, r_outer)
    return ring_inertia(mass, r_inner, r_outer)


def turret_inertia(plate_mass: float, plate_size: tuple[float, float], extra_mass: float, extra_r: float) -> float:
    """Turret plate about its centre plus everything bolted to it at radius extra_r."""
    return plate_inertia(plate_mass, *plate_size) + extra_mass * extra_r**2


def arm_inertia_4bar(mass: float, length: float) -> float:
    """Four-bar arm approximated as a rod about one end (conservative)."""
    return mass * length**2 / 3.0
