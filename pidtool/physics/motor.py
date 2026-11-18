"""Brushed DC motor + gearbox model.

All quantities SI: volts, amps, N*m, rad/s, kg*m^2, ohms.

Datasheet identities (brief, section "Physics"):

    V = I*R + k_e*w          (electrical)
    tau = k_t*I              (torque)
    R   = V_nom / I_stall
    k_t = tau_stall / I_stall
    k_e = (V_nom - I_free*R) / w_free

Through a gearbox of ratio G (motor turns per output turn) and efficiency eta:

    tau_out = eta*G*tau_m
    w_m     = G*w_out

Note on k_t vs k_e: for a real motor in SI they are numerically equal. Datasheet
numbers are rounded and the no-load speed already contains friction, so the two
estimates differ by a few percent. We keep both: k_e is what sets volts-per-speed
(kV), k_t is what sets volts-per-torque (kA, kG). `kt_ke_mismatch` reports the
discrepancy so it is visible rather than hidden.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

RPM_TO_RAD_S = 2.0 * math.pi / 60.0
KGF_CM_TO_NM = 0.0980665


@dataclass(frozen=True)
class Motor:
    """A brushed DC motor, described only by datasheet-style numbers."""

    key: str
    name: str
    v_nom: float
    free_speed_rpm: float
    free_current_a: float
    stall_current_a: float
    stall_torque_nm: float
    rotor_inertia_kgm2: float
    source: str = ""
    verified: bool = True
    notes: str = ""

    # ---- datasheet identities -------------------------------------------------
    @property
    def free_speed_rad_s(self) -> float:
        return self.free_speed_rpm * RPM_TO_RAD_S

    @property
    def resistance(self) -> float:
        """R = V_nom / I_stall."""
        return self.v_nom / self.stall_current_a

    @property
    def kt(self) -> float:
        """Torque constant k_t = tau_stall / I_stall  [N*m/A]."""
        return self.stall_torque_nm / self.stall_current_a

    @property
    def ke(self) -> float:
        """Back-EMF constant k_e = (V_nom - I_free*R) / w_free  [V*s/rad]."""
        return (self.v_nom - self.free_current_a * self.resistance) / self.free_speed_rad_s

    @property
    def kt_ke_mismatch(self) -> float:
        """Relative disagreement between k_e and k_t (0.0 means perfect datasheet)."""
        return self.ke / self.kt - 1.0

    @property
    def stall_torque_at(self) -> float:
        return self.v_nom * self.kt / self.resistance

    @property
    def viscous_damping(self) -> float:
        """Motor-side damping b = k_t*I_free / w_free  [N*m*s/rad].

        Free-run current is exactly the torque needed to hold free speed against
        friction; treating it as viscous friction gives the linear term used by
        the feedforward model. Coulomb friction is handled separately by kS.
        """
        return self.kt * self.free_current_a / self.free_speed_rad_s

    @classmethod
    def from_datasheet(
        cls,
        key: str,
        name: str,
        *,
        v_nom: float = 12.0,
        free_speed_rpm: float,
        free_current_a: float,
        stall_current_a: float,
        stall_torque_kgcm: float,
        rotor_inertia_kgm2: float,
        source: str = "",
        verified: bool = True,
        notes: str = "",
    ) -> "Motor":
        return cls(
            key=key,
            name=name,
            v_nom=v_nom,
            free_speed_rpm=free_speed_rpm,
            free_current_a=free_current_a,
            stall_current_a=stall_current_a,
            stall_torque_nm=stall_torque_kgcm * KGF_CM_TO_NM,
            rotor_inertia_kgm2=rotor_inertia_kgm2,
            source=source,
            verified=verified,
            notes=notes,
        )

    def current_for_torque(self, tau_motor: float) -> float:
        """Motor current needed for a motor-side torque [A]."""
        return tau_motor / self.kt

    def voltage_for_speed(self, w_motor: float, tau_motor: float = 0.0) -> float:
        """Voltage needed to hold motor speed w at motor torque tau (steady state)."""
        return self.ke * w_motor + self.current_for_torque(tau_motor) * self.resistance

    def to_dict(self) -> dict:
        return {
            "key": self.key,
            "name": self.name,
            "v_nom": self.v_nom,
            "free_speed_rpm": self.free_speed_rpm,
            "free_current_a": self.free_current_a,
            "stall_current_a": self.stall_current_a,
            "stall_torque_kgcm": self.stall_torque_nm / KGF_CM_TO_NM,
            "rotor_inertia_kgm2": self.rotor_inertia_kgm2,
            "source": self.source,
            "verified": self.verified,
            "notes": self.notes,
            "derived": {
                "resistance_ohm": self.resistance,
                "kt_nm_per_a": self.kt,
                "ke_v_s_per_rad": self.ke,
                "free_speed_rad_s": self.free_speed_rad_s,
                "kt_ke_mismatch": self.kt_ke_mismatch,
                "viscous_damping": self.viscous_damping,
            },
        }


@dataclass(frozen=True)
class Gearbox:
    """Ratio G = motor turns per output turn (G > 1 is a reduction)."""

    ratio: float
    efficiency: float = 0.85
    source: str = "assumed"

    def output_speed(self, motor_speed: float) -> float:
        return motor_speed / self.ratio

    def output_torque(self, motor_torque: float) -> float:
        return self.efficiency * self.ratio * motor_torque


def reflected_inertia(motor: Motor, gearbox: Gearbox, n_motors: int = 1) -> float:
    """Rotor inertia reflected to the output shaft: n*G^2*J_rotor [kg*m^2].

    Inertia reflects through a reduction by G^2. Gearbox inertia is neglected
    (it is an order of magnitude below the reflected rotor for FTC gearboxes).
    """
    return n_motors * gearbox.ratio**2 * motor.rotor_inertia_kgm2


def disc_inertia(mass: float, radius: float) -> float:
    """Solid disc about its axis: J = 1/2 m r^2."""
    return 0.5 * mass * radius**2


def ring_inertia(mass: float, r_inner: float, r_outer: float) -> float:
    """Thick ring/annulus about its axis: J = 1/2 m (r_i^2 + r_o^2)."""
    return 0.5 * mass * (r_inner**2 + r_outer**2)


def rod_about_end_inertia(mass: float, length: float) -> float:
    """Slender rod pivoted at one end: J = 1/3 m L^2."""
    return mass * length**2 / 3.0


def plate_inertia(mass: float, width: float, height: float) -> float:
    """Rectangular plate about the axis normal to it: J = 1/12 m (w^2 + h^2)."""
    return mass * (width**2 + height**2) / 12.0
