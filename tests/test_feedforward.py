"""Feedforward formulas, checked by hand against the brief's equations.

The hand case uses deliberately round numbers so the expected values are worked
out independently of the implementation:

    motor : 12 V, 6000 rpm free, 0.25 A free, 9.2 A stall, 0.1442 N m stall
    R     = 12/9.2                  = 1.3043478 ohm
    kt    = 0.1442/9.2              = 0.0156739 N m/A
    ke    = (12 - 0.25*R)/628.3185  = 0.0185796 V s/rad
    rotor J = 7e-6 kg m^2, gearbox G = 10, eta = 1, load J = 0.001
    J_eff = 0.001 + 10^2*7e-6       = 0.0017 kg m^2   (reflected rotor included)

    kV = G*ke                       = 0.185796
    kA = J_eff*R/(G*kt)             = 0.0141470
    kG = m*g*r*R/(G*kt)  (m=2, r=0.1) = 16.32175

Forgetting the G^2 term is the classic way to get kA wrong by an order of
magnitude, so the reflected rotor inertia is spelled out on purpose.
"""

import math

import pytest

from pidtool.physics.feedforward import (
    FrictionModel,
    arm_gains,
    elevator_gains,
    ks_from_friction,
    linear_gains,
    rotary_gains,
    steady_state_speed,
    voltage_at_state,
)
from pidtool.physics.motor import Gearbox, Motor

R_OHM = 12.0 / 9.2
KT = 0.1442 / 9.2
KE = (12.0 - 0.25 * R_OHM) / (6000.0 * 2 * math.pi / 60.0)


@pytest.fixture()
def motor():
    return Motor.from_datasheet(
        key="hand",
        name="hand-checked motor",
        v_nom=12.0,
        free_speed_rpm=6000.0,
        free_current_a=0.25,
        stall_current_a=9.2,
        stall_torque_kgcm=0.1442 / 0.0980665,
        rotor_inertia_kgm2=7e-6,
    )


@pytest.fixture()
def gb():
    return Gearbox(ratio=10.0, efficiency=1.0)


def test_kv_is_ratio_times_ke(motor, gb):
    g = rotary_gains("x", motor, gb, load_inertia=0.001, n_motors=1)
    assert g.kV == pytest.approx(10.0 * KE, rel=1e-9)
    assert g.kV == pytest.approx(0.185796, rel=1e-5)


def test_ka_matches_hand_calculation(motor, gb):
    g = rotary_gains("x", motor, gb, load_inertia=0.001, n_motors=1)
    j_eff = 0.001 + 10.0**2 * 7e-6
    expected = j_eff * R_OHM / (10.0 * KT)
    assert g.kA == pytest.approx(expected, rel=1e-9)
    assert g.kA == pytest.approx(0.0141470, rel=1e-4)


def test_ka_scales_with_reflected_rotor(motor, gb):
    """kA must use J_eff = J_load + G^2*J_rotor, not J_load alone."""
    g = rotary_gains("x", motor, gb, load_inertia=0.001, n_motors=1)
    j_eff = 0.001 + 10.0**2 * motor.rotor_inertia_kgm2
    assert g.basis["j_eff"] == pytest.approx(j_eff)
    assert g.kA == pytest.approx(j_eff * R_OHM / (10.0 * KT), rel=1e-9)


def test_ka_with_multiple_motors_shares_the_work(motor, gb):
    one = rotary_gains("x", motor, gb, load_inertia=0.001, n_motors=1)
    two = rotary_gains("x", motor, gb, load_inertia=0.001, n_motors=2)
    # twice the motors, twice the reflected inertia, half the kA contribution each
    assert two.kA < one.kA


def test_kg_hand_calculation(motor, gb):
    g = arm_gains("arm", motor, gb, mass=2.0, r_com=0.1, load_inertia=0.001, n_motors=1)
    expected = 2.0 * 9.80665 * 0.1 * R_OHM / (10.0 * KT)
    assert g.kG == pytest.approx(expected, rel=1e-9)
    assert g.kG == pytest.approx(16.32175, rel=1e-5)


def test_elevator_kg_uses_spool_radius(motor, gb):
    g = elevator_gains("lift", motor, gb, mass=2.0, spool_radius=0.1, n_motors=1)
    assert g.kG == pytest.approx(2.0 * 9.80665 * 0.1 * R_OHM / (10.0 * KT), rel=1e-9)


def test_linear_gains_units(motor, gb):
    g = linear_gains("drive", motor, gb, mass=10.0, wheel_radius=0.05, n_motors=4)
    assert g.kV == pytest.approx(10.0 * KE / 0.05, rel=1e-9)
    ka = 10.0 * 0.05 * R_OHM / (4 * 1.0 * 10.0 * KT)
    assert g.kA == pytest.approx(ka, rel=1e-9)


def test_ks_is_free_run_current_times_resistance(motor, gb):
    """With no mechanism friction, kS reduces to I_free*R -- the unstick voltage."""
    ks = ks_from_friction(motor, gb, friction_torque=0.0, n_motors=1)
    assert ks == pytest.approx(0.25 * R_OHM, rel=1e-9)
    assert ks == pytest.approx(0.32609, rel=1e-4)


def test_ks_without_motor_free_run():
    motor = Motor.from_datasheet(
        key="m", name="m", free_speed_rpm=6000.0, free_current_a=0.25,
        stall_current_a=9.2, stall_torque_kgcm=0.1442 / 0.0980665, rotor_inertia_kgm2=7e-6,
    )
    ks = ks_from_friction(motor, Gearbox(10.0, 1.0), 0.0, include_motor_free_run=False)
    assert ks == pytest.approx(0.0, abs=1e-12)


def test_friction_model_is_flagged_not_measured(motor, gb):
    g = rotary_gains("x", motor, gb, load_inertia=0.001, friction=FrictionModel())
    assert g.basis["kS_measured"] is False
    assert any("measure" in n.lower() for n in g.notes)


def test_measured_ks_overrides_model(motor, gb):
    g = rotary_gains("x", motor, gb, load_inertia=0.001, kS_measured=0.42)
    assert g.kS == pytest.approx(0.42)
    assert g.basis["kS_measured"] is True


def test_voltage_at_state_and_steady_speed(motor, gb):
    g = rotary_gains("x", motor, gb, load_inertia=0.001)
    v = voltage_at_state(g, velocity=1.0, acceleration=0.0)
    assert v == pytest.approx(g.kS + g.kV, rel=1e-12)
    assert steady_state_speed(g, g.kS + g.kV) == pytest.approx(1.0, rel=1e-9)
    assert steady_state_speed(g, g.kS * 0.5) == 0.0


def test_bad_inputs(motor):
    with pytest.raises(ValueError):
        rotary_gains("x", motor, Gearbox(ratio=0.0), 0.001)
    with pytest.raises(ValueError):
        rotary_gains("x", motor, Gearbox(ratio=1.0), 0.001, n_motors=0)
    with pytest.raises(ValueError):
        linear_gains("d", motor, Gearbox(ratio=1.0), mass=1.0, wheel_radius=0.0)
