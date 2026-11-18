"""Motor model: the datasheet identities have to come out exactly."""

import math

import pytest

from pidtool.motors import get_motor, load_db
from pidtool.physics.motor import Gearbox, Motor, reflected_inertia


@pytest.fixture()
def rs555():
    return get_motor("gobilda-5203-2402-0001")


def test_resistance_from_stall(rs555):
    assert rs555.resistance == pytest.approx(12.0 / 9.2, rel=1e-12)


def test_torque_constant(rs555):
    assert rs555.kt == pytest.approx(0.1442 / 9.2, rel=1e-12)


def test_back_emf_constant(rs555):
    expected = (12.0 - 0.25 * (12.0 / 9.2)) / (6000.0 * 2 * math.pi / 60.0)
    assert rs555.ke == pytest.approx(expected, rel=1e-12)


def test_free_speed_round_trip(rs555):
    """Applying V_nom and drawing I_free must land on the datasheet free speed."""
    v = rs555.ke * rs555.free_speed_rad_s + rs555.free_current_a * rs555.resistance
    assert v == pytest.approx(12.0, rel=1e-12)


def test_stall_current_round_trip(rs555):
    """At stall (w=0) the motor draws V/R amps."""
    assert 12.0 / rs555.resistance == pytest.approx(9.2, rel=1e-12)


def test_kt_ke_are_close_but_not_equal(rs555):
    """Datasheet rounding: the two constants differ by a few percent, not wildly."""
    assert abs(rs555.kt_ke_mismatch) < 0.25


def test_gearbox_speed_torque():
    gb = Gearbox(ratio=10.0, efficiency=0.5)
    assert gb.output_speed(100.0) == pytest.approx(10.0)
    assert gb.output_torque(1.0) == pytest.approx(5.0)


def test_reflected_inertia_scales_with_ratio_squared(rs555):
    gb = Gearbox(ratio=13.73)
    assert reflected_inertia(rs555, gb, n_motors=2) == pytest.approx(
        2 * 13.73**2 * rs555.rotor_inertia_kgm2
    )


def test_rejects_bad_gearbox(rs555):
    with pytest.raises(ValueError):
        # efficiency > 1 is not physical
        Gearbox(ratio=1.0, efficiency=1.5)
        from pidtool.physics.feedforward import rotary_gains

        rotary_gains("x", rs555, Gearbox(ratio=1.0, efficiency=1.5), 0.001)


def test_database_covers_required_motors():
    db = load_db()["motors"]
    for key in (
        "gobilda-5203-2402-0014", "gobilda-5203-2402-0001", "rev-hd-hex", "rev-core-hex",
        "kraken-x60", "falcon-500", "neo", "neo-vortex", "cim",
    ):
        assert key in db, key
    for key, motor in db.items():
        assert motor["source"], f"{key} has no cited source"


def test_all_motors_sane():
    """Every entry must produce finite positive derived constants."""
    for motor in load_db()["motors"].values():
        m = Motor(
            key=motor["sku"], name=motor["name"], v_nom=motor["nominal_voltage_v"],
            free_speed_rpm=motor["free_speed_rpm"], free_current_a=motor["free_current_a"],
            stall_current_a=motor["stall_current_a"], stall_torque_nm=motor["stall_torque_nm"],
            rotor_inertia_kgm2=motor["rotor_inertia_kgm2"],
        )
        assert m.resistance > 0
        assert m.kt > 0
        assert m.ke > 0
        assert m.free_speed_rad_s > 0
