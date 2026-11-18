"""The simulator has to reproduce the datasheet, or the tuning table is fiction."""

import math

import numpy as np
import pytest

from pidtool.motors import get_servo, motors as load_motors
from pidtool.physics.mechanisms import build_robot
from pidtool.physics.sim import PID, ServoAxis, simulate_drive, simulate_rotary, simulate_servo


@pytest.fixture(scope="module")
def model():
    doc = {
        "schema_version": 1,
        "name": "test",
        "mass_kg": 12.0,
        "mechanisms": [
            {
                "type": "flywheel",
                "name": "fly",
                "motor": "gobilda-5203-2402-0001",
                "count": 1,
                "gearing": {"ratio": 1.0, "efficiency": 1.0},
                "inertia_kgm2": 1e-4,
                # zero extra friction so the plant must reproduce the datasheet
                "friction": {"n_bearings": 0, "seal_drag_nm": 0.0, "extra_nm": 0.0},
            },
            {
                "type": "mecanum_drive",
                "name": "drive",
                "motor": "gobilda-5203-2402-0001",
                "gearing": {"ratio": 11.44, "efficiency": 0.88},
                "robot_mass_kg": 12.0,
                "wheel_radius_m": 0.048,
                "strafe_efficiency": 0.78,
                "mu_rolling": 0.08,
            },
        ],
    }
    return build_robot(doc, load_motors())


def test_free_speed_matches_datasheet(model):
    axis = model.rotary["fly"]
    ctrl = PID(kp=50.0, ki=0.0, kd=0.0, limit=12.0)
    res = simulate_rotary(axis, ctrl, target=axis.free_speed, mode="velocity",
                          use_feedforward=False, t_end=3.0)
    assert res.velocity[-1] == pytest.approx(axis.motor.free_speed_rad_s, rel=0.01)


def test_zero_command_does_not_move(model):
    axis = model.rotary["fly"]
    ctrl = PID(kp=1.0, ki=0.0, kd=0.0, limit=12.0)
    res = simulate_rotary(axis, ctrl, target=0.0, mode="velocity", t_end=1.0)
    assert np.max(np.abs(res.velocity)) < 1e-9


def test_battery_sag_reduces_voltage(model):
    axis = model.rotary["fly"]
    ctrl = PID(kp=500.0, ki=0.0, kd=0.0, limit=12.0)
    res = simulate_rotary(axis, ctrl, target=axis.free_speed, mode="velocity",
                          use_feedforward=False, t_end=0.05)
    assert np.max(res.voltage) < 12.4
    assert np.max(res.voltage) > 10.0


def test_position_step_settles(model):
    axis = model.rotary["fly"]
    ctrl = PID(kp=50.0, ki=10.0, kd=5.0, limit=12.0)
    res = simulate_rotary(axis, ctrl, target=2.0, mode="position", t_end=2.0)
    assert res.position[-1] == pytest.approx(2.0, abs=0.05)
    m = res.metrics(tolerance=0.05)
    assert math.isfinite(m["settle_time"])
    assert m["settle_time"] <= 2.0
    assert abs(m["steady_state_error"]) < 0.05


def test_hard_limits_are_respected(model):
    axis = model.rotary["fly"]
    axis.limits = (-0.5, 0.5)
    ctrl = PID(kp=800.0, ki=0.0, kd=20.0, limit=12.0)
    res = simulate_rotary(axis, ctrl, target=3.0, mode="position", t_end=1.5)
    assert np.max(res.position) <= 0.5 + 1e-9
    assert np.min(res.position) >= -0.5 - 1e-9


def test_metrics_rise_and_settle_ordering(model):
    axis = model.rotary["fly"]
    ctrl = PID(kp=50.0, ki=0.0, kd=0.0, limit=12.0)
    res = simulate_rotary(axis, ctrl, target=axis.free_speed * 0.5, mode="velocity",
                          use_feedforward=True, t_end=1.5)
    m = res.metrics(tolerance=axis.free_speed * 0.02)
    assert m["rise_time"] >= 0.0
    assert m["settle_time"] >= m["rise_time"]


def test_drive_reaches_requested_speed(model):
    drive = model.drive
    ctrl = PID(kp=20.0, ki=2.0, kd=0.0, limit=12.0)
    target = drive.free_speed_forward * 0.8
    res = simulate_drive(drive, ctrl, target_speed=target, t_end=2.5, use_feedforward=True)
    assert res.velocity[-1] == pytest.approx(target, rel=0.05)


def test_strafe_is_slower_than_forward(model):
    assert model.drive.free_speed_strafe < model.drive.free_speed_forward
    assert model.drive.free_speed_strafe == pytest.approx(
        model.drive.free_speed_forward * 0.78)


def test_servo_follows_a_rate_limit():
    axis = ServoAxis(name="turret", servo=get_servo("swyft-balance"), ratio=2.4, servo_count=2)
    target = math.radians(90.0)
    res = simulate_servo(axis, target=target, t_end=1.0)
    expected = target / axis.output_speed_rad_s
    m = res.metrics(math.radians(1.0))
    assert m["settle_time"] == pytest.approx(expected, rel=0.25)
    assert m["overshoot_pct"] <= 0.0 + 1e-9


def test_servo_ratio_reduces_travel_and_speeds_it_up():
    servo = get_servo("swyft-balance")
    direct = ServoAxis(name="a", servo=servo, ratio=1.0)
    geared = ServoAxis(name="b", servo=servo, ratio=2.4)
    assert geared.output_speed_rad_s < direct.output_speed_rad_s
    assert geared.output_torque_nm > direct.output_torque_nm


def test_sim_is_deterministic(model):
    axis = model.rotary["fly"]
    a = simulate_rotary(axis, PID(kp=30.0, kd=1.0), target=5.0, mode="position", t_end=0.5)
    b = simulate_rotary(axis, PID(kp=30.0, kd=1.0), target=5.0, mode="position", t_end=0.5)
    assert np.array_equal(a.position, b.position)
