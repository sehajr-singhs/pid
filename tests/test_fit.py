"""Fit recovery on synthetic logs from sim.py, where the truth is known."""

import numpy as np
import pytest

from spool.motors import motors as load_motors
from spool.physics.mechanisms import build_robot
from spool.physics.sim import synthetic_calibration_log
from spool.runtime import fit as fitmod


def _axis():
    doc = {
        "schema_version": 1,
        "name": "test",
        "mass_kg": 1.0,
        "mechanisms": [
            {
                "type": "flywheel",
                "name": "fly",
                "motor": "gobilda-5203-2402-0001",
                "count": 1,
                "gearing": {"ratio": 1.0, "efficiency": 1.0},
                "inertia_kgm2": 1e-4,
                "friction": {"n_bearings": 0, "extra_nm": 0.0, "seal_drag_nm": 0.0},
            }
        ],
    }
    return build_robot(doc, load_motors()).rotary["fly"]


@pytest.fixture(scope="module")
def fitted():
    axis = _axis()
    truth = {
        "kS": 0.35,
        "kV": axis.gains.kV,
        "kA": axis.gains.kA,
    }
    log = synthetic_calibration_log(
        axis, ks_true=truth["kS"], kv_true=truth["kV"], ka_true=truth["kA"], rng_seed=11
    )
    return truth, log, fitmod.fit_feedforward(log)


def test_recovers_ks(fitted):
    truth, _, f = fitted
    assert f.kS == pytest.approx(truth["kS"], rel=0.10)


def test_recovers_kv(fitted):
    truth, _, f = fitted
    assert f.kV == pytest.approx(truth["kV"], rel=0.02)


def test_recovers_ka(fitted):
    truth, _, f = fitted
    assert f.kA == pytest.approx(truth["kA"], rel=0.10)


def test_fit_is_good(fitted):
    _, _, f = fitted
    assert f.r2 > 0.99
    assert f.rmse_volts < 0.05


def test_standard_errors_reported(fitted):
    _, _, f = fitted
    assert set(f.stderr) == {"kS", "kV", "kA"}
    assert all(np.isfinite(v) for v in f.stderr.values())


def test_recursive_least_squares_agrees(fitted):
    truth, log, _ = fitted
    r = fitmod.fit_rls(log)
    assert r.kV == pytest.approx(truth["kV"], rel=0.05)


def test_compare_reports_relative_error(fitted):
    truth, _, f = fitted
    cmp = fitmod.compare(f, truth)
    assert cmp["kV"]["rel_diff"] == pytest.approx(0.0, abs=0.02)
    assert cmp["kS"]["cad"] == truth["kS"]


def test_csv_round_trip(tmp_path, fitted):
    _, log, _ = fitted
    path = fitmod.write_csv(tmp_path / "log.csv", log)
    back = fitmod.load_csv(path)
    assert len(back["t"]) == len(log["t"])
    assert back["velocity"][10] == pytest.approx(log["velocity"][10], abs=1e-5)


def test_noise_makes_fit_worse_not_wild():
    """A noisy log must degrade gracefully: still within 15% on kV."""
    axis = _axis()
    log = synthetic_calibration_log(
        axis, ks_true=0.35, kv_true=axis.gains.kV, ka_true=axis.gains.kA,
        rng_seed=3, noise_volts=0.15, noise_speed=8.0,
    )
    f = fitmod.fit_feedforward(log)
    assert f.kV == pytest.approx(axis.gains.kV, rel=0.15)


def test_rejects_tiny_logs():
    with pytest.raises(ValueError):
        fitmod.fit_feedforward({"t": [0, 1, 2], "voltage": [0, 0, 0], "velocity": [0, 0, 0]})


def test_large_ks_is_recovered_despite_stall_region():
    """Regression: a belt-driven axis needs ~1.1 V to break away.

    Its log spends the first second applying voltage with nothing turning. If the
    fit starts integrating there (e.g. because one noisy sample cleared the speed
    gate) kS comes out ~2x too high and the whole model collapses. The default
    FAT MAN flywheel has exactly this shape.
    """
    axis = _axis()
    truth = {"kS": 1.12, "kV": axis.gains.kV, "kA": axis.gains.kA}
    log = synthetic_calibration_log(
        axis, ks_true=truth["kS"], kv_true=truth["kV"], ka_true=truth["kA"], rng_seed=5
    )
    f = fitmod.fit_feedforward(log)
    assert f.kS == pytest.approx(truth["kS"], rel=0.10)
    assert f.kV == pytest.approx(truth["kV"], rel=0.02)
    assert f.kA == pytest.approx(truth["kA"], rel=0.10)
    assert f.r2 > 0.99
