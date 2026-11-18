"""Fit recovery on synthetic logs from sim.py, where the truth is known."""

import json
import math

import numpy as np
import pytest

from pidtool.motors import motors as load_motors
from pidtool.physics.mechanisms import build_robot
from pidtool.physics.sim import synthetic_calibration_log
from pidtool.runtime import fit as fitmod


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


# ---------------------------------------------------------- real FTC log format

FTC_HEADER = "time_s,voltage_v,velocity"


def _write_log(tmp_path, header: str, rows: list[str]):
    path = tmp_path / "ftc_log.csv"
    path.write_text(header + "\n" + "\n".join(rows) + "\n", encoding="utf-8")
    return path


def test_loads_ftc_columns_time_s_voltage_v_velocity(tmp_path):
    path = _write_log(tmp_path, FTC_HEADER, ["0.000,1.0,2.0", "0.010,1.2,2.5"])
    log = fitmod.load_csv(path)
    assert list(log["t"]) == pytest.approx([0.0, 0.01])
    assert list(log["voltage"]) == pytest.approx([1.0, 1.2])
    assert list(log["velocity"]) == pytest.approx([2.0, 2.5])
    assert log["units"] == "rad/s"
    assert log["source_units"] == "rad"


def test_units_in_header_are_tolerated_and_inferred(tmp_path):
    path = _write_log(tmp_path, "Time (s),Voltage (V),Velocity (ticks/s)",
                      ["0.000,1.0,14.0", "0.010,1.2,28.0"])
    log = fitmod.load_csv(path, ticks_per_rev=28.0)
    assert log["source_units"] == "ticks"
    assert log["velocity"][1] == pytest.approx(2.0 * math.pi)


def test_ticks_are_converted_to_rad_per_s(tmp_path):
    path = _write_log(tmp_path, FTC_HEADER, ["0.0,1.0,281.25", "0.5,1.1,562.5"])
    log = fitmod.load_csv(path, units="ticks", ticks_per_rev=281.25)
    assert log["velocity"][0] == pytest.approx(2.0 * math.pi)
    assert log["velocity"][1] == pytest.approx(4.0 * math.pi)
    assert log["source_units"] == "ticks"
    assert any("ticks/rev" in note for note in log["notes"])


def test_ticks_without_ticks_per_rev_is_an_error(tmp_path):
    path = _write_log(tmp_path, FTC_HEADER, ["0.0,1.0,100.0"])
    with pytest.raises(ValueError, match="ticks-per-rev"):
        fitmod.load_csv(path, units="ticks")


def test_missing_required_column_names_the_column(tmp_path):
    path = _write_log(tmp_path, "time_s,voltage_v", ["0.0,1.0"])
    with pytest.raises(ValueError, match="velocity"):
        fitmod.load_csv(path)


def test_non_numeric_row_is_reported_with_its_line(tmp_path):
    path = _write_log(tmp_path, FTC_HEADER, ["0.0,1.0,2.0", "0.1,oops,2.5"])
    with pytest.raises(ValueError, match="row 3"):
        fitmod.load_csv(path)


def test_calibrate_cli_reads_a_ticks_log(tmp_path, capsys):
    """The documented path: time_s,voltage_v,velocity in ticks/s plus a scale."""
    axis = _axis()
    log = synthetic_calibration_log(
        axis, ks_true=0.35, kv_true=axis.gains.kV, ka_true=axis.gains.kA, rng_seed=11
    )
    ticks_per_rev = 28.0
    rows = [
        f"{log['t'][i]:.6f},{log['voltage'][i]:.6f},"
        f"{log['velocity'][i] * ticks_per_rev / (2 * math.pi):.6f}"
        for i in range(len(log["t"]))
    ]
    path = _write_log(tmp_path, FTC_HEADER, rows)

    from pidtool.cli import main

    assert main(["calibrate", str(path), "--units", "ticks",
                 "--ticks-per-rev", str(ticks_per_rev)]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["kS"] == pytest.approx(0.35, rel=0.10)
    assert out["kV"] == pytest.approx(axis.gains.kV, rel=0.02)
    assert any("ticks/rev" in note for note in out["notes"])
