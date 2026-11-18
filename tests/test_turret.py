"""Turret travel: the CAD holds two readings, and Piπd must not pick one."""

import math

import pytest

from pidtool.ingest.fatman_local import turret_travel_report

CAD = dict(
    ratio=60.0 / 25.0,
    usable_range_deg=320.0,
    soft_min_rad=-2.356,
    soft_max_rad=2.356,
    hard_min_rad=-2.443,
    hard_max_rad=2.443,
)


def test_cad_two_readings_produce_a_warning():
    report = turret_travel_report(**CAD)
    assert report["resolution"] == "cad"
    assert report["cad_soft_span_deg"] == pytest.approx(270.0, abs=0.1)
    assert report["cad_hard_span_deg"] == pytest.approx(280.0, abs=0.1)
    assert report["servo_capable_span_deg"] == pytest.approx(133.33, abs=0.01)
    warning = report["warning"]
    assert warning
    assert "turret-travel-deg" in warning
    assert "270 deg" in warning and "280 deg" in warning


def test_cad_limits_are_kept_unchanged():
    report = turret_travel_report(**CAD)
    assert report["soft_limits_deg"] == pytest.approx([-135.0, 135.0], abs=0.1)
    assert report["hard_limits_deg"] == pytest.approx([-140.0, 140.0], abs=0.1)


def test_measured_travel_resolves_the_ambiguity():
    report = turret_travel_report(**CAD, measured_deg=132.0)
    assert report["resolution"] == "measured"
    assert report["warning"] is None
    assert report["measured_span_deg"] == pytest.approx(132.0)
    assert report["hard_limits_deg"] == [-66.0, 66.0]
    assert report["soft_limits_deg"] == [-61.0, 61.0]


def test_measured_travel_beyond_the_gearing_still_warns():
    report = turret_travel_report(**CAD, measured_deg=268.0)
    assert report["resolution"] == "measured"
    assert report["warning"]
    assert "cannot reach" in report["warning"]


def test_impossible_measurement_is_rejected():
    for bad in (0.0, -5.0, 721.0):
        with pytest.raises(ValueError, match="turret-travel-deg"):
            turret_travel_report(**CAD, measured_deg=bad)


def test_summary_warning_names_the_conflict():
    from pidtool.cli import _turret_travel_lines

    doc = {"turret_travel": turret_travel_report(**CAD)}
    lines = "\n".join(_turret_travel_lines(doc))
    assert lines.startswith("## WARNING: turret travel is ambiguous")
    assert "270" in lines and "280" in lines and "133" in lines
    assert "--turret-travel-deg" in lines


def test_summary_reports_a_measured_travel_without_warning():
    from pidtool.cli import _turret_travel_lines

    doc = {"turret_travel": turret_travel_report(**CAD, measured_deg=132.0)}
    text = "\n".join(_turret_travel_lines(doc))
    assert "WARNING" not in text
    assert "132.0" in text
    assert "61.0" in text
