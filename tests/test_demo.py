"""The static demo page: real numbers, plot links, no fetch, no server."""

import json

import pytest

from pidtool import demo


def _gains() -> dict:
    return {
        "robot": "FAT MAN",
        "pid_version": "0.1.0",
        "mechanisms": {
            "drivetrain": {
                "forward": {"kS": 0.4873, "kV": 4.4288, "kA": 1.2596, "kG": 0.0},
                "strafe": {"kS": 0.4873, "kV": 5.6780, "kA": 1.6149, "kG": 0.0},
                "sources": {"kS": "estimated", "kV": "datasheet", "kA": "estimated"},
            },
            "flywheel": {
                "gains": {"kS": 1.1167, "kV": 0.01858, "kA": 0.005173, "kG": 0.0},
                "sources": {"kS": "estimated", "kV": "datasheet", "kA": "estimated"},
            },
        },
        "frc_equivalent": {
            "mechanisms": {
                "flower_arm": {
                    "gains": {"kS": 0.1563, "kV": 0.4859, "kA": 0.03447, "kG": 1.4537},
                }
            }
        },
        "servo_axes": [{
            "name": "turret", "servo": "SR-Servo-02", "gear_ratio": 2.4,
            "output_speed_deg_s": 271.7, "stall_torque_out_nm": 11.57,
            "required_torque_nm": 0.901, "torque_margin": 12.8,
        }],
        "ball_recovery": {
            "energy_taken_j": 5.30, "dip_pct": 17.8, "recovery_time_s": 0.214,
            "recovery_ok": True, "tolerance_pct": 2.0,
        },
        "scenarios": {
            "flywheel spin-up": {
                "before": {"rise_time": 0.544, "overshoot_pct": -5.96,
                           "settle_time": 2.0},
                "after": {"rise_time": 0.425, "overshoot_pct": 0.0,
                          "settle_time": 0.575},
            },
            "drive forward": {
                "before": {"rise_time": 0.706, "overshoot_pct": -10.3,
                           "settle_time": 2.0},
                "after": {"rise_time": 0.413, "overshoot_pct": -0.0,
                          "settle_time": 0.545},
            },
        },
    }


def _robot(warning: bool = True) -> dict:
    return {
        "name": "FAT MAN",
        "mass_kg": 12.7,
        "turret_travel": {
            "resolution": "cad",
            "warning": "The turret travel in the CAD is ambiguous: measure it." if warning else None,
            "cad_soft_span_deg": 270.0,
            "cad_hard_span_deg": 280.0,
            "servo_capable_span_deg": 133.33,
        },
    }


def test_render_shows_gains_scenarios_and_loading_text():
    html = demo.render_demo(_gains(), _robot(), {})
    assert "tuning..." in html
    assert "FAT MAN" in html
    assert "12.70" in html
    assert "1.1167" in html
    assert "1.4537" in html
    assert "flywheel spin-up" in html
    assert "0.425" in html
    assert "17.8" in html
    assert "SR-Servo-02" in html
    assert "WARNING" not in html  # the warning lives in the turret note, not the header


def test_render_includes_the_turret_warning_when_present():
    html = demo.render_demo(_gains(), _robot(warning=True), {})
    assert "Turret travel warning" in html
    assert "measure it" in html
    plain = demo.render_demo(_gains(), _robot(warning=False), {})
    assert "Turret travel warning" not in plain


def test_page_has_no_fetch_and_no_em_dash():
    html = demo.render_demo(_gains(), _robot(), {})
    assert "fetch(" not in html
    assert "\u2014" not in html
    assert "<script src" not in html


def test_build_demo_links_plots_relative_to_the_page(tmp_path):
    out = tmp_path / "out"
    (out / "plots").mkdir(parents=True)
    (out / "gains.json").write_text(json.dumps(_gains()), encoding="utf-8")
    (out / "robot.json").write_text(json.dumps(_robot()), encoding="utf-8")
    for _key, filename, _caption in demo.PLOTS:
        (out / "plots" / filename).write_bytes(b"\x89PNG\r\n\x1a\n")

    dest = tmp_path / "docs" / "demo" / "index.html"
    path = demo.build_demo(out, dest)
    html = path.read_text(encoding="utf-8")
    assert path == dest
    assert "../../out/plots/flywheel.png" in html
    assert "../../out/plots/drive_strafe.png" in html
    assert html.count("<figure>") == len(demo.PLOTS)


def test_build_demo_needs_gains_json(tmp_path):
    with pytest.raises(FileNotFoundError, match="pid fatman"):
        demo.build_demo(tmp_path)
