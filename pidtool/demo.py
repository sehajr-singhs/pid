"""Static demo page: docs/demo/index.html, no server and no network.

`pid fatman` rebuilds it at the end of every run and `pid demo` rebuilds it
from an existing output directory. The tables are rendered here, from
`out/fatman/gains.json` and `robot.json`; the plots are plain `<img>` tags that
point at `out/fatman/plots/`. So the page opens straight from file:// with no
fetch, no CORS and no server.

The page shows "tuning..." until the window load event, which is when the plots
have finished decoding.
"""

from __future__ import annotations

import html
import json
import math
import os
from pathlib import Path

from . import __version__

PLOTS = (
    ("flywheel", "flywheel.png", "Flywheel spin-up and ball recovery"),
    ("turret_slew", "turret_slew.png", "Turret slew, 0 to 90 deg"),
    ("hood_slew", "hood_slew.png", "Hood slew, 0 to 45 deg"),
    ("arm_hold", "arm_hold.png", "Arm hold at 20 deg against gravity"),
    ("drive_forward", "drive_forward.png", "Drivetrain forward step"),
    ("drive_strafe", "drive_strafe.png", "Drivetrain strafe step"),
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DEST = REPO_ROOT / "docs" / "demo" / "index.html"


def _fmt(value, nd: int = 3) -> str:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(v):
        return "-"
    return f"{v:.{nd}f}"


def _chips(sources: dict | None, keys=("kS", "kV", "kA", "kG")) -> str:
    if not sources:
        return '<span class="dash">-</span>'
    out = []
    for key in keys:
        tag = sources.get(key)
        if tag:
            out.append(f'<span class="chip t-{html.escape(str(tag))}">{html.escape(str(tag))}</span>')
    return " ".join(out) or '<span class="dash">-</span>'


def _gain_row(label: str, gains: dict, units: str, sources: dict | None = None) -> str:
    kg = gains.get("kG")
    return (
        "<tr>"
        f'<td class="mech">{html.escape(label)}</td>'
        f'<td class="num">{_fmt(gains.get("kS"), 4)}</td>'
        f'<td class="num">{_fmt(gains.get("kV"), 5)}</td>'
        f'<td class="num">{_fmt(gains.get("kA"), 5)}</td>'
        f'<td class="num">{_fmt(kg, 4) if kg else "-"}</td>'
        f'<td class="units">{html.escape(units)}</td>'
        f"<td>{_chips(sources)}</td>"
        "</tr>"
    )


def _gain_rows(gains: dict) -> str:
    rows = []
    mechanisms = gains.get("mechanisms", {})
    drive = mechanisms.get("drivetrain", {})
    for label, key in (("drivetrain forward", "forward"), ("drivetrain strafe", "strafe")):
        block = drive.get(key)
        if isinstance(block, dict):
            rows.append(_gain_row(label, block, "V, V/(m/s), V/(m/s^2)", drive.get("sources")))
    for name, block in mechanisms.items():
        if name == "drivetrain" or not isinstance(block, dict):
            continue
        if isinstance(block.get("gains"), dict):
            rows.append(_gain_row(name, block["gains"], "V, V/(rad/s), V/(rad/s^2)",
                                  block.get("sources")))
    arm = gains.get("frc_equivalent", {}).get("mechanisms", {}).get("flower_arm", {})
    if isinstance(arm.get("gains"), dict):
        rows.append(_gain_row("FRC arm (equivalent)", arm["gains"],
                              "V, V/(rad/s), V/(rad/s^2)", arm.get("sources")))
    return "\n".join(rows) or '<tr><td colspan="7" class="dash">no gains in gains.json</td></tr>'


def _scenario_rows(gains: dict) -> str:
    rows = []
    for name, scn in gains.get("scenarios", {}).items():
        before = scn.get("before", {}) or {}
        after = scn.get("after", {}) or {}
        rows.append(
            "<tr>"
            f'<td class="mech">{html.escape(name)}</td>'
            f'<td class="num">{_fmt(before.get("rise_time"))}</td>'
            f'<td class="num">{_fmt(after.get("rise_time"))}</td>'
            f'<td class="num">{_fmt(before.get("overshoot_pct"), 1)}</td>'
            f'<td class="num">{_fmt(after.get("overshoot_pct"), 1)}</td>'
            f'<td class="num">{_fmt(before.get("settle_time"))}</td>'
            f'<td class="num">{_fmt(after.get("settle_time"))}</td>'
            "</tr>"
        )
    return "\n".join(rows) or '<tr><td colspan="7" class="dash">no scenarios in gains.json</td></tr>'


def _servo_rows(gains: dict) -> str:
    rows = []
    for axis in gains.get("servo_axes", []):
        margin = axis.get("torque_margin")
        rows.append(
            "<tr>"
            f'<td class="mech">{html.escape(str(axis.get("name", "?")))}</td>'
            f'<td>{html.escape(str(axis.get("servo", "-")))}</td>'
            f'<td class="num">{_fmt(axis.get("gear_ratio"), 2)}</td>'
            f'<td class="num">{_fmt(axis.get("output_speed_deg_s"), 0)}</td>'
            f'<td class="num">{_fmt(axis.get("stall_torque_out_nm"), 2)}</td>'
            f'<td class="num">{_fmt(axis.get("required_torque_nm"), 3)}</td>'
            f'<td class="num">{(_fmt(margin, 1) + "x") if margin else "-"}</td>'
            "</tr>"
        )
    return "\n".join(rows) or '<tr><td colspan="7" class="dash">no servo axes in gains.json</td></tr>'


def _ball_line(gains: dict) -> str:
    ball = gains.get("ball_recovery") or {}
    if not ball:
        return ""
    tolerance = _fmt(ball.get("tolerance_pct"), 0)
    recovery = ball.get("recovery_time_s")
    if recovery is None:
        tail = "did not recover inside the window"
    else:
        tail = f"back inside {tolerance} percent in {_fmt(recovery)} s"
    verdict = ('<span class="ok">PASS</span>' if ball.get("recovery_ok")
               else '<span class="bad">FAIL</span>')
    return (
        f"A shot takes {_fmt(ball.get('energy_taken_j'), 2)} J and drops the wheel "
        f"{_fmt(ball.get('dip_pct'), 1)} percent, {tail} "
        f"({verdict} at 2 percent in 0.5 s)."
    )


def _travel_note(robot: dict) -> str:
    info = robot.get("turret_travel") or {}
    if info.get("warning"):
        return (
            '<div class="warn"><b>Turret travel warning.</b> '
            f'{html.escape(str(info["warning"]))}</div>'
        )
    if info.get("resolution") == "measured":
        soft = info.get("soft_limits_deg", [0, 0])
        hard = info.get("hard_limits_deg", [0, 0])
        return (
            '<div class="note">Turret travel was measured at '
            f'{_fmt(info.get("measured_span_deg"), 1)} deg between hard stops: soft stops '
            f'{_fmt(soft[0], 1)} to {_fmt(soft[1], 1)} deg, hard stops '
            f'{_fmt(hard[0], 1)} to {_fmt(hard[1], 1)} deg.</div>'
        )
    return ""


def _plots_section(plot_hrefs: dict[str, str]) -> str:
    figures = []
    for key, _filename, caption in PLOTS:
        href = plot_hrefs.get(key)
        if not href:
            continue
        figures.append(
            "<figure>"
            f'<img src="{html.escape(href)}" alt="{html.escape(caption)}">'
            f"<figcaption>{html.escape(caption)}</figcaption>"
            "</figure>"
        )
    if not figures:
        return ('<p class="dash">No plots found next to this page. Run `pid fatman` '
                "to generate out/fatman/plots.</p>")
    return '<div class="grid">\n' + "\n".join(figures) + "\n</div>"


_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>pid demo - {robot_name}</title>
<style>
  :root {{
    --bg:#0a0c10; --panel:#11151b; --panel-2:#141a22; --line:#232b36;
    --ink:#e8ecf2; --muted:#8b97a7; --faint:#5d6877;
    --accent:#7cc4ff; --amber:#f0b86e; --bad:#ff7a7a; --good:#67d391;
    --cad:#8ea6ff; --bom:#f0b86e; --datasheet:#67d391; --estimated:#c9a0ff;
    --radius:12px;
  }}
  * {{ box-sizing:border-box; }}
  html, body {{ margin:0; padding:0; }}
  body {{
    background:var(--bg); color:var(--ink);
    font:14px/1.55 ui-sans-serif, system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
    -webkit-font-smoothing:antialiased;
  }}
  code {{ font-family:ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }}
  header {{
    display:flex; align-items:center; gap:14px; flex-wrap:wrap;
    padding:18px 26px; border-bottom:1px solid var(--line); background:var(--panel);
  }}
  h1 {{ margin:0; font-size:21px; font-weight:650; letter-spacing:-0.03em; }}
  h1 .dot {{ color:var(--accent); }}
  .sub {{ color:var(--muted); font-size:12.5px; }}
  .status {{
    margin-left:auto; display:inline-flex; align-items:center; gap:8px;
    font-size:12px; color:var(--muted);
    border:1px solid var(--line); border-radius:999px; padding:5px 12px;
    background:var(--panel-2);
  }}
  .spinner {{
    width:11px; height:11px; border-radius:50%;
    border:2px solid var(--line); border-top-color:var(--accent);
    animation:spin .8s linear infinite;
  }}
  @keyframes spin {{ to {{ transform:rotate(360deg); }} }}
  main {{ max-width:1180px; margin:0 auto; padding:20px 26px 70px; }}
  section {{
    background:var(--panel); border:1px solid var(--line); border-radius:var(--radius);
    padding:18px 20px; margin-bottom:18px;
  }}
  h2 {{
    margin:0 0 14px; font-size:11.5px; font-weight:650; text-transform:uppercase;
    letter-spacing:.09em; color:var(--muted);
  }}
  .facts {{ display:flex; gap:22px; flex-wrap:wrap; color:var(--muted); font-size:12.5px; margin-top:8px; }}
  .facts b {{ color:var(--ink); font-weight:600; }}
  table {{ width:100%; border-collapse:collapse; font-variant-numeric:tabular-nums; }}
  th {{
    text-align:left; padding:7px 9px; border-bottom:1px solid var(--line);
    font-size:10.5px; text-transform:uppercase; letter-spacing:.06em;
    color:var(--faint); font-weight:600; white-space:nowrap;
  }}
  td {{ padding:9px; border-bottom:1px solid var(--line); font-size:13px; vertical-align:top; }}
  tr:last-child td {{ border-bottom:none; }}
  td.num {{ text-align:right; }}
  .mech {{ font-weight:600; }}
  .units {{ color:var(--faint); font-size:11px; }}
  .chip {{
    display:inline-block; margin:1px 2px 1px 0; padding:1px 7px; border-radius:999px;
    font-size:9.5px; letter-spacing:.04em; text-transform:uppercase; font-weight:600;
    border:1px solid currentColor; opacity:.9;
  }}
  .t-cad {{ color:var(--cad); }} .t-bom {{ color:var(--bom); }}
  .t-datasheet {{ color:var(--datasheet); }} .t-estimated {{ color:var(--estimated); }}
  .dash {{ color:var(--faint); }}
  .note, .warn {{
    border-radius:10px; padding:12px 14px; margin:0 0 16px; font-size:12.5px;
    border:1px solid var(--line); background:var(--panel-2); color:var(--muted);
  }}
  .warn {{ border-color:#4a3520; background:#1d1610; color:var(--amber); }}
  .grid {{ display:grid; grid-template-columns:repeat(auto-fit, minmax(330px, 1fr)); gap:16px; }}
  figure {{ margin:0; background:#fff; border:1px solid var(--line); border-radius:10px; overflow:hidden; }}
  figure img {{ display:block; width:100%; height:auto; }}
  figcaption {{
    background:var(--panel-2); color:var(--muted); font-size:11.5px;
    padding:7px 10px; border-top:1px solid var(--line);
  }}
  .ok {{ color:var(--good); }} .bad {{ color:var(--bad); }}
  .muted {{ color:var(--muted); font-size:12.5px; }}
  footer {{ color:var(--faint); font-size:11.5px; text-align:center; padding:8px 0 0; }}
  @media (max-width:640px) {{
    header {{ padding:14px 16px; }} main {{ padding:16px; }} section {{ padding:14px; }}
  }}
</style>
</head>
<body>
<header>
  <h1>pid<span class="dot">.</span> demo</h1>
  <span class="sub">CAD in, tuned robot out</span>
  <span class="status" id="loading"><span class="spinner"></span>tuning...</span>
</header>

<main>
  <section>
    <h2>Robot</h2>
    <div><b>{robot_name}</b></div>
    <div class="facts">
      <span>mass <b>{mass} kg</b></span>
      <span>pid <b>{version}</b></span>
      <span>source <b>out/fatman</b></span>
    </div>
  </section>

  {travel_note}

  <section>
    <h2>Gains</h2>
    <table>
      <thead><tr>
        <th>Mechanism</th><th>kS (V)</th><th>kV</th><th>kA</th><th>kG (V)</th>
        <th>Units</th><th>Source</th>
      </tr></thead>
      <tbody>
{gain_rows}
      </tbody>
    </table>
    <p class="muted" style="margin:12px 0 0">
      kS is always estimated: friction cannot come from CAD, so
      <code>pid calibrate</code> measures it. Every other source tag is in
      robot.json and gains.json.
    </p>
  </section>

  <section>
    <h2>Before / after</h2>
    <table>
      <thead><tr>
        <th>Scenario</th><th>Rise before</th><th>Rise after</th>
        <th>Overshoot before %</th><th>Overshoot after %</th>
        <th>Settle before</th><th>Settle after</th>
      </tr></thead>
      <tbody>
{scenario_rows}
      </tbody>
    </table>
    <p class="muted" style="margin:12px 0 0">
      Times in seconds. Negative overshoot means the axis came to rest below the
      target: a steady-state error, not a dip. "before" is a PID with no
      feedforward, "after" is Piπd's feedforward plus a tuned PID.
    </p>
    <p class="muted" style="margin:8px 0 0">{ball_line}</p>
  </section>

  <section>
    <h2>Servo axes (no PIDF)</h2>
    <table>
      <thead><tr>
        <th>Axis</th><th>Servo</th><th>Ratio</th><th>Speed (deg/s)</th>
        <th>Torque out (N m)</th><th>Required (N m)</th><th>Margin</th>
      </tr></thead>
      <tbody>
{servo_rows}
      </tbody>
    </table>
    <p class="muted" style="margin:12px 0 0">
      A position servo closes its own loop, so there is no PIDF to tune. Piπd
      exports ratio, speed, torque and limits instead.
    </p>
  </section>

  <section>
    <h2>Plots</h2>
{plots}
  </section>

  <footer>
    Generated by pid {version} from out/fatman. Static page: no server, no fetch,
    opens from file. Rebuild with <code>pid demo</code> or <code>pid fatman</code>.
  </footer>
</main>

<script>
(function () {{
  function ready() {{
    var el = document.getElementById("loading");
    if (el) {{ el.style.display = "none"; }}
  }}
  if (document.readyState === "complete") {{ ready(); }}
  else {{ window.addEventListener("load", ready); }}
}})();
</script>
</body>
</html>
"""


def render_demo(gains: dict, robot: dict, plot_hrefs: dict[str, str]) -> str:
    """Render the whole page. plot_hrefs maps a PLOTS key to a relative img src."""
    name = robot.get("name") or gains.get("robot") or "robot"
    mass = _fmt(robot.get("mass_kg"), 2)
    ball = _ball_line(gains)
    return _TEMPLATE.format(
        robot_name=html.escape(str(name)),
        mass=html.escape(mass),
        version=html.escape(__version__),
        travel_note=_travel_note(robot),
        gain_rows=_gain_rows(gains),
        scenario_rows=_scenario_rows(gains),
        servo_rows=_servo_rows(gains),
        ball_line=ball,
        plots=_plots_section(plot_hrefs),
    )


def _href(target: Path, start: Path) -> str:
    try:
        return Path(os.path.relpath(target.resolve(), start.resolve())).as_posix()
    except ValueError:  # different drive on Windows
        return target.resolve().as_uri()


def build_demo(out_dir: str | Path = "out/fatman", dest: str | Path | None = None) -> Path:
    """Write the static demo page from a pid output directory."""
    out = Path(out_dir)
    gains_path = out / "gains.json"
    if not gains_path.exists():
        raise FileNotFoundError(
            f"{gains_path} not found: run 'pid fatman' first, or pass --from <dir>"
        )
    gains = json.loads(gains_path.read_text(encoding="utf-8"))
    robot_path = out / "robot.json"
    robot = (json.loads(robot_path.read_text(encoding="utf-8"))
             if robot_path.exists() else {"name": gains.get("robot", "robot")})

    dest = Path(dest) if dest is not None else DEFAULT_DEST
    hrefs = {}
    for key, filename, _caption in PLOTS:
        plot = out / "plots" / filename
        if plot.exists():
            hrefs[key] = _href(plot, dest.parent)

    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(render_demo(gains, robot, hrefs), encoding="utf-8", newline="\n")
    return dest
