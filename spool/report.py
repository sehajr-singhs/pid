"""Plot and summary generation for the report."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless, CPU only
import matplotlib.pyplot as plt  # noqa: E402

plt.rcParams.update({
    "figure.dpi": 130,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "font.size": 9,
})

BEFORE = "#b0413e"
AFTER = "#2f6f9f"


def _ensure(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def fig_flywheel(spin, shot, target_rpm: float, path: str | Path) -> Path:
    path = _ensure(Path(path))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.6))

    for key, color, label in (("before", BEFORE, "PID only"), ("after", AFTER, "Spool feedforward + tuned PID")):
        tr = spin.traces[key]
        ax1.plot(tr.t, tr.velocity * 60 / (2 * math.pi), color=color, lw=1.4, label=label)
    ax1.axhline(target_rpm, color="k", ls=":", lw=1, label="target")
    ax1.axhspan(target_rpm * 0.98, target_rpm * 1.02, color="k", alpha=0.06)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("flywheel speed (rpm)")
    ax1.set_title("Flywheel spin-up")
    ax1.legend(fontsize=7, loc="lower right")

    tr = shot["trace"]
    ax2.plot(tr.t, tr.velocity * 60 / (2 * math.pi), color=AFTER, lw=1.4)
    ax2.axhline(shot["target_rpm"], color="k", ls=":", lw=1)
    ax2.axhspan(shot["target_rpm"] * (1 - shot["tolerance_pct"] / 100),
                shot["target_rpm"] * (1 + shot["tolerance_pct"] / 100),
                color="k", alpha=0.06, label=f"+/-{shot['tolerance_pct']:.0f}% band")
    ax2.annotate(
        f"drop {shot['dip_pct']:.1f}%\nrecovered in {shot['recovery_time_s']:.2f} s"
        if shot["recovery_time_s"] is not None else f"drop {shot['dip_pct']:.1f}%",
        xy=(0.5, 0.25), xycoords="axes fraction", fontsize=8,
    )
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("flywheel speed (rpm)")
    ax2.set_title("Ball recovery (one shot at t = 0.45 s)")
    ax2.legend(fontsize=7, loc="lower right")

    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def fig_step(scn, ylabel: str, title: str, path: str | Path, scale: float = 1.0) -> Path:
    path = _ensure(Path(path))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.4))
    for key, color, label in (("before", BEFORE, "before"), ("after", AFTER, "after (Spool)")):
        tr = scn.traces[key]
        ax1.plot(tr.t, tr.position * scale, color=color, lw=1.3, label=label)
    ax1.axhline(scn.after.get("target", 0.0) * scale, color="k", ls=":", lw=1)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel(ylabel)
    ax1.set_title(title)
    ax1.legend(fontsize=7)

    for key, color, label in (("before", BEFORE, "before"), ("after", AFTER, "after (Spool)")):
        tr = scn.traces[key]
        ax2.plot(tr.t, tr.command, color=color, lw=0.9, alpha=0.85, label=f"{label} command (V)")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("applied voltage (V)")
    ax2.set_title("Commanded voltage")
    ax2.legend(fontsize=7)

    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def fig_arm(scn, path: str | Path) -> Path:
    path = _ensure(Path(path))
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for key, color, label in (("before", BEFORE, "no kG (gravity ignored)"),
                              ("after", AFTER, "with CAD kG")):
        tr = scn[f"trace_{key}"]
        ax.plot(tr.t, [math.degrees(v) for v in tr.position], color=color, lw=1.4, label=label)
    ax.axhline(math.degrees(scn["after"]["target"]), color="k", ls=":", lw=1, label="setpoint")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("arm angle (deg)")
    ax.set_title("Arm hold at 20 deg from horizontal")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def fig_drive(scn, path: str | Path, label: str) -> Path:
    path = _ensure(Path(path))
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for key, color, name in (("before", BEFORE, "before"), ("after", AFTER, "after (Spool)")):
        tr = scn.traces[key]
        ax.plot(tr.t, tr.velocity * 39.3701, color=color, lw=1.4, label=name)
    ax.axhline(scn.after.get("target", 0.0) * 39.3701, color="k", ls=":", lw=1)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("chassis speed (in/s)")
    ax.set_title(f"Drivetrain {label} step")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def table_rows(scenarios: dict) -> list[dict]:
    rows = []
    for name, scn in scenarios.items():
        rows.append({
            "scenario": name,
            "rise_before": scn.before.get("rise_time"),
            "rise_after": scn.after.get("rise_time"),
            "os_before": scn.before.get("overshoot_pct"),
            "os_after": scn.after.get("overshoot_pct"),
            "settle_before": scn.before.get("settle_time"),
            "settle_after": scn.after.get("settle_time"),
        })
    return rows


def markdown_table(rows: list[dict], unit_scale: float = 1.0, unit_name: str = "") -> str:
    def fmt(v, nd=3):
        if v is None:
            return "-"
        if isinstance(v, float) and not math.isfinite(v):
            return "-"
        return f"{v:.{nd}f}"

    head = (
        "| Scenario | Rise before | Rise after | Overshoot before % | Overshoot after % | "
        "Settle before | Settle after |\n"
        "|---|---|---|---|---|---|---|\n"
    )
    body = "\n".join(
        f"| {r['scenario']} | {fmt(r['rise_before'] * unit_scale)} | {fmt(r['rise_after'] * unit_scale)} | "
        f"{fmt(r['os_before'], 1)} | {fmt(r['os_after'], 1)} | "
        f"{fmt(r['settle_before'] * unit_scale)} | {fmt(r['settle_after'] * unit_scale)} |"
        for r in rows
    )
    unit = f"\n\nTimes in {'seconds' if unit_scale == 1.0 else unit_name}." if unit_scale == 1.0 else ""
    return head + body + unit
