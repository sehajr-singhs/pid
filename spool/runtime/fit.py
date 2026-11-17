"""Fit kS, kV, kA (and kG) from a logged test run.

The robot does not have to be tuned for this: SpoolCalibrate applies a slow
voltage ramp (kS), steady voltages (kV) and voltage steps (kA), logging voltage,
velocity and current. This module solves

    V = kS*sign(w) + kV*w + kA*dw/dt [+ kG*cos(theta)]

by weighted least squares, then compares the answer with the CAD prediction.

Why not plain `np.polyfit`: the acceleration has to be differentiated out of a
noisy velocity signal, and the regressors are correlated (sign(w) and w). The fit
here smooths velocity and acceleration first, drops the near-zero-speed samples
where sign(w) is meaningless, and reports per-parameter standard errors so a bad
log is visible instead of silently producing a confident wrong number.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


def _smooth(y: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or y.size < window:
        return y
    pad = window // 2
    padded = np.pad(y, pad, mode="edge")
    kernel = np.ones(window) / window
    return np.convolve(padded, kernel, mode="valid")[: y.size]


@dataclass
class FitResult:
    kS: float
    kV: float
    kA: float
    kG: float
    r2: float
    rmse_volts: float
    stderr: dict
    n_samples: int
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "kS": self.kS,
            "kV": self.kV,
            "kA": self.kA,
            "kG": self.kG,
            "r2": self.r2,
            "rmse_volts": self.rmse_volts,
            "stderr": self.stderr,
            "n_samples": self.n_samples,
            "notes": self.notes,
        }


# Header names a real team log might use. The canonical SpoolCalibrate header is
# time_s,voltage_v,velocity,position,current; the older t,voltage,velocity,
# position,current form (and headers with units in brackets) still load.
_FIELD_ALIASES = {
    "time": ("time_s", "t", "time", "timestamp_s", "timestamp"),
    "voltage": ("voltage_v", "voltage", "volts", "v"),
    "velocity": ("velocity_rad_s", "velocity_ticks_s", "velocity_rads",
                 "velocity_tick_s", "velocity_counts_s", "velocity", "speed"),
    "position": ("position_rad", "position", "pos", "angle_rad", "angle"),
    "current": ("current_a", "current", "amps", "current_amps"),
}


def _normalise_header(name: str) -> str:
    """`Velocity (ticks/s)` -> `velocity_ticks_s`."""
    return re.sub(r"[^a-z0-9]+", "_", name.strip().lower()).strip("_")


def _pick_column(columns: dict[str, str], aliases: tuple[str, ...]) -> str | None:
    for alias in aliases:
        if alias in columns:
            return columns[alias]
    return None


def load_csv(path: str | Path, *, units: str | None = None,
             ticks_per_rev: float | None = None) -> dict:
    """Read a calibration CSV, in rad/s or from an FTC encoder log.

    Required columns: time_s, voltage_v and velocity. `position` is optional (it
    is needed for --gravity) and `current` is optional (the fit does not use it).
    Header names are matched loosely, so `t`, `voltage`, `volts`, `speed` and units
    written into the name (`velocity (ticks/s)`) all work.

    `units` is "rad" or "ticks". When it is None the units are inferred from the
    velocity column name, falling back to rad/s. A ticks/s log is converted with
    `2*pi / ticks_per_rev`; position is converted with the same factor so a
    --gravity fit still sees radians. `ticks_per_rev` is required for ticks.
    """
    import csv

    if units not in (None, "rad", "ticks"):
        raise ValueError(f"units must be 'rad' or 'ticks', got {units!r}")

    t, v, w, pos, cur = [], [], [], [], []
    expected = "time_s,voltage_v,velocity,position,current"
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        fieldnames = reader.fieldnames or []
        columns = {_normalise_header(n): n for n in fieldnames if n and n.strip()}
        picked = {key: _pick_column(columns, aliases)
                  for key, aliases in _FIELD_ALIASES.items()}
        missing = [key for key in ("time", "voltage", "velocity") if not picked[key]]
        if missing:
            raise ValueError(
                f"missing a {'/'.join(missing)} column. Expected "
                f"{expected}; found {', '.join(fieldnames) or 'no header row'}"
            )

        for lineno, row in enumerate(reader, start=2):
            try:
                values = []
                for key in ("time", "voltage", "velocity"):
                    raw = (row.get(picked[key]) or "").strip()
                    if not raw:
                        raise ValueError(f"empty {key} value")
                    values.append(float(raw))
                for key in ("position", "current"):
                    col = picked[key]
                    raw = ((row.get(col) or "").strip() if col else "")
                    values.append(float(raw) if raw else 0.0)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"row {lineno} is not numeric: {exc}") from exc
            t.append(values[0])
            v.append(values[1])
            w.append(values[2])
            pos.append(values[3])
            cur.append(values[4])

    if not t:
        raise ValueError(f"{path}: no data rows")

    velocity = np.array(w, dtype=float)
    position = np.array(pos, dtype=float)
    velocity_col = picked["velocity"] or "velocity"
    source_units = units
    if source_units is None:
        normalised = _normalise_header(velocity_col)
        source_units = "ticks" if ("ticks" in normalised or "counts" in normalised) else "rad"

    notes = []
    if source_units == "ticks":
        if ticks_per_rev is None:
            raise ValueError(
                "a ticks/s log needs --ticks-per-rev (encoder counts per revolution "
                "of the shaft the velocity is measured on), e.g. 28 on a goBILDA "
                "5203 at 1:1 or 537.7 for a 312 rpm gearmotor"
            )
        if float(ticks_per_rev) <= 0:
            raise ValueError("--ticks-per-rev must be positive")
        scale = 2.0 * math.pi / float(ticks_per_rev)
        velocity = velocity * scale
        position = position * scale
        notes.append(
            f"velocity and position converted from encoder ticks to rad with "
            f"{float(ticks_per_rev):g} ticks/rev"
        )
    else:
        notes.append("velocity read as rad/s")
    if units is None and ("ticks" in _normalise_header(velocity_col)
                          or "rad" in _normalise_header(velocity_col)):
        notes.append(f"units inferred from the column name {velocity_col!r}; "
                     "pass --units to override")

    return {
        "t": np.array(t, dtype=float),
        "voltage": np.array(v, dtype=float),
        "velocity": velocity,
        "position": position,
        "current": np.array(cur, dtype=float),
        "units": "rad/s",
        "source_units": source_units,
        "notes": notes,
    }


def write_csv(path: str | Path, log: dict) -> Path:
    import csv

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = ["t", "voltage", "velocity", "position", "current"]
    n = len(log["t"])
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(keys)
        for i in range(n):
            row = []
            for k in keys:
                series = log.get(k)
                row.append(f"{float(series[i]) if series is not None else 0.0:.6f}")
            w.writerow(row)
    return path


def _cumtrapz(y: np.ndarray, x: np.ndarray) -> np.ndarray:
    """Cumulative trapezoidal integral, starting at 0."""
    out = np.zeros_like(y, dtype=float)
    if len(y) > 1:
        out[1:] = np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(x))
    return out


def fit_feedforward(
    log: dict,
    *,
    include_gravity: bool = False,
    smooth_window: int = 9,
    min_speed: float = 1.0,
    method: str = "integral",
) -> FitResult:
    """Fit the feedforward model to a log.

    Default method is the *integral* form, not pointwise regression:

        integral(V dt) = kS*integral(sign(w) dt) + kV*integral(w dt) + kA*dw

    It is exactly equivalent to V = kS sign(w) + kV w + kA dw/dt, but it never
    differentiates the measured speed. Differentiating a noisy encoder signal
    puts that noise straight into the acceleration regressor, which biases kA and
    leaks the bias into kS (errors-in-variables). Integrating instead is
    numerically stable and recovers the gains from the same data far more tightly.

    Samples before the motor first moves are dropped: while the wheel is at rest
    a voltage below kS is applied and produces no motion, which the model does not
    describe.
    """
    t = np.asarray(log["t"], dtype=float)
    volts = np.asarray(log["voltage"], dtype=float)
    speed = np.asarray(log["velocity"], dtype=float)

    order = np.argsort(t)
    t, volts, speed = t[order], volts[order], speed[order]

    if t.size < 8:
        raise ValueError("need at least 8 samples to fit a feedforward model")

    v_smooth = _smooth(speed, smooth_window)
    theta = np.asarray(log.get("position", np.zeros_like(t)), dtype=float)[order]

    moving = np.abs(v_smooth) > min_speed
    if moving.sum() < 8:
        raise ValueError(
            "the mechanism never exceeded the minimum speed: check that it turned freely"
        )

    # Start where the mechanism is *sustainedly* moving, not at the first sample
    # that happens to clear the gate. One noisy sample above the threshold during
    # the stall region would pull the whole integral through samples where voltage
    # was applied but nothing turned -- which inflates kS badly (it did: 2.09 V
    # fitted against 1.12 V truth before this guard).
    window = min(51, max(3, len(moving) // 20))
    sustained = np.convolve(moving.astype(int), np.ones(window, dtype=int), mode="valid") == window
    start = int(np.argmax(sustained)) if sustained.any() else int(np.argmax(moving))
    t2, v2, w2, th2 = t[start:], volts[start:], v_smooth[start:], theta[start:]
    t2 = t2 - t2[0]
    w2 = w2 - 0.0
    sgn = np.sign(w2)

    cols = [_cumtrapz(sgn, t2), _cumtrapz(w2, t2), (w2 - w2[0])]
    names = ["kS", "kV", "kA"]
    if include_gravity:
        cols.append(_cumtrapz(np.cos(th2), t2))
        names.append("kG")

    lhs = _cumtrapz(v2, t2)
    X = np.vstack(cols).T
    if np.linalg.matrix_rank(X) < X.shape[1] and method == "integral":
        method = "derivative"

    if method == "integral":
        coef, *_ = np.linalg.lstsq(X, lhs, rcond=None)
    else:  # pragma: no cover - fallback path
        accel = _smooth(np.gradient(w2, t2), smooth_window)
        Xd = np.vstack([sgn, w2, accel]).T
        coef, *_ = np.linalg.lstsq(Xd, v2, rcond=None)

    out = {n: float(c) for n, c in zip(names, coef)}

    # Goodness of fit, quoted on the voltage equation (what a reader expects).
    accel = _smooth(np.gradient(v_smooth, t), smooth_window)[start:]
    pred = (out["kS"] * sgn + out["kV"] * w2 + out["kA"] * accel
            + (out.get("kG", 0.0) * np.cos(th2) if include_gravity else 0.0))
    resid = v2 - pred
    ss_res = float(np.sum(resid**2))
    ss_tot = float(np.sum((v2 - v2.mean()) ** 2)) or 1.0
    r2 = 1.0 - ss_res / ss_tot

    # Equivalent voltage error: the integral fit is scored on the integrated
    # equation, so the honest error metric is the integrated residual divided by
    # the window. (A pointwise rmse here would be dominated by differentiation
    # noise in the alpha regressor, which is not what the fit minimised.)
    duration = float(t2[-1]) or 1.0
    int_resid = lhs - X @ coef
    rmse = float(np.sqrt(np.mean(int_resid**2)) / duration)

    # Approximate standard errors from the pointwise regression's covariance.
    stderr = {}
    try:
        Xd = np.vstack([sgn, w2, accel]).T
        coefd, *_ = np.linalg.lstsq(Xd, v2, rcond=None)
        res = v2 - Xd @ coefd
        sigma2 = float(res @ res) / max(len(v2) - Xd.shape[1], 1)
        cov = sigma2 * np.linalg.inv(Xd.T @ Xd)
        stderr = {n: float(np.sqrt(max(cov[i, i], 0.0))) for i, n in enumerate(names[:3])}
        if include_gravity:
            stderr["kG"] = float("nan")
    except (np.linalg.LinAlgError, ValueError):
        stderr = {n: float("nan") for n in names}

    method_notes = [
        f"{method} identification on {len(v2)} samples (|w| > {min_speed}), "
        f"starting at t={t[start]:.2f} s",
        "integral form: no differentiation of the measured speed",
    ] if method == "integral" else [
        f"derivative identification on {len(v2)} samples",
        f"acceleration from smoothed velocity, window {smooth_window} samples",
    ]
    notes = list(log.get("notes") or []) + method_notes
    if r2 < 0.9:
        notes.append("R^2 below 0.9: the log is noisy or the model does not fit this axis")

    return FitResult(
        kS=out["kS"],
        kV=out["kV"],
        kA=out["kA"],
        kG=out.get("kG", 0.0),
        r2=r2,
        rmse_volts=rmse,
        stderr=stderr,
        n_samples=int(len(v2)),
        notes=notes,
    )


def fit_rls(
    log: dict,
    *,
    forgetting: float = 0.999,
    include_gravity: bool = False,
    smooth_window: int = 9,
) -> FitResult:
    """Recursive least squares, for streaming a fit while the robot runs."""
    t = np.asarray(log["t"], dtype=float)
    volts = np.asarray(log["voltage"], dtype=float)
    speed = _smooth(np.asarray(log["velocity"], dtype=float), smooth_window)
    accel = _smooth(np.gradient(speed, t), smooth_window)

    cols = [np.sign(speed), speed, accel]
    names = ["kS", "kV", "kA"]
    if include_gravity:
        cols.append(np.cos(np.asarray(log.get("position", np.zeros_like(t)), dtype=float)))
        names.append("kG")
    X = np.vstack(cols).T

    n = X.shape[1]
    theta = np.zeros(n)
    P = np.eye(n) * 1e3
    for i in range(len(t)):
        x = X[i]
        denom = forgetting + x @ P @ x
        if denom <= 0:
            continue
        k = (P @ x) / denom
        theta = theta + k * (volts[i] - x @ theta)
        P = (P - np.outer(k, x @ P)) / forgetting

    pred = X @ theta
    ss_res = float(np.sum((volts - pred) ** 2))
    ss_tot = float(np.sum((volts - volts.mean()) ** 2)) or 1.0
    return FitResult(
        kS=float(theta[0]),
        kV=float(theta[1]),
        kA=float(theta[2]),
        kG=float(theta[3]) if n > 3 else 0.0,
        r2=1.0 - ss_res / ss_tot,
        rmse_volts=float(np.sqrt(ss_res / max(len(t), 1))),
        stderr={name: float("nan") for name in names},
        n_samples=len(t),
        notes=["recursive least squares", f"forgetting factor {forgetting}"],
    )


def compare(fit: FitResult, predicted: dict) -> dict:
    """Difference between the fitted gains and the CAD prediction."""
    out = {}
    for key in ("kS", "kV", "kA", "kG"):
        cad = float(predicted.get(key, 0.0))
        got = float(getattr(fit, key))
        out[key] = {
            "cad": cad,
            "fitted": got,
            "abs_diff": got - cad,
            "rel_diff": (got - cad) / cad if abs(cad) > 1e-12 else None,
        }
    out["r2"] = fit.r2
    out["rmse_volts"] = fit.rmse_volts
    return out
