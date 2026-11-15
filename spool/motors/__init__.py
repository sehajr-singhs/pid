"""Motor and servo database access."""

from __future__ import annotations

import json
from functools import lru_cache
from importlib.resources import files

from ..physics.motor import Motor


@lru_cache(maxsize=1)
def load_db() -> dict:
    path = files("spool.motors").joinpath("db.json")
    return json.loads(path.read_text(encoding="utf-8"))


@lru_cache(maxsize=1)
def motors() -> dict[str, Motor]:
    db = load_db()["motors"]
    out: dict[str, Motor] = {}
    for key, m in db.items():
        out[key] = Motor(
            key=key,
            name=m["name"],
            v_nom=float(m.get("nominal_voltage_v", 12.0)),
            free_speed_rpm=float(m["free_speed_rpm"]),
            free_current_a=float(m["free_current_a"]),
            stall_current_a=float(m["stall_current_a"]),
            stall_torque_nm=float(m["stall_torque_nm"]),
            rotor_inertia_kgm2=float(m["rotor_inertia_kgm2"]),
            source=m.get("source", ""),
            verified=bool(m.get("verified", False)),
            notes=m.get("notes", ""),
        )
    return out


def get_motor(key: str) -> Motor:
    db = motors()
    if key not in db:
        raise KeyError(f"unknown motor {key!r}; known: {', '.join(sorted(db))}")
    return db[key]


def servos() -> dict[str, dict]:
    return load_db()["servos"]


def get_servo(key: str) -> dict:
    db = servos()
    if key not in db:
        raise KeyError(f"unknown servo {key!r}; known: {', '.join(sorted(db))}")
    return db[key]
