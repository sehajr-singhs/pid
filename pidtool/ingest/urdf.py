"""Generic ingest path: URDF.

URDF is the lowest-friction way to get a CAD model into Piπd without an Onshape
API key. We read mass and inertia from <inertial> and turns joints into
mechanisms. URDF does not say which motor drives which joint, so the caller
supplies a {joint name: motor key} map (or takes the default).

Nothing here guesses a gear ratio: if the URDF has no <dynamics> or no
transmission element, ratio 1.0 is assumed and tagged `estimated`.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


def _floats(text: str | None) -> list[float]:
    if not text:
        return []
    return [float(x) for x in text.split()]


def _inertia(link: ET.Element) -> dict:
    inertial = link.find("inertial")
    out = {"mass_kg": 0.0, "inertia_kgm2": None}
    if inertial is None:
        return out
    m = inertial.find("mass")
    if m is not None:
        out["mass_kg"] = float(m.get("value", 0.0))
    i = inertial.find("inertia")
    if i is not None:
        # Full inertia tensor is available, but a single joint needs the inertia
        # about its own axis. We take the largest principal value as a bound and
        # let the caller override: a wrong axis silently produces wrong kA.
        out["inertia_kgm2"] = max(
            float(i.get("ixx", 0.0)), float(i.get("iyy", 0.0)), float(i.get("izz", 0.0))
        )
        out["tensor"] = {
            k: float(i.get(k, 0.0))
            for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        }
    return out


def parse_urdf(path: str | Path) -> dict:
    root = ET.parse(str(path)).getroot()
    links = []
    for link in root.findall("link"):
        info = _inertia(link)
        info["name"] = link.get("name")
        links.append(info)
    joints = []
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        dyn = joint.find("dynamics")
        joints.append(
            {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": (joint.find("parent").get("link") if joint.find("parent") is not None else None),
                "child": (joint.find("child").get("link") if joint.find("child") is not None else None),
                "axis": _floats(joint.find("axis").get("xyz") if joint.find("axis") is not None else None),
                "lower": float(limit.get("lower", 0.0)) if limit is not None else 0.0,
                "upper": float(limit.get("upper", 0.0)) if limit is not None else 0.0,
                "damping": float(dyn.get("damping", 0.0)) if dyn is not None else 0.0,
                "friction": float(dyn.get("friction", 0.0)) if dyn is not None else 0.0,
            }
        )
    return {"links": links, "joints": joints}


def robot_from_urdf(
    path: str | Path,
    *,
    motor_map: dict[str, str] | None = None,
    default_motor: str = "gobilda-5203-2402-0014",
    default_ratio: float = 1.0,
    name: str | None = None,
) -> dict:
    """Turn a URDF into a pid robot.json document.

    Joint type mapping:
      continuous            -> flywheel (no limits, speed controlled)
      revolute              -> turret (limits from URDF, position controlled)
      prismatic             -> elevator
    """
    parsed = parse_urdf(path)
    link_i = {l["name"]: l for l in parsed["links"]}
    mass_total = sum(l["mass_kg"] for l in parsed["links"]) or 1.0
    mechanisms = []
    for j in parsed["joints"]:
        child = link_i.get(j["child"] or "", {})
        inertia = child.get("inertia_kgm2") or 1e-4
        mech = {
            "name": j["name"],
            "motor": (motor_map or {}).get(j["name"], default_motor),
            "count": 1,
            "gearing": {"ratio": default_ratio, "efficiency": 0.85, "source": "estimated"},
            "inertia_kgm2": inertia,
            "notes": ["ratio 1.0 assumed: URDF has no transmission element"],
        }
        if j["type"] == "continuous":
            mech["type"] = "flywheel"
        elif j["type"] == "prismatic":
            mech["type"] = "elevator"
        else:
            mech["type"] = "turret"
            lo, hi = j["lower"], j["upper"]
            if hi > lo:
                mech["limits_rad"] = [lo, hi]
        mechanisms.append(mech)
    return {
        "schema_version": 1,
        "name": name or Path(path).stem,
        "generator": "pidtool.ingest.urdf",
        "mass_kg": mass_total,
        "provenance": {"mass_kg": "cad"},
        "mechanisms": mechanisms,
        "links": parsed["links"],
        "joints": parsed["joints"],
    }
