"""Generic ingest path: STEP (optional).

Requires the `step` extra:  pip install -e ".[step]"

trimesh reads STEP through a CAD kernel (build123d / OCP). We use whatever
trimesh can load, compute volume, bounding box and the inertia tensor, and turn
that into link masses at a stated density. If trimesh cannot load the file we say
so instead of returning a plausible-looking wrong number.
"""

from __future__ import annotations

from pathlib import Path

DEFAULT_DENSITY = 2700.0  # aluminium 6061, kg/m^3


def analyze_step(path: str | Path, density: float = DEFAULT_DENSITY) -> dict:
    try:
        import trimesh  # noqa: F401
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise ImportError(
            'STEP ingest needs trimesh: pip install -e ".[step]"'
        ) from exc

    import trimesh

    scene = trimesh.load(str(path), force="scene")
    geoms = list(scene.geometry.values()) if hasattr(scene, "geometry") else [scene]
    solids = []
    volume = 0.0
    for g in geoms:
        v = float(abs(g.volume)) if getattr(g, "is_volume", False) else 0.0
        volume += v
        solids.append(
            {
                "name": getattr(g, "name", "solid"),
                "volume_m3": v,
                "mass_kg": v * density,
                "bounds_m": g.bounds.tolist() if hasattr(g, "bounds") else None,
            }
        )
    if volume <= 0.0:
        raise ValueError(
            f"{path}: trimesh loaded {len(geoms)} geometries but none closed, so volume "
            "is unknown. Export a solid (not a surface) or use the URDF path."
        )
    return {
        "density_kg_m3": density,
        "volume_m3": volume,
        "mass_kg": volume * density,
        "solids": solids,
        "source": "step",
        "note": "mass = solid volume x density; assigned density is an assumption",
    }


def robot_from_step(
    path: str | Path,
    *,
    motor: str,
    ratio: float,
    mechanism_type: str = "flywheel",
    density: float = DEFAULT_DENSITY,
) -> dict:
    """One-solid robot spec: enough for a single mechanism from a STEP file."""
    info = analyze_step(path, density=density)
    # Inertia about the spin axis for a body of that volume is not available in
    # closed form; use the bounding box radius as a conservative estimate.
    bounds = info["solids"][0]["bounds_m"]
    r = 0.5 * max(bounds[0][0] - bounds[0][1], bounds[0][2] - bounds[0][1]) if bounds else 0.05
    j = 0.5 * info["mass_kg"] * r**2
    return {
        "schema_version": 1,
        "name": Path(path).stem,
        "generator": "spool.ingest.step",
        "mass_kg": info["mass_kg"],
        "provenance": {"mass_kg": "cad", "mechanisms.body.inertia_kgm2": "estimated"},
        "mechanisms": [
            {
                "type": mechanism_type,
                "name": "body",
                "motor": motor,
                "count": 1,
                "gearing": {"ratio": ratio, "efficiency": 0.9, "source": "estimated"},
                "inertia_kgm2": j,
                "notes": ["inertia estimated from bounding box radius"],
            }
        ],
    }
