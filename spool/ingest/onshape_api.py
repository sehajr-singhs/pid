"""Optional Onshape API ingest -- OFF BY DEFAULT.

The team's free Onshape API calls are exhausted, so Spool never depends on this
path. It exists for teams that do have quota, and it is disabled unless both
environment variables are set:

    ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY
    ONSHAPE_DID / ONSHAPE_WID          (document + workspace ids)

The FAT MAN pipeline uses the local export instead (spool/ingest/fatman_local.py)
and never calls the network.
"""

from __future__ import annotations

import base64
import json
import os
import urllib.request
from urllib.error import HTTPError

API = "https://cad.onshape.com/api/v6"


def enabled() -> bool:
    return all(os.environ.get(k) for k in ("ONSHAPE_ACCESS_KEY", "ONSHAPE_SECRET_KEY"))


def _auth_header() -> str:
    raw = f"{os.environ['ONSHAPE_ACCESS_KEY']}:{os.environ['ONSHAPE_SECRET_KEY']}"
    return "Basic " + base64.b64encode(raw.encode()).decode()


def _get(path: str) -> dict:
    if not enabled():
        raise RuntimeError(
            "Onshape API is disabled: set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY. "
            "Spool uses local CAD exports by default and does not need it."
        )
    req = urllib.request.Request(f"{API}{path}", headers={"Authorization": _auth_header(),
                                                          "Accept": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            return json.loads(resp.read().decode())
    except HTTPError as exc:
        raise RuntimeError(f"Onshape API error {exc.code} on {path}: {exc.reason}") from exc


def assembly_bbox(did: str, wid: str, eid: str) -> dict:
    """Bounding box of an assembly element (metres)."""
    data = _get(f"/partstudios/d/{did}/w/{wid}/e/{eid}/boundingboxes")
    return data


def mass_properties(did: str, wid: str, eid: str) -> dict:
    """Mass properties (mass in kg, inertia in kg*m^2)."""
    return _get(f"/partstudios/d/{did}/w/{wid}/e/{eid}/massproperties")


def to_robot(did: str, wid: str, eid: str, name: str) -> dict:
    """Minimal robot spec from a live assembly. Mechanism tagging stays manual."""
    props = mass_properties(did, wid, eid)
    bbox = assembly_bbox(did, wid, eid)
    return {
        "schema_version": 1,
        "name": name,
        "generator": "spool.ingest.onshape_api",
        "mass_kg": float(props.get("mass", [0.0])[0]),
        "envelope_m": bbox,
        "provenance": {"mass_kg": "cad"},
        "mechanisms": [],
        "note": "Onshape API path: tag mechanisms before spool can compute gains",
    }
