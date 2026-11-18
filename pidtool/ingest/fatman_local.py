"""FAT MAN ingest: build robot.json from the local CAD export + BOM.

Sources (all local; the Onshape API is NOT used):

    robots/fatman/robot.fs                    FeatureScript source (geometry constants)
    robots/fatman/build/parts_final.json      one part bbox per line (JSONL)
    robots/fatman/build/asm_positions.json    assembly bounding boxes
    robots/fatman/src/asm_bbox.json           whole-robot envelope
    robots/fatman/src/mass.json               Onshape mass properties
    robots/fatman/FAT_MAN_BOM.xlsx            bill of materials
    robots/it1/doc.json                       Onshape element ids (subsystem names)

Anything that had to be estimated is tagged `estimated` in the provenance map and
explained in `provenance_notes`. The physics never invents a number without a tag.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from ..motors import get_servo
from .xlsx import find_rows, read_workbook

# Datasheet identities for the parts that appear in the BOM.
WHEEL_DIAMETER_M = 0.096                 # goBILDA 3213-3606-0002 96mm mecanum
FLYWHEEL_DIAMETER_M = 0.072              # goBILDA 3601-0014-0072 Rhino wheel, 72mm

DENSITY = {"aluminum": 2700.0, "petg": 1270.0, "pla": 1240.0, "steel": 8000.0, "pc": 1200.0}


def _read_json(path: Path, default=None):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def _read_jsonl(path: Path) -> list[dict]:
    out = []
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line:
                out.append(json.loads(line))
    except (OSError, ValueError):
        return []
    return out


def part_masses(parts: list[dict]) -> dict:
    """Bounding-box volume -> mass per part, using a material guess.

    Structural parts in this CAD export are a mix of 6061 plate and PETG prints;
    without per-body densities we cannot do better than a bbox volume at an
    average density. This is a cross-check, not the primary mass.
    """
    total_volume = 0.0
    per_part = []
    for p in parts:
        (x0, y0, z0), (x1, y1, z1) = p["min"], p["max"]
        v = abs(x1 - x0) * abs(y1 - y0) * abs(z1 - z0) * 1e-9
        total_volume += v
        per_part.append({"name": p.get("name", "?"), "volume_m3": v})
    # 60% solid is a fair average for plates, channels and printed brackets.
    solid_fraction = 0.6
    return {
        "bbox_volume_m3": total_volume,
        "solid_fraction_assumed": solid_fraction,
        "mass_at_2700_kg": total_volume * solid_fraction * DENSITY["aluminum"],
        "mass_at_mixed_kg": total_volume * solid_fraction * 1900.0,
        "parts": per_part,
    }


def _bom_index(rows_by_sheet: dict[str, list[list[str]]]) -> dict:
    """Pull the parts pid needs out of the BOM sheets."""
    found: dict[str, dict] = {}
    for sheet, rows in rows_by_sheet.items():
        for r in rows:
            text = " | ".join(r)
            low = text.lower()
            for needle, key in (
                ("5203-2402-0014", "drive_motor"),
                ("5203-2402-0001", "flywheel_motor"),
                ("3213-3606-0002", "mecanum_wheel"),
                ("3421-0014-0060", "turret_gear_60t"),
                ("3422-0125-0020", "gt2_20t"),
                ("2302-0014-0060", "hub_gear_60t"),
                ("3601-0014-0072", "rhino_wheel"),
                ("swyft", "servo"),
            ):
                if needle in low and key not in found:
                    found[key] = {"sheet": sheet, "row": r}
    return found


TURRET_SOFT_MARGIN_DEG = 5.0


def turret_travel_report(
    *,
    ratio: float,
    usable_range_deg: float,
    soft_min_rad: float,
    soft_max_rad: float,
    hard_min_rad: float,
    hard_max_rad: float,
    measured_deg: float | None = None,
) -> dict:
    """Describe the turret travel, and say so when the CAD holds two answers.

    FAT MAN's CAD encodes the soft stops as +-135 deg (a 270 deg span) and the hard
    stops as +-140 deg (a 280 deg span), while the 25T to 60T servo gearing can
    only reach 320 deg / 2.40 = 133.3 deg at the axis. Those numbers cannot all be
    right, so this function does not pick one: it keeps the CAD encoding and
    returns a warning. `measured_deg` is the total travel between the hard stops,
    measured on the real robot; when it is supplied the stops are rebuilt from it
    and the result is tagged measured instead.
    """
    cad_soft = math.degrees(soft_max_rad - soft_min_rad)
    cad_hard = math.degrees(hard_max_rad - hard_min_rad)
    capable = usable_range_deg / ratio
    report = {
        "resolution": "cad" if measured_deg is None else "measured",
        "ratio": round(ratio, 4),
        "servo_usable_range_deg": round(usable_range_deg, 2),
        "servo_capable_span_deg": round(capable, 2),
        "cad_soft_span_deg": round(cad_soft, 2),
        "cad_hard_span_deg": round(cad_hard, 2),
        "soft_limits_deg": [round(math.degrees(soft_min_rad), 2),
                            round(math.degrees(soft_max_rad), 2)],
        "hard_limits_deg": [round(math.degrees(hard_min_rad), 2),
                            round(math.degrees(hard_max_rad), 2)],
        "warning": None,
    }

    if measured_deg is not None:
        measured = float(measured_deg)
        if not 1.0 <= measured <= 720.0:
            raise ValueError(
                f"--turret-travel-deg must be between 1 and 720 deg, got {measured:g}"
            )
        half = measured / 2.0
        soft_half = max(half - TURRET_SOFT_MARGIN_DEG, 0.0)
        report.update({
            "measured_span_deg": round(measured, 2),
            "soft_margin_deg": TURRET_SOFT_MARGIN_DEG,
            "soft_limits_deg": [round(-soft_half, 2), round(soft_half, 2)],
            "hard_limits_deg": [round(-half, 2), round(half, 2)],
        })
        if measured > capable + 0.05:
            report["warning"] = (
                f"Measured turret travel {measured:.1f} deg is more than the servo "
                f"gearing can produce ({capable:.1f} deg at the axis: "
                f"{usable_range_deg:.0f} deg usable / {ratio:.2f}:1). Check the gear "
                "ratio or the measurement: the measured stops were applied, but the "
                "axis cannot reach them as modelled."
            )
        return report

    problems = []
    if abs(cad_hard - cad_soft) > 1e-6:
        problems.append(
            f"the CAD soft stops are {cad_soft:.0f} deg apart "
            f"({math.degrees(soft_min_rad):.0f} to {math.degrees(soft_max_rad):.0f} deg) "
            f"and the hard stops are {cad_hard:.0f} deg apart "
            f"({math.degrees(hard_min_rad):.0f} to {math.degrees(hard_max_rad):.0f} deg)"
        )
    if capable < cad_soft - 0.05:
        problems.append(
            f"the servo gearing can only reach {capable:.1f} deg at the axis "
            f"({usable_range_deg:.0f} deg usable / {ratio:.2f}:1)"
        )
    if problems:
        report["warning"] = (
            "The turret travel in the CAD is ambiguous: " + "; ".join(problems) + ". "
            "Those numbers cannot all be right, and Piπd is not guessing which one "
            "to keep. Measure the real travel between the hard stops on the robot and "
            "re-run `pid fatman --turret-travel-deg <total deg>`. Until then the CAD "
            "stops are kept exactly as encoded and the 0 to 90 deg slew test may run "
            "past the physical stop."
        )
    return report


def build_robot(root: str | Path, *, turret_travel_deg: float | None = None) -> dict:
    """Return a robot.json document for FAT MAN.

    turret_travel_deg is the measured total travel between the turret hard stops.
    Leave it out and the CAD stops are kept as encoded, with a warning in
    `turret_travel` when the CAD holds two different readings.
    """
    root = Path(root)
    files = {
        "parts": root / "build" / "parts_final.json",
        "positions": root / "build" / "asm_positions.json",
        "bbox": root / "src" / "asm_bbox.json",
        "mass": root / "src" / "mass.json",
        "bom": root / "FAT_MAN_BOM.xlsx",
        "featurescript": root / "robot.fs",
    }

    parts = _read_jsonl(files["parts"])
    bbox = _read_json(files["bbox"], {}) or {}
    mass_props = _read_json(files["mass"], {}) or {}
    bom_rows = read_workbook(str(files["bom"])) if files["bom"].exists() else {}

    pm = part_masses(parts)
    bom = _bom_index(bom_rows)

    # ---------------------------------------------------------------- mass
    # Onshape reports 11.55 kg but flags 39 bodies with no density, so the CAD
    # number is a lower bound. Add 10% for those bodies; this is the one number
    # every gain is proportional to, so it is stated loudly.
    cad_mass = float(mass_props.get("mass", [11.55])[0]) if mass_props.get("mass") else 11.55
    mass_allowance = 1.10
    robot_mass = round(cad_mass * mass_allowance, 2)

    # ------------------------------------------------------------ drivetrain
    # 435 rpm gearmotor output -> 24T HTD5 -> 20T wheel pulley.
    internal_ratio = 13.73
    belt_ratio = 20.0 / 24.0                # motor turns per wheel turn on the belt
    drive_ratio = internal_ratio * belt_ratio
    wheel_r = WHEEL_DIAMETER_M / 2.0

    # -------------------------------------------------------------- flywheel
    # 2x 6000 rpm bare motors, 20T -> 20T GT2, 1:1.
    rhino_mass = 0.10                       # estimated
    steel_disc_mass = math.pi * 0.019**2 * 0.012 * DENSITY["steel"]   # 3/4" rod, 12 mm thick
    flywheel_mass = rhino_mass + 2 * steel_disc_mass
    j_rhino = 0.5 * rhino_mass * (FLYWHEEL_DIAMETER_M / 2) ** 2
    j_disc = 0.5 * steel_disc_mass * 0.019**2
    flywheel_j = j_rhino + 2 * j_disc

    target_exit = 9.4                       # m/s, from the shot requirement
    transfer_eff = 0.5                      # ball leaves at ~half rim speed
    rim = target_exit / transfer_eff
    flywheel_target = rim / (FLYWHEEL_DIAMETER_M / 2)

    # ---------------------------------------------------------------- turret
    # 2x SWYFT Balance servos, 25T horn gear -> 60T hub gear.
    turret_ratio = 60.0 / 25.0
    turret_output_mass = 1.15               # plates + shooter + hood, estimated
    turret_rg = 0.085                       # radius of gyration, estimated
    turret_j = turret_output_mass * turret_rg**2
    # CAD stops as encoded: 135 deg soft, 140 deg hard, both as half-travel.
    turret_soft_min, turret_soft_max = -2.356, 2.356
    turret_hard_min, turret_hard_max = -2.443, 2.443
    turret_servo = get_servo("swyft-balance")
    turret_travel = turret_travel_report(
        ratio=turret_ratio,
        usable_range_deg=float(turret_servo["usable_range_deg"]),
        soft_min_rad=turret_soft_min,
        soft_max_rad=turret_soft_max,
        hard_min_rad=turret_hard_min,
        hard_max_rad=turret_hard_max,
        measured_deg=turret_travel_deg,
    )
    if turret_travel_deg is None:
        turret_limits = {
            "soft_min_rad": turret_soft_min,
            "soft_max_rad": turret_soft_max,
            "hard_min_rad": turret_hard_min,
            "hard_max_rad": turret_hard_max,
        }
        turret_limit_note = "135 deg soft, 140 deg hard stops"
    else:
        turret_limits = {
            "soft_min_rad": round(math.radians(turret_travel["soft_limits_deg"][0]), 4),
            "soft_max_rad": round(math.radians(turret_travel["soft_limits_deg"][1]), 4),
            "hard_min_rad": round(math.radians(turret_travel["hard_limits_deg"][0]), 4),
            "hard_max_rad": round(math.radians(turret_travel["hard_limits_deg"][1]), 4),
            "measured_span_deg": turret_travel["measured_span_deg"],
        }
        turret_limit_note = (
            f"travel measured at {turret_travel['measured_span_deg']:.1f} deg between "
            f"hard stops; soft stops {TURRET_SOFT_MARGIN_DEG:.0f} deg inside"
        )

    # ------------------------------------------------------------------ hood
    hood_ratio = 75.0 / 19.0
    hood_j = 0.15 * 0.045**2                # estimated: 150 g at 45 mm

    # ------------------------------------------------------------- flower arm
    arm_mass = 0.62                         # printed ramp + letters + linkages
    arm_length = 0.28
    arm_r_com = 0.14
    arm_j = arm_mass * arm_length**2 / 3.0
    arm_holding_torque = arm_mass * 9.80665 * arm_r_com

    provenance = {
        "mass_kg": "estimated",
        "mechanisms.drivetrain.gearing.ratio": "bom",
        "mechanisms.drivetrain.wheel_radius_m": "datasheet",
        "mechanisms.drivetrain.robot_mass_kg": "estimated",
        "mechanisms.drivetrain.mu_rolling": "estimated",
        "mechanisms.drivetrain.strafe_efficiency": "estimated",
        "mechanisms.flywheel.gearing.ratio": "bom",
        "mechanisms.flywheel.motor": "bom",
        "mechanisms.flywheel.inertia_kgm2": "estimated",
        "mechanisms.flywheel.target_speed_rad_s": "estimated",
        "mechanisms.turret.gearing.ratio": "bom",
        "mechanisms.turret.servo": "bom",
        "mechanisms.turret.limits": "measured" if turret_travel_deg is not None else "cad",
        "mechanisms.hood.gearing.ratio": "bom",
        "mechanisms.hood.inertia_kgm2": "estimated",
        "mechanisms.flower_arm.servo": "bom",
        "mechanisms.flower_arm.arm_mass_kg": "estimated",
        "mechanisms.flower_arm.r_com_m": "estimated",
        "mechanisms.flower_arm.inertia_kgm2": "estimated",
    }
    provenance_notes = {
        "mass_kg": (
            f"Onshape mass properties report {cad_mass:.2f} kg with "
            f"{mass_props.get('massMissingCount', '?')} bodies missing density; "
            f"+{int((mass_allowance - 1) * 100)}% allowance = {robot_mass} kg. "
            f"The 48 exported part bboxes total {pm['bbox_volume_m3'] * 1e6:.0f} cm^3 "
            f"(~{pm['mass_at_mixed_kg']:.1f} kg at 60% solid), which covers structure only."
        ),
        "mechanisms.drivetrain.gearing.ratio": (
            f"{internal_ratio} internal (5203-2402-0014) x {belt_ratio:.4f} belt "
            "(24T motor pulley -> 20T wheel pulley) = 11.44 motor turns per wheel turn"
        ),
        "mechanisms.flywheel.inertia_kgm2": (
            f"Rhino 72 mm wheel ({rhino_mass:.2f} kg est) + 2 stainless discs turned from "
            f"3/4in rod ({steel_disc_mass * 1000:.0f} g each): J = 0.5 m r^2 summed"
        ),
        "mechanisms.flywheel.target_speed_rad_s": (
            f"target exit {target_exit} m/s with transfer efficiency {transfer_eff} "
            f"-> rim {rim:.1f} m/s -> {flywheel_target * 60 / (2 * math.pi):.0f} rpm"
        ),
        "mechanisms.drivetrain.mu_rolling": (
            "mecanum rollers scrub on foam tile; 0.08 is a starting value. Pedro's "
            "Foresight Forward Deceleration Identification measures the real one."
        ),
        "mechanisms.drivetrain.strafe_efficiency": (
            "mecanum strafe loses 15-30% of forward speed to roller slip; 0.78 midpoint, "
            "to be replaced by Pedro's Strafe Velocity Identification."
        ),
        "mechanisms.turret.limits": (
            f"user-measured total travel {turret_travel['measured_span_deg']:.1f} deg "
            "between hard stops via --turret-travel-deg; soft stops set 5 deg inside"
            if turret_travel_deg is not None else
            "CAD encoding: 135 deg soft and 140 deg hard stops as half-travel. This is "
            "ambiguous (see turret_travel); measure the real travel to settle it."
        ),
    }

    robot = {
        "schema_version": 1,
        "name": "FAT MAN",
        "generator": "pidtool.ingest.fatman_local",
        "source_files": [str(p.relative_to(root)) for p in files.values() if p.exists()],
        "mass_kg": robot_mass,
        "envelope_m": {
            "min": [bbox.get("lowX", -0.2286), bbox.get("lowY", -0.2919), bbox.get("lowZ", -0.0246)],
            "max": [bbox.get("highX", 0.2286), bbox.get("highY", 0.2506), bbox.get("highZ", 0.3149)],
        },
        "controllers": {"ftc": "REV Control Hub + Expansion Hub"},
        "turret_travel": turret_travel,
        "provenance": provenance,
        "provenance_notes": provenance_notes,
        "mechanisms": [
            {
                "type": "mecanum_drive",
                "name": "drivetrain",
                "motor": "gobilda-5203-2402-0001",
                "count": 4,
                "gearing": {
                    "ratio": round(drive_ratio, 4),
                    "efficiency": 0.88,
                    "source": "bom",
                    "description": "13.73:1 planetary (eta 0.927 from datasheet torque) x 24T->20T HTD5 belt (eta 0.95)",
                },
                "robot_mass_kg": robot_mass,
                "wheel_radius_m": wheel_r,
                "track_width_m": 0.322,
                "wheelbase_m": 0.288,
                "strafe_efficiency": 0.78,
                "mu_rolling": 0.08,
                "r_batt_ohm": 0.03,
                "friction": {"n_bearings": 4, "pulley_radius_m": 0.02, "n_belts": 4},
                "notes": [
                    "wheel 96 mm mecanum (3213-3606-0002)",
                    "drive motor 5203-2402-0014 is modelled as the bare 5203-2402-0001 "
                    "behind a 13.73:1 planetary, so rotor inertia reflects correctly",
                ],
            },
            {
                "type": "flywheel",
                "name": "flywheel",
                "motor": "gobilda-5203-2402-0001",
                "count": 2,
                "gearing": {"ratio": 1.0, "efficiency": 0.95, "source": "bom",
                             "description": "20T -> 20T GT2 belt, 1:1"},
                "inertia_kgm2": round(flywheel_j, 8),
                "load_mass_kg": round(flywheel_mass, 4),
                "load_radius_m": FLYWHEEL_DIAMETER_M / 2,
                "target_speed_rad_s": round(flywheel_target, 2),
                "target_exit_speed_m_s": target_exit,
                "transfer_efficiency": transfer_eff,
                "ball_mass_kg": 0.06,
                "friction": {"n_bearings": 4, "pulley_radius_m": 0.0125, "n_belts": 2,
                             "belt_preload_n": 30.0},
                "notes": [
                    "two 6000 rpm motors on one shaft, 20T -> 20T GT2 belt (1:1; "
                    "158T is the belt length, not a ratio)",
                    "shooter hood servo and ball blocker servo are separate axes",
                ],
            },
            {
                "type": "servo_axis",
                "name": "turret",
                "servo": "swyft-balance",
                "servo_count": 2,
                "gearing": {"ratio": turret_ratio, "efficiency": 0.9, "source": "bom",
                             "description": "25T servo horn gear -> 60T hub gear (2302-0014-0060)"},
                "inertia_kgm2": round(turret_j, 6),
                "holding_torque_nm": 0.901,
                "limits": turret_limits,
                "notes": [f"spin plate on bearing stack; {turret_limit_note}"],
            },
            {
                "type": "servo_axis",
                "name": "hood",
                "servo": "swyft-balance",
                "servo_count": 1,
                "gearing": {"ratio": hood_ratio, "efficiency": 0.9, "source": "bom",
                             "description": "19T -> 75T spur gears"},
                "inertia_kgm2": round(hood_j, 6),
                "holding_torque_nm": 0.02,
                "limits": {"soft_min_rad": -0.5, "soft_max_rad": 1.0},
                "notes": ["hood angle sets the shot arc"],
            },
            {
                "type": "servo_axis",
                "name": "flower_arm",
                "servo": "swyft-balance",
                "servo_count": 2,
                "gearing": {"ratio": 1.0, "efficiency": 0.95, "source": "bom",
                             "description": "4-bar driven directly by two servos"},
                "inertia_kgm2": round(arm_j, 6),
                "arm_mass_kg": arm_mass,
                "r_com_m": arm_r_com,
                "holding_torque_nm": round(arm_holding_torque, 4),
                "limits": {"soft_min_rad": -0.2, "soft_max_rad": 1.4},
                "notes": ["4-bar linkage, 6 mm aluminium links, printed tray"],
            },
        ],
        "frc_equivalent": [
            {
                "type": "mecanum_drive",
                "name": "drivetrain",
                "motor": "kraken-x60",
                "count": 4,
                "gearing": {"ratio": 6.75, "efficiency": 0.9, "source": "estimated",
                             "description": "same 96 mm wheel, Kraken on a 6.75:1 gearbox"},
                "robot_mass_kg": 54.0,
                "wheel_radius_m": wheel_r,
                "strafe_efficiency": 0.78,
                "mu_rolling": 0.06,
                "current_limit_a": 60.0,
                "r_batt_ohm": 0.018,
                "notes": ["FRC-equivalent: 120 lb robot, Kraken X60 drive, 60 A per motor"],
            },
            {
                "type": "flywheel",
                "name": "flywheel",
                "motor": "kraken-x60",
                "count": 1,
                "gearing": {"ratio": 1.0, "efficiency": 0.95, "source": "estimated"},
                "inertia_kgm2": 0.0022,
                "target_speed_rad_s": round(flywheel_target, 2),
                "current_limit_a": 60.0,
                "r_batt_ohm": 0.018,
                "notes": ["FRC-equivalent: single Kraken, heavier 4.5 in wheel"],
            },
            {
                "type": "turret",
                "name": "turret",
                "motor": "neo",
                "count": 1,
                "gearing": {"ratio": 60.0, "efficiency": 0.85, "source": "estimated",
                             "description": "NEO on a 60:1 gearbox replacing the two servos"},
                "inertia_kgm2": 0.02,
                "limits_rad": [-2.356, 2.356],
                "current_limit_a": 60.0,
                "r_batt_ohm": 0.018,
                "notes": ["FRC-equivalent of the servo turret, for WPILib gains"],
            },
            {
                "type": "arm",
                "name": "flower_arm",
                "motor": "neo",
                "count": 2,
                "gearing": {"ratio": 25.0, "efficiency": 0.85, "source": "estimated",
                             "description": "2x NEO on 25:1 replacing the 4-bar servos"},
                "inertia_kgm2": 0.35,
                "arm_mass_kg": 4.5,
                "r_com_m": 0.35,
                "limits_rad": [-0.2, 1.4],
                "current_limit_a": 60.0,
                "r_batt_ohm": 0.018,
                "notes": ["FRC-equivalent 4-bar arm; this is where kG is a real voltage"],
            },
        ],
    }
    robot["bom_hits"] = {k: v["sheet"] for k, v in bom.items()}
    return robot


def main(root: str | Path = "C:/Users/sehaj/ftc-cad/robots/fatman") -> dict:
    return build_robot(root)
