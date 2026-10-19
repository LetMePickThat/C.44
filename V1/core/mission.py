#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MISSION -- mission preparation data model: sector templates, mission-prep validation ("Validate mission prep"),
and JSON/CSV load/save, kept strictly compatible with the optimiser's input format (same field names, same box/cap
shapes) so a saved mission file runs unchanged through core.optimizer.run().

Active tracking (PA) and confirmation inputs, per sector (core/tracking.py):
  "tracking": list of range brackets {r_min_km, r_max_km, n_targets, rr_s, angle_zone_deg (opening = full width at
              +/-2 sigma), range_acc_m and vr_acc_mps (1 sigma)};
  "confirm_mode": "auto" (default) or a forced mode of tracking.CONFIRM_MODES;
  "new_targets_per_rr": true targets newly detected per refresh (confirmed like the false alarms).

Validation messages are operator-facing.
"""
from __future__ import annotations

import copy
import csv
import json
import math
import re
from typing import Dict, List

# Re-use the optimiser's loader so JSON/CSV parsing never diverges from what run() expects.
from . import optimizer as mo

GLOBAL_DEFAULTS: Dict[str, float] = {"eps_min_deg": -90.0, "eps_max_deg": 90.0, "face_elec_mw": 2.0}

_SECTOR_DEFAULTS_NEED: Dict[str, object] = {
    "rcs_m2": 1000.0, "swerling": 1, "range_km": 2000.0, "rr_target_s": 30.0,
    "v_close_min_mps": 1000.0, "v_close_max_mps": 5000.0,
    "confirm_mode": "auto", "new_targets_per_rr": 1.0, "tracking": [],
}
SECTOR_DEFAULTS_BOX: Dict[str, object] = {
    "shape": "box", "phi_min_deg": 0.0, "phi_max_deg": 90.0, "eps_min_deg": 0.0, "eps_max_deg": 60.0,
    **_SECTOR_DEFAULTS_NEED,
}
SECTOR_DEFAULTS_CAP: Dict[str, object] = {
    "shape": "cap", "phi0_deg": 0.0, "eps0_deg": 20.0, "radius_deg": 15.0, **_SECTOR_DEFAULTS_NEED,
}
BRACKET_DEFAULTS: Dict[str, float] = {
    "r_min_km": 0.0, "r_max_km": 1000.0, "n_targets": 5.0, "rr_s": 2.0,
    "angle_zone_deg": 0.2, "range_acc_m": 10.0, "vr_acc_mps": 5.0,
}
BRACKET_FIELDS = tuple(BRACKET_DEFAULTS)
CONFIRM_MODE_CHOICES = ("auto", "KU1+7", "KU7", "S2")   # = tracking.CONFIRM_MODES, plus automatic selection

_ID_RE = re.compile(r"^[A-Za-z0-9_\-]+$")


def empty_mission() -> dict:
    return {"global": dict(GLOBAL_DEFAULTS), "sectors": []}


def new_sector_id(existing: List[str]) -> str:
    n = 1
    existing_set = set(existing)
    while f"S{n:02d}" in existing_set:
        n += 1
    return f"S{n:02d}"


def new_sector(existing_ids: List[str], shape: str = "box") -> dict:
    template = SECTOR_DEFAULTS_BOX if shape == "box" else SECTOR_DEFAULTS_CAP
    sec = copy.deepcopy(template)
    sec["id"] = new_sector_id(existing_ids)
    return sec


def duplicate_sector(sec: dict, existing_ids: List[str]) -> dict:
    dup = copy.deepcopy(sec)
    dup["id"] = new_sector_id(existing_ids)
    return dup


def new_bracket(sec: dict) -> dict:
    """A PA range bracket that fills the first gap of the sector's brackets: above the farthest one up to the
    required range, else below the nearest one; with no gap, the farthest bracket is split in two (the new one is
    its near half). Other values are copied from the farthest bracket (defaults if none)."""
    brackets = sec.get("tracking", [])
    r_req = float(sec.get("range_km", BRACKET_DEFAULTS["r_max_km"]))
    if not brackets:
        return {**BRACKET_DEFAULTS, "r_min_km": 0.0, "r_max_km": r_req}
    far = max(brackets, key=lambda b: b["r_max_km"])
    near = min(brackets, key=lambda b: b["r_min_km"])
    b = dict(far)
    if far["r_max_km"] < r_req:
        b["r_min_km"], b["r_max_km"] = far["r_max_km"], r_req
    elif near["r_min_km"] > 0:
        b["r_min_km"], b["r_max_km"] = 0.0, near["r_min_km"]
    else:
        mid = round((far["r_min_km"] + far["r_max_km"]) / 2.0)
        b["r_min_km"], b["r_max_km"] = far["r_min_km"], mid
        far["r_min_km"] = mid
    return b


# ----------------------------------------------------------------------------- validation
def validate_global(g: dict) -> List[str]:
    errs = []
    eps_min = g.get("eps_min_deg", -90.0)
    eps_max = g.get("eps_max_deg", 90.0)
    if not (-90.0 <= eps_min <= 90.0):
        errs.append(f"rear limit eps_min ({eps_min}) outside [-90, 90] deg")
    if not (-90.0 <= eps_max <= 90.0):
        errs.append(f"front limit eps_max ({eps_max}) outside [-90, 90] deg")
    if eps_min > eps_max:
        errs.append(f"eps_min ({eps_min}) > eps_max ({eps_max}): empty scan window")
    elec = g.get("face_elec_mw", 2.0)
    if elec <= 0:
        errs.append(f"electrical power per face ({elec} MW) must be > 0")
    elif elec > 8.0:
        errs.append(f"electrical power per face ({elec} MW) exceeds the ship total budget (8 MW)")
    return errs


def validate_sector(sec: dict, index: int) -> List[str]:
    errs = []
    tag = f"sector #{index + 1}"
    sid = sec.get("id")
    if not sid:
        errs.append(f"{tag}: missing id")
    elif not _ID_RE.match(str(sid)):
        errs.append(f"{tag} ({sid}): invalid id (letters/digits/_/- only)")
    else:
        tag = f"sector {sid}"

    shape = sec.get("shape")
    if shape not in ("box", "cap"):
        errs.append(f"{tag}: unknown shape '{shape}' (expected box or cap)")
    elif shape == "box":
        for f in ("phi_min_deg", "phi_max_deg", "eps_min_deg", "eps_max_deg"):
            if f not in sec:
                errs.append(f"{tag}: missing field '{f}' for a box shape")
        if "eps_min_deg" in sec and "eps_max_deg" in sec:
            if not (-90.0 <= sec["eps_min_deg"] <= 90.0):
                errs.append(f"{tag}: eps_min_deg ({sec['eps_min_deg']}) outside [-90, 90] deg")
            if not (-90.0 <= sec["eps_max_deg"] <= 90.0):
                errs.append(f"{tag}: eps_max_deg ({sec['eps_max_deg']}) outside [-90, 90] deg")
            if sec["eps_min_deg"] >= sec["eps_max_deg"]:
                errs.append(f"{tag}: eps_min_deg >= eps_max_deg (empty sector)")
        if "phi_min_deg" in sec and "phi_max_deg" in sec:
            span = sec["phi_max_deg"] - sec["phi_min_deg"]
            # phi_min > phi_max is a legitimate wrap through 0 (e.g. 315 -> 45); only a zero-width span is empty
            if span < 360.0 and math.isclose(sec["phi_min_deg"] % 360.0, sec["phi_max_deg"] % 360.0, abs_tol=1e-9):
                errs.append(f"{tag}: phi_min_deg = phi_max_deg (zero-width azimuth span, empty sector; "
                            f"use 0 -> 360 for a full turn)")
    elif shape == "cap":
        for f in ("phi0_deg", "eps0_deg", "radius_deg"):
            if f not in sec:
                errs.append(f"{tag}: missing field '{f}' for a cap shape")
        if "eps0_deg" in sec and not (-90.0 <= sec["eps0_deg"] <= 90.0):
            errs.append(f"{tag}: eps0_deg ({sec['eps0_deg']}) outside [-90, 90] deg")
        if "radius_deg" in sec and not (0.0 < sec["radius_deg"] <= 180.0):
            errs.append(f"{tag}: radius_deg ({sec['radius_deg']}) must be in ]0, 180] deg")

    if sec.get("rcs_m2", 0) is None or sec.get("rcs_m2", 0) <= 0:
        errs.append(f"{tag}: rcs_m2 must be > 0")
    if sec.get("range_km", 0) is None or sec.get("range_km", 0) <= 0:
        errs.append(f"{tag}: range_km must be > 0")
    if sec.get("rr_target_s", 0) is None or sec.get("rr_target_s", 0) <= 0:
        errs.append(f"{tag}: rr_target_s must be > 0")
    vmin, vmax = sec.get("v_close_min_mps"), sec.get("v_close_max_mps")
    if vmin is None or vmax is None:
        errs.append(f"{tag}: missing v_close_min_mps / v_close_max_mps")
    else:
        if vmin < 0 or vmax < 0:
            errs.append(f"{tag}: negative closing speeds")
        if vmax <= 0:
            errs.append(f"{tag}: v_close_max_mps must be > 0 (sizes migration, Doppler and time to impact)")
        if vmin > vmax:
            errs.append(f"{tag}: v_close_min_mps ({vmin}) > v_close_max_mps ({vmax})")
    sw = sec.get("swerling", 1)
    if sw not in (0, 1, 2, 3, 4):
        errs.append(f"{tag}: swerling ({sw}) must be 0..4")
    res_max = sec.get("res_max_m")
    if res_max is not None and res_max <= 0:
        errs.append(f"{tag}: res_max_m must be > 0")
    errs.extend(validate_tracking(sec, tag))
    return errs


def validate_tracking(sec: dict, tag: str) -> List[str]:
    """Confirmation mode, new targets per refresh and the PA range brackets of one sector."""
    errs = []
    mode = sec.get("confirm_mode", "auto")
    if mode not in CONFIRM_MODE_CHOICES:
        errs.append(f"{tag}: unknown confirmation mode '{mode}' (expected {', '.join(CONFIRM_MODE_CHOICES)})")
    n_new = sec.get("new_targets_per_rr", 1.0)
    if n_new is None or n_new < 0:
        errs.append(f"{tag}: new_targets_per_rr must be >= 0")
    brackets = sec.get("tracking", [])
    if not isinstance(brackets, list):
        return errs + [f"{tag}: tracking must be a list of range brackets"]
    r_req = sec.get("range_km")
    spans = []
    for i, b in enumerate(brackets):
        bt = f"{tag}, PA bracket #{i + 1}"
        missing = [f for f in BRACKET_FIELDS if b.get(f) is None]
        if missing:
            errs.append(f"{bt}: missing {', '.join(missing)}")
            continue
        if b["r_min_km"] < 0 or b["r_max_km"] <= b["r_min_km"]:
            errs.append(f"{bt}: needs 0 <= r_min_km < r_max_km ({b['r_min_km']:g} -> {b['r_max_km']:g} km)")
        elif r_req is not None and b["r_max_km"] > r_req:
            errs.append(f"{bt}: r_max_km ({b['r_max_km']:g}) beyond the required detection range ({r_req:g} km): "
                        f"targets are tracked only once detected")
        else:
            spans.append((b["r_min_km"], b["r_max_km"], i + 1))
        if b["n_targets"] < 0:
            errs.append(f"{bt}: number of targets must be >= 0")
        for f, label in (("rr_s", "update period"), ("angle_zone_deg", "angular zone opening"),
                         ("range_acc_m", "range accuracy"), ("vr_acc_mps", "velocity accuracy")):
            if b[f] <= 0:
                errs.append(f"{bt}: {label} ({f}) must be > 0")
    spans.sort()
    for (a0, a1, ia), (b0, b1, ib) in zip(spans, spans[1:]):
        if b0 < a1:
            errs.append(f"{tag}: PA brackets #{ia} and #{ib} overlap ({a0:g}-{a1:g} / {b0:g}-{b1:g} km)")
    return errs


def validate_mission(mission: dict) -> List[str]:
    """Full mission-prep consistency check ('Validate mission prep'): unique ids, bounds, phi/eps, v_min<=v_max,
    rr/range > 0, etc. Returns a list of error messages (empty = mission prep valid)."""
    errs = list(validate_global(mission.get("global", {})))
    sectors = mission.get("sectors", [])
    if not sectors:
        errs.append("no sector defined")
    ids = [s.get("id") for s in sectors]
    dupes = {i for i in ids if i and ids.count(i) > 1}
    if dupes:
        errs.append(f"duplicate sector ids: {', '.join(sorted(dupes))}")
    for i, sec in enumerate(sectors):
        errs.extend(validate_sector(sec, i))
    return errs


# ----------------------------------------------------------------------------- I/O
def load_mission(path: str) -> dict:
    """Mission file (JSON or CSV) with its global parameters completed by the defaults (a CSV has none)."""
    mission = mo.load_mission(path)
    mission.setdefault("global", dict(GLOBAL_DEFAULTS))
    return mission


def save_mission_json(path: str, mission: dict) -> None:
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(mission, fh, indent=2, ensure_ascii=False)


_CSV_FIELDS = [
    "id", "shape", "phi_min_deg", "phi_max_deg", "eps_min_deg", "eps_max_deg",
    "phi0_deg", "eps0_deg", "radius_deg", "rcs_m2", "swerling", "range_km", "rr_target_s",
    "v_close_min_mps", "v_close_max_mps", "res_max_m", "allow_ambiguous",
    "confirm_mode", "new_targets_per_rr", "tracking",
]


def save_mission_csv(path: str, mission: dict) -> None:
    """Sector table only (global parameters have no column in mission_example.csv's format;
    they stay at the optimiser's defaults when a CSV is reloaded -- same as the provided example).
    The PA brackets go in one column as a JSON list."""
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=_CSV_FIELDS, extrasaction="ignore")
        w.writeheader()
        for sec in mission.get("sectors", []):
            row = dict(sec)
            row["tracking"] = json.dumps(sec["tracking"]) if sec.get("tracking") else ""
            w.writerow(row)
