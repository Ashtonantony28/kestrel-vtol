"""Load spec/vehicle.yaml (single source of truth) and derived mass properties."""
from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC_PATH = ROOT / "spec" / "vehicle.yaml"
CAD_MASS = ROOT / "cad" / "mass_props.json"


@lru_cache(maxsize=None)
def load(path: str | None = None) -> dict:
    with open(path or SPEC_PATH) as f:
        return yaml.safe_load(f)


def wing_geometry(spec: dict | None = None) -> dict:
    """Planform quantities of the trapezoidal swept wing (no winglets)."""
    w = (spec or load())["wing"]
    b, cr, ct = w["span"], w["root_chord"], w["tip_chord"]
    lam = ct / cr
    area = 0.5 * (cr + ct) * b
    mac = 2.0 / 3.0 * cr * (1 + lam + lam**2) / (1 + lam)
    y_mac = b / 6.0 * (1 + 2 * lam) / (1 + lam)
    x_le_mac = -y_mac * math.tan(math.radians(w["le_sweep_deg"]))  # FRD x (aft negative)
    return {
        "span": b, "area": area, "mac": mac, "y_mac": y_mac, "x_le_mac": x_le_mac,
        "aspect_ratio": b**2 / area, "taper": lam,
        "x_quarter_mac": x_le_mac - 0.25 * mac,
    }


def mass_properties(spec: dict | None = None, use_cad: bool = True) -> dict:
    """Total mass, CG and inertia (point masses + CAD inertia if available)."""
    s = spec or load()
    if use_cad and CAD_MASS.exists():
        return json.loads(CAD_MASS.read_text())
    items = s["mass_items"]
    m = sum(v["m"] for v in items.values())
    cg = np.array([
        sum(v["m"] * v.get("x", 0.0) for v in items.values()) / m,
        sum(v["m"] * v.get("y", 0.0) for v in items.values()) / m,
        sum(v["m"] * v.get("z", 0.0) for v in items.values()) / m,
    ])
    # Rough inertia: distributed items spread over their physical extent (point model underestimates roll).
    b = s["wing"]["span"]
    ry = s["lift_rotors"]["y"]
    ixx = (items["wing_foam"]["m"] + items["spar_and_glue"]["m"]) * b**2 / 12.0
    ixx += (items["lift_motors"]["m"] + items["lift_props"]["m"] + items["lift_escs"]["m"]
            + items["booms"]["m"] + items["printed_mounts"]["m"]) * ry**2
    ixx += items["winglets"]["m"] * (b / 2) ** 2
    dx = 0.5 * (s["lift_rotors"]["x_front"] - s["lift_rotors"]["x_rear"])
    iyy = 0.0
    for v in items.values():
        iyy += v["m"] * ((v.get("x", 0.0) - cg[0]) ** 2 + (v.get("z", 0.0) - cg[2]) ** 2)
    iyy += (items["lift_motors"]["m"] + items["lift_props"]["m"]) * dx**2
    iyy += items["wing_foam"]["m"] * s["wing"]["root_chord"] ** 2 / 12.0
    izz = ixx + iyy
    return {"mass": m, "cg": cg.tolist(), "inertia": [[ixx, 0, 0], [0, iyy, 0], [0, 0, izz]], "source": "spec"}
