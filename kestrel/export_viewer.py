"""Export the CAD assembly as one binary STL per part group for the web 3D viewer (cad/viewer/).

Verify: python -m kestrel.export_viewer   (writes cad/viewer/*.stl and parts.json)
"""
from __future__ import annotations

import json
from pathlib import Path

import build123d as bd

from . import cad
from . import spec as S

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "cad" / "viewer"

GROUPS = {
    "wing": ["wing_L", "wing_R"],
    "winglets": ["winglet_L", "winglet_R"],
    "pod": ["pod"],
    "booms": ["boom_L", "boom_R"],
    "spars": ["cross_spar", "outer_spar_L", "outer_spar_R"],
    "motors": ["motor_FR", "motor_RL", "motor_FL", "motor_RR", "pusher_motor"],
    "mounts": ["mount_FR", "mount_RL", "mount_FL", "mount_RR"],
    "legs": ["leg_FR", "leg_RL", "leg_FL", "leg_RR"],
    "props": ["propdisc_FR", "propdisc_RL", "propdisc_FL", "propdisc_RR", "propdisc_pusher"],
    "battery": ["battery"],
    "avionics": ["flight_controller", "gps", "receiver", "pusher_esc", "servos", "wiring"],
}


def main():
    s = S.load()
    parts, _ = cad.build(s)
    OUT.mkdir(parents=True, exist_ok=True)
    info = {}
    for g, names in GROUPS.items():
        comp = bd.Compound([parts[n] for n in names if n in parts])
        path = OUT / f"{g}.stl"
        bd.export_stl(comp.scale(1000.0), str(path), tolerance=0.4, angular_tolerance=0.25)   # mm
        info[g] = {"parts": [n for n in names if n in parts], "bytes": path.stat().st_size}
    mp = json.loads((ROOT / "cad" / "mass_props.json").read_text())
    t = json.loads((ROOT / "aero" / "tables.json").read_text())
    cg_flu = [mp["cg"][0] * 1000, -mp["cg"][1] * 1000, -mp["cg"][2] * 1000]
    info["_markers"] = {"cg_mm": cg_flu, "np_x_mm": -t["x_np_asb"] * 1000, "mass_kg": mp["mass"]}
    (OUT / "parts.json").write_text(json.dumps(info, indent=1))
    for g, v in info.items():
        print(g, v)


if __name__ == "__main__":
    main()
