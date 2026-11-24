"""Export the detailed assembly (kestrel.cad_detail) for the web 3D viewer (cad/viewer/).

Verify: python -m kestrel.export_viewer   (writes cad/viewer/model.json, parts.json and dims.json)
model.json maps each layer to an indexed mesh {v: base64 Float32 xyz, i: base64 Uint32} in mm (FLU, origin root LE).
"""
from __future__ import annotations

import base64
import json
from pathlib import Path

import numpy as np

from . import cad_detail as D
from . import dimensions as DIM
from . import spec as S

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "cad" / "viewer"

GROUPS = {
    "wing": ["wing_L", "wing_R"],
    "elevons": ["elevon_L", "elevon_R"],
    "winglets": ["winglet_L", "winglet_R"],
    "pod": ["pod", "hatch"],
    "booms": ["boom_L", "boom_R"],
    "spars": ["cross_spar", "outer_spar_L", "outer_spar_R", "torque_rod_L", "torque_rod_R"],
    "printed": ["mount_FR", "mount_RL", "mount_FL", "mount_RR", "leg_FR", "leg_RL", "leg_FL", "leg_RR",
                "clamp_L", "clamp_R", "pusher_mount", "horn_L", "horn_R"],
    "motors": ["motor_FR", "motor_RL", "motor_FL", "motor_RR", "pusher_motor"],
    "hardware": ["shaft_FR", "shaft_RL", "shaft_FL", "shaft_RR", "pusher_shaft", "pushrod_L", "pushrod_R", "gps_mast"],
    "props": ["prop_FR", "prop_RL", "prop_FL", "prop_RR", "prop_pusher"],
    "discs": ["propdisc_FR", "propdisc_RL", "propdisc_FL", "propdisc_RR", "propdisc_pusher"],
    "battery": ["battery", "strap", "xt60"],
    "electronics": ["flight_controller", "receiver", "gps", "pusher_esc", "esc_FR", "esc_RL", "esc_FL", "esc_RR",
                    "servo_L", "servo_R"],
    "wiring": ["wire_FR", "wire_RL", "wire_FL", "wire_RR", "harness_L", "harness_R", "ant_A", "ant_B"],
}
TOL = {"wing": 0.0003, "elevons": 0.0003, "pod": 0.0006, "props": 0.0008, "discs": 0.001, "battery": 0.0008,
       "motors": 0.0004, "hardware": 0.0004}
ANG = {"props": 0.6, "battery": 0.6, "motors": 0.4, "hardware": 0.5}


def mesh(shape, tol, ang):
    """Indexed triangle mesh in mm: (float32 vertices, uint32 indices)."""
    V, T = shape.tessellate(tol, ang)
    if not T:
        return np.zeros((0, 3), "<f4"), np.zeros(0, "<u4")
    return (np.array([[v.X, v.Y, v.Z] for v in V]) * 1000.0).astype("<f4"), np.array(T, "<u4").reshape(-1)


def main(P=None):
    s = S.load()
    P = P or D.build_detail(s)
    OUT.mkdir(parents=True, exist_ok=True)
    model, info = {}, {}
    for g, names in GROUPS.items():
        vs, ix, off = [], [], 0
        for n in names:
            if n not in P:
                continue
            v, i = mesh(P[n], TOL.get(g, 0.0003), ANG.get(g, 0.3))
            vs.append(v)
            ix.append(i + off)
            off += len(v)
        v = np.concatenate(vs)
        i = np.concatenate(ix).astype("<u4")
        model[g] = {"v": base64.b64encode(v.tobytes()).decode(), "i": base64.b64encode(i.tobytes()).decode()}
        info[g] = {"parts": [n for n in names if n in P], "triangles": int(len(i) // 3)}
    mp = json.loads((ROOT / "cad" / "mass_props.json").read_text())
    t = json.loads((ROOT / "aero" / "tables.json").read_text())
    cg_flu = [mp["cg"][0] * 1000, -mp["cg"][1] * 1000, -mp["cg"][2] * 1000]
    info["_markers"] = {"cg_mm": cg_flu, "np_x_mm": -t["x_np_asb"] * 1000, "mass_kg": mp["mass"]}
    (OUT / "model.json").write_text(json.dumps(model))
    (OUT / "parts.json").write_text(json.dumps(info, indent=1))
    dims = json.loads((ROOT / "build" / "dimensions.json").read_text()) if (ROOT / "build" / "dimensions.json").exists() else DIM.compute(s, P)
    (OUT / "dims.json").write_text(json.dumps(dims, ensure_ascii=False))
    ij = ROOT / "cad" / "detail" / "interference.json"
    if ij.exists():
        r = json.loads(ij.read_text())
        gaps = [x["gap_mm"] for x in r["swept"] if "gap_mm" in x]
        (OUT / "audit.json").write_text(json.dumps({"parts": r["parts"], "conflicts": len(r["conflicts"]),
                                                    "embeds": len(r["embeds"]), "ok": r["ok"],
                                                    "min_prop_gap_mm": min(gaps) if gaps else None}))
    for old in OUT.glob("*.stl"):
        old.unlink()
    print({g: v["triangles"] for g, v in info.items() if not g.startswith("_")},
          f"model.json {(OUT / 'model.json').stat().st_size / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
