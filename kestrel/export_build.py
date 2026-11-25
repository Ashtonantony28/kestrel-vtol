"""Build files: the detailed assembly as one STEP, and every printed part as STEP and STL, in mm.

Run: python -m kestrel.export_build      -> cad/build/ (see cad/build/README.md)

The assembly is the audited model from kestrel.cad_detail (python -m kestrel.interference), with one named
solid per part, in the FLU body frame (x forward, y left, z up), origin at the root leading edge.
Printed parts are exported in their print frames, with quantities in the file names.
"""
from __future__ import annotations

import json

import build123d as bd

from . import cad
from . import cad_detail as D
from . import spec as S

OUT = S.ROOT / "cad" / "build"


def _named(shape, name):
    shape.label = name
    return shape


def _to_origin_mm(shape_m):
    """Metres in the assembly -> mm, centred on the origin with the base on z = 0 (for a slicer)."""
    s = shape_m.scale(1000.0)
    b = s.bounding_box()
    return s.translate((-b.center().X, -b.center().Y, -b.min.Z))


def printed(s, P):
    pp = cad.printed_parts(s)                       # already mm, print frames
    mount_r = pp["motor_mount"]
    return {
        "motor_mount_right_x2": mount_r,
        "motor_mount_left_x2": mount_r.mirror(bd.Plane.XZ),
        "leg_x4": pp["leg"],
        "boom_clamp_x2": _to_origin_mm(P["clamp_L"]),
        "pusher_pylon_x1": D.pusher_pylon(s),
    }


def main():
    s = S.load()
    P = D.build_detail(s)
    OUT.mkdir(parents=True, exist_ok=True)
    kids = [_named(P[k].scale(1000.0), k) for k in P if not k.startswith("propdisc")]
    asm = bd.Compound(children=kids, label="kestrel_assembly")
    bd.export_step(asm, str(OUT / "kestrel_detailed_assembly.step"))
    info = {"assembly": {"file": "kestrel_detailed_assembly.step", "parts": len(kids), "units": "mm",
                         "frame": "FLU, origin at the wing root leading edge"}, "printed": {}}
    for name, shp in printed(s, P).items():
        bd.export_step(_named(shp, name), str(OUT / f"{name}.step"))
        bd.export_stl(shp, str(OUT / f"{name}.stl"), tolerance=0.01, angular_tolerance=0.1)
        b = shp.bounding_box()
        info["printed"][name] = {"size_mm": [round(b.size.X, 1), round(b.size.Y, 1), round(b.size.Z, 1)],
                                 "volume_cm3": round(D.vol(shp.scale(0.001)) * 1e6, 2)}
    (OUT / "manifest.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()
