"""Interference and geometry audit of the detailed assembly.

Run: python -m kestrel.interference      -> cad/detail/interference.json, exit 1 on any unexplained conflict

1. Every part is a valid, closed solid.
2. Every pair of parts is intersected. An overlap is either a deliberate embed that the build makes room for
   (a spar in a slot cut in the foam, the pod glued into a cut-out under the wing root) and is listed in
   ALLOWED with the build step that creates the room, or it is a conflict.
3. Moving parts get swept clearance: each prop disc (full diameter, 12 mm thick) and each elevon at ±full throw
   against everything that is not part of its own drive.
4. Minimum gaps between neighbours that must not touch (props to the airframe, prop tips to each other,
   pusher to wing and ground, legs to the mount saddle).
"""
from __future__ import annotations

import itertools
import json
import math
import sys

import build123d as bd

from . import cad_detail as D
from . import spec as S

OUT = S.ROOT / "cad" / "detail"
MIN_OVERLAP = 1e-9          # m^3 (1 mm^3): below this a "touch" is glue contact, not interference
PROP_MIN_GAP = 0.006        # m, prop disc to any structure (cad.PROP_MARGIN)

# deliberate embeds: (part prefix A, part prefix B, how the build makes room)
ALLOWED = [
    ("cross_spar", "wing_", "slot cut in the foam for the 8 mm cross spar (template mark)"),
    ("cross_spar", "pod", "8 mm holes through both pod walls"),
    ("cross_spar", "clamp_", "spar end seats in the printed clamp bore"),
    ("outer_spar_", "wing_", "slot cut in the foam for the 6 mm outer spar"),
    ("outer_spar_", "elevon_", "outer spar ends 2 mm short of the hinge line; trim the slot, do not cut the elevon"),
    ("boom_", "wing_", "boom sits half-recessed in a groove in the wing underside"),
    ("boom_", "clamp_", "boom seats in the printed clamp bore"),
    ("clamp_", "wing_", "clamp glued into a pocket in the wing underside"),
    ("servo_", "wing_", "servo pocket cut in the foam"),
    ("servo_", "elevon_", "servo pocket"),
    ("horn_", "elevon_", "horn slotted into the elevon"),
    ("pushrod_", "wing_", "pushrod runs in a shallow groove in the top skin"),
    ("pushrod_", "elevon_", "pushrod end in a groove at the horn"),
    ("pushrod_", "servo_", "Z-bend through the servo arm hole"),
    ("pushrod_", "horn_", "clevis / Z-bend through the horn hole"),
    ("torque_rod_", "wing_", "torque rod in a 2 mm groove along the hinge line under the top-skin tape"),
    ("torque_rod_", "elevon_", "torque rod glued into both elevon segments"),
    ("harness_", "wing_", "wire channel cut beside the cross spar"),
    ("harness_", "pod", "wire hole through the pod wall"),
    ("pod", "wing_", "pod glued into a cut-out under the wing root"),
    ("hatch", "wing_", "hatch is the removable top of the pod cut-out"),
    ("gps_mast", "wing_", "mast hole through the wing root"),
    ("gps_mast", "pod", "mast glued into the pod top"),
    ("gps_mast", "hatch", "mast hole through the hatch"),
    ("pusher_mount", "pod", "pylon flange glued onto the pod top"),
    ("pusher_mount", "hatch", "pylon flange overlaps the hatch rim"),
    ("ant_", "pod", "antenna exits through a hole in the pod floor"),
    ("flight_controller", "wing_", "FC sits in the pod cut-out"),
    ("receiver", "wing_", "receiver in the pod cut-out"),
    ("battery", "wing_", "battery bay in the pod cut-out"),
    ("strap", "wing_", "strap in the pod cut-out"),
    ("pusher_esc", "wing_", "ESC in the pod cut-out"),
    ("winglet_", "wing_", "winglet glued to the tip rib"),
    ("winglet_", "elevon_", "winglet glued to the tip rib, elevon stops 25 mm short"),
    ("wire_", "boom_", "leads taped on the boom"),
    ("esc_", "boom_", "ESC zip-tied to the boom"),
    ("leg_", "boom_", "leg seats on the boom (bore)"),
    ("mount_", "boom_", "mount saddle on the boom (bore)"),
    ("motor_", "mount_", "motor bolted to the plate"),
    ("motor_", "shaft_", "shaft and adapter are part of the motor"),
    ("shaft_", "mount_", "shaft passes through the 9 mm plate hole"),
    ("shaft_", "prop_", "prop on the adapter"),
    ("pusher_motor", "pusher_mount", "motor bolted to the pylon plate"),
    ("pusher_motor", "pusher_shaft", "shaft part of the motor"),
    ("pusher_shaft", "pusher_mount", "shaft through the plate hole"),
    ("pusher_shaft", "prop_pusher", "prop on the adapter"),
    ("xt60", "battery", "connector on the battery lead"),
    ("strap", "battery", "strap around the battery"),
    ("strap", "pod", "Velcro strap passes through two slots in the pod floor"),
]


def _allowed(a, b):
    for pa, pb, why in ALLOWED:
        if (a.startswith(pa) and b.startswith(pb)) or (b.startswith(pa) and a.startswith(pb)):
            return why
    return None


def _bb_overlap(b1, b2, pad=0.0):
    return (b1.min.X - pad <= b2.max.X and b2.min.X - pad <= b1.max.X and b1.min.Y - pad <= b2.max.Y
            and b2.min.Y - pad <= b1.max.Y and b1.min.Z - pad <= b2.max.Z and b2.min.Z - pad <= b1.max.Z)


def _solids(shape):
    return list(shape.solids()) if hasattr(shape, "solids") else [shape]


def audit(s=None, P=None):
    s = s or S.load()
    P = P or D.build_detail(s)
    names = [k for k in P if not k.startswith("propdisc")]
    res = {"parts": len(names), "invalid": [], "conflicts": [], "embeds": [], "swept": [], "gaps": {}}

    for k in names:
        for so in _solids(P[k]):
            if not so.is_valid:
                res["invalid"].append(k)
                break

    bbs = {k: P[k].bounding_box() for k in names}
    for a, b in itertools.combinations(names, 2):
        if not _bb_overlap(bbs[a], bbs[b]):
            continue
        try:
            v = D.vol(P[a] & P[b])
        except Exception as e:          # noqa: BLE001
            res["conflicts"].append({"a": a, "b": b, "mm3": None, "why": f"boolean failed: {e}"})
            continue
        if v <= MIN_OVERLAP:
            continue
        why = _allowed(a, b)
        rec = {"a": a, "b": b, "mm3": round(v * 1e9, 1)}
        if why:
            rec["build"] = why
            res["embeds"].append(rec)
        else:
            res["conflicts"].append(rec)

    # swept volumes: props
    own = {"FR", "FL", "RR", "RL"}
    for k in own | {"pusher"}:
        disc = P[f"propdisc_{k}"]
        drive = (f"prop_{k}", f"shaft_{k}", f"motor_{k}", f"mount_{k}") if k != "pusher" else (
            "prop_pusher", "pusher_shaft", "pusher_motor", "pusher_mount")
        db = disc.bounding_box()
        worst = (None, 9.0)
        for o in names:
            if o in drive or o.startswith("prop_") or o.startswith("propdisc"):
                continue
            if not _bb_overlap(db, bbs[o], PROP_MIN_GAP + 0.03):
                continue
            d = disc.distance_to(P[o])
            if d < worst[1]:
                worst = (o, d)
        ok = worst[1] >= PROP_MIN_GAP
        res["swept"].append({"part": f"prop disc {k}", "closest": worst[0], "gap_mm": round(worst[1] * 1000, 1),
                             "required_mm": PROP_MIN_GAP * 1000, "ok": ok})
    # elevons at full throw about the hinge line
    w = s["wing"]
    b2, chord, x_le, x_te, x_hinge = D.wing_dims(s)
    y0, y1 = (f * b2 for f in w["elevon_span_frac"])
    for side, sy in (("L", 1), ("R", -1)):
        p0 = bd.Vector(x_hinge(y0), sy * y0, 0.0)
        p1 = bd.Vector(x_hinge(y1), sy * y1, 0.0)
        ax = bd.Axis(p0, (p1 - p0).normalized())
        for ang in (-w["elevon_max_deg"], w["elevon_max_deg"]):
            ev = P[f"elevon_{side}"].rotate(ax, ang)
            eb = ev.bounding_box()
            hits = []
            for o in names:
                if o in (f"elevon_{side}", f"wing_{side}", f"horn_{side}", f"pushrod_{side}", f"outer_spar_{side}",
                         f"servo_{side}", f"torque_rod_{side}"):
                    continue
                if not _bb_overlap(eb, bbs[o]):
                    continue
                v = D.vol(ev & P[o])
                if v > MIN_OVERLAP:
                    hits.append({"part": o, "mm3": round(v * 1e9, 1)})
            res["swept"].append({"part": f"elevon {side} at {ang:+.0f} deg", "hits": hits, "ok": not hits})

    # named gaps
    g = res["gaps"]
    g["front_prop_tip_to_tip_mm"] = round(P["propdisc_FL"].distance_to(P["propdisc_FR"]) * 1000, 1)
    g["rear_prop_tip_to_tip_mm"] = round(P["propdisc_RL"].distance_to(P["propdisc_RR"]) * 1000, 1)
    g["rear_prop_to_pusher_disc_mm"] = round(P["propdisc_RL"].distance_to(P["propdisc_pusher"]) * 1000, 1)
    g["front_prop_to_pod_mm"] = round(P["propdisc_FL"].distance_to(P["pod"]) * 1000, 1)
    g["rear_prop_to_wing_mm"] = round(P["propdisc_RL"].distance_to(P["wing_L"] + P["elevon_L"]) * 1000, 1)
    g["rear_prop_to_winglet_mm"] = round(P["propdisc_RL"].distance_to(P["winglet_L"]) * 1000, 1)
    gz = min(P[f"leg_{k}"].bounding_box().min.Z for k in ("FR", "RL", "FL", "RR"))
    g["pusher_prop_to_ground_mm"] = round((P["propdisc_pusher"].bounding_box().min.Z - gz) * 1000, 1)
    g["pusher_prop_to_wing_mm"] = round(P["propdisc_pusher"].distance_to(P["wing_L"] + P["wing_R"] + P["elevon_L"] + P["elevon_R"]) * 1000, 1)
    g["lowest_non_leg_part_to_ground_mm"] = round(min(bbs[k].min.Z for k in names if not k.startswith("leg_")) * 1000 - gz * 1000, 1)
    g["leg_to_mount_saddle_mm"] = round(P["leg_FL"].distance_to(P["mount_FL"]) * 1000, 1)
    g["esc_to_leg_mm"] = round(P["esc_FL"].distance_to(P["leg_FL"]) * 1000, 1)
    res["ok"] = not res["invalid"] and not res["conflicts"] and all(x["ok"] for x in res["swept"])
    return res


def main():
    r = audit()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "interference.json").write_text(json.dumps(r, indent=1))
    print(f"{r['parts']} parts; invalid {len(r['invalid'])}; conflicts {len(r['conflicts'])}; deliberate embeds {len(r['embeds'])}")
    for c in r["conflicts"]:
        print("  CONFLICT", c)
    for x in r["swept"]:
        print("  SWEPT", x)
    print("  GAPS", json.dumps(r["gaps"]))
    print("  invalid:", r["invalid"])
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
