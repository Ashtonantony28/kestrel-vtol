"""Master dimension list for builders, derived from spec/vehicle.yaml and the detailed CAD (never typed by hand).

Run: python -m kestrel.dimensions     -> build/dimensions.json, docs/DIMENSIONS.md
Used by the dimensioned drawing (kestrel.drawing) and the 3D viewer's dimension overlay.
All lengths in mm. Stations: x is measured from the wing root leading edge, positive forward (so aft is
negative); y from the centreline, positive left; z from the root chord line, positive up (FLU, ADR 0001).
"""
from __future__ import annotations

import json
import math

from . import cad_detail as D
from . import spec as S

ROOT = S.ROOT


def _bb(P, names):
    import build123d as bd
    return bd.Compound([P[n] for n in names]).bounding_box()


def compute(s=None, P=None):
    s = s or S.load()
    P = P or D.build_detail(s)
    w, bm, lr, fu, st, lg, pu = (s[k] for k in ("wing", "booms", "lift_rotors", "fuselage", "structure", "landing_legs", "pusher"))
    b2, chord, x_le, x_te, x_hinge = D.wing_dims(s)
    g = S.wing_geometry(s)
    mm = lambda v: round(v * 1000.0, 1)
    tsw = math.tan(math.radians(w["le_sweep_deg"]))
    lp = s["props"][s["propulsion"]["lift_prop"]]
    ppu = s["props"][s["propulsion"]["pusher_prop"]]
    mp = json.loads((ROOT / "cad" / "mass_props.json").read_text())
    tab = json.loads((ROOT / "aero" / "tables.json").read_text())
    import aerosandbox as asb
    tc = float(asb.Airfoil(w["airfoil"]).max_thickness())

    allnames = [k for k in P if not k.startswith("propdisc")]
    bb_all = _bb(P, allnames + [k for k in P if k.startswith("propdisc")])
    ground = min(P[f"leg_{k}"].bounding_box().min.Z for k in ("FR", "RL", "FL", "RR"))
    top = bb_all.max.Z
    y0e, y1e = (f * b2 for f in w["elevon_span_frac"])
    qc = lambda y: (-(y * tsw + 0.25 * chord(y)), y)
    os_ = st["outer_spar"]
    outer_len = math.dist(qc(os_["y0"]), qc(os_["y1"]))
    xf, xr = lr["x_front"], lr["x_rear"]
    legx = {k: P[f"leg_{k}"].bounding_box().center().X for k in ("FR", "RL")}
    pdp = P["propdisc_pusher"].bounding_box()
    plate = P["pusher_mount"].bounding_box()
    servo = P["servo_L"].bounding_box()
    bat = P["battery"].bounding_box()
    esc = P["esc_FL"].bounding_box()
    winglet_top_chord = 0.6 * w["winglet_root_chord"]

    T = []

    def add(group, name, value, unit="mm", note=""):
        T.append({"group": group, "name": name, "value": value, "unit": unit, "note": note})

    G = "Overall"
    add(G, "Wingspan (tip to tip, without winglets)", mm(w["span"]))
    add(G, "Overall width (outside of winglets)", mm(bb_all.size.Y))
    add(G, "Overall length (front prop tip to rear prop tip)", mm(bb_all.size.X))
    wl_top = P["winglet_L"].bounding_box().max.Z
    add(G, "Overall height (ground to top of the pusher prop arc)", mm(top - ground))
    add(G, "Height to winglet tip (ground to top)", mm(wl_top - ground))
    add(G, "All-up weight", round(mp["mass"] * 1000), "g", "CAD mass properties")
    add(G, "CG station (behind root LE)", mm(-mp["cg"][0]), "mm", "balance point: mark it under both wing roots")
    add(G, "Neutral point (behind root LE)", mm(tab["x_np_asb"]), "mm", "static margin 9.4 % MAC")
    G = "Wing"
    add(G, "Airfoil", f"S5020, {tc * 100:.1f} % thick", "")
    add(G, "Semispan", mm(b2))
    add(G, "Root chord", mm(w["root_chord"]))
    add(G, "Tip chord", mm(w["tip_chord"]))
    add(G, "Leading-edge sweep", w["le_sweep_deg"], "deg")
    add(G, "Tip leading-edge setback (behind root LE)", mm(b2 * tsw))
    add(G, "Tip trailing edge (behind root LE)", mm(-x_te(b2)))
    add(G, "Trailing-edge sweep", round(math.degrees(math.atan((-x_te(b2) - w["root_chord"]) / b2)), 1), "deg")
    add(G, "Washout at tip (nose down, about the quarter chord)", abs(w["tip_twist_deg"]), "deg", "set with the printed twist jig at glue-up")
    add(G, "Root / tip thickness", f"{mm(tc * w['root_chord'])} / {mm(tc * w['tip_chord'])}", "mm")
    add(G, "Wing area", round(g["area"] * 1e4), "cm²")
    add(G, "Aspect ratio", round(g["aspect_ratio"], 2), "")
    add(G, "Mean aerodynamic chord", mm(g["mac"]), "mm", f"at {mm(g['y_mac'])} mm from centreline, LE {mm(-g['x_le_mac'])} mm behind root LE")
    G = "Elevons"
    add(G, "Span stations (from centreline)", f"{mm(y0e)} to {mm(y1e)}", "mm")
    add(G, "Elevon length (along the hinge)", mm(math.dist((x_hinge(y0e), y0e), (x_hinge(y1e), y1e))))
    add(G, "Chord, inboard / outboard (25 % of local chord)", f"{mm(w['elevon_chord_frac'] * chord(y0e))} / {mm(w['elevon_chord_frac'] * chord(y1e))}", "mm")
    add(G, "Hinge line, inboard / outboard end (behind root LE)", f"{mm(-x_hinge(y0e))} / {mm(-x_hinge(y1e))}", "mm")
    add(G, "Throw", f"±{w['elevon_max_deg']:.0f}", "deg", "±27 mm at the elevon root trailing edge")
    add(G, "Servo pocket centre (span / behind root LE)", f"{mm(servo.center().Y)} / {mm(-servo.center().X)}", "mm", "SG90 lying flat, 45 % chord")
    add(G, "Servo pocket (L × W × D)", f"{mm(servo.size.Y)} × {mm(D.SERVO[2])} × {mm(D.SERVO[1])}", "mm")
    G = "Winglets"
    add(G, "Height", mm(w["winglet_height"]))
    add(G, "Root / top chord", f"{mm(w['winglet_root_chord'])} / {mm(winglet_top_chord)}", "mm", "trailing edge vertical, flush with the tip TE")
    add(G, "Leading-edge sweep", round(math.degrees(math.atan((w["winglet_root_chord"] - winglet_top_chord) / w["winglet_height"])), 1), "deg")
    G = "Pod"
    add(G, "Length × width × height", f"{mm(fu['length'])} × {mm(fu['width'])} × {mm(fu['height'])}", "mm", "2 mm wall")
    add(G, "Nose ahead of root LE", mm(fu["nose_x"]))
    add(G, "Tail behind root LE", mm(fu["length"] - fu["nose_x"]))
    add(G, "Nose corner radius", 22.0)
    hb = P["hatch"].bounding_box()
    add(G, "Hatch (L × W)", f"{mm(hb.size.X)} × {mm(hb.size.Y)}", "mm")
    add(G, "Battery (L × W × H)", f"{mm(bat.size.X)} × {mm(bat.size.Y)} × {mm(bat.size.Z)}", "mm", f"centre {mm(-bat.center().X)} mm behind root LE; slide to balance")
    asp = s.get("airspeed_sensor") or {}
    if asp.get("fitted") and "pitot" in P:
        add(G, "Pitot tube (OD × length)", f"{mm(asp['pitot_od'])} × {mm(asp['pitot_length'])}", "mm",
            f"tip {mm(asp['protrusion'])} mm ahead of the nose, on the centreline (ADR 0007)")
        sb = P["airspeed_sensor"].bounding_box()
        add(G, "Airspeed sensor board (L × W × H)", " × ".join(str(mm(v)) for v in asp["board"]), "mm",
            f"centre {mm(-sb.center().X)} mm behind root LE (negative = ahead), on a {mm(asp.get('pad', 0.004))} mm foam pad")
    G = "Booms and spars"
    add(G, "Boom tube (OD / ID × length)", f"{mm(bm['diameter'])} / {mm(bm['inner_diameter'])} × {mm(bm['length'])}", "mm", "carbon, cut from 1 m")
    add(G, "Boom lateral position (each side) / spacing", f"{mm(bm['y'])} / {mm(2 * bm['y'])}", "mm")
    add(G, "Boom front / rear end station", f"+{mm(bm['x_front'])} / {mm(bm['x_front'] - bm['length'])}", "mm")
    add(G, "Boom axis below root chord line", mm(0.012))
    add(G, "Cross spar (OD / ID × length)", f"{mm(st['cross_spar']['od'])} / {mm(st['cross_spar']['id'])} × {mm(st['cross_spar']['span'])}", "mm")
    add(G, "Cross spar station (behind root LE) / height", f"{mm(st['cross_spar']['x_aft'])} / +{mm(st['cross_spar']['z'])}", "mm")
    add(G, "Outer spars (OD / ID × length)", f"{mm(os_['od'])} / {mm(os_['id'])} × {mm(outer_len)}", "mm", f"along the quarter chord, {mm(os_['y0'])} to {mm(os_['y1'])} mm out")
    add(G, "Boom-to-spar clamp (L × W × H)", f"{mm(P['clamp_L'].bounding_box().size.X)} × {mm(P['clamp_L'].bounding_box().size.Y)} × {mm(P['clamp_L'].bounding_box().size.Z)}", "mm", "printed, one per side")
    G = "Lift rotors"
    add(G, "Motor stations, front / rear (from root LE)", f"+{mm(xf)} / {mm(xr)}", "mm")
    add(G, "Motor spacing, fore-aft / side-to-side / diagonal", f"{mm(xf - xr)} / {mm(2 * lr['y'])} / {mm(math.hypot(xf - xr, 2 * lr['y']))}", "mm")
    add(G, "Prop", f"10×4.5 in ({mm(lp['d'])} mm)", "")
    add(G, "Prop plane above boom axis", mm(-lr["z"] + 0.012))
    add(G, "Motor tilt (outboard, about the boom)", lr.get("yaw_tilt_deg", 0.0), "deg")
    add(G, "Gap between left and right prop tips", mm(2 * lr["y"] - lp["d"]))
    add(G, "Prop tip to pod side", mm(lr["y"] - lp["d"] / 2 - fu["width"] / 2))
    add(G, "Motor mount plate (mm)", "34 × 34 × 7, M3 holes on 16 / 19 mm", "")
    add(G, "ESC centre from its motor (toward the wing)", mm(abs(esc.center().X - xf)))
    G = "Landing gear"
    add(G, "Leg height (boom axis to ground)", mm(lg["height"]))
    add(G, "Leg stations, front / rear", f"+{mm(legx['FR'])} / {mm(legx['RL'])}", "mm", f"{mm(D.LEG_INBOARD)} mm inboard of each motor")
    add(G, "Footprint (fore-aft × side-to-side)", f"{mm(legx['FR'] - legx['RL'])} × {mm(2 * bm['y'])}", "mm")
    chk = json.loads((ROOT / "cad" / "checks.json").read_text())
    add(G, "Pod ground clearance", mm(chk["pod_clearance_m"]))
    G = "Pusher"
    add(G, "Motor axis above root chord line", mm(-pu["z"]))
    add(G, "Mount plate aft face (behind root LE)", mm(-plate.min.X if False else -(fu["nose_x"] - fu["length"] + 0.002)))
    add(G, "Prop", f"8×6 in ({mm(ppu['d'])} mm)", "")
    add(G, "Prop plane (behind root LE)", mm(-pdp.center().X))
    add(G, "Prop ground clearance", mm(pdp.min.Z - ground))

    # overlay for the 3D viewer: endpoints in FLU mm, plus the label
    def p(x, y, z):
        return [round(x * 1000, 1), round(y * 1000, 1), round(z * 1000, 1)]
    ov = [
        {"id": "span", "label": f"span {mm(w['span']):.0f}", "a": p(-0.70, b2, 0.0), "b": p(-0.70, -b2, 0.0), "views": ["top", "iso", "under"]},
        {"id": "span_f", "label": f"span {mm(w['span']):.0f}", "a": p(0.0, b2, top + 0.02), "b": p(0.0, -b2, top + 0.02), "views": ["front"]},
        {"id": "length", "label": f"overall length {mm(bb_all.size.X):.0f}", "a": p(bb_all.max.X, b2 + 0.07, ground), "b": p(bb_all.min.X, b2 + 0.07, ground), "views": ["top", "iso", "side", "under"]},
        {"id": "root", "label": f"root chord {mm(w['root_chord']):.0f}", "a": p(0, 0.06, 0.03), "b": p(-w["root_chord"], 0.06, 0.03), "views": ["top", "iso"]},
        {"id": "tip", "label": f"tip chord {mm(w['tip_chord']):.0f}", "a": p(x_le(b2), -b2 - 0.04, 0.0), "b": p(x_te(b2), -b2 - 0.04, 0.0), "views": ["top", "iso"]},
        {"id": "booms", "label": f"boom spacing {mm(2 * bm['y']):.0f}", "a": p(0.36, bm["y"], -0.012), "b": p(0.36, -bm["y"], -0.012), "views": ["top", "iso", "under"]},
        {"id": "booms_f", "label": f"boom spacing {mm(2 * bm['y']):.0f}", "a": p(0.36, bm["y"], ground - 0.02), "b": p(0.36, -bm["y"], ground - 0.02), "views": ["front"]},
        {"id": "rotors", "label": f"motor spacing {mm(xf - xr):.0f}", "a": p(xf, -b2 - 0.07, -lr["z"]), "b": p(xr, -b2 - 0.07, -lr["z"]), "views": ["top", "iso", "side"]},
        {"id": "prop", "label": f"prop Ø{mm(lp['d']):.0f}", "a": p(xf + lp["d"] / 2, lr["y"], -lr["z"] + 0.012), "b": p(xf - lp["d"] / 2, lr["y"], -lr["z"] + 0.012), "views": ["top", "iso", "side"]},
        {"id": "pod", "label": f"pod {mm(fu['length']):.0f}", "a": p(fu["nose_x"], 0, ground + 0.012), "b": p(fu["nose_x"] - fu["length"], 0, ground + 0.012), "views": ["side", "under"]},
        {"id": "height", "label": f"winglet tip {mm(wl_top - ground):.0f}", "a": p(x_te(b2), b2 + 0.04, ground), "b": p(x_te(b2), b2 + 0.04, wl_top), "views": ["side", "front", "iso"]},
        {"id": "legs", "label": f"leg {mm(lg['height']):.0f}", "a": p(xf - 0.028, -lr["y"] - 0.03, -0.012), "b": p(xf - 0.028, -lr["y"] - 0.03, ground), "views": ["front", "side"]},
        {"id": "sweep", "label": f"LE sweep {w['le_sweep_deg']:.0f}°", "a": p(0, 0, 0.0), "b": p(x_le(b2), b2, 0.0), "kind": "edge", "views": ["top"]},
    ]
    return {"table": T, "overlay": ov,
            "frame": "FLU mm, origin at wing root leading edge; x forward, y left, z up",
            "ground_z_mm": mm(ground)}


def write_md(d):
    lines = ["# Kestrel dimensions", "",
             "Generated by `python -m kestrel.dimensions` from `spec/vehicle.yaml` and the detailed CAD. Do not edit by hand.", "",
             "Stations are measured from the **wing root leading edge**: fore-aft along the root chord (\"behind root LE\" = aft),",
             "sideways from the centreline, and up from the root chord line. Drawing: `cad/drawings/kestrel_drawings.pdf`.", ""]
    grp = None
    for r in d["table"]:
        if r["group"] != grp:
            grp = r["group"]
            lines += ["", f"## {grp}", "", "| Dimension | Value | Note |", "|---|---|---|"]
        v = f"{r['value']} {r['unit']}".strip()
        lines.append(f"| {r['name']} | {v} | {r['note']} |")
    (ROOT / "docs" / "DIMENSIONS.md").write_text("\n".join(lines) + "\n")


def main(P=None):
    d = compute(P=P)
    (ROOT / "build" / "dimensions.json").write_text(json.dumps(d, indent=1, ensure_ascii=False))
    write_md(d)
    for r in d["table"]:
        print(f"{r['group']:16s} {r['name']:55s} {r['value']} {r['unit']}")
    return d


if __name__ == "__main__":
    main()
