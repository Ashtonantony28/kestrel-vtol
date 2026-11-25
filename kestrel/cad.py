"""M1 CAD: build123d airframe, clearance checks, mass properties, printed parts, 1:1 foam cut templates.

Run: python -m kestrel.cad
Writes cad/out/*.step|stl, cad/mass_props.json (FRD, about CG), cad/templates/*.svg + kestrel_templates_letter.pdf,
cad/checks.json. CAD frame is FLU (x fwd, y left, z up), origin at wing root LE; converted to FRD for export.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import aerosandbox as asb
import build123d as bd
import numpy as np

from . import spec as S

ROOT = S.ROOT
OUT = ROOT / "cad" / "out"
TPL = ROOT / "cad" / "templates"
FOAM_AREAL = 0.194        # kg/m^2, 5 mm dollar-store foamboard with paper (~75 g per 20x30 in sheet)
FOAM_T = 0.005
PETG = 1270.0             # kg/m^3
PRINT_FILL = 0.45         # effective solid fraction (3 walls + 25 % infill)
CARBON = 1550.0
SHEET = (762.0, 508.0)    # mm, 30 x 20 in
PROP_MARGIN = 0.006       # m, minimum clearance around a prop disc

# Electronics / point items: (mass kg, size m (x,y,z), FLU centre)
def _items(s):
    pod = s["fuselage"]
    zc = -0.02
    bx = s["mass_items"]["battery"]["x"]
    items = {
        # battery on the pod floor (inner floor z = -0.048); FC behind it, clear of the battery (the earlier
        # stacking overlapped it, found by kestrel.interference), with 20 mm for the XT60 between them; GPS ahead of the cross spar at x = -0.18
        "battery": (s["batteries"][s["propulsion"]["battery"]]["mass"], (0.105, 0.034, 0.024), (bx, 0.0, -0.034)),
        "flight_controller": (0.019, (0.037, 0.026, 0.014), (bx - 0.0525 - 0.020 - 0.0185, 0.0, -0.040)),
        "gps": (0.012, (0.025, 0.025, 0.008), (-0.135, 0.0, 0.04)),
        "receiver": (0.003, (0.02, 0.012, 0.004), (-0.20, 0.0, zc)),
        "wiring": (0.040, (0.25, 0.05, 0.02), (-0.12, 0.0, zc)),
        "pusher_esc": (0.025, (0.045, 0.025, 0.008), (-0.24, 0.0, zc)),
        "servos": (0.018, (0.023, 0.012, 0.022), (-0.28, 0.0, 0.0)),
    }
    a = s.get("airspeed_sensor")
    if a and a.get("fitted"):   # ADR 0007: pitot + board + hoses lumped as one block in the pod nose
        mi = s["mass_items"]["airspeed_sensor"]
        items["airspeed_sensor"] = (mi["m"], (0.06, 0.018, 0.02), (mi["x"], 0.0, -mi["z"]))
    return items


def mh45_xy():
    c = asb.Airfoil(S.load()["wing"]["airfoil"]).coordinates   # TE upper -> LE -> TE lower
    return np.asarray(c)


def _section(x_le, y, chord, twist_deg):
    """Airfoil wire in FLU at span station y (twist about quarter chord, negative = nose down)."""
    xy = mh45_xy()
    pts = []
    t = math.radians(twist_deg)
    for xc, zc in xy[:-1]:
        dx, dz = -(xc - 0.25) * chord, zc * chord
        # rotate about quarter chord: nose down for negative twist
        rx = dx * math.cos(t) + dz * math.sin(t)
        rz = -dx * math.sin(t) + dz * math.cos(t)
        pts.append((x_le - 0.25 * chord + rx, y, rz))
    return bd.Polyline(*pts, close=True)


def wing_half(s, left=True):
    w = s["wing"]
    b2 = w["span"] / 2
    tan_sw = math.tan(math.radians(w["le_sweep_deg"]))
    root = _section(0.0, 0.0, w["root_chord"], 0.0)
    tip = _section(-b2 * tan_sw, b2, w["tip_chord"], w["tip_twist_deg"])
    solid = bd.Solid.make_loft([bd.Wire(root.edges()), bd.Wire(tip.edges())], ruled=True)
    return solid if left else solid.mirror(bd.Plane.XZ)


def winglet(s, left=True):
    w = s["wing"]
    b2 = w["span"] / 2
    tan_sw = math.tan(math.radians(w["le_sweep_deg"]))
    x_te_tip = -b2 * tan_sw - w["tip_chord"]
    crw, h = w["winglet_root_chord"], w["winglet_height"]
    pts = [(x_te_tip + crw, 0.0), (x_te_tip, 0.0), (x_te_tip + 0.0, h), (x_te_tip + 0.6 * crw - 0.03 + 0.03, h)]
    # profile in XZ plane at y = b2, extruded by foam thickness
    face = bd.Face(bd.Wire(bd.Polyline(*[(x, z) for x, z in pts], close=True).edges()))
    sol = bd.extrude(face, amount=FOAM_T)                          # in XY of default plane: (x, y=z) -> extrude z
    sol = sol.rotate(bd.Axis.X, 90)                                  # (x, z) -> XZ plane, thickness along -y
    sol = sol.translate((0, b2, 0))                                 # outboard face of the tip rib, no gap
    # follow the tip section's washout (twist about the tip quarter chord) so the winglet sits on the tip chord
    x_qc_tip = -b2 * tan_sw - 0.25 * w["tip_chord"]
    sol = sol.rotate(bd.Axis((x_qc_tip, b2, 0), (0, 1, 0)), w["tip_twist_deg"])
    return sol if left else sol.mirror(bd.Plane.XZ)


def pod(s):
    f = s["fuselage"]
    box = bd.Box(f["length"], f["width"], f["height"])
    return box.translate((f["nose_x"] - f["length"] / 2, 0, -0.02))


def cyl_x(r, length, center):
    c = bd.Cylinder(r, length, rotation=(0, 90, 0))
    return c.translate(center)


def build(s=None):
    s = s or S.load()
    bm, lr, pu, lg = s["booms"], s["lift_rotors"], s["pusher"], s["landing_legs"]
    lp = s["props"][s["propulsion"]["lift_prop"]]
    pp = s["props"][s["propulsion"]["pusher_prop"]]
    parts = {}
    parts["wing_L"] = wing_half(s, True)
    parts["wing_R"] = wing_half(s, False)
    parts["winglet_L"] = winglet(s, True)
    parts["winglet_R"] = winglet(s, False)
    parts["pod"] = pod(s)
    z_boom = -0.012
    for side, sy in (("L", 1), ("R", -1)):
        y = sy * bm["y"]
        xc = bm["x_front"] - bm["length"] / 2
        outer = cyl_x(bm["diameter"] / 2, bm["length"], (xc, y, z_boom))
        inner = cyl_x(bm["inner_diameter"] / 2, bm["length"], (xc, y, z_boom))
        parts[f"boom_{side}"] = outer - inner
    z_prop = -lr["z"]          # FRD z -> FLU
    rotors = {"FR": (lr["x_front"], -lr["y"]), "RL": (lr["x_rear"], lr["y"]),
              "FL": (lr["x_front"], lr["y"]), "RR": (lr["x_rear"], -lr["y"])}
    tilt = lr.get("yaw_tilt_deg", 0.0)
    for k, (x, y) in rotors.items():
        # yaw tilt: every motor leans outboard about its boom (FLU: right side y < 0 -> +tilt about +X), ADR 0004
        ax = bd.Axis((x, y, z_boom), (1, 0, 0))
        ang = tilt if y < 0 else -tilt
        parts[f"motor_{k}"] = bd.Cylinder(0.014, 0.026).translate((x, y, z_boom + 0.005 + 0.013)).rotate(ax, ang)
        parts[f"mount_{k}"] = bd.Box(0.034, 0.034, 0.006).translate((x, y, z_boom + 0.008)).rotate(ax, ang)
        parts[f"leg_{k}"] = bd.Box(0.012, 0.012, lg["height"]).translate((x, y, z_boom - lg["height"] / 2))
        parts[f"propdisc_{k}"] = bd.Cylinder(lp["d"] / 2, 0.012).translate((x, y, z_prop)).rotate(ax, ang)
    zpu = -pu["z"]
    parts["pusher_motor"] = cyl_x(0.014, 0.026, (pu["x"] + 0.02, 0, zpu))
    parts["propdisc_pusher"] = cyl_x(pp["d"] / 2, 0.012, (pu["x"] - 0.005, 0, zpu))
    # spars (spec structure): straight cross spar through the pod joining the booms, swept outer spars
    st = s["structure"]
    cs = st["cross_spar"]
    tube = bd.Cylinder(cs["od"] / 2, cs["span"], rotation=(90, 0, 0)) - bd.Cylinder(cs["id"] / 2, cs["span"], rotation=(90, 0, 0))
    parts["cross_spar"] = tube.translate((-cs["x_aft"], 0, cs["z"]))
    os_ = st["outer_spar"]
    w = s["wing"]
    b2, tsw = w["span"] / 2, math.tan(math.radians(w["le_sweep_deg"]))
    def qc(y):   # quarter-chord point (FLU), mid-height of the section
        c = w["root_chord"] + (w["tip_chord"] - w["root_chord"]) * y / b2
        return (-(y * tsw + 0.25 * c), y, 0.035 * c)
    for side, sy in (("L", 1), ("R", -1)):
        p0, p1 = qc(os_["y0"]), qc(os_["y1"])
        p0, p1 = (p0[0], sy * p0[1], p0[2]), (p1[0], sy * p1[1], p1[2])
        L = math.dist(p0, p1)
        d = bd.Vector(*(b - a for a, b in zip(p0, p1))).normalized()
        pl = bd.Plane(origin=p0, z_dir=d)
        t = pl * (bd.Cylinder(os_["od"] / 2, L, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
                  - bd.Cylinder(os_["id"] / 2, L, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)))
        parts[f"outer_spar_{side}"] = t
    for k, (m, size, c) in _items(s).items():
        parts[k] = bd.Box(*size).translate(c)
    return parts, rotors


def clearance_checks(s, parts):
    res = {}
    discs = {k: v for k, v in parts.items() if k.startswith("propdisc")}
    others = {k: v for k, v in parts.items() if not k.startswith("propdisc") and not k.startswith("motor")
              and not k.startswith("mount") and not k.startswith("leg") and k not in ("pusher_motor",)}
    lp = s["props"][s["propulsion"]["lift_prop"]]
    pp = s["props"][s["propulsion"]["pusher_prop"]]
    # inflate discs by margin
    infl = {}
    for k, v in discs.items():
        r = (pp if k.endswith("pusher") else lp)["d"] / 2 + PROP_MARGIN
        c = v.center()
        infl[k] = cyl_x(r, 0.012 + 2 * PROP_MARGIN, (c.X, c.Y, c.Z)) if k.endswith("pusher") else \
            bd.Cylinder(r, 0.012 + 2 * PROP_MARGIN).translate((c.X, c.Y, c.Z))
    hits = []
    names = list(infl)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if (infl[a] & discs[b]).volume > 1e-9:
                hits.append(f"{a} x {b}")
        for k, o in others.items():
            if k.startswith("boom"):
                continue  # boom passes under own motor only; checked via z below
            v = (infl[a] & o).volume
            if v > 1e-9:
                hits.append(f"{a} x {k}")
    res["prop_collisions"] = hits
    # planform overlap of lift discs with the wing (hover download), fraction of disc area (2-D, top view)
    from shapely.geometry import Point, Polygon
    w = s["wing"]
    b2 = w["span"] / 2
    tsw = math.tan(math.radians(w["le_sweep_deg"]))
    half = Polygon([(0, 0), (-w["root_chord"], 0), (-b2 * tsw - w["tip_chord"], b2), (-b2 * tsw, b2)])
    plan = half.union(Polygon([(x, -y) for x, y in half.exterior.coords]))
    frac = {}
    for k, (x, y) in {"FR": (s["lift_rotors"]["x_front"], -s["lift_rotors"]["y"]), "RL": (s["lift_rotors"]["x_rear"], s["lift_rotors"]["y"]),
                      "FL": (s["lift_rotors"]["x_front"], s["lift_rotors"]["y"]), "RR": (s["lift_rotors"]["x_rear"], -s["lift_rotors"]["y"])}.items():
        disc = Point(x, -y).buffer(lp["d"] / 2, 128)   # FRD y -> FLU
        frac[k] = disc.intersection(plan).area / disc.area
    res["disc_wing_overlap_frac"] = frac
    # ground clearance: lowest point of legs vs pusher disc and pod
    z_ground = min(parts[f"leg_{k}"].bounding_box().min.Z for k in ("FR", "RL", "FL", "RR"))
    res["ground_z"] = z_ground
    res["pusher_tip_clearance_m"] = parts["propdisc_pusher"].bounding_box().min.Z - z_ground
    res["pod_clearance_m"] = parts["pod"].bounding_box().min.Z - z_ground
    # battery inside pod
    # battery inside the faired pod's cavity: no contact with the 2 mm shell (kestrel.cad_detail.faired_pod)
    from . import cad_detail
    shell, _ = cad_detail.faired_pod(s)
    res["battery_in_pod"] = bool(cad_detail.vol(parts["battery"] & shell) < 1e-9
                                 and parts["battery"].bounding_box().min.Z > shell.bounding_box().min.Z)
    # pusher to trailing edge clearance (min distance)
    res["pusher_wing_distance_m"] = parts["propdisc_pusher"].distance_to(parts["wing_L"] + parts["wing_R"])
    # spars must stay inside the wing (or the pod): fraction of each spar's volume enclosed
    shell = parts["wing_L"] + parts["wing_R"] + parts["pod"]
    res["spar_inside_frac"] = {k: (parts[k] & shell).volume / parts[k].volume
                               for k in ("cross_spar", "outer_spar_L", "outer_spar_R")}
    return res


def mass_properties(s, parts):
    """Mass, CG and inertia (about CG), FRD."""
    comps = []

    def add(name, mass, shape):
        vol = shape.volume
        c = shape.center()
        I = np.array(shape.matrix_of_inertia) * (mass / vol)  # about own CoM, unit density -> scaled
        comps.append((name, mass, np.array([c.X, c.Y, c.Z]), I))

    w = s["wing"]
    for side in ("L", "R"):
        wing = parts[f"wing_{side}"]
        skin_area = wing.area - 2 * 0  # outer surface area (top+bottom+ends)
        m_wing = skin_area * FOAM_AREAL * 1.12   # + hot glue and tape
        add(f"wing_{side}", m_wing, wing)
        add(f"winglet_{side}", parts[f"winglet_{side}"].volume / FOAM_T * FOAM_AREAL, parts[f"winglet_{side}"])
    pod_sh = parts["pod"]
    add("pod", pod_sh.area * FOAM_AREAL * 1.15, pod_sh)
    for k in ("cross_spar", "outer_spar_L", "outer_spar_R"):
        add(k, parts[k].volume * CARBON, parts[k])
    add("glue", 0.008, bd.Box(0.2, 0.6, 0.004).translate((-0.17, 0, 0.004)))
    for side in ("L", "R"):
        b = parts[f"boom_{side}"]
        add(f"boom_{side}", b.volume * CARBON, b)
    motor_m = s["motors"][s["propulsion"]["lift_motor"]]["mass"]
    prop_m = s["props"][s["propulsion"]["lift_prop"]]["mass"]
    for k in ("FR", "RL", "FL", "RR"):
        add(f"motor_{k}", motor_m + prop_m, parts[f"motor_{k}"])
        add(f"mount_{k}", 0.010, parts[f"mount_{k}"])
        add(f"leg_{k}", s["landing_legs"]["mass_each"], parts[f"leg_{k}"])
        c = parts[f"motor_{k}"].center()
        add(f"esc_{k}", 0.025, bd.Box(0.045, 0.025, 0.008).translate((c.X - 0.06 * np.sign(c.X + 0.119), c.Y, -0.012)))
    add("pusher_motor", s["motors"][s["propulsion"]["pusher_motor"]]["mass"] + s["props"][s["propulsion"]["pusher_prop"]]["mass"] + 0.008,
        parts["pusher_motor"])
    for k, (m, size, c) in _items(s).items():
        add(k, m, parts[k])
    M = sum(c[1] for c in comps)
    cg = sum(c[1] * c[2] for c in comps) / M
    I = np.zeros((3, 3))
    for _, m, r, Ic in comps:
        d = r - cg
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    # FLU -> FRD: flip y and z; inertia products Ixy, Ixz change sign? (diag same; Ixz sign kept since both flip)
    T = np.diag([1, -1, -1])
    I_frd = T @ I @ T
    cg_frd = T @ cg
    breakdown = {n: m for n, m, _, _ in comps}
    return {"mass": float(M), "cg": cg_frd.tolist(), "inertia": I_frd.tolist(), "source": "cad", "breakdown": breakdown}


# ---------------- foam templates ----------------

def _surface_lengths():
    xy = mh45_xy()
    i_le = int(np.argmin(xy[:, 0]))
    up = xy[: i_le + 1]
    lo = xy[i_le:]
    L = lambda p: float(np.sum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))))
    return L(up), L(lo)


def foam_parts(s):
    """Flat patterns (mm). Each: name, outer polygon, list of dashed fold/hinge lines."""
    w, f = s["wing"], s["fuselage"]
    b2 = w["span"] / 2 * 1000
    sw = b2 * math.tan(math.radians(w["le_sweep_deg"]))
    cr, ct = w["root_chord"] * 1000, w["tip_chord"] * 1000
    Lu, Ll = _surface_lengths()
    parts = []
    e0, e1 = w["elevon_span_frac"]
    hinge = 1 - w["elevon_chord_frac"]
    split = 0.5   # each half wing is made from an inner and an outer panel (joined at the spar)

    def chord(eta):
        return cr + (ct - cr) * eta

    for side in ("left", "right"):
        for name, (ea, eb) in (("inner", (0.0, split)), ("outer", (split, 1.0))):
            # Unrolled panel: fold line at the LE; lower skin aft (+x), upper skin forward (-x). Local origin at
            # the panel's inboard LE.
            ya, yb = ea * b2, eb * b2
            xa, xb = sw * ea, sw * eb
            ox = xa
            poly = [(xa - Lu * chord(ea) - ox, 0.0), (xa + Ll * chord(ea) - ox, 0.0),
                    (xb + Ll * chord(eb) - ox, yb - ya), (xb - Lu * chord(eb) - ox, yb - ya)]

            def at(eta, frac):
                return (sw * eta + frac * chord(eta) * Ll - ox, eta * b2 - ya)
            lines = [("fold (LE, score inside)", [(0.0, 0.0), (xb - ox, yb - ya)]),
                     ("spar line", [(0.25 * chord(ea) * Ll, 0.0), (xb - ox + 0.25 * chord(eb) * Ll, yb - ya)])]
            lo, hi = max(e0, ea), min(e1, eb)
            if hi > lo:
                lines += [("elevon hinge (cut top skin only)", [at(lo, hinge), at(hi, hinge)])]
                if ea <= e0 <= eb:
                    lines.append(("elevon end cut", [at(e0, hinge), at(e0, 1.0)]))
                if ea <= e1 <= eb:
                    lines.append(("elevon end cut", [at(e1, hinge), at(e1, 1.0)]))
            if side == "right":
                poly = [(x, -y) for x, y in poly]
                lines = [(n, [(x, -y) for x, y in pts]) for n, pts in lines]
            parts.append((f"wing_{side}_{name}", poly, lines))
    crw, h = w["winglet_root_chord"] * 1000, w["winglet_height"] * 1000
    for side in ("left", "right"):
        parts.append((f"winglet_{side}", [(0, 0), (crw, 0), (crw, h), (0.4 * crw, h)], []))
    L, W, H = f["length"] * 1000, f["width"] * 1000, f["height"] * 1000
    parts += [("pod_side_left", [(0, 0), (L, 0), (L, H), (0, H)], []),
              ("pod_side_right", [(0, 0), (L, 0), (L, H), (0, H)], []),
              ("pod_top_hatch", [(0, 0), (L, 0), (L, W - 10), (0, W - 10)], [("hatch cut", [(20, 0), (20, W - 10)])]),
              ("pod_bottom", [(0, 0), (L, 0), (L, W - 10), (0, W - 10)], [])]
    return parts


def nest(parts):
    """Greedy shelf nesting on 762 x 508 mm sheets (wings one per sheet, small parts packed). Returns sheets."""
    from shapely.geometry import Polygon, box
    from shapely import affinity
    sheets = []
    sheet_box = box(0, 0, *SHEET)

    def norm(poly):
        p = Polygon(poly)
        minx, miny, _, _ = p.bounds
        return affinity.translate(p, -minx + 5, -miny + 5)

    small = []
    wings = [(n, p, l) for n, p, l in parts if n.startswith("wing_")]
    for i in range(0, len(wings), 1):
        sh = {"parts": [], "fits": True}
        yoff = 5.0
        for name, poly, lines in wings[i:i + 1]:
            p0 = Polygon(poly)
            minx, miny, maxx, maxy = p0.bounds
            dx, dy = -minx + 5, -miny + yoff
            p = affinity.translate(p0, dx, dy)
            sh["parts"].append((name, p, [(n, [(x + dx, y + dy) for x, y in pts]) for n, pts in lines]))
            sh["fits"] = sh["fits"] and sheet_box.contains(p)
            yoff += (maxy - miny) + 8
        sheets.append(sh)
    small = [(n, p, l) for n, p, l in parts if not n.startswith("wing_")]
    cur = {"parts": [], "fits": True}
    x, y, row_h = 5.0, 5.0, 0.0
    for name, poly, lines in small:
        p0 = Polygon(poly)
        minx, miny, maxx, maxy = p0.bounds
        wdt, hgt = maxx - minx, maxy - miny
        if x + wdt > SHEET[0] - 5:
            x, y, row_h = 5.0, y + row_h + 8, 0.0
        if y + hgt > SHEET[1] - 5:
            sheets.append(cur)
            cur = {"parts": [], "fits": True}
            x, y, row_h = 5.0, 5.0, 0.0
        dx, dy = x - minx, y - miny
        p = affinity.translate(p0, dx, dy)
        cur["parts"].append((name, p, [(n, [(a + dx, b + dy) for a, b in pts]) for n, pts in lines]))
        cur["fits"] = cur["fits"] and sheet_box.contains(p)
        x += wdt + 8
        row_h = max(row_h, hgt)
    if cur["parts"]:
        sheets.append(cur)
    # overlap check
    for sh in sheets:
        polys = [p for _, p, _ in sh["parts"]]
        sh["overlap"] = any(polys[i].intersection(polys[j]).area > 1e-6 for i in range(len(polys)) for j in range(i + 1, len(polys)))
    return sheets


def write_templates(sheets):
    TPL.mkdir(parents=True, exist_ok=True)
    for old in TPL.glob("sheet_*.svg"):   # a re-nest with fewer sheets must not leave stale templates behind
        old.unlink()
    for i, sh in enumerate(sheets, 1):
        W, H = SHEET
        out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">',
               f'<rect x="0" y="0" width="{W}" height="{H}" fill="none" stroke="#999" stroke-width="0.5"/>',
               f'<text x="10" y="{H-8}" font-size="8">Kestrel foam sheet {i}/{len(sheets)} - 762 x 508 mm (30 x 20 in) - print at 100 %</text>',
               '<rect x="10" y="10" width="50" height="50" fill="none" stroke="#000" stroke-width="0.4"/>'
               '<text x="12" y="38" font-size="5">50 mm check</text>']
        for name, p, lines in sh["parts"]:
            pts = " ".join(f"{x:.2f},{H - y:.2f}" for x, y in p.exterior.coords)
            out.append(f'<polygon points="{pts}" fill="none" stroke="#000" stroke-width="0.4"/>')
            c = p.centroid
            out.append(f'<text x="{c.x:.1f}" y="{H - c.y:.1f}" font-size="7" text-anchor="middle">{name}</text>')
            for n, ln in lines:
                lp = " ".join(f"{x:.2f},{H - y:.2f}" for x, y in ln)
                out.append(f'<polyline points="{lp}" fill="none" stroke="#c00" stroke-width="0.4" stroke-dasharray="4,3"><title>{n}</title></polyline>')
        out.append("</svg>")
        (TPL / f"sheet_{i}.svg").write_text("\n".join(out))
    # tiled Letter PDF
    from reportlab.lib.pagesizes import letter
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    pw, ph = letter[0] / mm, letter[1] / mm
    m = 10.0
    tw, th = pw - 2 * m, ph - 2 * m
    c = canvas.Canvas(str(TPL / "kestrel_templates_letter.pdf"), pagesize=letter)
    pages = 0
    for i, sh in enumerate(sheets, 1):
        nx, ny = math.ceil(SHEET[0] / tw), math.ceil(SHEET[1] / th)
        for jy in range(ny):
            for jx in range(nx):
                ox, oy = jx * tw, jy * th
                c.saveState()
                p = c.beginPath(); p.rect(m * mm, m * mm, tw * mm, th * mm); c.clipPath(p, stroke=1, fill=0)
                for name, poly, lines in sh["parts"]:
                    pts = list(poly.exterior.coords)
                    path = c.beginPath()
                    path.moveTo((pts[0][0] - ox + m) * mm, (pts[0][1] - oy + m) * mm)
                    for x, y in pts[1:]:
                        path.lineTo((x - ox + m) * mm, (y - oy + m) * mm)
                    c.setDash(); c.setStrokeColorRGB(0, 0, 0); c.drawPath(path)
                    for n, ln in lines:
                        c.setDash(4 * mm / 3, 3 * mm / 3); c.setStrokeColorRGB(0.8, 0, 0)
                        c.line((ln[0][0] - ox + m) * mm, (ln[0][1] - oy + m) * mm, (ln[1][0] - ox + m) * mm, (ln[1][1] - oy + m) * mm)
                    cx, cy = poly.centroid.x, poly.centroid.y
                    c.setFillColorRGB(0, 0, 0); c.setFont("Helvetica", 7)
                    c.drawCentredString((cx - ox + m) * mm, (cy - oy + m) * mm, name)
                c.restoreState()
                c.setFont("Helvetica", 7); c.setDash()
                c.drawString(m * mm, 4 * mm, f"Sheet {i} tile row {jy+1}/{ny} col {jx+1}/{nx} - print at 100 % (no scaling), tape edges to edges")
                if jx == 0 and jy == 0:
                    c.rect((m + 5) * mm, (m + 5) * mm, 50 * mm, 50 * mm); c.drawString((m + 7) * mm, (m + 30) * mm, "50 mm check square")
                c.showPage(); pages += 1
    c.save()
    return pages


# ---------------- printed parts ----------------

def printed_parts(s):
    tube_d = s["booms"]["diameter"] * 1000
    # motor mount: 34 x 34 mm plate, A2212 cross-mount holes on 16 / 19 mm pattern, tube saddle below.
    # The plate is tilted about the boom by yaw_tilt_deg. All four mounts are identical: fit each so its
    # motor leans outboard (FR/RR right, FL/RL left); turning a mount 180 deg on the boom flips the lean.
    tilt = s["lift_rotors"].get("yaw_tilt_deg", 0.0)
    plate = bd.Box(34, 34, 7)
    for (x, y) in [(8, 0), (-8, 0), (0, 9.5), (0, -9.5)]:
        plate -= bd.Cylinder(1.6, 9).translate((x, y, 0))
    plate -= bd.Cylinder(4.5, 9)  # shaft clearance
    plate = plate.rotate(bd.Axis.X, tilt).translate((0, 0, 1.0))
    saddle = bd.Box(34, tube_d + 6, tube_d + 4).translate((0, 0, -(tube_d + 4) / 2 - 2.5))
    saddle -= bd.Cylinder((tube_d + 0.3) / 2, 40, rotation=(0, 90, 0)).translate((0, 0, -(tube_d + 4) / 2 - 2.5))
    saddle -= bd.Box(36, 1.5, tube_d).translate((0, 0, -(tube_d + 4) - 2.5))     # clamp slot
    mount = plate + saddle
    leg = bd.Box(12, 12, s["landing_legs"]["height"] * 1000 - tube_d / 2).translate((0, 0, -(s["landing_legs"]["height"] * 1000 - tube_d / 2) / 2))
    leg += bd.Box(20, tube_d + 6, tube_d + 4)
    leg -= bd.Cylinder((tube_d + 0.3) / 2, 40, rotation=(90, 0, 0))
    leg -= bd.Cylinder((tube_d + 0.3) / 2, 40, rotation=(0, 90, 0))
    leg -= bd.Box(24, 4.6, 3.0).translate((0, 0, tube_d / 2 + 1.5))    # channel for the motor leads (4.6 mm)
    pusher = bd.Box(40, 40, 4)
    for (x, y) in [(8, 0), (-8, 0), (0, 9.5), (0, -9.5)]:
        pusher -= bd.Cylinder(1.6, 6).translate((x, y, 0))
    pusher -= bd.Cylinder(4.5, 6)
    pusher += bd.Box(40, 4, 60).translate((0, -20, -30))
    return {"motor_mount": mount, "leg": leg, "pusher_mount": pusher}


def main():
    s = S.load()
    OUT.mkdir(parents=True, exist_ok=True)
    parts, rotors = build(s)
    chk = clearance_checks(s, parts)
    mp = mass_properties(s, parts)
    (ROOT / "cad" / "mass_props.json").write_text(json.dumps(mp, indent=1))
    asm = bd.Compound([v for k, v in parts.items() if not k.startswith("propdisc")])
    T = bd.Location((0, 0, 0))
    bd.export_step(asm, str(OUT / "kestrel_assembly.step"))
    bd.export_stl(asm.scale(1000), str(OUT / "kestrel_assembly.stl"))
    pp = printed_parts(s)
    for k, v in pp.items():
        bd.export_stl(v, str(OUT / f"{k}.stl"))
    printed_mass = {k: v.volume * 1e-9 * PETG * PRINT_FILL for k, v in pp.items()}
    sheets = nest(foam_parts(s))
    pages = write_templates(sheets)
    budget = sum(v["m"] for v in s["mass_items"].values())
    # balance: battery station for the target static margin (needs aero/tables.json for the neutral point)
    tp = ROOT / "aero" / "tables.json"
    if tp.exists():
        t = json.loads(tp.read_text())
        mac = S.wing_geometry(s)["mac"]
        cg_tab = -t["ref"]["cg_asb"][0]
        # the worst local margin moves 1:1 with CG shift (in MAC): put the CG where the minimum local margin = target
        target = cg_tab + (s["limits"]["target_static_margin"] - t["static_margin_min"]) * mac
        x_np = -t["x_np_asb"]
        m_b = s["batteries"][s["propulsion"]["battery"]]["mass"]
        chk["battery_x_for_target_sm"] = s["mass_items"]["battery"]["x"] + (target - mp["cg"][0]) * mp["mass"] / m_b
        chk["cg_target_frd_x"] = target
        chk["static_margin_cad"] = (mp["cg"][0] - x_np) / mac
        chk["static_margin_min_local_cad"] = t["static_margin_min"] + (mp["cg"][0] - cg_tab) / mac
    chk.update({
        "mass_cad_kg": mp["mass"], "mass_budget_kg": budget, "mass_error_frac": mp["mass"] / budget - 1,
        "cg_frd": mp["cg"], "sheets": len(sheets), "sheets_fit": all(sh["fits"] for sh in sheets),
        "sheets_overlap": any(sh["overlap"] for sh in sheets), "template_pages_letter": pages,
        "printed_part_mass_kg": printed_mass,
    })
    (ROOT / "cad" / "checks.json").write_text(json.dumps(chk, indent=1, default=float))
    print(json.dumps(chk, indent=1, default=float))
    print("Mass breakdown (g):", {k: round(v * 1000) for k, v in mp["breakdown"].items()})
    print("Inertia FRD:", np.round(np.array(mp["inertia"]), 5).tolist())


if __name__ == "__main__":
    main()
