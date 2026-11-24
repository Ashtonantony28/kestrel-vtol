"""Dimensioned build drawings (A3 landscape PDF + PNG previews), projected from the detailed CAD.

Run: python -m kestrel.drawing      -> cad/drawings/kestrel_drawings.pdf, cad/drawings/sheet_*.png
Sheet 1 general arrangement (1:6), sheet 2 wing and structure (1:3, sections 1:2), sheet 3 lift unit,
pusher and printed parts (1:2 and 1:1). Every number printed comes from build/dimensions.json or the spec.
Hidden-line removal is done by OpenCascade (build123d project_to_viewport).
"""
from __future__ import annotations

import datetime as _dt
import json
import math

import build123d as bd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import Arc, Circle, Rectangle  # noqa: E402
import numpy as np                                  # noqa: E402

from . import cad_detail as D                       # noqa: E402
from . import dimensions as DIM                     # noqa: E402
from . import spec as S                             # noqa: E402

OUTD = S.ROOT / "cad" / "drawings"
A3 = (420.0, 297.0)
INK, HID, DIMC, CL = "#111111", "#8a8a8a", "#1f4f8a", "#b0503a"
FS = 6.0           # dimension text, pt
LW, LWH, LWD = 0.6, 0.35, 0.3

# view directions: name -> (viewport origin, up, mapping FLU(x,y,z) -> view (u,v) as build123d returns it)
VIEWS = {
    "top": ((0, 0, 50), (1, 0, 0), lambda x, y, z: (-y, x)),
    "front": ((50, 0, 0), (0, 0, 1), lambda x, y, z: (y, z)),
    "side": ((0, 50, 0), (0, 0, 1), lambda x, y, z: (-x, z)),
}


def _edge_pts(e, n=28):
    try:
        if e.geom_type == bd.GeomType.LINE:
            a, b = e @ 0, e @ 1
            return np.array([[a.X, a.Y], [b.X, b.Y]])
    except Exception:
        pass
    pts = [e @ t for t in np.linspace(0, 1, n)]
    return np.array([[p.X, p.Y] for p in pts])


def project(shapes, view):
    org, up, _ = VIEWS[view]
    def ok(sh):
        try:
            return sh._wrapped is not None and sh.volume > 1e-12
        except Exception:
            return False
    comp = bd.Compound([sh for sh in shapes if ok(sh)])
    vis, hid = comp.project_to_viewport(org, viewport_up=up, look_at=(0, 0, 0))
    return [_edge_pts(e) * 1000 for e in vis], [_edge_pts(e) * 1000 for e in hid]


class Sheet:
    def __init__(self, pdf, title, number, total, scale_note):
        self.fig = plt.figure(figsize=(A3[0] / 25.4, A3[1] / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, A3[0])
        self.ax.set_ylim(0, A3[1])
        self.ax.set_aspect("equal")
        self.ax.axis("off")
        self.pdf, self.number = pdf, number
        ax = self.ax
        ax.add_patch(Rectangle((8, 8), A3[0] - 16, A3[1] - 16, fill=False, lw=1.0, ec=INK))
        # title block
        x0, y0, w, h = A3[0] - 8 - 170, 8, 170, 30
        ax.add_patch(Rectangle((x0, y0), w, h, fill=False, lw=0.8, ec=INK))
        ax.plot([x0, x0 + w], [y0 + 18, y0 + 18], lw=0.4, c=INK)
        ax.plot([x0 + 110, x0 + 110], [y0, y0 + 18], lw=0.4, c=INK)
        ax.text(x0 + 3, y0 + 24, f"KESTREL QUADPLANE  —  {title}", fontsize=9, weight="bold", va="center")
        ax.text(x0 + 3, y0 + 12.5, "All dimensions mm unless noted. Stations from the wing root leading edge.", fontsize=5.5, va="center")
        ax.text(x0 + 3, y0 + 6.5, "Generated from spec/vehicle.yaml by python -m kestrel.drawing. Do not scale; use the numbers.", fontsize=5.5, va="center")
        ax.text(x0 + 113, y0 + 12.5, scale_note, fontsize=6, va="center")
        ax.text(x0 + 113, y0 + 6.5, f"Sheet {number} of {total}   {_dt.date.today().isoformat()}", fontsize=6, va="center")

    # ----- primitives in paper mm ------------------------------------------------------------
    def lines(self, polys, color=INK, lw=LW, ls="-", z=2):
        for p in polys:
            self.ax.plot(p[:, 0], p[:, 1], color=color, lw=lw, ls=ls, solid_capstyle="round", zorder=z)

    def text(self, x, y, s, size=FS, color=DIMC, rot=0, ha="center", va="center", weight="normal"):
        self.ax.text(x, y, s, fontsize=size, color=color, rotation=rot, ha=ha, va=va, weight=weight, zorder=6,
                     bbox=dict(boxstyle="square,pad=0.15", fc="white", ec="none", alpha=0.85))

    def arrow(self, a, b):
        self.ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="<|-|>", lw=LWD, color=DIMC,
                                                               mutation_scale=4, shrinkA=0, shrinkB=0), zorder=5)

    def dim(self, p1, p2, at, text, orient="h", ext_gap=0.8):
        """Linear dimension between paper points p1, p2. orient 'h': horizontal line at y=at; 'v': vertical at x=at."""
        ax = self.ax
        (x1, y1), (x2, y2) = p1, p2
        if orient == "h":
            for (x, y) in (p1, p2):
                s = 1 if at > y else -1
                ax.plot([x, x], [y + s * ext_gap, at + s * 1.2], lw=LWD, c=DIMC, zorder=4)
            self.arrow((x1, at), (x2, at))
            self.text((x1 + x2) / 2, at + 1.6, text, va="bottom")
        else:
            for (x, y) in (p1, p2):
                s = 1 if at > x else -1
                ax.plot([x + s * ext_gap, at + s * 1.2], [y, y], lw=LWD, c=DIMC, zorder=4)
            self.arrow((at, y1), (at, y2))
            self.text(at - 1.6, (y1 + y2) / 2, text, rot=90, ha="right")

    def leader(self, p, txt_xy, text, ha="left"):
        self.ax.annotate(text, xy=p, xytext=txt_xy, fontsize=FS, color=DIMC, ha=ha, va="center", zorder=6,
                         arrowprops=dict(arrowstyle="-|>", lw=LWD, color=DIMC, mutation_scale=4, shrinkA=1, shrinkB=0))

    def angle(self, c, r, a0, a1, text, tpos=None):
        self.ax.add_patch(Arc(c, 2 * r, 2 * r, theta1=a0, theta2=a1, lw=LWD, ec=DIMC, zorder=4))
        am = math.radians((a0 + a1) / 2)
        tp = tpos or (c[0] + (r + 4) * math.cos(am), c[1] + (r + 4) * math.sin(am))
        self.text(*tp, text)

    def label(self, x, y, s, size=7.5):
        self.ax.text(x, y, s, fontsize=size, weight="bold", ha="center", va="top", color=INK)

    def save(self, png_name):
        self.fig.savefig(OUTD / png_name, dpi=130)
        self.pdf.savefig(self.fig)
        plt.close(self.fig)


class View:
    """Maps model mm in a view's (u, v) to paper mm: paper = origin + (u, v) / scale."""

    def __init__(self, sheet, name, origin, scale):
        self.sh, self.name, self.o, self.k = sheet, name, origin, scale

    def p(self, u, v):
        return (self.o[0] + u / self.k, self.o[1] + v / self.k)

    def m(self, x, y, z):           # FLU metres -> paper
        u, v = VIEWS[self.name][2](x * 1000, y * 1000, z * 1000)
        return self.p(u, v)

    def draw(self, vis, hid=None, hidden=False, color=INK, lw=LW):
        tr = lambda P: np.column_stack([self.o[0] + P[:, 0] / self.k, self.o[1] + P[:, 1] / self.k])
        if hidden and hid:
            self.sh.lines([tr(p) for p in hid], color=HID, lw=LWH, ls=(0, (2, 1.4)), z=1)
        self.sh.lines([tr(p) for p in vis], color=color, lw=lw)


def _f(v):
    return f"{v:.0f}" if abs(v - round(v)) < 0.05 else f"{v:.1f}"


def build_all():
    s = S.load()
    P = D.build_detail(s)
    dims = DIM.main(P)
    OUTD.mkdir(parents=True, exist_ok=True)
    w, bm, lr, fu, st, lg, pu = (s[k] for k in ("wing", "booms", "lift_rotors", "fuselage", "structure", "landing_legs", "pusher"))
    b2, chord, x_le, x_te, x_hinge = D.wing_dims(s)
    tsw = math.tan(math.radians(w["le_sweep_deg"]))
    lp = s["props"][s["propulsion"]["lift_prop"]]
    ppu = s["props"][s["propulsion"]["pusher_prop"]]
    mp = json.loads((S.ROOT / "cad" / "mass_props.json").read_text())
    cgx = mp["cg"][0]
    gz = dims["ground_z_mm"] / 1000
    solid = [v for k, v in P.items() if not k.startswith("propdisc")]
    xf, xr, yb = lr["x_front"], lr["x_rear"], lr["y"]
    wl = P["winglet_L"].bounding_box()
    proj = {v: project(solid, v) for v in ("top", "front", "side")}

    with PdfPages(OUTD / "kestrel_drawings.pdf") as pdf:
        # ================= SHEET 1: general arrangement ===================================
        sh = Sheet(pdf, "GENERAL ARRANGEMENT", 1, 3, "Scale 1:6 (A3)")
        k = 6.0
        T = View(sh, "top", (122, 200), k)
        T.draw(*proj["top"], hidden=True)
        for kk, (x, y) in {"FR": (xf, -yb), "FL": (xf, yb), "RR": (xr, -yb), "RL": (xr, yb)}.items():
            sh.ax.add_patch(Circle(T.m(x, y, 0), lp["d"] * 1000 / 2 / k, fill=False, ls=(0, (3, 2)), lw=LWH, ec=HID))
        sh.lines([np.array([T.m(0.25, 0, 0), T.m(-0.70, 0, 0)])], color=CL, lw=LWD, ls=(0, (6, 2, 1, 2)))
        sh.label(122, 288, "TOP VIEW (nose up)")
        # span at the tip trailing edges, below
        yb_dim = T.m(-0.70, 0, 0)[1]
        sh.dim(T.m(x_te(b2), b2, 0), T.m(x_te(b2), -b2, 0), yb_dim - 4, f"wingspan {_f(w['span'] * 1000)}")
        # widths above the nose
        top_y = T.m(0.31, 0, 0)[1]
        sh.dim(T.m(bm["x_front"], bm["y"], 0), T.m(bm["x_front"], -bm["y"], 0), top_y + 6, f"boom spacing {_f(2 * bm['y'] * 1000)}")
        sh.dim(T.m(wl.min.X, wl.max.Y, 0), T.m(wl.min.X, -wl.max.Y, 0), top_y + 14, f"overall width {_f(wl.max.Y * 2000)}")
        # right side: motor spacing and overall length
        rx = T.m(0, -0.5, 0)[0]
        sh.dim(T.m(xf, -yb, 0), T.m(xr, -yb, 0), rx + 4, f"motor spacing {_f((xf - xr) * 1000)}", orient="v")
        bb = bd.Compound(list(P.values())).bounding_box()
        sh.dim(T.m(bb.max.X, -yb, 0), T.m(bb.min.X, -yb, 0), rx + 13, f"overall length {_f(bb.size.X * 1000)}", orient="v")
        # left side chain: pod nose | tip LE setback | tip chord
        lx = T.m(0, 0.505, 0)[0] - 6
        sh.dim(T.m(fu["nose_x"], 0.035, 0), T.m(0, 0, 0), lx, _f(fu["nose_x"] * 1000), orient="v")
        sh.dim(T.m(0, 0, 0), T.m(x_le(b2), b2, 0), lx, f"tip LE {_f(b2 * tsw * 1000)}", orient="v")
        sh.dim(T.m(x_le(b2), b2, 0), T.m(x_te(b2), b2, 0), lx, f"tip chord {_f(w['tip_chord'] * 1000)}", orient="v")
        sh.dim(T.m(0, 0, 0), T.m(-w["root_chord"], 0, 0), lx - 9, f"root chord {_f(w['root_chord'] * 1000)}", orient="v")
        # CG
        cgp = T.m(cgx, 0, 0)
        sh.ax.add_patch(Circle(cgp, 1.8, fill=False, lw=0.6, ec=CL, zorder=6))
        sh.ax.plot([cgp[0] - 2.6, cgp[0] + 2.6], [cgp[1], cgp[1]], c=CL, lw=0.6, zorder=6)
        sh.ax.plot([cgp[0], cgp[0]], [cgp[1] - 2.6, cgp[1] + 2.6], c=CL, lw=0.6, zorder=6)
        sh.dim(T.m(0, 0, 0), cgp, T.m(0, -0.09, 0)[0], f"CG {_f(-cgx * 1000)}", orient="v")
        # sweep
        c0 = T.m(0, 0, 0)
        sh.lines([np.array([c0, T.m(0, 0.22, 0)])], color=DIMC, lw=LWD, ls=(0, (3, 2)))
        sh.angle(c0, 30, 180 - 0, 180 + w["le_sweep_deg"], f"{w['le_sweep_deg']:.0f}° LE sweep", tpos=(c0[0] - 44, c0[1] - 9))
        sh.leader(T.m(xf + lp["d"] * 0.35, yb + lp["d"] * 0.35, 0), (T.m(0, 0.30, 0)[0] - 8, T.m(0.36, 0, 0)[1]), f"Ø{_f(lp['d'] * 1000)} (10×4.5 in), 4 places", ha="right")
        sh.leader(T.m(-0.43, -0.07, 0), (T.m(0, -0.10, 0)[0], T.m(-0.75, 0, 0)[1]), f"pusher Ø{_f(ppu['d'] * 1000)} (8×6 in)")

        # side view (from the left; nose to the left)
        Sd = View(sh, "side", (278, 232), k)
        Sd.draw(*proj["side"])
        g0, g1 = Sd.m(0.35, 0, gz), Sd.m(-0.68, 0, gz)
        sh.lines([np.array([g0, g1])], color=INK, lw=0.4)
        for i in range(24):
            xx = g0[0] + i * (g1[0] - g0[0]) / 23
            sh.ax.plot([xx, xx - 1.5], [g0[1], g0[1] - 1.5], c=INK, lw=0.3)
        sh.label(330, 288, "SIDE VIEW (from left)")
        legx = P["leg_FL"].bounding_box().center().X
        sh.dim(Sd.m(legx, 0, -0.012), Sd.m(legx, 0, gz), Sd.m(0.24, 0, 0)[0] - 2, _f(lg["height"] * 1000), orient="v")
        sh.dim(Sd.m(xf, 0, -0.012), Sd.m(xf, 0, -lr["z"]), Sd.m(0.32, 0, 0)[0] - 2, _f((-lr["z"] + 0.012) * 1000), orient="v")
        sh.dim(Sd.m(wl.max.X, 0, wl.max.Z), Sd.m(wl.max.X, 0, gz), Sd.m(-0.68, 0, 0)[0] + 3, f"{_f((wl.max.Z - gz) * 1000)} winglet", orient="v")
        pz = -pu["z"]
        sh.dim(Sd.m(-0.44, 0, pz), Sd.m(-0.44, 0, gz), Sd.m(-0.68, 0, 0)[0] + 12, f"{_f((pz - gz) * 1000)} pusher axis", orient="v")
        podb = P["pod"].bounding_box()
        sh.dim(Sd.m(0.10, 0, podb.min.Z), Sd.m(0.10, 0, gz), Sd.m(0.10, 0, 0)[0] + 3, _f((podb.min.Z - gz) * 1000), orient="v")

        # front view
        F = View(sh, "front", (330, 140), k)
        F.draw(*proj["front"])
        sh.lines([np.array([F.m(0, 0.55, gz), F.m(0, -0.55, gz)])], color=INK, lw=0.4)
        sh.label(330, 196, "FRONT VIEW")
        fz = F.m(0, 0, -lr["z"])[1]
        sh.dim(F.m(0, yb - lp["d"] / 2, -lr["z"]), F.m(0, -(yb - lp["d"] / 2), -lr["z"]), fz + 9, f"prop tip gap {_f((2 * yb - lp['d']) * 1000)}")
        sh.dim(F.m(0, yb, -0.012), F.m(0, -yb, -0.012), F.m(0, 0, gz)[1] - 6, f"boom spacing {_f(2 * yb * 1000)}")
        sh.dim(F.m(0, -0.505, bb.max.Z), F.m(0, -0.505, gz), F.m(0, -0.56, 0)[0], f"{_f((bb.max.Z - gz) * 1000)}", orient="v")
        sh.leader(F.m(0, -0.28, -lr["z"] + 0.004), (F.m(0, -0.44, 0)[0], fz + 18), f"motors lean {lr.get('yaw_tilt_deg', 0):.0f}° outboard")

        # key numbers box
        rows = [(r["name"], f"{r['value']} {r['unit']}".strip()) for r in dims["table"] if r["group"] == "Overall"]
        x0, y0 = 246, 46
        sh.ax.text(x0, y0 + 8 + 5.2 * len(rows), "KEY NUMBERS", fontsize=7, weight="bold")
        for i, (a, b) in enumerate(rows):
            yy = y0 + 5 + 5.2 * (len(rows) - 1 - i)
            sh.ax.text(x0, yy, a, fontsize=5.8)
            sh.ax.text(x0 + 165, yy, b, fontsize=5.8, ha="right", family="monospace")
        sh.save("sheet_1_general_arrangement.png")

        # ================= SHEET 2: wing and structure =======================================
        sh = Sheet(pdf, "WING, ELEVON AND STRUCTURE", 2, 3, "Plan 1:3, sections 1:2 (A3)")
        k = 3.0
        wingL = [P[n] for n in ("wing_L", "elevon_L", "winglet_L", "servo_L", "horn_L", "pushrod_L", "outer_spar_L",
                                "clamp_L", "harness_L")]
        wingL.append(P["boom_L"] & bd.Box(0.56, 1, 1, align=(bd.Align.MAX, bd.Align.MIN, bd.Align.CENTER)).translate((0.02, 0, 0)))
        csL = P["cross_spar"] & bd.Box(1, 1, 1, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.CENTER))
        vis, hid = project(wingL + [csL], "top")
        T = View(sh, "top", (200, 262), k)
        T.draw(vis, hid, hidden=True)
        sh.lines([np.array([T.m(0.03, 0, 0), T.m(-0.56, 0, 0)])], color=CL, lw=LWD, ls=(0, (6, 2, 1, 2)))
        sh.label(115, 288, "LEFT WING, TOP VIEW (right wing is a mirror image)")
        y0e, y1e = (f * b2 for f in w["elevon_span_frac"])
        botv = T.m(x_te(b2) - 0.02, 0, 0)[1]
        # spanwise stations along the bottom
        segs = D.elevon_segments(s)
        servo_y = P["servo_L"].bounding_box().center().Y
        ys_sorted = sorted({0.0, *(round(v, 4) for sg in segs for v in sg), round(bm["y"], 4), b2})
        for a, bq in zip(ys_sorted[:-1], ys_sorted[1:]):
            sh.dim(T.m(x_te(a), a, 0), T.m(x_te(bq), bq, 0), botv - 6, _f((bq - a) * 1000))
        sh.dim(T.m(x_te(0), 0, 0), T.m(x_te(b2), b2, 0), botv - 15, f"semispan {_f(b2 * 1000)}")
        sh.text(T.m(0, 0.5, 0)[0] - 2, botv - 22, "stations from centreline: elevon segments " + ", ".join(f"{_f(a*1000)}–{_f(b*1000)}" for a, b in segs)
                + f" (split at the boom, {_f(bm['y']*1000)}), servo {_f(servo_y*1000)}, outer spar {_f(st['outer_spar']['y0']*1000)}–{_f(st['outer_spar']['y1']*1000)}",
                size=5.6, ha="left", color=INK)
        # chords and hinge
        rx = T.m(0, -0.03, 0)[0]
        sh.dim(T.m(0, 0, 0), T.m(-w["root_chord"], 0, 0), rx + 6, f"root chord {_f(w['root_chord'] * 1000)}", orient="v")
        sh.dim(T.m(0, 0, 0), T.m(x_hinge(y0e), y0e, 0), rx + 15, f"hinge {_f(-x_hinge(y0e) * 1000)}", orient="v")
        sh.dim(T.m(0, 0, 0), T.m(-st["cross_spar"]["x_aft"], 0, 0), rx + 24, f"cross spar {_f(st['cross_spar']['x_aft'] * 1000)}", orient="v")
        lx = T.m(0, 0.52, 0)[0] - 4
        sh.dim(T.m(0, 0, 0), T.m(x_le(b2), b2, 0), lx, f"{_f(b2 * tsw * 1000)}", orient="v")
        sh.dim(T.m(x_le(b2), b2, 0), T.m(x_hinge(y1e), y1e, 0), lx, f"{_f((x_le(b2) - x_hinge(y1e)) * 1000)}", orient="v")
        sh.dim(T.m(x_hinge(y1e), y1e, 0), T.m(x_te(b2), b2, 0), lx, f"{_f((x_hinge(y1e) - x_te(b2)) * 1000)}", orient="v")
        sh.dim(T.m(x_le(b2), b2, 0), T.m(x_te(b2), b2, 0), lx - 9, f"tip chord {_f(w['tip_chord'] * 1000)}", orient="v")
        # labels with leaders
        sv = P["servo_L"].bounding_box().center()
        sh.leader(T.m(sv.X, sv.Y, 0), (T.m(0, 0.50, 0)[0], T.m(-0.12, 0, 0)[1]),
                  f"SG90 servo pocket {_f(D.SERVO[0]*1000)}×{_f(D.SERVO[2]*1000)}×{_f(D.SERVO[1]*1000)} deep,\ncentre {_f(sv.Y*1000)} out, {_f(-sv.X*1000)} behind root LE (45 % chord)")
        ym = (y0e + y1e) / 2
        sh.leader(T.m(x_hinge(ym) - 0.02, ym, 0), (T.m(0, 0.50, 0)[0], botv - 33),
                  f"elevon: 25 % chord ({_f(0.25*chord(y0e)*1000)} inboard, {_f(0.25*chord(y1e)*1000)} outboard), ±{w['elevon_max_deg']:.0f}°,\ntape hinge on the top skin, bevel the lower edge 45°")
        oy = (st["outer_spar"]["y0"] + st["outer_spar"]["y1"]) / 2
        oxq = -(oy * tsw + 0.25 * chord(oy))
        sh.leader(T.m(oxq, oy, 0), (T.m(0, 0.52, 0)[0] - 2, T.m(-0.03, 0, 0)[1]),
                  f"outer spar Ø{_f(st['outer_spar']['od']*1000)}/{_f(st['outer_spar']['id']*1000)} carbon along the quarter chord")
        sh.leader(T.m(-st["cross_spar"]["x_aft"], 0.12, 0), (T.m(0, 0.36, 0)[0], T.m(0.035, 0, 0)[1]),
                  f"cross spar Ø{_f(st['cross_spar']['od']*1000)}/{_f(st['cross_spar']['id']*1000)} × {_f(st['cross_spar']['span']*1000)} through the pod")
        sh.leader(T.m(-st["cross_spar"]["x_aft"], bm["y"], 0), (T.m(0, 0.12, 0)[0], T.m(0.068, 0, 0)[1]),
                  "printed clamp: boom to cross spar")
        g = S.wing_geometry(s)
        sh.lines([np.array([T.m(g["x_le_mac"], g["y_mac"], 0), T.m(g["x_le_mac"] - g["mac"], g["y_mac"], 0)])], color=CL, lw=0.5)
        sh.text(*T.m(g["x_le_mac"] - g["mac"] / 2, g["y_mac"] + 0.012, 0), f"MAC {_f(g['mac']*1000)}", color=CL, size=5.6)

        # sections (1:2): root and tip, with spar, servo and hinge
        import aerosandbox as asb
        af = asb.Airfoil(w["airfoil"]).coordinates

        def section(x0p, y0p, c, twist, label, marks):
            ks = 2.0
            t = math.radians(twist)
            pts = []
            for xc, zc in af:
                dx, dz = (xc - 0.25) * c, zc * c
                rx_ = dx * math.cos(t) - dz * math.sin(t)
                rz_ = dx * math.sin(t) * -1 + dz * math.cos(t)
                pts.append((x0p + (0.25 * c + rx_) * 1000 / ks, y0p + rz_ * 1000 / ks))
            pts = np.array(pts)
            sh.lines([pts], lw=LW)
            sh.lines([np.array([[x0p - 3, y0p], [x0p + c * 1000 / ks + 3, y0p]])], color=CL, lw=LWD, ls=(0, (6, 2, 1, 2)))
            sh.dim((x0p, y0p), (x0p + c * 1000 / ks, y0p), y0p - 9, f"{label} chord {_f(c*1000)}")
            th = float(asb.Airfoil(w["airfoil"]).max_thickness()) * c
            sh.text(x0p + c * 1000 / ks / 2, y0p + 10, f"{label}: S5020, max thickness {_f(th*1000)}" + (f", {abs(twist):.0f}° washout" if twist else ""),
                    color=INK, size=5.8)
            for (xm, zm, r, txt) in marks:
                pc = (x0p + xm * 1000 / ks, y0p + zm * 1000 / ks)
                sh.ax.add_patch(Circle(pc, r * 1000 / ks, fill=False, lw=0.5, ec=INK, zorder=5))
                sh.leader(pc, (pc[0] + 6, pc[1] - 7), txt)
            return pts

        cs = st["cross_spar"]
        section(232, 200, w["root_chord"], 0.0, "root", [(cs["x_aft"], cs["z"], cs["od"] / 2, f"cross spar at {_f(cs['x_aft']*1000)}")])
        sh.lines([np.array([[232 + 0.75 * w["root_chord"] * 500, 196], [232 + 0.75 * w["root_chord"] * 500, 204]])], color=DIMC, lw=LWD)
        ct = w["tip_chord"]
        section(232, 135, ct, w["tip_twist_deg"], "tip", [])
        sh.text(232 + ct * 250, 122, "hinge line at 75 % chord on both; washout is set with the printed twist jig", size=5.6, color=INK)
        sh.label(310, 230, "SECTIONS 1:2 (chord line dash-dot)")
        sh.save("sheet_2_wing_structure.png")

        # ================= SHEET 3: lift unit, pusher, printed parts ==========================
        sh = Sheet(pdf, "LIFT UNIT, PUSHER AND PRINTED PARTS", 3, 3, "1:2 and 1:1 as marked (A3)")

        def fit(name, vis, hid, box, kk, hidden=False):
            """Centre a projected view in a paper box (x0, y0, x1, y1); warn if it does not fit."""
            allp = np.vstack(vis + (hid or []))
            u0, v0 = allp.min(axis=0)
            u1, v1 = allp.max(axis=0)
            if (u1 - u0) / kk > box[2] - box[0] or (v1 - v0) / kk > box[3] - box[1]:
                print(f"warning: {name} view {(u1 - u0) / kk:.0f}x{(v1 - v0) / kk:.0f} mm overflows box {box}")
            o = ((box[0] + box[2]) / 2 - (u0 + u1) / 2 / kk, (box[1] + box[3]) / 2 - (v0 + v1) / 2 / kk)
            V = View(sh, name, o, kk)
            V.draw(vis, hid, hidden=hidden)
            return V

        k = 2.0
        unit = [P[n] for n in ("boom_L", "mount_FL", "motor_FL", "shaft_FL", "prop_FL", "leg_FL", "esc_FL", "wire_FL")]
        x_aft = P["esc_FL"].bounding_box().min.X - 0.012
        x_fwd = xf + lp["d"] / 2 + 0.01
        cut = bd.Box(x_fwd - x_aft, 0.3, 0.4).translate(((x_fwd + x_aft) / 2, 0.25, 0))
        unit = [u & cut for u in unit]
        unit = [u for u in unit if u is not None and getattr(u, "volume", 1) != 0]
        vis, hid = project(unit, "side")
        Sv = fit("side", vis, None, (50, 175, 200, 262), k)
        sh.label(125, 280, "FRONT-LEFT LIFT UNIT, SIDE VIEW 1:2 (all four identical; rear units face aft)")
        zb = -0.012
        sh.lines([np.array([Sv.m(x_fwd, 0, gz), Sv.m(x_aft, 0, gz)])], color=INK, lw=0.4)
        mount_top = zb + 0.014
        motor_top = mount_top + 0.026
        for zz, txt, off in ((mount_top, f"mount plate top +{_f((mount_top - zb)*1000)}", 0), (motor_top, f"motor top +{_f((motor_top - zb)*1000)}", 9),
                             (-lr["z"], f"prop plane +{_f((-lr['z'] - zb)*1000)}", 18)):
            sh.dim(Sv.m(xf, 0, zb), Sv.m(xf, 0, zz), Sv.m(x_fwd, 0, 0)[0] - 4 - off, txt, orient="v")
        legx = P["leg_FL"].bounding_box().center().X
        sh.dim(Sv.m(legx, 0, zb), Sv.m(legx, 0, gz), Sv.m(x_aft, 0, 0)[0] + 4, f"leg {_f(lg['height']*1000)}", orient="v")
        sh.dim(Sv.m(xf, 0, gz), Sv.m(legx, 0, gz), Sv.m(0, 0, gz)[1] - 6, _f((xf - legx) * 1000))
        ex = P["esc_FL"].bounding_box().center().X
        sh.dim(Sv.m(xf, 0, gz), Sv.m(ex, 0, gz), Sv.m(0, 0, gz)[1] - 13, f"ESC {_f((xf - ex)*1000)}")
        sh.dim(Sv.m(xf + lp["d"] / 2, 0, -lr["z"]), Sv.m(xf - lp["d"] / 2, 0, -lr["z"]), Sv.m(0, 0, -lr["z"])[1] + 12, f"prop Ø{_f(lp['d']*1000)}")
        sh.leader(Sv.m(ex, 0, zb - 0.01), (Sv.m(legx, 0, 0)[0] + 8, Sv.m(0, 0, zb)[1] - 22), "30 A ESC zip-tied under\nthe boom, in the prop wash")
        sh.leader(Sv.m(legx, 0, gz + 0.03), (Sv.m(legx, 0, 0)[0] - 30, Sv.m(0, 0, gz)[1] + 18), f"printed leg, {_f(D.LEG_INBOARD*1000)} inboard of the motor, rubber foot", ha="right")

        # front view of the lift unit: the 4 deg outboard lean
        vis, hid = project(unit, "front")
        Fv = fit("front", vis, None, (230, 175, 395, 262), k)
        sh.label(312, 280, "SAME UNIT, LOOKING AFT 1:2")
        c0 = Fv.m(xf, yb, zb)
        sh.lines([np.array([c0, Fv.m(xf, yb, zb + 0.07)])], color=CL, lw=LWD, ls=(0, (6, 2, 1, 2)))
        sh.angle(c0, 30, 90, 90 + lr.get("yaw_tilt_deg", 4), "", None)
        sh.leader(Fv.m(xf, yb + 0.004, zb + 0.045), (c0[0] + 30, c0[1] + 30), f"{lr.get('yaw_tilt_deg', 4):.0f}° outboard lean\n(mount printed with the tilt)")
        sh.text(c0[0], Fv.m(0, 0, gz)[1] - 8, "left boom: motors lean left; right boom: lean right", size=5.6, color=INK)

        # pusher side view
        pus = [P[n] for n in ("pod", "pusher_mount", "pusher_motor", "pusher_shaft", "prop_pusher", "pusher_esc", "hatch")]
        cutp = bd.Box(0.20, 0.3, 0.4).translate((-0.37, 0, 0))
        pus = [u & cutp for u in pus]
        vis, hid = project(pus, "side")
        Pv = fit("side", vis, None, (35, 40, 140, 150), k)
        sh.label(85, 160, "PUSHER ON THE POD TAIL, SIDE VIEW 1:2")
        pz = -pu["z"]
        pod_top = -0.02 + fu["height"] / 2
        plate_x = fu["nose_x"] - fu["length"] + 0.002
        sh.dim(Pv.m(-0.30, 0, pod_top), Pv.m(-0.30, 0, pz), Pv.m(-0.27, 0, 0)[0], f"{_f((pz - pod_top)*1000)} above pod top", orient="v")
        pdz = P["propdisc_pusher"].bounding_box()
        sh.lines([np.array([Pv.m(pdz.center().X, 0, pz + ppu["d"] / 2), Pv.m(pdz.center().X, 0, pz - ppu["d"] / 2)])], color=CL, lw=LWD, ls=(0, (6, 2, 1, 2)))
        sh.text(Pv.m(pdz.center().X, 0, 0)[0] + 2, Pv.m(0, 0, pz + ppu["d"] / 2)[1] - 3, "prop disc", size=5.2, color=CL, ha="left")
        sh.dim(Pv.m(plate_x, 0, pz - 0.105), Pv.m(pdz.center().X, 0, pz - 0.105), Pv.m(0, 0, pz - 0.105)[1] - 5, f"{_f((plate_x - pdz.center().X)*1000)}")
        sh.dim(Pv.m(-0.31, 0, pz + ppu['d'] / 2), Pv.m(-0.31, 0, pz - ppu['d'] / 2), Pv.m(-0.47, 0, 0)[0] + 4, f"Ø{_f(ppu['d']*1000)}", orient="v")
        sh.leader(Pv.m(plate_x + 0.03, 0, pod_top + 0.004), (40, 44),
                  f"printed pylon: plate aft face {_f(-plate_x*1000)} behind root LE,\n70 × 34 flange glued to the pod top, two webs")

        # printed parts at 1:1: motor mount top, leg, pusher pylon
        pp = D.cad.printed_parts(s)
        mnt = pp["motor_mount"].scale(0.001)
        v_, h_ = project([mnt], "top")
        Mv = fit("top", v_, h_, (170, 80, 235, 135), 1.0, hidden=True)
        mb = mnt.bounding_box()
        sh.label(202, 150, "MOTOR MOUNT 1:1 (top)")
        sh.dim(Mv.m(mb.min.X, mb.max.Y, 0), Mv.m(mb.min.X, mb.min.Y, 0), Mv.m(mb.min.X, 0, 0)[1] - 6, _f((mb.max.Y - mb.min.Y) * 1000))
        sh.dim(Mv.m(mb.min.X, mb.max.Y, 0), Mv.m(mb.max.X, mb.max.Y, 0), Mv.m(0, mb.max.Y, 0)[0] - 6, _f((mb.max.X - mb.min.X) * 1000), orient="v")
        sh.text(202, Mv.m(mb.min.X, 0, 0)[1] - 18, "M3 holes on 16 and 19 mm (A2212 cross mount), Ø9 shaft hole,\nboom saddle for Ø10 tube with clamp slot", size=5.6, color=INK)
        leg = pp["leg"].scale(0.001)
        v_, h_ = project([leg], "side")
        Lv = fit("side", v_, h_, (245, 50, 285, 145), 1.0, hidden=True)
        sh.label(265, 150, "LEG 1:1 (side)")
        lb = leg.bounding_box()
        sh.dim(Lv.m(lb.min.X, 0, lb.max.Z), Lv.m(lb.min.X, 0, lb.min.Z), Lv.m(lb.min.X, 0, 0)[0] + 6, _f((lb.max.Z - lb.min.Z) * 1000), orient="v")
        pyl = D.pusher_pylon(s).scale(0.001)
        v_, h_ = project([pyl], "side")
        Yv = fit("side", v_, h_, (305, 60, 395, 140), 1.0, hidden=True)
        sh.label(350, 150, "PUSHER PYLON 1:1 (side)")
        yb_ = pyl.bounding_box()
        sh.dim(Yv.m(yb_.max.X, 0, yb_.min.Z), Yv.m(yb_.min.X, 0, yb_.min.Z), Yv.m(0, 0, yb_.min.Z)[1] - 6, _f((yb_.max.X - yb_.min.X) * 1000))
        sh.dim(Yv.m(yb_.min.X, 0, yb_.max.Z), Yv.m(yb_.min.X, 0, yb_.min.Z), Yv.m(yb_.min.X, 0, 0)[0] + 6, _f((yb_.max.Z - yb_.min.Z) * 1000), orient="v")
        sh.save("sheet_3_details.png")
    return OUTD / "kestrel_drawings.pdf"


def main():
    p = build_all()
    print("wrote", p)


if __name__ == "__main__":
    main()
