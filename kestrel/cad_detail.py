"""Detailed build assembly: every part a builder handles, for the 3D viewer and the dimensioned drawing.

Run: python -m kestrel.cad_detail      (writes cad/detail/checks.json; used by export_viewer and drawing)

`cad.build()` stays the model for mass properties and clearances (simple shapes, analysed in validate.py).
This module adds what a builder needs to see: elevons cut at the hinge line, servos with horns and pushrods,
real printed motor mounts, legs and pusher pylon (the same solids as the STLs in cad/out), stepped motors,
two-blade props, ESCs, boom-to-spar clamps, wire runs, a hollow pod with hatch, GPS mast and receiver
antennas. Frame: FLU metres, origin at the wing root leading edge (ADR 0001).
"""
from __future__ import annotations

import json
import math

import aerosandbox as asb
import build123d as bd

from . import cad
from . import spec as S

DETAIL = S.ROOT / "cad" / "detail"

SERVO = (0.023, 0.0122, 0.0225)     # SG90 body: length, width, height (m)
LEG_INBOARD = 0.028                  # legs sit 28 mm along the boom toward the CG, clear of the mount saddle
MOTOR_BELL_R = 0.01375               # A2212: 27.5 mm can, 26 mm long incl. base
MOTOR_BASE_R = 0.0140
ESC = (0.045, 0.025, 0.008)


def wing_dims(s):
    w = s["wing"]
    b2 = w["span"] / 2
    tsw = math.tan(math.radians(w["le_sweep_deg"]))

    def chord(y):
        return w["root_chord"] + (w["tip_chord"] - w["root_chord"]) * abs(y) / b2

    def x_le(y):
        return -abs(y) * tsw

    def x_te(y):
        return x_le(y) - chord(y)

    def x_hinge(y):
        return x_te(y) + w["elevon_chord_frac"] * chord(y)

    return b2, chord, x_le, x_te, x_hinge


def smooth_wing_half(s, left=True):
    """Same planform, airfoil and washout as cad.wing_half, but each section is one spline instead of a
    61-point polyline, so the skin is two smooth faces (clean drawings and shading). OCC's analytic volume is
    unreliable on this B-spline loft; use vol() below, which integrates the tessellation."""
    w = s["wing"]
    b2 = w["span"] / 2
    tsw = math.tan(math.radians(w["le_sweep_deg"]))
    xy = cad.mh45_xy()

    def sec(x_le_, y, c, tw):
        t = math.radians(tw)
        pts = []
        for xc, zc in xy[:-1]:
            dx, dz = -(xc - 0.25) * c, zc * c
            rx = dx * math.cos(t) + dz * math.sin(t)
            rz = -dx * math.sin(t) + dz * math.cos(t)
            pts.append(bd.Vector(x_le_ - 0.25 * c + rx, y, rz))
        return bd.Wire([bd.Spline(*pts), bd.Line(pts[-1], pts[0])])

    sol = bd.Solid.make_loft([sec(0.0, 0.0, w["root_chord"], 0.0), sec(-b2 * tsw, b2, w["tip_chord"], w["tip_twist_deg"])], ruled=True)
    return sol if left else sol.mirror(bd.Plane.XZ)


def vol(shape, tol=0.0002):
    """Volume from the tessellation (robust on B-spline lofts where Shape.volume is not)."""
    import numpy as np
    try:
        if shape is None or shape._wrapped is None:
            return 0.0
        V, T = shape.tessellate(tol)
    except Exception:
        return 0.0
    if not T:
        return 0.0
    V = np.array([[v.X, v.Y, v.Z] for v in V])
    T = np.array(T)
    return float(abs(np.einsum("ij,ij->i", V[T[:, 0]], np.cross(V[T[:, 1]], V[T[:, 2]])).sum() / 6))


def _ymirror(shape, sy):
    return shape if sy > 0 else shape.mirror(bd.Plane.XZ)


def _prism_xy(pts, z0=-0.2, z1=0.2):
    face = bd.Face(bd.Wire(bd.Polyline(*pts, close=True).edges()))
    return bd.extrude(face.translate((0, 0, z0)), amount=z1 - z0)


def elevon_segments(s):
    """Elevon spans (left side, y from centreline): split around the boom (spec wing.elevon_gap, ADR 0006)."""
    w = s["wing"]
    b2 = w["span"] / 2
    y0, y1 = (f * b2 for f in w["elevon_span_frac"])
    g = w.get("elevon_gap")
    if not g:
        return [(y0, y1)]
    return [(y0, g["center"] - g["width"] / 2), (g["center"] + g["width"] / 2, y1)]


def elevon_prism(s, ya, yb, hinge_gap=0.0, end_gap=0.0):
    b2, chord, x_le, x_te, x_hinge = wing_dims(s)
    ya, yb = ya + end_gap, yb - end_gap
    return _prism_xy([(x_hinge(ya) - hinge_gap, ya), (x_hinge(yb) - hinge_gap, yb),
                      (x_te(yb) - 0.05, yb), (x_te(ya) - 0.05, ya)])


def _ellipse_section(r, c, t, beta):
    """Blade section at radius r (blade along +x): flat ellipse, chord c, thickness t, pitch angle beta (rad)."""
    pl = bd.Plane(origin=(r, 0, 0), x_dir=(0, math.cos(beta), math.sin(beta)), z_dir=(1, 0, 0))
    return pl * bd.Wire(bd.Ellipse(c / 2, t / 2).edges())


def prop(radius, pitch, hub_r=0.007, clock_deg=0.0, ccw=False):
    """Two-blade prop, axis +z, lofted through elliptic sections with the real helical twist
    beta(r) = atan(pitch / (2 pi r)) and a slow-fly chord distribution (max ~0.16 R at 40 % R)."""
    stations = [(0.17, 0.11, 0.11), (0.30, 0.15, 0.10), (0.45, 0.165, 0.085), (0.65, 0.15, 0.07),
                (0.85, 0.115, 0.06), (0.99, 0.055, 0.06)]     # (r/R, chord/R, t/c)
    secs = []
    for rr, cc, tc in stations:
        r = rr * radius
        c = cc * radius
        beta = math.atan(pitch / (2 * math.pi * r))
        secs.append(_ellipse_section(r, c, max(c * tc, 0.0008), beta))
    blade = bd.Solid.make_loft(secs, ruled=False)
    blades = [blade, blade.rotate(bd.Axis.Z, 180)]
    hub = bd.Cylinder(hub_r, 0.009)
    hub = bd.fillet(hub.edges(), 0.0015)
    out = bd.Compound(blades + [hub])
    if ccw:
        out = out.mirror(bd.Plane.YZ)
    return out.rotate(bd.Axis.Z, clock_deg)


def lift_motor(base_z):
    """A2212 outrunner standing on +z from base_z: cross-mount base, stator gap, bell with three vented
    windows and a chamfered top, 3.17 mm shaft, collet prop adapter with a bullet nut."""
    A = (bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)
    base = bd.Cylinder(MOTOR_BASE_R, 0.003, align=A)
    for a in (0, 90):
        arm = bd.Box(0.032, 0.006, 0.0025, align=A).rotate(bd.Axis.Z, a + 45)
        base = base + arm
    stator_ring = bd.Cylinder(0.0115, 0.002, align=A).translate((0, 0, 0.003))
    bell = bd.Cylinder(MOTOR_BELL_R, 0.021, align=A).translate((0, 0, 0.005))
    try:
        bell = bd.chamfer(bell.edges().group_by(bd.Axis.Z)[-1], 0.0012)
    except Exception:
        pass
    bell -= bd.Cylinder(MOTOR_BELL_R - 0.0012, 0.019, align=A).translate((0, 0, 0.005))
    for a in (0, 120, 240):
        bell -= bd.Cylinder(0.0035, 0.004, align=A).translate((0.0072, 0, 0.0235)).rotate(bd.Axis.Z, a)
    hubboss = bd.Cylinder(0.0045, 0.0025, align=A).translate((0, 0, 0.026))
    motor = bd.Compound([base, stator_ring, bell, hubboss]).translate((0, 0, base_z))
    shaft = bd.Cylinder(0.0016, 0.014, align=A).translate((0, 0, 0.022))
    collet = bd.Cylinder(0.004, 0.006, align=A).translate((0, 0, 0.0285))
    nut = bd.Cone(0.0045, 0.0018, 0.006, align=A).translate((0, 0, 0.0375))
    return {"motor": motor, "shaft": bd.Compound([shaft, collet, nut]).translate((0, 0, base_z))}


def sg90(arm_angle_deg=0.0):
    """SG90 9 g servo, output shaft along +y (servo lying on its side in the wing), body centred at origin.
    Body 23 (x) x 22.5 (y, shaft axis) x 12.2 (z); tabs 32.2 long; output boss and arm on the +y face."""
    body = bd.Box(0.023, 0.0225, 0.0122)
    try:
        body = bd.fillet(body.edges().filter_by(bd.Axis.Y), 0.001)
    except Exception:
        pass
    tabs = bd.Box(0.0322, 0.0025, 0.0122).translate((0, 0.0225 / 2 - 0.0066, 0))
    for sx in (-1, 1):
        tabs -= bd.Cylinder(0.001, 0.004, rotation=(90, 0, 0)).translate((sx * 0.0139, 0.0225 / 2 - 0.0066, 0))
    boss = bd.Cylinder(0.0059, 0.004, rotation=(90, 0, 0)).translate((0.0055, 0.0225 / 2 + 0.002, 0))
    spline = bd.Cylinder(0.0024, 0.003, rotation=(90, 0, 0)).translate((0.0055, 0.0225 / 2 + 0.0055, 0))
    arm = bd.Box(0.0045, 0.0016, 0.019).translate((0, 0, 0.0075))
    arm = arm.rotate(bd.Axis.Y, arm_angle_deg).translate((0.0055, 0.0225 / 2 + 0.0072, 0))
    return bd.Compound([body, tabs, boss, spline, arm])


def faired_pod(s):
    """Pod lofted through rounded-rectangle stations: blunt rounded nose, parallel battery bay, boat-tail.
    The top stays flat at z = top (for the hatch and the pusher pylon); the boat-tail tapers from below."""
    fu = s["fuselage"]
    L, W, H = fu["length"], fu["width"], fu["height"]
    top = -0.02 + H / 2
    st = [(0.0, 0.42, 0.45, 0.010), (0.05, 0.78, 0.80, 0.004), (0.14, 0.97, 0.98, 0.0), (0.30, 1.0, 1.0, 0.0),
          (0.72, 1.0, 1.0, 0.0), (1.0, 0.72, 0.55, 0.0)]          # (t, width factor, height factor, top drop)

    def sec(t, kw, kh, drop, inset=0.0):
        w_, h_ = W * kw - 2 * inset, H * kh - 2 * inset
        r = min(w_, h_) * 0.32
        x = fu["nose_x"] - L * t
        zc = top - drop - inset - (H * kh - 2 * inset) / 2 - inset * 0
        pl = bd.Plane(origin=(x, 0, zc), x_dir=(0, 1, 0), z_dir=(-1, 0, 0))
        return pl * bd.Wire(bd.RectangleRounded(w_, h_, r).edges()), x

    outer = bd.Solid.make_loft([sec(*a)[0] for a in st], ruled=False)
    inner_st = [(0.03, 0.66, 0.68, 0.007), (0.10, 0.92, 0.94, 0.001)] + st[2:5] + [(0.985, 0.74, 0.58, 0.0)]
    inner = bd.Solid.make_loft([sec(*a, inset=0.002)[0] for a in inner_st], ruled=False)
    return outer - inner, top


def build_detail(s=None):
    s = s or S.load()
    simple, rotors = cad.build(s)
    pp = cad.printed_parts(s)                      # mm, printed-part frames
    w, bm, lr, lg = s["wing"], s["booms"], s["lift_rotors"], s["landing_legs"]
    st, pu, fu = s["structure"], s["pusher"], s["fuselage"]
    b2, chord, x_le, x_te, x_hinge = wing_dims(s)
    tilt = lr.get("yaw_tilt_deg", 0.0)
    z_boom = -0.012
    z_prop = -lr["z"]
    lp = s["props"][s["propulsion"]["lift_prop"]]
    ppu = s["props"][s["propulsion"]["pusher_prop"]]
    P = {}

    # --- wing: fixed panels, elevons, winglets -------------------------------------------------
    # elevons split around the boom (ADR 0006); 0.8 mm hinge slot and 0.75 mm end gaps so nothing rubs.
    # A 2 mm carbon torque rod on the hinge line joins the two segments, so one servo drives both.
    segs = elevon_segments(s)
    for side, sy in (("L", 1), ("R", -1)):
        wing = smooth_wing_half(s, sy > 0)
        fixed_cut = None
        evs = []
        for (ya, yb) in segs:
            full = _ymirror(elevon_prism(s, ya, yb), sy)
            fixed_cut = full if fixed_cut is None else fixed_cut + full
            evs.append(wing & _ymirror(elevon_prism(s, ya, yb, hinge_gap=0.0008, end_gap=0.00075), sy))
        P[f"elevon_{side}"] = bd.Compound(evs)
        P[f"wing_{side}"] = wing - fixed_cut
        wl = simple[f"winglet_{side}"]
        try:   # rounded leading and top edges on the 5 mm foam winglet
            wl = bd.fillet(wl.edges().filter_by(bd.Axis.Y, reverse=True).filter_by(lambda e: e.length > 0.05), 0.002)
        except Exception:
            pass
        P[f"winglet_{side}"] = wl

    # --- servos, horns, pushrods ---------------------------------------------------------------
    # SG90 on its side in a pocket at 45 % chord (where the S5020 is ~22 mm thick), 200 mm out; its arm comes
    # through the top skin and a pushrod runs aft to a horn on the elevon, 8 mm behind the hinge line.
    af = asb.Airfoil(w["airfoil"])
    ys = 0.40 * b2      # 200 mm out: the arm and horn land 15 mm inboard of the boom gap

    def surf(y, frac, top=True):
        c = chord(y)
        t = math.radians(w["tip_twist_deg"] * abs(y) / b2)
        zc = float(af.local_camber(frac)) + (0.5 if top else -0.5) * float(af.local_thickness(frac))
        return (frac - 0.25) * c * math.sin(t) + zc * c * math.cos(t)

    for side, sy in (("L", 1), ("R", -1)):
        c = chord(ys)
        xs = x_le(ys) - 0.45 * c
        zmid = 0.5 * (surf(ys, 0.45) + surf(ys, 0.45, False))
        sv = sg90()
        if sy < 0:
            sv = sv.mirror(bd.Plane.XZ)
        P[f"servo_{side}"] = sv.translate((xs, sy * ys, zmid))
        # arm tip: shaft at (xs + 5.5 mm, zmid), arm 15 mm up -> tip 13 mm above the shaft
        y_arm = sy * (ys + 0.0225 / 2 + 0.0072)
        tip = bd.Vector(xs + 0.0055, y_arm, zmid + 0.0145)
        xh = x_hinge(abs(y_arm))           # hinge line at the horn's own span station (it is swept)
        fr_h = (x_le(abs(y_arm)) - (xh - 0.010)) / chord(abs(y_arm))
        zt = surf(abs(y_arm), fr_h)
        horn = bd.Box(0.014, 0.0015, 0.013).translate((xh - 0.010, y_arm, zt + 0.0045))
        horn -= bd.Cylinder(0.0006, 0.004, rotation=(90, 0, 0)).translate((xh - 0.006, y_arm, zt + 0.0085))
        P[f"horn_{side}"] = horn
        hole = bd.Vector(xh - 0.006, y_arm, zt + 0.0085)
        a, bvec = tip + (hole - tip).normalized() * 0.002, hole - (hole - tip).normalized() * 0.002
        P[f"pushrod_{side}"] = bd.Plane(origin=a, z_dir=(bvec - a).normalized()) * bd.Cylinder(
            0.0006, (bvec - a).length, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        # torque rod joining the two elevon segments across the boom gap, on the hinge line under the top skin
        if len(segs) > 1:
            ya, yb = segs[0][1] - 0.010, segs[1][0] + 0.010
            za = surf(ya, 0.75) - 0.0016
            zb = surf(yb, 0.75) - 0.0016
            pa = bd.Vector(x_hinge(ya) - 0.0018, sy * ya, za)
            pb = bd.Vector(x_hinge(yb) - 0.0018, sy * yb, zb)
            P[f"torque_rod_{side}"] = bd.Plane(origin=pa, z_dir=(pb - pa).normalized()) * bd.Cylinder(
                0.001, (pb - pa).length, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

    # --- pod: faired, hollow (2 mm wall), removable hatch cut from its top --------------------
    L, W, H = fu["length"], fu["width"], fu["height"]
    pod, pod_top = faired_pod(s)
    hatch_box = bd.Box(0.26, W + 0.01, 0.006).translate((fu["nose_x"] - 0.03 - 0.13, 0, pod_top - 0.002))
    P["hatch"] = pod & hatch_box
    P["pod"] = pod - hatch_box
    bat = simple["battery"]
    try:
        bat = bd.fillet(bat.edges(), 0.002)
    except Exception:
        pass
    bb = bat.bounding_box()
    P["battery"] = bat
    strap = bd.Box(0.020, bb.size.Y + 0.003, bb.size.Z + 0.003) - bd.Box(0.022, bb.size.Y, bb.size.Z)
    P["strap"] = strap.translate((bb.center().X, 0, bb.center().Z))
    P["xt60"] = bd.Box(0.016, 0.008, 0.016).translate((bb.min.X - 0.012, 0, bb.center().Z))

    # --- avionics -----------------------------------------------------------------------------
    P["flight_controller"] = simple["flight_controller"]
    P["receiver"] = simple["receiver"]
    rb = simple["receiver"].bounding_box()
    for k, ang in (("ant_A", 35), ("ant_B", -35)):          # 2.4 GHz antennas splayed below the pod
        a = bd.Vector(rb.center().X, 0, rb.min.Z)            # out through the pod floor, clear of the wing
        d = bd.Vector(0, math.sin(math.radians(ang)), -math.cos(math.radians(ang)))
        P[k] = bd.Plane(origin=a, z_dir=d) * bd.Cylinder(0.0008, 0.055, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    gb = simple["gps"].bounding_box()
    pod_top = -0.02 + H / 2
    gps = bd.Box(0.025, 0.025, 0.008)
    try:
        gps = bd.fillet(gps.edges().filter_by(bd.Axis.Z), 0.003)
    except Exception:
        pass
    P["gps"] = gps.translate((gb.center().X, 0, pod_top + 0.030))
    P["gps_mast"] = bd.Cylinder(0.003, 0.026, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).translate((gb.center().X, 0, pod_top))
    P["pusher_esc"] = simple["pusher_esc"]
    try:
        P["flight_controller"] = bd.fillet(simple["flight_controller"].edges().filter_by(bd.Axis.Z), 0.002)
    except Exception:
        pass

    # --- booms, spars, clamps -------------------------------------------------------------------
    for side, sy in (("L", 1), ("R", -1)):
        P[f"boom_{side}"] = simple[f"boom_{side}"]
        P[f"outer_spar_{side}"] = simple[f"outer_spar_{side}"]
        # printed clamp joining the cross-spar end (z +6 mm) to the boom (z -12 mm) at x = -x_aft
        cs = st["cross_spar"]
        blk = bd.Box(0.022, 0.016, (cs["z"] - z_boom) + 0.016).translate((-cs["x_aft"], sy * bm["y"], (cs["z"] + z_boom) / 2))
        blk -= bd.Cylinder(bm["diameter"] / 2 + 0.0002, 0.03, rotation=(0, 90, 0)).translate((-cs["x_aft"], sy * bm["y"], z_boom))
        blk -= bd.Cylinder(cs["od"] / 2 + 0.0002, 0.03, rotation=(90, 0, 0)).translate((-cs["x_aft"], sy * bm["y"], cs["z"]))
        try:
            blk = bd.fillet(blk.edges().filter_by(bd.Axis.Z), 0.002)
        except Exception:
            pass
        P[f"clamp_{side}"] = blk
    P["cross_spar"] = simple["cross_spar"]

    # --- lift units: printed mount, motor, prop, leg, ESC, wire run --------------------------
    sad_z = 0.0095            # printed mount: boom axis is 9.5 mm below the mount origin (cad.printed_parts)
    plate_top = sad_z + 0.0045
    clocks = {"FR": 20, "RL": 65, "FL": 110, "RR": 155}
    for k, (x, y) in rotors.items():
        sy = 1 if y > 0 else -1
        m = pp["motor_mount"].scale(0.001)            # tilted +tilt about X (leans toward -y): right side
        m = _ymirror(m, -sy)                          # left side: mirrored, leans toward +y (outboard)
        P[f"mount_{k}"] = m.translate((x, y, z_boom + sad_z))
        ax = bd.Axis((x, y, z_boom), (1, 0, 0))
        ang = tilt if y < 0 else -tilt
        parts = lift_motor(z_boom + plate_top)
        P[f"motor_{k}"] = parts["motor"].translate((x, y, 0)).rotate(ax, ang)
        P[f"shaft_{k}"] = parts["shaft"].translate((x, y, 0)).rotate(ax, ang)
        pr_ = prop(lp["d"] / 2, 4.5 * 0.0254, clock_deg=clocks[k], ccw=k in ("FR", "RL"))   # ADR 0001 rotation
        P[f"prop_{k}"] = pr_.translate((x, y, z_prop)).rotate(ax, ang)
        P[f"propdisc_{k}"] = simple[f"propdisc_{k}"]
        xl = x - LEG_INBOARD if x > -0.167 else x + LEG_INBOARD
        leg = pp["leg"].scale(0.001).translate((xl, y, z_boom))
        foot = bd.Cylinder(0.008, 0.005, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX)).translate(
            (xl, y, z_boom - (lg["height"] - bm["diameter"] / 2)))
        P[f"leg_{k}"] = bd.Compound([leg, foot])
        xe = x - 0.105 if x > -0.167 else x + 0.105    # ESC zip-tied under the boom, in the prop wash
        esc = bd.Box(*ESC)
        try:
            esc = bd.fillet(esc.edges().filter_by(bd.Axis.X), 0.0025)
        except Exception:
            pass
        P[f"esc_{k}"] = esc.translate((xe, y, z_boom - bm["diameter"] / 2 - ESC[2] / 2))
        zw = z_boom + bm["diameter"] / 2 + 0.0021          # leads ride in the leg's 4.6 mm channel
        a = bd.Vector(xe, y, zw)
        bvec = bd.Vector(x - math.copysign(0.0175, x + 0.167), y, zw)   # stop at the mount saddle, then up to the motor
        P[f"wire_{k}"] = bd.Plane(origin=a, z_dir=(bvec - a).normalized()) * bd.Cylinder(
            0.0018, (bvec - a).length, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    # power runs from the pod to each boom, inside the wing beside the cross spar
    cs = st["cross_spar"]
    for side, sy in (("L", 1), ("R", -1)):
        a = bd.Vector(-cs["x_aft"] + 0.012, sy * (W / 2), cs["z"])
        bvec = bd.Vector(-cs["x_aft"] + 0.012, sy * (bm["y"] - 0.01), cs["z"])
        P[f"harness_{side}"] = bd.Plane(origin=a, z_dir=(bvec - a).normalized()) * bd.Cylinder(
            0.0025, (bvec - a).length, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

    # --- pusher: printed pylon on the pod tail, motor, prop -------------------------------------
    zpu = -pu["z"]
    plate_x = fu["nose_x"] - L + 0.002              # aft face of the pylon plate, 2 mm ahead of the pod tail
    pyl = pusher_pylon(s).scale(0.001)
    P["pusher_mount"] = pyl.translate((plate_x, 0, zpu))
    pm = lift_motor(0.0)
    rot = lambda shp: shp.rotate(bd.Axis.Y, -90).translate((plate_x, 0, zpu))   # motor axis +z -> -x (aft)
    P["pusher_motor"] = rot(pm["motor"])
    P["pusher_shaft"] = rot(pm["shaft"])
    pr = prop(ppu["d"] / 2, 6.0 * 0.0254, clock_deg=90)
    P["prop_pusher"] = pr.rotate(bd.Axis.Y, -90).translate((plate_x - 0.0335, 0, zpu))
    P["propdisc_pusher"] = bd.Cylinder(ppu["d"] / 2, 0.012, rotation=(0, 90, 0)).translate((plate_x - 0.0335, 0, zpu))
    return P


def pusher_pylon(s):
    """Printed pusher pylon (mm): motor plate on the pod tail, flange glued to the pod top, two webs.

    Frame: origin on the motor axis at the plate's aft face, x forward (into the pod), z up.
    The pod top is 30 mm below the motor axis (spec pusher.z vs fuselage height).
    """
    fu, pu = s["fuselage"], s["pusher"]
    drop = (-pu["z"] - (-0.02 + fu["height"] / 2)) * 1000      # motor axis above pod top, mm
    plate = bd.Box(4, 40, 40).translate((2, 0, 0))
    for (y, z) in [(8, 0), (-8, 0), (0, 9.5), (0, -9.5)]:
        plate -= bd.Cylinder(1.6, 8, rotation=(0, 90, 0)).translate((2, y, z))
    plate -= bd.Cylinder(4.5, 8, rotation=(0, 90, 0)).translate((2, 0, 0))
    flange = bd.Box(70, 34, 3).translate((35, 0, -drop + 1.5))
    webs = []
    for y in (-13, 13):
        f = bd.Face(bd.Wire(bd.Polyline((0, -drop + 3), (0, 14), (60, -drop + 3), close=True).edges()))
        web = bd.extrude(f, amount=4).rotate(bd.Axis.X, 90).translate((0, y + 2, 0))
        webs.append(web)
    return plate + flange + webs[0] + webs[1]


def detail_checks(s, P):
    """Fit checks for the added parts (fractions inside the wing; collisions between solid parts)."""
    res = {}
    for side in ("L", "R"):
        wing = P[f"wing_{side}"] + P[f"elevon_{side}"]
        sv = P[f"servo_{side}"]
        res[f"servo_inside_wing_{side}"] = float((sv & wing).volume / sv.volume)
    # props must not hit anything solid
    solid = ["wing_L", "wing_R", "elevon_L", "elevon_R", "pod", "boom_L", "boom_R", "winglet_L", "winglet_R",
             "clamp_L", "clamp_R", "pusher_mount", "gps", "ant_A", "ant_B"]
    hits = []
    for k in list(P):
        if k.startswith("prop_"):
            for o in solid:
                if (P[k] & P[o]).volume > 1e-12:
                    hits.append((k, o))
    res["prop_hits"] = hits
    gz = min(P[f"leg_{k}"].bounding_box().min.Z for k in ("FR", "RL", "FL", "RR"))
    res["ground_z_m"] = gz
    res["pusher_prop_ground_clearance_m"] = P["propdisc_pusher"].bounding_box().min.Z - gz
    res["pusher_prop_to_wing_m"] = P["propdisc_pusher"].distance_to(P["wing_L"] + P["wing_R"] + P["elevon_L"] + P["elevon_R"])
    return res


def main():
    s = S.load()
    P = build_detail(s)
    DETAIL.mkdir(parents=True, exist_ok=True)
    chk = detail_checks(s, P)
    (DETAIL / "checks.json").write_text(json.dumps(chk, indent=1, default=str))
    print(json.dumps(chk, indent=1, default=str))
    print(len(P), "parts")


if __name__ == "__main__":
    main()
