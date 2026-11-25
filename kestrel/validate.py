"""Independent validation of the Kestrel design (cross-checks that do not reuse the design tools' own models).

Verify: python -m kestrel.validate      -> build/validation.json, exit code 1 if any check fails

Each check compares a design number against a second, independent method with a stated tolerance:
  V1  aerodynamics: AeroBuildup (design tool) vs vortex-lattice (inviscid, separate solver) vs DATCOM/textbook
  V2  hand calculations: stall speed, hover power (momentum theory), cruise power, endurance
  V3  structures: cross spar, outer spars, booms (stress, deflection, first bending mode, torsion under motor torque)
  V4  propulsion limits: prop tip Mach, ESC / battery current margins, hover T/W
  V5  6-DOF numerics: time-step convergence of a hover and a glide response
  V6  control authority: elevon roll rate, pitch authority at stall, quad yaw authority with motor tilt
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import aerosandbox as asb
import numpy as np

from . import aero as A
from . import propulsion as P
from . import spec as S
from . import trim

ROOT = Path(__file__).resolve().parents[1]
G = 9.80665


def _check(name, value, ref, tol_rel=None, lo=None, hi=None, unit="", note=""):
    if tol_rel is not None:
        ok = abs(value - ref) <= tol_rel * abs(ref)
        crit = f"within {tol_rel * 100:.0f} % of {ref:.4g}{unit}"
    else:
        ok = (lo is None or value >= lo) and (hi is None or value <= hi)
        crit = " and ".join(x for x in [f">= {lo:.4g}{unit}" if lo is not None else "",
                                          f"<= {hi:.4g}{unit}" if hi is not None else ""] if x)
    return {"check": name, "value": float(value), "criterion": crit, "pass": bool(ok), "unit": unit, "note": note}


# ---------------- V1 aerodynamics ----------------

def v1_aero(s, t):
    out = []
    ref = t["ref"]
    cg = A._cg_asb(s)
    ap = A.build_airplane(s)
    wing_only = asb.Airplane(name="wing", wings=[ap.wings[0]])

    def vlm(alpha, plane):
        r = asb.VortexLatticeMethod(airplane=plane, op_point=asb.OperatingPoint(velocity=14.0, alpha=alpha),
                                    xyz_ref=cg, spanwise_resolution=12, chordwise_resolution=8).run()
        return float(np.ravel(r["CL"])[0]), float(np.ravel(r["Cm"])[0])

    def bu(alpha, plane):
        r = asb.AeroBuildup(airplane=plane, op_point=asb.OperatingPoint(velocity=14.0, alpha=alpha), xyz_ref=cg).run()
        return float(np.ravel(r["CL"])[0]), float(np.ravel(r["Cm"])[0])

    a0, a1 = 1.0, 5.0
    (cl0v, cm0v), (cl1v, cm1v) = vlm(a0, wing_only), vlm(a1, wing_only)
    (cl0b, cm0b), (cl1b, cm1b) = bu(a0, wing_only), bu(a1, wing_only)
    da = math.radians(a1 - a0)
    cla_v, cla_b = (cl1v - cl0v) / da, (cl1b - cl0b) / da
    xnp_v = cg[0] - (cm1v - cm0v) / (cl1v - cl0v) * ref["mac"]
    xnp_b = cg[0] - (cm1b - cm0b) / (cl1b - cl0b) * ref["mac"]
    # DATCOM / Helmbold-Diederich lift-curve slope for the trapezoidal wing (no winglets)
    w = s["wing"]
    Sref, b = ref["area"], w["span"]
    AR = b * b / Sref
    lam = w["tip_chord"] / w["root_chord"]
    tan_le = math.tan(math.radians(w["le_sweep_deg"]))
    tan_half = tan_le - 2 * (1 - lam) / (AR * (1 + lam))   # tan(L_c/2) = tan(L_le) - (4/AR)(1/2)(1-l)/(1+l)
    eta = 0.95
    cla_datcom = 2 * math.pi * AR / (2 + math.sqrt(AR ** 2 / eta ** 2 * (1 + tan_half ** 2) + 4))
    # analytic aerodynamic centre: quarter chord of the mean aerodynamic chord
    mac = 2 / 3 * w["root_chord"] * (1 + lam + lam ** 2) / (1 + lam)
    y_mac = b / 6 * (1 + 2 * lam) / (1 + lam)
    x_ac = y_mac * tan_le + 0.25 * mac
    out += [
        _check("V1a wing CL_alpha: AeroBuildup vs vortex lattice", cla_b, cla_v, tol_rel=0.15, unit="/rad"),
        _check("V1b wing CL_alpha: vortex lattice vs DATCOM", cla_v, cla_datcom, tol_rel=0.12, unit="/rad"),
        _check("V1c wing neutral point: vortex lattice vs MAC quarter chord", xnp_v, x_ac, tol_rel=0.05, unit=" m aft of root LE"),
        _check("V1d wing neutral point: AeroBuildup vs vortex lattice", xnp_b, xnp_v, tol_rel=0.05, unit=" m aft of root LE"),
    ]
    # static margin of the wing alone from the vortex-lattice neutral point. The design value includes the pod,
    # which is destabilising, so the wing-alone margin must be the larger of the two.
    sm_vlm = (xnp_v - cg[0]) / ref["mac"]
    out.append(_check("V1e wing-alone static margin (vortex lattice) >= design margin incl. pod", sm_vlm, None, lo=t["static_margin"],
                      note=f"design {t['static_margin'] * 100:.1f} % incl. pod"))
    # a tailless wing trims at positive lift only if Cm about its neutral point is positive (reflex + washout).
    # Moving the reference aft by dx adds CL * dx / c (nose up).
    cm_np = cm0b + cl0b * (xnp_b - cg[0]) / ref["mac"]
    out.append(_check("V1f wing Cm about its neutral point > 0 (reflex + washout let a tailless wing trim)", cm_np, None, lo=0.0))
    return out, {"cla_vlm": cla_v, "cla_buildup": cla_b, "cla_datcom": cla_datcom, "xnp_vlm": xnp_v, "xnp_buildup": xnp_b,
                 "x_ac_analytic": x_ac, "sm_vlm_no_pod": sm_vlm, "AR": AR}


# ---------------- V2 hand calculations ----------------

def v2_hand(s, t, sz, mp):
    out = []
    m = mp["mass"]
    W = m * G
    rho = s["environment"]["rho"]
    Sref = t["ref"]["area"]
    v_stall = math.sqrt(2 * W / (rho * Sref * t["clmax"]))
    out.append(_check("V2a stall speed sqrt(2W/(rho S CLmax)) vs sizing", v_stall, sz["stall_mps"], tol_rel=0.02, unit=" m/s"))
    # momentum theory hover power with a typical small-prop figure of merit and motor+ESC efficiency
    lp = P.get("prop", s["propulsion"]["lift_prop"], s)
    A_disc = math.pi * (lp.d / 2) ** 2
    T = W / 4
    p_ideal = T ** 1.5 / math.sqrt(2 * rho * A_disc)
    fm, eta_me = 0.60, 0.75
    p_hover = 4 * p_ideal / fm / eta_me
    out.append(_check("V2b hover power: momentum theory (FM 0.6, motor+ESC 75 %) vs propulsion model", sz["lift"]["hover_power_w"],
                      p_hover, tol_rel=0.25, unit=" W", note="model implies FM x eta = %.2f" % (4 * p_ideal / sz["lift"]["hover_power_w"])))
    # cruise: P = D V / (eta_prop eta_motor) with D from L/D
    V = sz["cruise"]["v_mps"]
    D = W / sz["cruise"]["ld"]
    p_cruise = D * V / (0.60 * 0.75)
    out.append(_check("V2c cruise power D V / (eta_prop 0.6 x eta_motor 0.75) vs propulsion model", sz["cruise"]["power_w"], p_cruise,
                      tol_rel=0.25, unit=" W"))
    # endurance: usable energy / mean power for the standard flight (2 min hover + rest cruise)
    batt = P.get("battery", s["propulsion"]["battery"], s)
    e_use = batt.energy_wh * batt.usable_frac
    t_end = (e_use - sz["lift"]["hover_power_w"] * 2 / 60) / sz["cruise"]["power_w"] * 60 + 2
    out.append(_check("V2d endurance: energy / power hand calc vs sizing", sz["endurance_min"], t_end, tol_rel=0.10, unit=" min"))
    # SITL cross-check: endurance runs fly until 30 % is left; scale to empty and compare with the model
    try:
        e = json.loads((ROOT / "sitl/runs/endurance/endurance_nominal.json").read_text())
        frac_used = e["mah_used"] / (batt.capacity_ah * 1000)
        t_empty = e["sim_time_s"] / 60 / frac_used * batt.usable_frac
        out.append(_check("V2e endurance: SITL flight (scaled to the usable capacity) vs sizing model", t_empty, sz["endurance_min"],
                          tol_rel=0.15, unit=" min", note=f"SITL flew {e['sim_time_s'] / 60:.1f} min using {frac_used * 100:.0f} %"))
    except FileNotFoundError:
        pass
    return out


# ---------------- V3 structures ----------------

def _tube(od, idd):
    I = math.pi / 64 * (od ** 4 - idd ** 4)
    return I, 2 * I, math.pi / 4 * (od ** 2 - idd ** 2)


def v3_structure(s, sz, mp):
    st = s["structure"]
    E, Gt, sig = (float(st[k]) for k in ("tube_E", "tube_G", "tube_sigma_allow"))   # YAML reads 1.0e9 as text
    out, info = [], {}
    m = mp["mass"]
    W = m * G
    n_ult = st["load_factor_limit"] * st["ultimate_factor"]
    T_max = sz["lift"]["full_thrust_each_n"]
    yb = s["booms"]["y"]
    # masses carried at each boom (motors, props, ESCs, mounts, legs, the boom itself)
    bd = mp.get("breakdown", {})
    m_boom_side = sum(v for k, v in bd.items() if k.split("_")[0] in ("motor", "esc", "mount", "leg") and k.endswith(("FR", "RR"))) \
        + bd.get("boom_R", 0.037)
    # cross spar root moment per side: (a) hover at full thrust on both motors of a boom; (b) n_limit pull-up:
    # elliptic lift centroid 4 s / (3 pi) minus inertia relief of the boom masses
    M_hover = 2 * T_max * yb - m_boom_side * G * yb
    s2 = s["wing"]["span"] / 2
    M_flight = st["load_factor_limit"] * (W / 2) * 4 * s2 / (3 * math.pi) - st["load_factor_limit"] * m_boom_side * G * yb
    M_design = max(M_hover, M_flight) * st["ultimate_factor"]
    cs = st["cross_spar"]
    I, J, Aa = _tube(cs["od"], cs["id"])
    sig_cs = M_design * (cs["od"] / 2) / I
    out.append(_check("V3a cross spar bending stress at ultimate load", sig_cs / 1e6, None, hi=sig / 1e6, unit=" MPa",
                      note=f"governing case {'hover full thrust' if M_hover > M_flight else 'pull-up'}: M = {M_design:.2f} N m incl. x{st['ultimate_factor']}"))
    info["cross_spar"] = {"M_hover": M_hover, "M_flight": M_flight, "M_design": M_design, "sigma_MPa": sig_cs / 1e6, "SF": sig / sig_cs}
    # outer spar: outer panel lift (area fraction) at n_ult, centroid ~45 % of the panel span from the boom
    os_ = st["outer_spar"]
    w = s["wing"]
    c_at = lambda y: w["root_chord"] + (w["tip_chord"] - w["root_chord"]) * y / s2
    area_out = (c_at(yb) + w["tip_chord"]) / 2 * (s2 - yb)
    frac = area_out / (s["wing"]["span"] * (w["root_chord"] + w["tip_chord"]) / 2)
    M_out = n_ult * W * frac * 0.45 * (s2 - yb)
    I2, _, _ = _tube(os_["od"], os_["id"])
    sig_os = M_out * (os_["od"] / 2) / I2
    out.append(_check("V3b outer spar bending stress at ultimate load", sig_os / 1e6, None, hi=sig / 1e6, unit=" MPa",
                      note=f"outer panel carries {frac * 100:.0f} % of lift; M = {M_out:.2f} N m"))
    # design review K8: torsion in the cross spar. The printed clamp joins boom, cross spar and outer-spar root,
    # so two torques reach the cross spar through it: (a) in hover, full pitch authority (front motor at full
    # thrust, rear at idle) on a boom; (b) in a pull-up, the outer panel's lift acting at its quarter chord, aft
    # of the cross spar. Conservative: the spar is fixed at the pod centreline, torque over its half span.
    tsw = math.tan(math.radians(w["le_sweep_deg"]))
    y_c = yb + 0.45 * (s2 - yb)
    dx_out = (y_c * tsw + 0.25 * c_at(y_c)) - cs["x_aft"]
    L_f0 = s["lift_rotors"]["x_front"] + cs["x_aft"]
    L_r0 = -cs["x_aft"] - s["lift_rotors"]["x_rear"]
    T_idle = 0.12 ** 2 * T_max                                 # Q_M_SPIN_MIN 12 %: thrust ~ duty^2
    Tq_hover = T_max * L_f0 - T_idle * L_r0
    Tq_flight = n_ult / st["ultimate_factor"] * W * frac / 2 * dx_out
    Tq = max(Tq_hover, Tq_flight) * st["ultimate_factor"]
    # Torsion path: the cross spar in parallel with the inboard wing's closed foamboard box (forward of the
    # elevon hinge), both fixed at the pod side and loaded at the boom. Foamboard shear stiffness is a
    # conservative guess (half of paper faces alone), measured by bench item B18.
    L_t = yb - s["fuselage"]["width"] / 2
    y_m = s["fuselage"]["width"] / 2 + L_t / 2
    c_m = c_at(y_m)
    tc = float(st.get("airfoil_tc", 0.084))
    c_box = (1 - w["elevon_chord_frac"]) * c_m
    A_box = 0.6 * c_box * tc * c_m                       # enclosed area of the folded section (~0.6 of the bounding box)
    perim = 2.05 * c_box
    Gt_foam = float(st["foamboard_shear_stiffness"])     # N/m
    GJ_box = 4 * A_box ** 2 * Gt_foam / perim
    GJ_tube = Gt * J
    Tq_lim = max(Tq_hover, Tq_flight)
    tau = Tq * GJ_tube / (GJ_box + GJ_tube) * (cs["od"] / 2) / J
    tau_allow = float(st.get("tube_tau_allow", 40e6))
    q_box = Tq * GJ_box / (GJ_box + GJ_tube) / (2 * A_box)
    case = "hover pitch command" if Tq_hover > Tq_flight else "outer panel lift"
    out.append(_check("V3g cross spar shear stress in torsion at ultimate load", tau / 1e6, None, hi=tau_allow / 1e6, unit=" MPa",
                      note=f"governing case {case}: T = {Tq:.2f} N m incl. x{st['ultimate_factor']}; tube takes "
                           f"{GJ_tube / (GJ_box + GJ_tube) * 100:.0f} %, wing box the rest"))
    out.append(_check("V3h inboard wing box shear flow at ultimate torque", q_box / 1000, None,
                      hi=float(st["foamboard_shear_flow_allow"]) / 1000, unit=" N/mm"))
    twist_cs = Tq_lim * L_t / (GJ_box + GJ_tube)
    out.append(_check("V3i boom pitch twist at the clamp, limit torque (cross spar + wing box)", math.degrees(twist_cs), None,
                      hi=2.0, unit=" deg", note=f"tube alone would twist {math.degrees(Tq_lim * L_t / GJ_tube):.0f} deg: the "
                                                 "boom must be glued to the wing along its whole chord; bench item B18"))
    info["cross_spar_torsion"] = {"T_hover": Tq_hover, "T_flight": Tq_flight, "dx_outer_panel_m": dx_out,
                                  "GJ_box": GJ_box, "GJ_tube": GJ_tube, "tau_MPa": tau / 1e6, "q_box_N_per_mm": q_box / 1000,
                                  "twist_deg": math.degrees(twist_cs)}
    # booms: front arm cantilevered from the cross-spar clamp (conservative: no support from the foam)
    bm = s["booms"]
    Ib, Jb, Ab = _tube(bm["diameter"], bm["inner_diameter"])
    x_clamp = -cs["x_aft"]
    L_f = s["lift_rotors"]["x_front"] - x_clamp
    L_r = x_clamp - s["lift_rotors"]["x_rear"]
    tip_m = s["motors"][s["propulsion"]["lift_motor"]]["mass"] + s["props"][s["propulsion"]["lift_prop"]]["mass"] + 0.010
    m_lin = Ab * 1550.0
    for name, L in (("front", L_f), ("rear", L_r)):
        sig_b = st["ultimate_factor"] * T_max * L * (bm["diameter"] / 2) / Ib
        defl = T_max * L ** 3 / (3 * E * Ib)
        k = 3 * E * Ib / L ** 3
        f1 = math.sqrt(k / (tip_m + 0.24 * m_lin * L)) / (2 * math.pi)
        out.append(_check(f"V3c boom {name} arm ({L * 1000:.0f} mm) stress at 1.5 x full thrust", sig_b / 1e6, None, hi=sig / 1e6, unit=" MPa"))
        out.append(_check(f"V3d boom {name} arm tip deflection at full thrust", defl * 1000, None, hi=5.0, unit=" mm"))
        hover_1p = sz["lift"]["hover_rpm"] / 60
        out.append(_check(f"V3e boom {name} arm first bending mode vs hover 1P ({hover_1p:.0f} Hz) and rate loop (< 15 Hz)", f1, None,
                          lo=15.0, hi=hover_1p / 1.5, unit=" Hz", note="keep clear of both; set INS_HNTCH on motor RPM for 1P"))
        info[f"boom_{name}"] = {"L": L, "f1_Hz": f1, "defl_mm": defl * 1000}
        # torsion: full motor torque twists the arm and changes the 4 deg yaw tilt
        lm = P.get("motor", s["propulsion"]["lift_motor"], s)
        q_full = (sz["lift"]["full_current_each_a"] - lm.i0) / lm.kv_rad     # motor torque constant 1/Kv
        twist = q_full * L / (Gt * Jb)
        out.append(_check(f"V3f boom {name} arm twist under full motor torque", math.degrees(twist), None, hi=1.0, unit=" deg",
                          note=f"vs {s['lift_rotors'].get('yaw_tilt_deg', 0):.0f} deg yaw tilt; pultruded tube (G 4.5 GPa)"))
    return out, info


# ---------------- V4 propulsion limits ----------------

def v4_limits(s, sz):
    out = []
    lp = P.get("prop", s["propulsion"]["lift_prop"], s)
    pp = P.get("prop", s["propulsion"]["pusher_prop"], s, role="pusher")
    # full-throttle rpm from the propulsion model (lift) and static pusher
    lm = P.get("motor", s["propulsion"]["lift_motor"], s)
    pm = P.get("motor", s["propulsion"]["pusher_motor"], s)
    batt = P.get("battery", s["propulsion"]["battery"], s)
    op_l = P.operating_point(lm, lp, batt.cells * batt.v_cell_full)
    thr_max = s["propulsion"].get("pusher_thr_max", 1.0)          # THR_MAX caps the pusher duty (review K4)
    op_p = P.operating_point(pm, pp, thr_max * batt.cells * batt.v_cell_full)
    for name, op, pr in (("lift", op_l, lp), ("pusher", op_p, pp)):
        tip = math.pi * pr.d * op.rpm / 60
        out.append(_check(f"V4a {name} prop tip Mach at full throttle", tip / 340.3, None, hi=0.5))
        out.append(_check(f"V4b {name} motor current at full throttle vs 30 A ESC", op.current, None, hi=30 / 1.5, unit=" A"))
    out.append(_check("V4e pusher motor current at THR_MAX, full pack, static vs motor continuous rating", op_p.current, None,
                      hi=pm.i_max_cont, unit=" A", note=f"THR_MAX {thr_max * 100:.0f} (design review K4)"))
    i_peak = 4 * op_l.current + 0.5 * op_p.current      # transition: lift motors high while the pusher spools up
    c_rate = s["batteries"][s["propulsion"]["battery"]].get("c_rating", 30)   # cheapest 3S 2200 packs are rated 25-30C
    out.append(_check("V4c peak pack current (4 lift full + pusher half) vs pack rating", i_peak, None,
                      hi=c_rate * batt.capacity_ah, unit=" A"))
    out.append(_check("V4d hover T/W with an empty pack", sz["lift"]["tw_empty_pack"], None, lo=1.6))
    return out


# ---------------- V5 numerics ----------------

def v5_numerics(s):
    from .sim import Vehicle, q_from_euler
    out = []

    def run(dt, mode):
        v = Vehicle(s)
        v.ground_on = False
        if mode == "hover":
            v.pos = [0.0, 0.0, -50.0]
            duty = [0.52, 0.52, 0.48, 0.48, 0.0]      # small asymmetric step: excites roll/pitch/yaw open loop
            T = 0.8
        else:
            g = trim.trim_glide(v, 13.0)
            v.pos = [0.0, 0.0, -100.0]
            v.vel = [13.0 * math.cos(g["gamma"]), 0.0, -13.0 * math.sin(g["gamma"])]
            v.q = list(q_from_euler(0.0, g["alpha"] + g["gamma"], 0.0))
            v.elevon = [g["de"], g["de"]]
            duty = [0.0] * 5
            T = 3.0
        n = int(round(T / dt))
        for i in range(n):
            de = v.elevon[0] if mode == "hover" else (g["de"] + (0.05 if i * dt < 0.5 else 0.0))
            v.step(dt, duty, (de, de))
        return list(v.pos) + list(v.vel) + list(v.w)

    for mode in ("hover", "glide"):
        ref = np.array(run(1 / 4800, mode))
        res = {}
        for f in (600, 1200, 2400):
            x = np.array(run(1 / f, mode))
            res[f] = float(np.max(np.abs(x - ref)) / (np.max(np.abs(ref[3:])) + 1e-9))
        out.append(_check(f"V5 {mode}: state error at the sim's 1.2 kHz step vs 4.8 kHz reference", res[1200], None, hi=0.01,
                          note="relative to the largest velocity/rate; errors at 600/1200/2400 Hz: " +
                               ", ".join(f"{res[k]:.1e}" for k in (600, 1200, 2400))))
    return out


# ---------------- V6 control authority ----------------

def v6_control(s, t, mp):
    out = []
    der = t["derivatives"]
    V = s["mission_profile"]["cruise_mps"]
    b = t["ref"]["span"]
    da = math.radians(s["wing"]["elevon_max_deg"]) * 0.5     # half throw used for roll
    pb2v = -der["Clda"] * da / der["Clp"]
    p_dps = math.degrees(pb2v * 2 * V / b)
    out.append(_check("V6a steady roll rate at half elevon throw, cruise", p_dps, None, lo=90.0, unit=" deg/s",
                      note=f"pb/2V = {pb2v:.3f} (>= 0.07 is brisk for model aircraft)"))
    # pitch: can full up-elevon hold CLmax-trim margin? elevon used at trimmed CLmax must leave >= 25 % throw
    de_needed = None
    tr = t.get("trim", {})
    cm_de = der["Cmde"]
    # elevon to go from cruise CL to CLmax at fixed CG: dCm/dCL * dCL / Cmde
    sm = t["static_margin"]
    de_needed = abs(sm * (t["clmax"] - tr["cl"]) / cm_de) + abs(math.radians(tr["elevon_deg"]))
    frac = math.degrees(de_needed) / s["wing"]["elevon_max_deg"]
    out.append(_check("V6b pitch: elevon to trim at CLmax as a fraction of full throw", frac, None, hi=0.75))
    # quad yaw authority with the 4 deg tilt: yaw moment with one diagonal pair at 80 % and the other at 20 % duty
    # (total thrust ~ hover), vs the moment needed to weathervane against a 7 m/s crosswind on the winglets
    from .sim import Vehicle
    v = Vehicle(s)
    duty = [0.80, 0.80, 0.20, 0.20, 0.0]
    rotor, vp = trim.steady_rotor(v, duty)
    rdot = v.derivs((0, 0, -50.0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0), rotor, [0, 0], duty, vp)[3][2]
    n_avail = rdot * v.I[2][2]
    w = s["wing"]
    a_wl = 2 * w["winglet_height"] * 0.8 * w["winglet_root_chord"]
    x_wl = (w["span"] / 2 * math.tan(math.radians(w["le_sweep_deg"])) + w["tip_chord"] - 0.5 * w["winglet_root_chord"]) + mp["cg"][0]
    n_req = 0.5 * s["environment"]["rho"] * 7.0 ** 2 * a_wl * 1.2 * abs(x_wl)
    out.append(_check("V6c quad yaw moment available (pairs 80/20 %) vs 2 x winglet moment in a 7 m/s crosswind", n_avail, None,
                      lo=2 * n_req, unit=" N m", note=f"required {n_req:.3f} N m (winglets {a_wl * 1e4:.0f} cm^2, {abs(x_wl) * 1000:.0f} mm behind CG)"))
    return out


def main():
    s = S.load()
    t = json.loads((ROOT / "aero" / "tables.json").read_text())
    sz = json.loads((ROOT / "build" / "sizing.json").read_text())["result"]
    mp = S.mass_properties(s)
    checks, info = [], {}
    c, i = v1_aero(s, t); checks += c; info["aero"] = i
    checks += v2_hand(s, t, sz, mp)
    c, i = v3_structure(s, sz, mp); checks += c; info["structure"] = i
    checks += v4_limits(s, sz)
    checks += v5_numerics(s)
    checks += v6_control(s, t, mp)
    n_ok = sum(c["pass"] for c in checks)
    (ROOT / "build" / "validation.json").write_text(json.dumps({"passed": n_ok, "n": len(checks), "checks": checks, "info": info},
                                                               indent=1, default=float))
    for c in checks:
        print(f"[{'PASS' if c['pass'] else 'FAIL'}] {c['check']}: {c['value']:.4g}{c['unit']}  ({c['criterion']}) {c['note']}")
    print(f"{n_ok}/{len(checks)} validation checks pass")
    return 0 if n_ok == len(checks) else 1


if __name__ == "__main__":
    sys.exit(main())
