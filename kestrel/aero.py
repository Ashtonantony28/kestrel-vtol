"""M1 aerodynamics: AeroSandbox model of the flying wing -> aero/tables.json for sizing and the 6-DOF sim.

Run: python -m kestrel.aero
- AeroBuildup (NeuralFoil section polars + lifting-line style buildup) over alpha -12..22 deg at cruise.
- Stability + control derivatives by AeroSandbox and finite differences (elevon symmetric / differential).
- Full +-180 deg tables: attached-flow data blended into flat-plate aero past stall (hover in wind, descent,
  back-transition all visit high alpha).
Coordinates here are AeroSandbox geometry axes (x aft, y right, z up); tables are exported as body-axis
coefficients CL, CD (stability axes) and Cm about the CG, plus derivatives in the usual sign convention.
"""
from __future__ import annotations

import json
import math

import aerosandbox as asb
import aerosandbox.numpy as anp
import numpy as np

from . import sizing
from . import spec as S

ROOT = S.ROOT
OUT = ROOT / "aero" / "tables.json"
FOAM_CLMAX_FACTOR = 0.90   # folded foamboard airfoil vs molded MH45: conservative CLmax knockdown


def build_airplane(s: dict, d_sym: float = 0.0, d_diff: float = 0.0) -> asb.Airplane:
    w, f = s["wing"], s["fuselage"]
    b2 = w["span"] / 2
    tan_sw = math.tan(math.radians(w["le_sweep_deg"]))
    af = asb.Airfoil(w["airfoil"])
    e0, e1 = w["elevon_span_frac"]

    def xsec(eta, cs=None):
        y = eta * b2
        c = w["root_chord"] + (w["tip_chord"] - w["root_chord"]) * eta
        tw = w["tip_twist_deg"] * eta
        return asb.WingXSec(xyz_le=[y * tan_sw, y, y * math.tan(math.radians(w["dihedral_deg"]))], chord=c,
                            twist=tw, airfoil=af, control_surfaces=cs or [])

    hinge = 1 - w["elevon_chord_frac"]
    cs = [asb.ControlSurface("elevon_sym", symmetric=True, deflection=d_sym, hinge_point=hinge),
          asb.ControlSurface("elevon_diff", symmetric=False, deflection=d_diff, hinge_point=hinge)]
    wing = asb.Wing(name="wing", symmetric=True, xsecs=[xsec(0.0), xsec(e0, cs), xsec(e1), xsec(1.0)])
    tip_x = b2 * tan_sw
    h, crw = w["winglet_height"], w["winglet_root_chord"]
    winglet = asb.Wing(name="winglets", symmetric=True, xsecs=[
        asb.WingXSec(xyz_le=[tip_x + (w["tip_chord"] - crw), b2, 0.0], chord=crw, airfoil=asb.Airfoil("naca0008")),
        asb.WingXSec(xyz_le=[tip_x + w["tip_chord"] - 0.6 * crw + 0.03, b2, h], chord=0.6 * crw, airfoil=asb.Airfoil("naca0008")),
    ])
    x0, L = -f["nose_x"], f["length"]
    fus = asb.Fuselage(name="pod", xsecs=[
        asb.FuselageXSec(xyz_c=[x0 + L * t, 0, 0.005], width=f["width"] * k, height=f["height"] * k, shape=3)
        for t, k in [(0.0, 0.2), (0.12, 0.8), (0.3, 1.0), (0.75, 1.0), (1.0, 0.5)]
    ])
    return asb.Airplane(name="kestrel", wings=[wing, winglet], fuselages=[fus])


def _cg_asb(s):
    mp = S.mass_properties(s)
    cg = mp["cg"]
    return [-cg[0], cg[1], -cg[2]]   # FRD -> asb (x aft, z up)


class _Scalar(dict):
    def __getitem__(self, k):
        v = dict.__getitem__(self, k)
        try:
            return float(np.asarray(v).reshape(-1)[0])
        except (TypeError, ValueError):
            return v


def run_point(s, alpha, beta=0.0, v=None, d_sym=0.0, d_diff=0.0, p=0.0, q=0.0, r=0.0):
    v = v or s["mission_profile"]["cruise_mps"]
    ap = build_airplane(s, d_sym, d_diff)
    op = asb.OperatingPoint(velocity=v, alpha=alpha, beta=beta, p=p, q=q, r=r)
    return _Scalar(asb.AeroBuildup(airplane=ap, op_point=op, xyz_ref=_cg_asb(s)).run())


def flat_plate(alpha_rad: np.ndarray, cd90: float = 1.20):
    """Flat-plate post-stall model (stability axes) for a low-AR wing."""
    cn = cd90 * np.sin(alpha_rad)
    cl = cn * np.cos(alpha_rad)
    cd = 0.02 + cn * np.sin(alpha_rad)
    return cl, cd, cn


DE_GRID = (-20.0, -10.0, 0.0, 10.0, 20.0)   # symmetric elevon deflection grid, deg (+ = trailing edge down)


def _trim_de(cm_row, de_grid):
    """Elevon (deg) giving Cm = 0 by piecewise-linear interpolation along the deflection grid; None if outside."""
    for i in range(len(de_grid) - 1):
        c0, c1 = cm_row[i], cm_row[i + 1]
        if (c0 - 0) * (c1 - 0) <= 0 and c0 != c1:
            return de_grid[i] + (0 - c0) * (de_grid[i + 1] - de_grid[i]) / (c1 - c0)
    return None


def generate(s: dict | None = None) -> dict:
    s = s or S.load()
    g = S.wing_geometry(s)
    v = s["mission_profile"]["cruise_mps"]
    rho = s["environment"]["rho"]
    mac, area, span = g["mac"], g["area"], g["span"]
    cg = _cg_asb(s)
    extra = sizing.drag_buildup(s)
    extra_cda = extra["booms"] + extra["lift_motors_stopped_props"]   # not modelled in AeroBuildup

    # attached-flow sweep over alpha x symmetric elevon
    alphas = np.arange(-12.0, 22.01, 1.0)
    grid = {}
    for de in DE_GRID:
        rows = [run_point(s, a, d_sym=de) for a in alphas]
        grid[de] = np.array([[r["CL"], r["CD"] + extra_cda / area, r["Cm"]] for r in rows])
    CL, CD, CM = grid[0.0].T
    i_max = int(np.argmax(CL))
    clmax_raw = float(CL[i_max])
    alpha_stall = float(alphas[i_max])

    # stability derivatives (AeroSandbox) at 3 deg
    stab = _Scalar(asb.AeroBuildup(airplane=build_airplane(s), op_point=asb.OperatingPoint(velocity=v, alpha=3.0),
                                   xyz_ref=cg).run_with_stability_derivatives())
    keep = ["CLa", "Cma", "CYb", "Clb", "Cnb", "Clp", "Cmq", "Cnr", "CYp", "Cnp", "Clr", "CLq", "CYr"]
    der = {k: float(stab[k]) for k in keep if k in stab}
    x_np = float(stab["x_np"])
    static_margin = (x_np - cg[0]) / mac
    # local static margin -dCm/dCL (NeuralFoil section Cm is nonlinear at low Re, so the margin at the cruise
    # alpha can differ from the small-alpha neutral point). Smoothed over 3 deg.
    k = 3
    cls = np.convolve(CL, np.ones(k) / k, mode="valid")
    cms = np.convolve(CM, np.ones(k) / k, mode="valid")
    al = alphas[k // 2: -(k // 2)]
    sm_local = -np.gradient(cms) / np.gradient(cls)
    band = (al >= 2) & (al <= 15)
    sm_min = float(sm_local[band].min())
    alpha_sm_min = float(al[band][int(np.argmin(sm_local[band]))])

    # small-deflection control derivatives (per rad) at 3 deg, for reference and the linear models
    i3 = int(np.where(alphas == 3.0)[0][0])
    der["CLde"] = float(grid[10.0][i3, 0] - grid[-10.0][i3, 0]) / math.radians(20)
    der["Cmde"] = float(grid[10.0][i3, 2] - grid[-10.0][i3, 2]) / math.radians(20)
    dd = 5.0
    rp, rm = run_point(s, 3.0, d_diff=dd), run_point(s, 3.0, d_diff=-dd)
    # AeroSandbox antisymmetric deflection > 0 puts the RIGHT trailing edge down. Kestrel defines
    # da = (left - right) / 2 (left TE down = roll right), so the derivatives change sign.
    der["Clda"] = -float(rp["Cl"] - rm["Cl"]) / math.radians(2 * dd)
    der["Cnda"] = -float(rp["Cn"] - rm["Cn"]) / math.radians(2 * dd)

    # trimmed polar and trimmed CLmax: elevon solved for Cm = 0 at every alpha on the deflection grid
    de_lim = s["wing"]["elevon_max_deg"]
    tr_rows = []
    for i, a in enumerate(alphas):
        cm_row = [grid[d][i, 2] for d in DE_GRID]
        de = _trim_de(cm_row, DE_GRID)
        if de is None or abs(de) > de_lim:
            continue
        cl_t = float(np.interp(de, DE_GRID, [grid[d][i, 0] for d in DE_GRID]))
        cd_t = float(np.interp(de, DE_GRID, [grid[d][i, 1] for d in DE_GRID]))
        tr_rows.append((a, de, cl_t, cd_t))
    tr_rows = np.array(tr_rows)
    pos = tr_rows[:, 0] >= 0
    i_pk = int(np.argmax(np.where(pos, tr_rows[:, 2], -9)))
    clmax_trim = float(tr_rows[i_pk, 2]) * FOAM_CLMAX_FACTOR
    pol = tr_rows[(tr_rows[:, 2] > 0.02) & (np.arange(len(tr_rows)) <= i_pk)]
    pol = pol[np.argsort(pol[:, 2])]

    # cruise trim from the trimmed polar
    W = S.mass_properties(s)["mass"] * s["environment"]["g"]
    cl_req = W / (0.5 * rho * v**2 * area)
    a_tr = float(np.interp(cl_req, pol[:, 2], pol[:, 0]))
    de_tr_deg = float(np.interp(cl_req, pol[:, 2], pol[:, 1]))
    ld_trim = cl_req / float(np.interp(cl_req, pol[:, 2], pol[:, 3]))

    # +-180 deg tables (per elevon setting) blended with flat plate past stall
    a_full = np.arange(-180.0, 180.01, 2.0)
    ar = np.radians(a_full)
    fp_cl, fp_cd, fp_cn = flat_plate(ar)
    x_ac = -g["x_quarter_mac"]
    x_cp = x_ac + (0.25 * mac) * np.abs(np.sin(ar))
    fp_cm = -fp_cn * (x_cp - cg[0]) / mac
    def sig(x): return 1 / (1 + np.exp(-x))
    wfp = np.clip(sig((a_full - (alpha_stall + 2)) / 1.5) + sig((-(a_full) - 14.0) / 1.5), 0, 1)
    ramp = np.clip((a_full - 0.6 * alpha_stall) / (0.4 * alpha_stall), 0, 1)
    knock = 1 - (1 - FOAM_CLMAX_FACTOR) * ramp * ramp * (3 - 2 * ramp)   # foam CLmax knockdown near stall only
    full = {}
    for de in DE_GRID:
        acl, acd, acm = (np.interp(a_full, alphas, grid[de][:, j]) for j in range(3))
        full[str(de)] = {
            "CL": ((1 - wfp) * acl * knock + wfp * fp_cl).tolist(),
            "CD": ((1 - wfp) * acd + wfp * (fp_cd + extra_cda / area)).tolist(),
            "Cm": ((1 - wfp) * acm + wfp * fp_cm).tolist(),
        }

    tables = {
        "ref": {"area": area, "mac": mac, "span": span, "v": v, "cg_asb": cg},
        "alpha_deg": a_full.tolist(), "de_grid_deg": list(DE_GRID), "full": full,
        "CL": full["0.0"]["CL"], "CD": full["0.0"]["CD"], "Cm": full["0.0"]["Cm"],
        "attached": {"alpha_deg": alphas.tolist(), "CL": CL.tolist(), "CD": CD.tolist(), "Cm": CM.tolist()},
        "polar": {"cl": pol[:, 2].tolist(), "cd": pol[:, 3].tolist(), "alpha_deg": pol[:, 0].tolist(),
                  "elevon_deg": pol[:, 1].tolist(), "trimmed": True},
        "extra_cda": 0.0,                   # already folded into CD above
        "clmax": clmax_trim, "clmax_untrimmed": clmax_raw * FOAM_CLMAX_FACTOR, "clmax_raw": clmax_raw,
        "alpha_stall_deg": alpha_stall,
        "derivatives": der, "x_np_asb": x_np, "static_margin": static_margin,
        "static_margin_min": sm_min, "alpha_static_margin_min": alpha_sm_min,
        "sm_local": {"alpha_deg": al.tolist(), "sm": sm_local.tolist()},
        "trim": {"cl": cl_req, "alpha_deg": a_tr, "elevon_deg": de_tr_deg, "ld": ld_trim},
        "cd90": 1.20,
    }
    OUT.write_text(json.dumps(tables, indent=1))
    return tables


def main():
    t = generate()
    d = t["derivatives"]
    print(f"Static margin {t['static_margin']*100:.1f} % MAC at 3 deg (x_np {t['x_np_asb']*1000:.0f} mm aft of root LE); "
          f"minimum local margin {t['static_margin_min']*100:.1f} % at {t['alpha_static_margin_min']:.0f} deg")
    print(f"Trimmed CLmax {t['clmax']:.2f} (untrimmed {t['clmax_untrimmed']:.2f}, raw {t['clmax_raw']:.2f} at {t['alpha_stall_deg']:.0f} deg)")
    print(f"Trim at cruise: CL {t['trim']['cl']:.3f}, alpha {t['trim']['alpha_deg']:.1f} deg, elevon {t['trim']['elevon_deg']:.1f} deg, L/D {t['trim']['ld']:.1f}")
    print("Derivatives:", {k: round(v, 3) for k, v in d.items()})


if __name__ == "__main__":
    main()
