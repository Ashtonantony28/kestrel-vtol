"""M0 sizing: mass, thrust/weight, hover and cruise power, endurance, stall, cost, and a motor/prop trade study.

Run: python -m kestrel.sizing            (prints report, writes build/sizing.json)
Cruise drag uses aero tables from M1 when present (aero/tables.json); otherwise a component drag buildup.
"""
from __future__ import annotations

import itertools
import json
import math
from pathlib import Path

import yaml

from . import propulsion as P
from . import spec as S

ROOT = S.ROOT


def drag_buildup(s: dict) -> dict:
    """Parasite drag area CdA (m^2) by component: hand estimate, superseded by aero tables in M1."""
    g = S.wing_geometry(s)
    cd0_wing = 0.015  # folded foamboard airfoil, transitional Re ~2e5 (vs 0.010 for a molded MH45)
    parts = {
        "wing_profile": cd0_wing * g["area"],
        "winglets": 0.012 * 2 * s["wing"]["winglet_height"] * s["wing"]["winglet_root_chord"],
        "fuselage_pod": 0.12 * s["fuselage"]["width"] * s["fuselage"]["height"] + 0.004 * 2 * s["fuselage"]["length"] * (s["fuselage"]["width"] + s["fuselage"]["height"]),
        "booms": 2 * (0.004 * math.pi * s["booms"]["diameter"] * s["booms"]["length"] + 0.8 * math.pi / 4 * s["booms"]["diameter"] ** 2),
        "lift_motors_stopped_props": 4 * (0.8 * 0.028 * 0.026 + 0.13 * 0.0048),  # bell frontal + pitched blades
    }
    parts["total"] = sum(parts.values())
    return parts


def cruise_drag(s: dict, v: float, mass: float, tables: dict | None = None) -> tuple[float, float, float]:
    """Returns (drag N, CL, L/D) in trimmed level flight."""
    env = s["environment"]
    g = S.wing_geometry(s)
    q = 0.5 * env["rho"] * v**2
    cl = mass * env["g"] / (q * g["area"])
    if tables and "polar" in tables:
        pol = tables["polar"]
        import numpy as np
        cd = float(np.interp(cl, pol["cl"], pol["cd"])) + tables.get("extra_cda", 0.0) / g["area"]
    else:
        e = 0.85
        cd = drag_buildup(s)["total"] / g["area"] + cl**2 / (math.pi * e * g["aspect_ratio"])
    return q * g["area"] * cd, cl, cl / cd


def load_tables() -> dict | None:
    p = ROOT / "aero" / "tables.json"
    return json.loads(p.read_text()) if p.exists() else None


def bom_totals(path: Path | None = None) -> dict:
    b = yaml.safe_load(open(path or ROOT / "build" / "bom.yaml"))
    out = {}
    for k, rows in b.items():
        out[k] = (sum(r["low"] for r in rows), sum(r["high"] for r in rows))
    out["total"] = (sum(v[0] for v in out.values()), sum(v[1] for v in out.values()))
    return out


def analyse(s: dict | None = None, lift_motor=None, lift_prop=None, pusher_motor=None, pusher_prop=None,
            use_cad: bool = True) -> dict:
    s = s or S.load()
    env, pr, lim, mp = s["environment"], s["propulsion"], s["limits"], s["mission_profile"]
    lm = P.get("motor", lift_motor or pr["lift_motor"], s)
    lp = P.get("prop", lift_prop or pr["lift_prop"], s)
    pm = P.get("motor", pusher_motor or pr["pusher_motor"], s)
    pp = P.get("prop", pusher_prop or pr["pusher_prop"], s, role="pusher")
    batt = P.get("battery", pr["battery"], s)
    massp = S.mass_properties(s, use_cad=use_cad)
    mass = massp["mass"]
    w = mass * env["g"]
    av_i = mp["avionics_w"] / batt.v_nom

    # Full-throttle thrust, nominal and at empty pack (worst case).
    full_nom, vp_nom = P.solve_with_battery(lm, lp, batt, 1.0, 4, av_i)
    full_empty, _ = P.solve_with_battery(lm, lp, batt, 1.0, 4, av_i, soc_v_cell=batt.v_cell_empty)
    tw_nom = 4 * full_nom.thrust / w
    tw_empty = 4 * full_empty.thrust / w
    hover, vp_hover = P.throttle_for_thrust(lm, lp, batt, w / 4, 4, av_i)
    p_hover = 4 * hover.current * hover.throttle * vp_hover + mp["avionics_w"]

    tables = load_tables()
    v = mp["cruise_mps"]
    drag, cl, ld = cruise_drag(s, v, mass, tables)
    cruise, vp_cruise = P.throttle_for_thrust(pm, pp, batt, drag, 1, av_i, airspeed=v)
    p_cruise = cruise.current * cruise.throttle * vp_cruise + mp["avionics_w"]
    thr_max = pr.get("pusher_thr_max", 1.0)            # THR_MAX: the pusher never sees more than this duty
    # worst case for motor current: a fresh pack (review K4); thrust margins use the nominal pack below
    pusher_static, _ = P.solve_with_battery(pm, pp, batt, thr_max, 1, av_i, soc_v_cell=batt.v_cell_full)
    pusher_at_v, _ = P.solve_with_battery(pm, pp, batt, thr_max, 1, av_i, airspeed=v)

    usable_wh = batt.energy_wh * batt.usable_frac
    hover_wh = p_hover * mp["hover_s"] / 3600.0
    cruise_wh = max(usable_wh - hover_wh, 0.0)
    endurance_min = mp["hover_s"] / 60 + cruise_wh / p_cruise * 60
    endurance_reserve_min = mp["hover_s"] / 60 + max(usable_wh * (1 - lim["reserve_frac"]) - hover_wh, 0) / p_cruise * 60
    hover_only_min = usable_wh / p_hover * 60

    g = S.wing_geometry(s)
    clmax = tables["clmax"] if tables else 0.85
    v_stall = math.sqrt(2 * w / (env["rho"] * g["area"] * clmax))

    # best-endurance / best-range speed sweep
    sweep = []
    for vv in [x * 0.5 for x in range(int(math.ceil(v_stall * 1.2 * 2)), 45)]:
        try:
            d, c, l = cruise_drag(s, vv, mass, tables)
            op, vpp = P.throttle_for_thrust(pm, pp, batt, d, 1, av_i, airspeed=vv)
            pw = op.current * op.throttle * vpp + mp["avionics_w"]
            sweep.append({"v": vv, "drag": d, "cl": c, "ld": l, "p": pw, "wh_per_km": pw / vv / 3.6})
        except ValueError:
            pass
    best_end = min(sweep, key=lambda r: r["p"])
    best_rng = min(sweep, key=lambda r: r["wh_per_km"])

    return {
        "mass_kg": mass, "cg": massp["cg"], "mass_source": massp.get("source", "spec"),
        "weight_n": w,
        "lift": {"motor": lift_motor or pr["lift_motor"], "prop": lift_prop or pr["lift_prop"],
                 "full_thrust_each_n": full_nom.thrust, "full_current_each_a": full_nom.current,
                 "tw_nominal": tw_nom, "tw_empty_pack": tw_empty,
                 "hover_throttle": hover.throttle, "hover_current_each_a": hover.current,
                 "hover_rpm": hover.rpm, "hover_power_w": p_hover, "pack_v_full_throttle": vp_nom},
        "cruise": {"v_mps": v, "cl": cl, "ld": ld, "drag_n": drag, "pusher_throttle": cruise.throttle,
                   "power_w": p_cruise, "pusher_current_a": cruise.current,
                   "pusher_static_thrust_n": pusher_static.thrust, "pusher_thrust_at_cruise_full_n": pusher_at_v.thrust,
                   "excess_thrust_ratio": pusher_at_v.thrust / drag, "pusher_thr_max": thr_max,
                   "climb_at_cruise_mps": (pusher_at_v.thrust - drag) * v / w,
                   "pusher_static_current_a": pusher_static.current, "pusher_motor_i_max_cont": pm.i_max_cont},
        "best_endurance": best_end, "best_range": best_rng,
        "battery": {"usable_wh": usable_wh, "hover_wh": hover_wh},
        "endurance_min": endurance_min, "endurance_with_reserve_min": endurance_reserve_min,
        "hover_only_min": hover_only_min,
        "stall_mps": v_stall, "clmax": clmax,
        "drag_buildup": drag_buildup(s), "aero_source": "tables" if tables else "buildup",
        "esc_ok": full_nom.current <= s["propulsion"]["esc_max_a"],
        "motor_current_ok": full_nom.current <= lm.i_max_cont * 1.15,
        "pusher_current_ok": pusher_static.current <= pm.i_max_cont,
    }


def trade_study(s: dict | None = None) -> list[dict]:
    s = s or S.load()
    rows = []
    for mot, prop in itertools.product(s["motors"], ["prop_10x45", "prop_9x45", "prop_8x6"]):
        try:
            r = analyse(s, lift_motor=mot, lift_prop=prop)
            rows.append({"lift": f"{mot}+{prop}", "tw": r["lift"]["tw_nominal"], "tw_empty": r["lift"]["tw_empty_pack"],
                         "hover_w": r["lift"]["hover_power_w"], "full_a": r["lift"]["full_current_each_a"],
                         "esc_ok": r["esc_ok"], "motor_ok": r["motor_current_ok"]})
        except ValueError as e:
            rows.append({"lift": f"{mot}+{prop}", "error": str(e)})
    for mot, prop in itertools.product(s["motors"], ["prop_9x6", "prop_8x6", "prop_9x45"]):
        try:
            r = analyse(s, pusher_motor=mot, pusher_prop=prop)
            rows.append({"pusher": f"{mot}+{prop}", "cruise_w": r["cruise"]["power_w"],
                         "excess": r["cruise"]["excess_thrust_ratio"], "static_n": r["cruise"]["pusher_static_thrust_n"],
                         "endurance_min": r["endurance_min"]})
        except ValueError as e:
            rows.append({"pusher": f"{mot}+{prop}", "error": str(e)})
    return rows


def checks(r: dict, s: dict | None = None) -> dict:
    s = s or S.load()
    lim = s["limits"]
    bom = bom_totals()
    return {
        "hover_tw>=target": r["lift"]["tw_nominal"] >= lim["target_hover_tw"],
        "hover_tw_empty>=1.6": r["lift"]["tw_empty_pack"] >= 1.6,
        "hover_throttle<=0.6": r["lift"]["hover_throttle"] <= 0.6,
        "endurance_reserve>=min": r["endurance_with_reserve_min"] >= lim["min_endurance_min"],
        "stall<=max": r["stall_mps"] <= lim["max_stall_mps"],
        # was "excess thrust >= 2 at 100 % throttle". With THR_MAX capping the pusher for its current limit (review K4)
        # no A2212 pusher reaches 2.0; the requirement is now the climb rate that ratio stood for (ADR 0008).
        "pusher_climb_at_cruise>=min": r["cruise"]["climb_at_cruise_mps"] >= lim["min_cruise_climb_mps"],
        "esc_current_ok": r["esc_ok"],
        "motor_current_ok": r["motor_current_ok"],
        "pusher_current_ok@THR_MAX": r["pusher_current_ok"],
        "aircraft_bom<=max": bom["aircraft"][1] <= lim["max_bom_usd"],
    }


def main():
    s = S.load()
    r = analyse(s)
    c = checks(r, s)
    bom = bom_totals()
    print(f"Kestrel sizing (mass source: {r['mass_source']}, aero: {r['aero_source']})")
    print(f"  AUW {r['mass_kg']*1000:.0f} g, CG x={r['cg'][0]*1000:.0f} mm")
    L = r["lift"]
    print(f"  Lift {L['motor']}+{L['prop']}: {L['full_thrust_each_n']/9.81*1000:.0f} gf each @ {L['full_current_each_a']:.1f} A; "
          f"T/W {L['tw_nominal']:.2f} (empty pack {L['tw_empty_pack']:.2f})")
    print(f"  Hover: throttle {L['hover_throttle']*100:.0f} %, {L['hover_current_each_a']:.1f} A/motor, {L['hover_power_w']:.0f} W")
    C = r["cruise"]
    print(f"  Cruise {C['v_mps']} m/s: CL {C['cl']:.2f}, L/D {C['ld']:.1f}, drag {C['drag_n']:.2f} N, {C['power_w']:.0f} W, pusher {C['pusher_throttle']*100:.0f} %")
    print(f"  Pusher at THR_MAX {C['pusher_thr_max']*100:.0f} %: {C['pusher_static_current_a']:.1f} A static "
          f"(motor {C['pusher_motor_i_max_cont']:.0f} A cont), excess thrust at cruise {C['excess_thrust_ratio']:.2f}, "
          f"climb {C['climb_at_cruise_mps']:.1f} m/s")
    print(f"  Best endurance {r['best_endurance']['v']} m/s ({r['best_endurance']['p']:.0f} W), best range {r['best_range']['v']} m/s ({r['best_range']['wh_per_km']:.2f} Wh/km)")
    print(f"  Stall {r['stall_mps']:.1f} m/s (CLmax {r['clmax']:.2f})")
    print(f"  Endurance {r['endurance_min']:.1f} min ({r['endurance_with_reserve_min']:.1f} with 20 % reserve), hover-only {r['hover_only_min']:.1f} min")
    print(f"  BOM aircraft ${bom['aircraft'][0]}-{bom['aircraft'][1]}, ground ${bom['ground'][0]}-{bom['ground'][1]}, total ${bom['total'][0]}-{bom['total'][1]}")
    for k, v in c.items():
        print(f"  [{'PASS' if v else 'FAIL'}] {k}")
    out = {"result": r, "checks": c, "bom": bom, "trade": trade_study(s)}
    (ROOT / "build" / "sizing.json").write_text(json.dumps(out, indent=2, default=float))
    return 0 if all(c.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
