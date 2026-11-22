"""Close the design loop: place the lift rotors on the CG and slide the battery so the minimum local static
margin equals the target. Iterates CAD mass properties <-> AeroSandbox until stations move < 0.5 mm, then
writes the stations back into spec/vehicle.yaml (the single source of truth).

Run: python -m kestrel.balance
"""
from __future__ import annotations

import json
import re

from . import spec as S

ROOT = S.ROOT


def _set(text, block, key, value):
    pat = re.compile(r"(^%s:.*?\n(?:[ ].*\n)*?\s+%s:\s*)(-?[0-9.]+)" % (block, key), re.M)
    new, n = pat.subn(lambda m: m.group(1) + f"{value:.3f}", text, count=1)
    assert n == 1, (block, key)
    return new


def _set_item_x(text, item, value):
    pat = re.compile(r"(\b%s:\s*\{m:\s*[0-9.]+,\s*x:\s*)(-?[0-9.]+)" % item)
    new, n = pat.subn(lambda m: m.group(1) + f"{value:.3f}", text, count=1)
    assert n == 1, item
    return new


def run(max_iter=8, verbose=True):
    from . import aero, cad
    for it in range(max_iter):
        S.load.cache_clear()
        s = S.load()
        parts, _ = cad.build(s)
        mp = cad.mass_properties(s, parts)
        (ROOT / "cad" / "mass_props.json").write_text(json.dumps(mp, indent=1))
        t = aero.generate(s)
        mac = S.wing_geometry(s)["mac"]
        cg_tab = -t["ref"]["cg_asb"][0]
        target = cg_tab + (s["limits"]["target_static_margin"] - t["static_margin_min"]) * mac
        half = s["booms"]["half_spacing_rotors"]
        lr = s["lift_rotors"]
        m_b = s["batteries"][s["propulsion"]["battery"]]["mass"]
        # moving the 4 lift units (motor+prop+mount+leg+esc ~ 0.104 kg each) shifts CG too: solve both together
        m_unit = 4 * (s["motors"][s["propulsion"]["lift_motor"]]["mass"] + s["props"][s["propulsion"]["lift_prop"]]["mass"] + 0.010 + 0.005 + 0.025)
        m_boom = mp["breakdown"].get("boom_L", 0) * 2
        M = mp["mass"]
        c_now = 0.5 * (lr["x_front"] + lr["x_rear"])
        d_rot = target - c_now
        cg_after_rot = mp["cg"][0] + d_rot * (m_unit + m_boom) / M
        bx_new = s["mass_items"]["battery"]["x"] + (target - cg_after_rot) * M / m_b
        xf, xr = target + half, target - half
        moved = max(abs(xf - lr["x_front"]), abs(bx_new - s["mass_items"]["battery"]["x"]))
        if verbose:
            print(f"iter {it}: cg {mp['cg'][0]*1000:.1f} mm, target {target*1000:.1f} mm, SMmin {t['static_margin_min']*100:.1f} %, "
                  f"rotors {xf*1000:.0f}/{xr*1000:.0f} mm, battery {bx_new*1000:.0f} mm")
        txt = S.SPEC_PATH.read_text()
        txt = _set(txt, "lift_rotors", "x_front", xf)
        txt = _set(txt, "lift_rotors", "x_rear", xr)
        txt = _set(txt, "booms", "x_front", xf + 0.035)
        txt = _set_item_x(txt, "battery", bx_new)
        for item in ("lift_motors", "lift_props", "lift_escs", "printed_mounts", "landing_legs"):
            txt = _set_item_x(txt, item, target)
        txt = _set_item_x(txt, "booms", target - (s["booms"]["length"] / 2 - half - 0.035))
        S.SPEC_PATH.write_text(txt)
        if moved < 5e-4:
            break
    S.load.cache_clear()
    return t


if __name__ == "__main__":
    run()
