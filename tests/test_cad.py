"""M1 CAD: clearances, balance, mass budget, foam templates (builds the model; ~5 s)."""
import json

import pytest

from kestrel import cad
from kestrel import spec as S


@pytest.fixture(scope="module")
def built():
    s = S.load()
    parts, rotors = cad.build(s)
    return s, parts, cad.clearance_checks(s, parts), cad.mass_properties(s, parts)


def test_no_prop_collisions(built):
    assert built[2]["prop_collisions"] == []


def test_ground_and_wing_clearances(built):
    c = built[2]
    assert c["pusher_tip_clearance_m"] >= 0.015
    assert c["pod_clearance_m"] >= 0.02
    assert c["pusher_wing_distance_m"] >= 0.03
    assert max(c["disc_wing_overlap_frac"].values()) < 0.10     # hover download under ~5 %
    assert c["battery_in_pod"]


def test_mass_within_budget(built):
    s, _, _, mp = built
    budget = sum(v["m"] for v in s["mass_items"].values())
    assert abs(mp["mass"] / budget - 1) < 0.05


def test_balanced_cg_and_rotor_centroid(built):
    s, _, _, mp = built
    t = json.loads((S.ROOT / "aero" / "tables.json").read_text())
    cg_tab = -t["ref"]["cg_asb"][0]
    assert mp["cg"][0] == pytest.approx(cg_tab, abs=0.002)      # aero tables computed about this CG
    lr = s["lift_rotors"]
    assert 0.5 * (lr["x_front"] + lr["x_rear"]) == pytest.approx(mp["cg"][0], abs=0.003)


def test_foam_templates_fit_five_sheets():
    sheets = cad.nest(cad.foam_parts(S.load()))
    assert len(sheets) <= 5
    assert all(sh["fits"] and not sh["overlap"] for sh in sheets)
