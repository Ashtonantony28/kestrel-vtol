"""M1 aerodynamics: stability, trim and table sanity (reads aero/tables.json produced by kestrel.aero)."""
import json

import numpy as np
import pytest

from kestrel import spec as S

T = json.loads((S.ROOT / "aero" / "tables.json").read_text())
LIM = S.load()["limits"]


def test_minimum_local_static_margin_in_band():
    lo, hi = LIM["static_margin"]
    assert lo <= T["static_margin_min"] <= hi


def test_trims_at_cruise_with_small_elevon():
    assert abs(T["trim"]["elevon_deg"]) < 5.0
    assert 2.0 < T["trim"]["alpha_deg"] < T["alpha_stall_deg"] - 6.0


def test_trimmed_stall_speed():
    mp = S.mass_properties()
    w = mp["mass"] * 9.80665
    vs = (2 * w / (1.225 * T["ref"]["area"] * T["clmax"])) ** 0.5
    assert vs <= LIM["max_stall_mps"]


def test_derivative_signs():
    d = T["derivatives"]
    assert d["Cma"] < 0 and d["Cmq"] < 0          # pitch stiffness and damping
    assert d["Cnb"] > 0                             # weathercock stability from winglets
    assert d["Clp"] < 0 and d["Cnr"] < 0            # roll and yaw damping
    assert d["Cmde"] < 0 and d["Clda"] > 0          # control signs (TE down = nose down; left TE down = roll right)


def test_tables_cover_full_circle_and_are_continuous():
    a = np.array(T["alpha_deg"])
    assert a[0] == -180 and a[-1] == 180
    for de in T["de_grid_deg"]:
        for k in ("CL", "CD", "Cm"):
            v = np.array(T["full"][str(de)][k])
            assert np.all(np.isfinite(v))
            assert np.max(np.abs(np.diff(v))) < 0.25      # no jumps between 2-deg points
        assert min(T["full"][str(de)]["CD"]) > 0
    i90 = list(a).index(90)
    assert T["CD"][i90] == pytest.approx(T["cd90"], rel=0.1)   # broadside flat plate
