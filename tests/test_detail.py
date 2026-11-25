"""Detailed assembly: interference audit (ADR 0006) and the dimension table. Builds the 67-part model (~2 min)."""
import pytest

from kestrel import cad_detail as D
from kestrel import dimensions as DIM
from kestrel import interference as I
from kestrel import spec as S


@pytest.fixture(scope="module")
def built():
    s = S.load()
    return s, D.build_detail(s)


@pytest.fixture(scope="module")
def audit(built):
    return I.audit(*built)


def test_all_solids_valid(audit):
    assert audit["invalid"] == []


def test_no_unexplained_overlaps(audit):
    assert audit["conflicts"] == []


def test_props_and_elevons_swept_clear(audit):
    bad = [x for x in audit["swept"] if not x["ok"]]
    assert bad == []
    assert min(x["gap_mm"] for x in audit["swept"] if "gap_mm" in x) >= I.PROP_MIN_GAP * 1000


def test_legs_are_the_lowest_parts(audit):
    assert audit["gaps"]["lowest_non_leg_part_to_ground_mm"] > 20
    assert audit["gaps"]["pusher_prop_to_ground_mm"] > 20


def test_dimension_table_matches_spec(built):
    s, P = built
    d = DIM.compute(s, P)
    rows = {r["name"]: r["value"] for r in d["table"]}
    span = next(v for k, v in rows.items() if k.startswith("Wingspan"))
    assert float(span) == pytest.approx(s["wing"]["span"] * 1000, abs=0.5)


def test_printed_masses_match_exported_volumes():
    """Mass model vs the exported printed parts at 50 % of PETG density (design review: BOM said ~130 g)."""
    import json
    from kestrel import cad, spec as S
    man = S.ROOT / "cad" / "build" / "manifest.json"
    if not man.exists():
        pytest.skip("run python -m kestrel.export_build first")
    v = {k: d["volume_cm3"] for k, d in json.loads(man.read_text())["printed"].items()}
    g = lambda cm3: cm3 * 1.27 * 0.5 / 1000
    s = S.load()
    for model, vol in ((cad.MOUNT_MASS, v["motor_mount_right_x2"]), (cad.CLAMP_MASS, v["boom_clamp_x2"]),
                       (cad.PYLON_MASS, v["pusher_pylon_x1"]), (s["landing_legs"]["mass_each"], v["leg_x4"])):
        assert model == pytest.approx(g(vol), rel=0.2)
