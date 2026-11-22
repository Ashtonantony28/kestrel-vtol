"""Motor/prop model reproduces the published A2212/13T 1000KV + 10x4.5 point and behaves physically."""
import pytest

from kestrel import propulsion as P


def test_datasheet_point():
    m, p = P.get("motor", "a2212_1000kv"), P.get("prop", "prop_10x45")
    op = P.operating_point(m, p, 11.1)
    assert op.rpm == pytest.approx(7600, rel=0.05)
    assert op.current == pytest.approx(11.5, rel=0.05)
    assert op.thrust / 9.80665 * 1000 == pytest.approx(840, rel=0.05)


@pytest.mark.parametrize("prop", ["prop_10x45", "prop_9x45", "prop_9x6", "prop_8x6"])
def test_figure_of_merit_physical(prop):
    p = P.get("prop", prop)
    fom = p.ct0 ** 1.5 / (2 ** 0.5 * p.cp0)
    assert 0.4 < fom < 0.8


def test_thrust_monotonic_in_throttle_and_drops_with_inflow():
    m, p = P.get("motor", "a2212_1000kv"), P.get("prop", "prop_10x45")
    b = P.get("battery", "lipo_3s_2200")
    t = [P.solve_with_battery(m, p, b, d, 4)[0].thrust for d in (0.2, 0.4, 0.6, 0.8, 1.0)]
    assert all(b2 > a2 for a2, b2 in zip(t, t[1:]))
    static = P.operating_point(m, p, 10.0).thrust
    moving = P.operating_point(m, p, 10.0, airspeed=8.0).thrust
    assert moving < static
