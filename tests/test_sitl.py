"""ArduPlane SITL flying the Kestrel physics (M3). Slow (~1-2 min each), so opt-in:

    KESTREL_SITL=1 pytest -q tests/test_sitl.py

Needs ArduPlane built for SITL (README). The full robustness suites are `python -m kestrel.mc`.
"""
import os

import pytest

from kestrel import fly

pytestmark = pytest.mark.skipif(
    os.environ.get("KESTREL_SITL") != "1" or not (fly.ARDUPILOT / "build/sitl/bin/arduplane").exists(),
    reason="set KESTREL_SITL=1 with ArduPlane SITL built",
)


def _check(r, max_land_err=2.0):
    assert not r.get("error"), r.get("error")
    assert r["completed"], r.get("statustext", [])[-5:]
    assert not r["physics"]["crashed"], r["physics"]["crash_reason"]
    assert r["land_error_m"] < max_land_err
    assert r["physics"]["max_touch_vz"] < 1.5          # gentle touchdown (m/s)


def test_hover_mission():
    r = fly.Run("hover", instance=7, name="pytest_hover", verbose=False).run()
    _check(r, 1.0)
    assert r["physics"]["max_tilt_deg"] < 15


def test_basic_mission_transitions_and_lands():
    r = fly.Run("basic", instance=8, name="pytest_basic", verbose=False).run()
    _check(r)
    msgs = " ".join(s for _, s in r["statustext"])
    assert "Transition done" in msgs
    assert r["wp_reached"] >= 5
