"""ArduPilot ignores unknown parameter names without an error, so a renamed parameter silently does nothing.

These tests check every name in the parameter files against the ArduPlane 4.7.3 name list
(sitl/arduplane_4.7_param_names.txt, generated from ArduPilot's own parameter metadata), and check the
hardware overlay against ground/README.md and the board's serial order.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _params(path):
    out = {}
    for line in (ROOT / path).read_text().splitlines():
        f = line.split("#")[0].split()
        if f:
            out[f[0]] = float(f[1])
    return out


NAMES = {l.strip() for l in (ROOT / "sitl/arduplane_4.7_param_names.txt").read_text().splitlines()
         if l.strip() and not l.startswith("#")}


def test_every_name_exists():
    for f in ("sitl/kestrel.parm", "sitl/kestrel_hw.parm"):
        unknown = sorted(set(_params(f)) - NAMES)
        assert not unknown, f"{f}: not ArduPlane 4.7.3 parameters: {unknown}"


def test_hw_overlay_matches_ground_station():
    hw = _params("sitl/kestrel_hw.parm")
    assert hw["SERIAL1_PROTOCOL"] == 23          # ELRS on USART1 (CRSF does not work on the inverted SBUS pad)
    assert hw["SERIAL3_PROTOCOL"] == 5           # GPS on USART3
    assert hw["ARSPD_BUS"] == 0                  # the board has one I2C bus; ArduPilot's default 1 finds nothing
    assert hw["FLTMODE_CH"] == 5 and hw["RC7_OPTION"] == 153 and hw["ARMING_RUDDER"] == 0
    # the CH5 table in ground/README.md
    assert [hw[f"FLTMODE{i}"] for i in range(1, 7)] == [19, 18, 7, 10, 11, 20]
    assert hw["FS_LONG_ACTN"] == 1 and hw["THR_FAILSAFE"] == 1


def test_overlay_does_not_retune_the_flight_parameters():
    """The overlay may only add hardware settings; anything SITL tested stays in kestrel.parm."""
    base, hw = _params("sitl/kestrel.parm"), _params("sitl/kestrel_hw.parm")
    clash = {k for k in set(base) & set(hw) if base[k] != hw[k] and k != "ARSPD_TYPE"}
    assert not clash, clash


def test_flight_parameters_follow_the_sizing_model():
    import json
    import math
    from kestrel import spec as S
    s, base = S.load(), _params("sitl/kestrel.parm")
    assert base["THR_MAX"] == round(100 * s["propulsion"]["pusher_thr_max"])
    sz = json.loads((ROOT / "build/sizing.json").read_text())["result"]
    # lift motors must assist before the wing stalls at the bank limit (design review K5)
    v_stall_bank = sz["stall_mps"] / math.sqrt(math.cos(math.radians(base["ROLL_LIMIT_DEG"])))
    assert base["Q_ASSIST_SPEED"] >= v_stall_bank
    assert base["AIRSPEED_MIN"] > v_stall_bank
    assert abs(base["Q_M_THST_HOVER"] - sz["lift"]["hover_throttle"]) < 0.03


def test_static_margin_survives_the_cg_tolerance():
    from kestrel import spec as S
    s = S.load()
    mac = S.wing_geometry(s)["mac"]
    lim = s["limits"]
    assert lim["target_static_margin"] - lim["cg_tolerance_m"] / mac >= lim["static_margin"][0]
