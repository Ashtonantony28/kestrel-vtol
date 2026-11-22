"""M2 verification of the 6-DOF model: analytic cases, sign conventions, trims vs AeroSandbox/sizing, stability."""
import math

import numpy as np
import pytest

from kestrel import sizing, trim
from kestrel.sim import Vehicle, euler_from_q, mv, rot

DT = 1 / 1200


@pytest.fixture
def v():
    return Vehicle()


def test_rests_on_legs(v):
    for _ in range(2400):
        v.step(DT, [0] * 5, (0, 0))
    assert abs(v.vel[2]) < 1e-6 and not v.crashed
    assert abs(math.degrees(euler_from_q(v.q)[1])) < 0.5


def test_free_fall_analytic(v):
    v.aero_on = False
    v.ground_on = False
    v.reset(pos=(0, 0, -100.0))
    T = 2.0
    for _ in range(int(T / DT)):
        v.step(DT, [0] * 5, (0, 0))
    z = -100.0 + 0.5 * v.g * T**2
    assert v.pos[2] == pytest.approx(z, rel=1e-9)
    assert v.last_accel[2] == pytest.approx(0.0, abs=1e-12)   # free fall: accelerometer reads zero


def test_torque_free_conserves_momentum_and_energy(v):
    v.aero_on = False
    v.ground_on = False
    v.reset(pos=(0, 0, -1000.0), w=(3.0, 1.0, 2.0))
    I = np.array(v.I)

    def H_world():
        return np.array(mv(rot(v.q), tuple(I @ np.array(v.w))))
    H0, E0 = H_world(), 0.5 * np.array(v.w) @ I @ np.array(v.w)
    for _ in range(int(3.0 / DT)):
        v.step(DT, [0] * 5, (0, 0))
    E1 = 0.5 * np.array(v.w) @ I @ np.array(v.w)
    assert np.linalg.norm(H_world() - H0) / np.linalg.norm(H0) < 1e-5
    assert abs(E1 - E0) / E0 < 1e-5


def test_hover_force_balance_matches_sizing(v):
    r = sizing.analyse()
    duty = r["lift"]["hover_throttle"]
    # sizing uses the nominal cell voltage; set the sim's pack to the same open-circuit voltage
    v.set_cell_voltage(v.batt.v_cell_nom)
    rotor, vp = trim.steady_rotor(v, [duty] * 4 + [0])
    d = v.derivs((0, 0, -50.0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0), rotor, [0, 0], [duty] * 4 + [0], vp)
    thrust = -d[5][2] * v.m
    assert thrust == pytest.approx(v.m * v.g, rel=0.05)   # within 5 % (sim adds rear-rotor download)
    assert rotor[0] * 60 / (2 * math.pi) == pytest.approx(r["lift"]["hover_rpm"], rel=0.05)


def test_yaw_torque_sign(v):
    # ArduPilot X frame: motors 1,2 CCW. Speeding them up must yaw the body right (+Mz).
    base = [0.5] * 4 + [0]
    rotor, vp = trim.steady_rotor(v, base)
    hi = list(rotor)
    hi[0] *= 1.05
    hi[1] *= 1.05
    d = v.derivs((0, 0, -50.0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0), hi, [0, 0], base, vp)
    assert d[3][2] > 0


def test_yaw_tilt_adds_yaw_authority_without_side_force(v):
    # The 4 deg motor tilt must add yaw torque of the same sign as prop drag (>2x total), cancel side force
    # and leave hover lift within 0.5 %.
    base = [0.5] * 4 + [0]
    rotor, vp = trim.steady_rotor(v, base)
    hi = list(rotor)
    hi[0] *= 1.05
    hi[1] *= 1.05
    args = ((0, 0, -50.0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0))
    yaw_tilted = v.derivs(*args, hi, [0, 0], base, vp)[3][2]
    side_tilted = v.derivs(*args, rotor, [0, 0], base, vp)[1][1]
    lift_tilted = v.derivs(*args, rotor, [0, 0], base, vp)[1][2]
    c, s_ = v.tilt_c, v.tilt_s
    v.tilt_c, v.tilt_s = 1.0, 0.0
    try:
        yaw_flat = v.derivs(*args, hi, [0, 0], base, vp)[3][2]
        lift_flat = v.derivs(*args, rotor, [0, 0], base, vp)[1][2]
    finally:
        v.tilt_c, v.tilt_s = c, s_
    assert yaw_flat > 0 and yaw_tilted > 2.0 * yaw_flat
    assert abs(side_tilted) < 1e-6
    assert abs((v.g - lift_tilted) / (v.g - lift_flat) - 1) < 0.005   # a_z = g - T/m (NED)


def test_pitch_roll_control_signs(v):
    V = 13.0
    up = trim.eval_state(v, V, 0.08, 0.0, -0.1)[3][1]
    dn = trim.eval_state(v, V, 0.08, 0.0, +0.1)[3][1]
    assert up > dn                          # trailing edges up -> nose up
    v.elevon = [0.1, -0.1]                  # left TE down, right TE up
    from kestrel.sim import q_from_euler
    d = v.derivs((0, 0, -100.0), (V, 0, 0), q_from_euler(0, 0.08, 0), (0, 0, 0), [0] * 5, v.elevon, [0] * 5, v.v_oc())
    assert d[3][0] > 0                      # roll right


def test_glide_trim_matches_aero_tables(v):
    import json
    from kestrel import spec as S
    t = json.loads((S.ROOT / "aero" / "tables.json").read_text())
    V = 12.0
    tr = trim.trim_glide(v, V)
    assert tr["ok"]
    # glide ratio equals the TRIMMED L/D of the airframe at the trim CL (aero tables, elevon set for Cm = 0)
    q = 0.5 * v.rho * V**2
    cl = v.m * v.g * math.cos(tr["gamma"]) / (q * v.S)
    cd = np.interp(cl, t["polar"]["cl"], t["polar"]["cd"])
    assert tr["glide_ratio"] == pytest.approx(cl / cd, rel=0.08)   # sim: linearised elevon + CLmax knockdown
    assert abs(tr["de"]) < math.radians(8)                        # trims with small elevon, far from the 20 deg limit


def test_glide_is_dynamically_stable_in_pitch(v):
    V = 12.0
    tr = trim.trim_glide(v, V)
    from kestrel.sim import q_from_euler
    theta = tr["alpha"] + tr["gamma"]
    v.ground_on = False
    v.reset(pos=(0, 0, -500.0), vel=(V * math.cos(tr["gamma"]), 0, -V * math.sin(tr["gamma"])),
            q=q_from_euler(0, theta + math.radians(3), 0))
    pitch = []
    for i in range(int(20 / DT)):
        v.step(DT, [0] * 5, (tr["de"], tr["de"]))
        pitch.append(euler_from_q(v.q)[1])
    p = np.degrees(np.array(pitch) - theta)
    first, last = np.abs(p[: int(2 / DT)]).max(), np.abs(p[-int(4 / DT):]).max()
    assert last < first                      # oscillation decays (short period + phugoid not divergent in 20 s)
    assert not v.crashed


def test_level_cruise_matches_sizing(v):
    r = sizing.analyse()
    v.set_cell_voltage(v.batt.v_cell_nom)    # nominal voltage, like sizing
    tr = trim.trim_level(v, r["cruise"]["v_mps"])
    assert tr["ok"]
    elec = tr["power"] + 2.5
    assert elec == pytest.approx(r["cruise"]["power_w"], rel=0.15)
    assert 0.3 < tr["duty"] < 0.9


def test_rotor_time_constant(v):
    v.reset(pos=(0, 0, -50))
    v.ground_on = False
    rotor, _ = trim.steady_rotor(v, [0.3] * 4 + [0])
    hi, _ = trim.steady_rotor(v, [0.6] * 4 + [0])
    v.rotor = list(rotor)
    target = rotor[0] + 0.632 * (hi[0] - rotor[0])
    t = 0.0
    while v.rotor[0] < target and t < 1.0:
        v.step(DT, [0.6] * 4 + [0], (0, 0))
        t += DT
    assert 0.015 < t < 0.08


def test_runs_faster_than_real_time(v):
    import time
    v.reset(pos=(0, 0, -50))
    t0 = time.time()
    for _ in range(1200):
        v.step(DT, [0.5] * 4 + [0.5], (0, 0))
    assert time.time() - t0 < 0.25           # > 4x real time at 1.2 kHz
