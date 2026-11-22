"""Trim and linear-analysis helpers on the 6-DOF model (used by tests and the design review)."""
from __future__ import annotations

import math

import numpy as np
from scipy.optimize import fsolve

from .sim import Vehicle, q_from_euler


def steady_rotor(v: Vehicle, duty, vb_air=(0.0, 0.0, 0.0), iters=4000, dt=1e-3):
    """Spin rotors to steady state for given duty with the body held fixed (no integration of body states)."""
    rotor = [0.0] * 5
    pos, vel, q, w = (0, 0, -50.0), vb_air, (1.0, 0, 0, 0), (0, 0, 0)
    v_pack = v.v_oc() - v.i_batt * v.r_int
    for _ in range(iters):
        d = v.derivs(pos, vel, q, w, rotor, v.elevon, duty, v_pack)
        rotor = [max(0.0, rotor[i] + dt * d[4][i]) for i in range(5)]
        v_pack = v.v_oc() - d[6] * v.r_int
    return rotor, v_pack


def eval_state(v: Vehicle, V, alpha, gamma, de, duty_pusher=0.0, duty_lift=0.0, rotor=None, v_pack=None):
    theta = alpha + gamma
    q = q_from_euler(0.0, theta, 0.0)
    vel = (V * math.cos(gamma), 0.0, -V * math.sin(gamma))
    duty = [duty_lift] * 4 + [duty_pusher]
    if rotor is None:
        rotor = [0.0] * 4 + [0.0]
    v.elevon = [de, de]
    vp = v_pack if v_pack is not None else v.v_oc()
    d = v.derivs((0, 0, -100.0), vel, q, (0, 0, 0), rotor, v.elevon, duty, vp)
    return d


def trim_glide(v: Vehicle, V: float):
    """Unpowered glide: solve alpha, gamma, de. Returns dict."""
    def f(x):
        a, g, de = x
        d = eval_state(v, V, a, g, de)
        acc, wd = d[1], d[3]
        return [acc[0], acc[2], wd[1]]
    sol, info, ier, msg = fsolve(f, [0.08, -0.12, -0.08], full_output=True)
    a, g, de = sol
    return {"alpha": a, "gamma": g, "de": de, "ok": ier == 1, "glide_ratio": -1 / math.tan(g), "resid": f(sol)}


def trim_level(v: Vehicle, V: float):
    """Level cruise with pusher: solve alpha, de, pusher rotor speed (via duty with steady rotor)."""
    def state(x):
        a, de, duty = x
        theta = a
        q = q_from_euler(0, theta, 0)
        vel = (V, 0, 0)
        v.elevon = [de, de]
        # steady pusher speed for this duty at this inflow
        rot = [0.0] * 5
        vb = (V * math.cos(a), 0, V * math.sin(a))
        vp = v.v_oc()
        for _ in range(1500):
            d = v.derivs((0, 0, -100.0), vel, q, (0, 0, 0), rot, v.elevon, [0, 0, 0, 0, duty], vp)
            rot[4] = max(0.0, rot[4] + 1e-3 * d[4][4])
            vp = v.v_oc() - d[6] * v.r_int
        return d, rot, vp

    def f(x):
        d, _, _ = state(x)
        return [d[1][0], d[1][2], d[3][1]]
    sol, info, ier, msg = fsolve(f, [0.09, -0.1, 0.65], full_output=True, xtol=1e-6)
    d, rot, vp = state(sol)
    return {"alpha": sol[0], "de": sol[1], "duty": sol[2], "ok": ier == 1, "current": d[6], "v_pack": vp,
            "power": d[6] * vp, "pusher_rpm": rot[4] * 60 / (2 * math.pi), "resid": f(sol)}
