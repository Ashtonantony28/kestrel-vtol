"""Motor/prop/battery models (steady state). Used by sizing and by the 6-DOF sim.

Prop: T = ct(J) rho n^2 D^4, P = cp(J) rho n^3 D^5, J = V/(n D); ct, cp linear in J (fit to UIUC data shape).
Motor: brushed-equivalent DC model of a BLDC: I = (V_m - w/Kv)/Rm, Q = (I - I0)/Kv  (Kv in rad/s/V).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from scipy.optimize import brentq

from . import spec as _spec

RPM_PER_RADS = 60.0 / (2 * math.pi)


@dataclass(frozen=True)
class Prop:
    d: float
    ct0: float
    cp0: float
    j0_ct: float
    j0_cp: float
    mass: float = 0.0

    def ct(self, j: float) -> float:
        return self.ct0 * (1.0 - j / self.j0_ct)

    def cp(self, j: float) -> float:
        return max(self.cp0 * (1.0 - (j / self.j0_cp) ** 2), 0.05 * self.cp0)


@dataclass(frozen=True)
class Motor:
    kv: float      # rpm/V
    rm: float
    i0: float
    mass: float = 0.0
    i_max_cont: float = 1e9

    @property
    def kv_rad(self) -> float:
        return self.kv / RPM_PER_RADS


@dataclass(frozen=True)
class Battery:
    cells: int
    capacity_ah: float
    r_int: float
    mass: float
    usable_frac: float
    v_cell_nom: float
    v_cell_full: float
    v_cell_empty: float
    c_rating: float = 30.0     # continuous C rating; the pack must cover peak draw (validation V4c)

    @property
    def v_nom(self) -> float:
        return self.cells * self.v_cell_nom

    @property
    def energy_wh(self) -> float:
        return self.v_nom * self.capacity_ah


def get(kind: str, name: str, s: dict | None = None, raw: bool = False, role: str = "lift"):
    """raw=True returns a prop exactly as tabulated (it reproduces the vendor datasheet point). Otherwise the
    power coefficient is scaled by spec prop_derate.<role>_cp_scale, so a lift prop's static figure of merit
    matches cheap moulded props on a thrust stand rather than the vendor's optimistic figure (review K3)."""
    s = s or _spec.load()
    table = {"motor": ("motors", Motor), "prop": ("props", Prop), "battery": ("batteries", Battery)}[kind]
    row = dict(s[table[0]][name])
    if kind == "prop" and not raw:
        row["cp0"] *= s.get("prop_derate", {}).get(f"{role}_cp_scale", 1.0)
    return table[1](**row)


def figure_of_merit(prop: "Prop") -> float:
    """Static figure of merit from propeller (rev/s) coefficients: ct^1.5 / (cp sqrt(pi/2))."""
    return prop.ct0 ** 1.5 / (prop.cp0 * math.sqrt(math.pi / 2))


@dataclass
class OpPoint:
    throttle: float
    rpm: float
    thrust: float
    torque: float
    current: float
    p_elec: float
    p_shaft: float
    p_thrust: float

    @property
    def eff_total(self) -> float:
        return self.p_thrust / self.p_elec if self.p_elec > 0 else 0.0


def operating_point(motor: Motor, prop: Prop, v_motor: float, airspeed: float = 0.0, rho: float = 1.225) -> OpPoint:
    """Steady state for an applied (average) motor voltage v_motor and axial inflow airspeed."""
    kvr = motor.kv_rad

    def torque_balance(w: float) -> float:
        n = w / (2 * math.pi)
        i = (v_motor - w / kvr) / motor.rm
        qm = (i - motor.i0) / kvr
        j = airspeed / (n * prop.d) if n > 1e-6 else 0.0
        qp = prop.cp(j) * rho * n**2 * prop.d**5 / (2 * math.pi)
        return qm - qp

    w_max = v_motor * kvr
    if v_motor <= motor.i0 * motor.rm + 1e-6:
        return OpPoint(0, 0, 0, 0, 0, 0, 0, 0)
    w = brentq(torque_balance, 1e-3, w_max * 0.999999)
    n = w / (2 * math.pi)
    j = airspeed / (n * prop.d)
    i = (v_motor - w / kvr) / motor.rm
    t = prop.ct(j) * rho * n**2 * prop.d**4
    q = prop.cp(j) * rho * n**2 * prop.d**5 / (2 * math.pi)
    return OpPoint(throttle=float("nan"), rpm=w * RPM_PER_RADS, thrust=t, torque=q, current=i,
                   p_elec=v_motor * i, p_shaft=q * w, p_thrust=max(t * airspeed, 0.0))


def solve_with_battery(motor: Motor, prop: Prop, batt: Battery, throttle: float, n_motors_on_pack: float,
                       other_current: float = 0.0, airspeed: float = 0.0, soc_v_cell: float | None = None,
                       rho: float = 1.225) -> tuple[OpPoint, float]:
    """Throttle (0..1 duty) with pack sag from all motors drawing the same current. Returns (op, v_pack)."""
    v_oc = batt.cells * (soc_v_cell if soc_v_cell is not None else batt.v_cell_nom)
    v_pack = v_oc
    for _ in range(50):
        op = operating_point(motor, prop, throttle * v_pack, airspeed, rho)
        i_pack = n_motors_on_pack * op.current * throttle + other_current  # ESC: I_batt = duty * I_motor
        v_new = v_oc - i_pack * batt.r_int
        if abs(v_new - v_pack) < 1e-6:
            break
        v_pack = 0.5 * (v_pack + v_new)
    op.throttle = throttle
    return op, v_pack


def throttle_for_thrust(motor: Motor, prop: Prop, batt: Battery, thrust: float, n_motors_on_pack: float,
                        other_current: float = 0.0, airspeed: float = 0.0, soc_v_cell: float | None = None,
                        rho: float = 1.225) -> tuple[OpPoint, float]:
    def f(d):
        return solve_with_battery(motor, prop, batt, d, n_motors_on_pack, other_current, airspeed, soc_v_cell, rho)[0].thrust - thrust
    if f(1.0) < 0:
        raise ValueError("thrust not reachable at full throttle")
    d = brentq(f, 0.05, 1.0, xtol=1e-6)
    return solve_with_battery(motor, prop, batt, d, n_motors_on_pack, other_current, airspeed, soc_v_cell, rho)
