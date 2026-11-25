"""M2: 6-DOF quadplane flight dynamics model (FRD body, NED world, quaternion body->NED, scalar first).

Pure-Python scalar maths (no numpy in the hot path) so it runs several times faster than real time when it
drives ArduPilot SITL through the JSON backend (kestrel/sitl_bridge.py).

Model content
- Rigid body: mass and full inertia from CAD (cad/mass_props.json), forces/moments about the CG.
- 4 lift rotors + pusher: motor speed ODE (DC motor model, effective resistance), prop thrust/torque with
  axial inflow (advance ratio), reaction torque, in-plane rotor drag, hover download on rear rotors (wing overlap).
- Battery: open-circuit voltage vs state of charge, internal resistance sag, coulomb counting.
- Aero: AeroSandbox tables over +-180 deg alpha (flat plate past stall), sideslip/rate/control derivatives,
  low-speed rotational damping, flat-plate side force at large sideslip.
- Elevon servos: first-order lag + rate limit.
- Ground: spring-damper legs with friction; crash flag if anything but a leg touches or touchdown > 3 m/s.
- Wind: steady wind + Dryden-like first-order gust filter (seeded), in NED.
"""
from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass, field

from . import propulsion as P
from . import spec as S

ROOT = S.ROOT


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def rot(q):
    """Rotation matrix body->NED as nested tuples."""
    w, x, y, z = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def mv(R, v):
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


def mtv(R, v):
    return (R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
            R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
            R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def euler_from_q(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def q_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


@dataclass
class Dispersion:
    """Multiplicative/additive model errors for robustness and Monte Carlo (1.0 / 0.0 = nominal)."""
    mass: float = 1.0
    cg_dx: float = 0.0           # m, CG shift (FRD x)
    thrust: tuple = (1.0, 1.0, 1.0, 1.0, 1.0)   # per motor (4 lift + pusher)
    drag: float = 1.0
    lift: float = 1.0
    cm0: float = 0.0             # Cm offset (foam build asymmetry / warp)
    cl0_roll: float = 0.0        # rolling-moment offset (warped wing)
    batt_capacity: float = 1.0
    batt_r: float = 1.0
    motor_fail: dict = field(default_factory=dict)   # {index: time_s} motor stops at time
    servo_fail: dict = field(default_factory=dict)   # {index: (time_s, stuck_rad)}


class Vehicle:
    LIFT_DIR = (+1, +1, -1, -1)  # yaw reaction sign per ArduPilot motor 1..4 (1,2 CCW -> +yaw torque; 3,4 CW)

    def __init__(self, spec=None, tables=None, mass_props=None, disp: Dispersion | None = None,
                 wind_ned=(0.0, 0.0, 0.0), gust_sigma=0.0, gust_tau=2.0, seed=0, home_alt=0.0):
        s = self.s = spec or S.load()
        t = self.t = tables or json.loads((ROOT / "aero" / "tables.json").read_text())
        mp = mass_props or S.mass_properties(s)
        d = self.d = disp or Dispersion()
        env = s["environment"]
        self.rho, self.g = env["rho"], env["g"]
        self.m = mp["mass"] * d.mass
        cg = mp["cg"]
        self.cg = (cg[0] + d.cg_dx, cg[1], cg[2])
        I = mp["inertia"]
        self.I = tuple(tuple(r) for r in I)
        self.Iinv = self._inv3(self.I)
        g = S.wing_geometry(s)
        self.S, self.b, self.c = t["ref"]["area"], t["ref"]["span"], t["ref"]["mac"]
        self.der = t["derivatives"]
        self.a0, self.da = t["alpha_deg"][0], t["alpha_deg"][1] - t["alpha_deg"][0]
        self.de_grid = [math.radians(x) for x in t["de_grid_deg"]]
        self.tabs = [(t["full"][str(x)]["CL"], t["full"][str(x)]["CD"], t["full"][str(x)]["Cm"]) for x in t["de_grid_deg"]]
        self.CLt, self.CDt, self.Cmt = t["CL"], t["CD"], t["Cm"]
        self.alpha_stall = math.radians(t["alpha_stall_deg"])
        # CG shift changes Cm: dCm = CN * dx / c (dx forward positive => nose-down)
        self.dx_cg_cm = d.cg_dx / self.c
        # propulsion
        pr = s["propulsion"]
        self.lm = P.get("motor", pr["lift_motor"], s)
        self.lp = P.get("prop", pr["lift_prop"], s)
        self.pm = P.get("motor", pr["pusher_motor"], s)
        self.pp = P.get("prop", pr["pusher_prop"], s, role="pusher")
        self.batt = P.get("battery", pr["battery"], s)
        self.cap_c = self.batt.capacity_ah * 3600 * d.batt_capacity
        self.r_int = self.batt.r_int * d.batt_r
        lr = s["lift_rotors"]
        x0, y0, z0 = self.cg
        zr = lr.get("z", -0.035)
        tau = math.radians(lr.get("yaw_tilt_deg", 0.0))
        self.tilt_c, self.tilt_s = math.cos(tau), math.sin(tau)
        # ArduPilot QuadPlane X numbering: 1 FR, 2 RL, 3 FL, 4 RR
        self.rotor_pos = ((lr["x_front"] - x0, lr["y"] - y0, zr - z0), (lr["x_rear"] - x0, -lr["y"] - y0, zr - z0),
                          (lr["x_front"] - x0, -lr["y"] - y0, zr - z0), (lr["x_rear"] - x0, lr["y"] - y0, zr - z0))
        try:
            ov = json.loads((ROOT / "cad" / "checks.json").read_text())["disc_wing_overlap_frac"]
            self.download = (1 - 0.5 * ov["FR"], 1 - 0.5 * ov["RL"], 1 - 0.5 * ov["FL"], 1 - 0.5 * ov["RR"])
        except Exception:
            self.download = (1.0, 0.965, 1.0, 0.965)
        pu = s["pusher"]
        self.pusher_pos = (pu["x"] - x0, pu.get("y", 0.0) - y0, pu.get("z", 0.0) - z0)
        self.j_rotor = 2.2e-5     # kg m^2, 10" prop + A2212 bell (estimate)
        self.j_pusher = 1.8e-5
        self.k_hforce = 3.6e-4    # N per (rev/s * m/s): rotor in-plane drag
        # contact points (FRD, relative to CG): 4 leg tips + crash points
        lg_z = s["booms"].get("z", 0.012) + s["landing_legs"]["height"]
        self.legs = tuple((p[0], p[1], lg_z - z0) for p in self.rotor_pos)
        w = s["wing"]
        b2 = w["span"] / 2
        tsw = math.tan(math.radians(w["le_sweep_deg"]))
        f = s["fuselage"]
        self.crash_pts = ((f["nose_x"] - x0, 0.0, 0.05 - z0), (-b2 * tsw - x0, b2, 0.0 - z0), (-b2 * tsw - x0, -b2, 0.0 - z0),
                          (pu["x"] - x0, 0.0, pu.get("z", 0.0) + self.pp.d / 2 - z0))
        self.k_leg, self.c_leg, self.mu = 2500.0, 30.0, 0.6
        # wind
        self.wind0 = tuple(wind_ned)
        self.gust_sigma, self.gust_tau = gust_sigma, gust_tau
        self.rng = random.Random(seed)
        self.gust = [0.0, 0.0, 0.0]
        self.home_alt = home_alt
        self.aero_on = True
        self.ground_on = True
        self.reset()

    @staticmethod
    def _inv3(m):
        a, b, c = m[0]; d, e, f = m[1]; g, h, i = m[2]
        det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
        return ((
            (e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det),
            ((f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det),
            ((d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det))

    def reset(self, pos=(0.0, 0.0, None), vel=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0), w=(0.0, 0.0, 0.0), yaw=0.0):
        z0 = -max(l[2] for l in self.legs) + 0.0 if pos[2] is None else pos[2]
        if pos[2] is None:
            # resting on legs: static compression m g / (4 k)
            z0 += self.m * self.g / (4 * self.k_leg)
        if yaw:
            q = q_from_euler(0.0, 0.0, yaw)
        self.pos = (pos[0], pos[1], z0)
        self.vel = tuple(vel)
        self.q = tuple(q)
        self.w = tuple(w)
        self.rotor = [0.0] * 5
        self.elevon = [0.0, 0.0]
        self.soc_c = 0.0
        self.i_batt = 0.0
        self.v_batt = self.batt.cells * self.batt.v_cell_full
        self.time = 0.0
        self.crashed = False
        self.crash_reason = ""
        self.max_touch_vz = 0.0
        self.on_ground = True
        self.last_accel = (0.0, 0.0, -self.g)
        self.energy_wh = 0.0

    # ---------------- sub-models ----------------
    def v_oc(self):
        frac = max(0.0, 1.0 - self.soc_c / self.cap_c)
        b = self.batt
        # simple LiPo curve: steep at ends, near-linear plateau
        if frac > 0:
            v_cell = b.v_cell_empty + (b.v_cell_full - b.v_cell_empty) * (0.15 * frac + 0.85 * frac ** 0.6)
        else:
            # past rated capacity the pack collapses (and the ESCs cut out): 3.0 V/cell falling to 0 within 10 %
            over = self.soc_c / self.cap_c - 1.0
            v_cell = max(0.0, 3.0 - 30.0 * over)
        if frac < 0.05:
            v_cell -= (0.05 - frac) * 8.0
        return b.cells * v_cell

    def set_cell_voltage(self, v_cell):
        """Set state of charge so the open-circuit cell voltage equals v_cell (bisection)."""
        lo, hi = 0.0, self.cap_c
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            self.soc_c = mid
            if self.v_oc() / self.batt.cells > v_cell:
                lo = mid
            else:
                hi = mid
        self.soc_c = 0.5 * (lo + hi)

    def aero_coeffs(self, alpha, de=0.0):
        """CL, CD, Cm(about CG) at angle of attack alpha and symmetric elevon de (rad): bilinear table lookup."""
        a = math.degrees(alpha)
        x = (a - self.a0) / self.da
        i = int(x)
        i = max(0, min(len(self.CLt) - 2, i))
        f = x - i
        g = self.de_grid
        de = max(g[0], min(g[-1], de))
        j = 0
        while j < len(g) - 2 and de > g[j + 1]:
            j += 1
        u = (de - g[j]) / (g[j + 1] - g[j])
        out = []
        for c in range(3):
            t0, t1 = self.tabs[j][c], self.tabs[j + 1][c]
            v0 = t0[i] + f * (t0[i + 1] - t0[i])
            v1 = t1[i] + f * (t1[i + 1] - t1[i])
            out.append(v0 + u * (v1 - v0))
        return out[0], out[1], out[2]

    def wind(self):
        return (self.wind0[0] + self.gust[0], self.wind0[1] + self.gust[1], self.wind0[2] + self.gust[2])

    # ---------------- dynamics ----------------
    def derivs(self, pos, vel, q, w, rotor, elev, duty, v_pack):
        R = rot(q)
        wind = self.wind()
        v_air_ned = (vel[0] - wind[0], vel[1] - wind[1], vel[2] - wind[2])
        vb = mtv(R, v_air_ned)
        Fx = Fy = Fz = 0.0
        Mx = My = Mz = 0.0
        rho = self.rho
        d = self.d
        drot = [0.0] * 5
        i_tot = 0.0
        # ---- lift rotors ----
        lm, lp = self.lm, self.lp
        kvr = lm.kv_rad
        for k in range(4):
            r = self.rotor_pos[k]
            om = rotor[k]
            n = om / (2 * math.pi)
            vloc = (vb[0] + w[1] * r[2] - w[2] * r[1], vb[1] + w[2] * r[0] - w[0] * r[2], vb[2] + w[0] * r[1] - w[1] * r[0])
            v_ax = -vloc[2]
            j = v_ax / (n * lp.d) if n > 1.0 else 0.0
            ct = lp.ct(max(j, -0.3)) if j > -0.3 else lp.ct0 * (1 - j * 0.5)
            t = max(ct, 0.0) * rho * n * n * lp.d ** 4 * d.thrust[k] * self.download[k]
            qp = lp.cp(abs(j)) * rho * n * n * lp.d ** 5 / (2 * math.pi)
            vm = duty[k] * v_pack
            ia = (vm - om / kvr) / lm.rm
            if duty[k] <= 0.0 or ia < 0:
                ia = max(ia, 0.0) if duty[k] > 0 else 0.0
            qm = max(ia - lm.i0, 0.0) / kvr if duty[k] > 0 else 0.0
            drot[k] = (qm - qp) / self.j_rotor
            if om <= 0.0 and drot[k] < 0:
                drot[k] = 0.0
            i_tot += duty[k] * ia
            hx, hy = -self.k_hforce * n * vloc[0], -self.k_hforce * n * vloc[1]
            # yaw tilt: lateral thrust whose moment about z (arm x) has the sign of this rotor's drag torque
            side = self.LIFT_DIR[k] * (1.0 if r[0] > 0 else -1.0)
            fx, fy, fz = hx, hy + side * t * self.tilt_s, -t * self.tilt_c
            Fx += fx; Fy += fy; Fz += fz
            Mx += r[1] * fz - r[2] * fy
            My += r[2] * fx - r[0] * fz
            Mz += r[0] * fy - r[1] * fx + self.LIFT_DIR[k] * qp
        # ---- pusher ----
        pm, pp = self.pm, self.pp
        om = rotor[4]
        n = om / (2 * math.pi)
        j = vb[0] / (n * pp.d) if n > 1.0 else 0.0
        t = max(pp.ct(max(j, 0.0)), 0.0) * rho * n * n * pp.d ** 4 * d.thrust[4]
        qp = pp.cp(max(j, 0.0)) * rho * n * n * pp.d ** 5 / (2 * math.pi)
        vm = duty[4] * v_pack
        ia = max((vm - om / pm.kv_rad) / pm.rm, 0.0) if duty[4] > 0 else 0.0
        qm = max(ia - pm.i0, 0.0) / pm.kv_rad if duty[4] > 0 else 0.0
        drot[4] = (qm - qp) / self.j_pusher
        if om <= 0.0 and drot[4] < 0:
            drot[4] = 0.0
        i_tot += duty[4] * ia
        r = self.pusher_pos
        Fx += t
        My += r[2] * t
        Mz += -r[1] * t
        Mx += -qp      # prop CW seen from behind -> reaction about -x
        # ---- aerodynamics ----
        V = math.sqrt(vb[0] ** 2 + vb[1] ** 2 + vb[2] ** 2)
        if V > 0.2 and self.aero_on:
            alpha = math.atan2(vb[2], vb[0])
            beta = math.asin(max(-1.0, min(1.0, vb[1] / V)))
            qd = 0.5 * rho * V * V
            att = 1.0 if abs(alpha) < self.alpha_stall else max(0.0, 1.0 - (abs(alpha) - self.alpha_stall) / math.radians(20))
            de = 0.5 * (elev[0] + elev[1])
            da = 0.5 * (elev[0] - elev[1])     # left TE down, right TE up = roll right (Clda > 0)
            D = self.der
            cl, cd, cm = self.aero_coeffs(alpha, de)   # tables carry elevon effect (fades past stall)
            cl = cl * d.lift
            cd = cd * d.drag
            cn_force = cl * math.cos(alpha) + cd * math.sin(alpha)
            cm = cm + d.cm0 - cn_force * self.dx_cg_cm
            sb = math.sin(beta)
            cy = D["CYb"] * beta * math.cos(alpha) - 1.2 * sb * abs(sb) * 0.047 / self.S * (1 - math.cos(alpha) ** 2 * 0.0)
            # rates (dimensional to avoid 1/V)
            p_, q_, r_ = w
            Cl = D["Clb"] * beta + att * D["Clda"] * da + d.cl0_roll
            Cn = D["Cnb"] * beta + att * D["Cnda"] * da
            L = qd * self.S * cl
            Dg = qd * self.S * cd
            Y = qd * self.S * cy
            ca, sa = math.cos(alpha), math.sin(alpha)
            cbt = math.cos(beta)
            Fx += -Dg * ca * cbt - Y * ca * sb + L * sa
            Fy += -Dg * sb + Y * cbt
            Fz += -Dg * sa * cbt - Y * sa * sb - L * ca
            kq = 0.25 * rho * V * self.S
            Mx += qd * self.S * self.b * Cl + kq * self.b * self.b * (D["Clp"] * p_ + D["Clr"] * r_)
            My += qd * self.S * self.c * cm + kq * self.c * self.c * D["Cmq"] * q_
            Mz += qd * self.S * self.b * Cn + kq * self.b * self.b * (D["Cnr"] * r_ + D["Cnp"] * p_)
        # low-speed rotational damping (flat-plate strips), always present
        p_, q_, r_ = w
        cdp = 0.5 * rho * 1.2 * (1.0 if self.aero_on else 0.0)
        Mx += -cdp * self.c * p_ * abs(p_) * self.b ** 4 / 32.0
        My += -cdp * self.b * q_ * abs(q_) * self.c ** 4 / 32.0 * 4.0
        Mz += -cdp * 0.02 * r_ * abs(r_) * self.b ** 4 / 32.0
        # ---- ground contact ----
        gF = [0.0, 0.0, 0.0]
        gM = [0.0, 0.0, 0.0]
        ground_z = 0.0
        touching = False
        for p in (self.legs if self.ground_on else ()):
            pn = mv(R, p)
            z = pos[2] + pn[2]
            if z > ground_z:
                touching = True
                wr = cross(w, p)
                vp = mv(R, wr)
                vz = vel[2] + vp[2]
                fn = self.k_leg * (z - ground_z) + self.c_leg * vz
                fn = max(fn, 0.0)
                vx, vy = vel[0] + vp[0], vel[1] + vp[1]
                ft = math.hypot(vx, vy)
                fric = min(self.mu * fn, 8.0 * ft) / ft if ft > 1e-6 else 0.0
                fned = (-vx * fric, -vy * fric, -fn)
                fb = mtv(R, fned)
                gF[0] += fb[0]; gF[1] += fb[1]; gF[2] += fb[2]
                mm = cross(p, fb)
                gM[0] += mm[0]; gM[1] += mm[1]; gM[2] += mm[2]
        Fx += gF[0]; Fy += gF[1]; Fz += gF[2]
        Mx += gM[0]; My += gM[1]; Mz += gM[2]
        # ---- equations of motion ----
        m = self.m
        sf = (Fx / m, Fy / m, Fz / m)     # specific force (body) = accelerometer
        an = mv(R, sf)
        acc = (an[0], an[1], an[2] + self.g)
        I = self.I
        Iw = mv(I, w)
        wxIw = cross(w, Iw)
        wd = mv(self.Iinv, (Mx - wxIw[0], My - wxIw[1], Mz - wxIw[2]))
        qd_ = qmul(q, (0.0, w[0], w[1], w[2]))
        qdot = (0.5 * qd_[0], 0.5 * qd_[1], 0.5 * qd_[2], 0.5 * qd_[3])
        return vel, acc, qdot, wd, drot, sf, i_tot, touching

    def step(self, dt, duty, elev_cmd):
        """Advance dt seconds. duty: 5 throttle duty 0..1 (4 lift, pusher). elev_cmd: (left, right) rad, + = TE down."""
        duty = list(duty)
        for k, tf in self.d.motor_fail.items():
            if self.time >= tf:
                duty[k] = 0.0
        # servos: lag 40 ms, rate 6 rad/s
        for k in range(2):
            cmd = max(-0.35, min(0.35, elev_cmd[k]))
            if k in self.d.servo_fail and self.time >= self.d.servo_fail[k][0]:
                cmd = self.d.servo_fail[k][1]
            dv = (cmd - self.elevon[k]) * min(1.0, dt / 0.04)
            dv = max(-6.0 * dt, min(6.0 * dt, dv))
            self.elevon[k] += dv
        # gusts (first-order Gauss-Markov per axis)
        if self.gust_sigma > 0:
            a = math.exp(-dt / self.gust_tau)
            bcoef = self.gust_sigma * math.sqrt(1 - a * a)
            for i in range(3):
                self.gust[i] = a * self.gust[i] + bcoef * self.rng.gauss(0, 1) * (0.5 if i == 2 else 1.0)
        v_pack = self.v_oc() - self.i_batt * self.r_int
        st = (self.pos, self.vel, self.q, self.w, self.rotor)
        # RK2 (midpoint)
        k1 = self.derivs(*st, self.elevon, duty, v_pack)
        h = 0.5 * dt
        mid = (tuple(self.pos[i] + h * k1[0][i] for i in range(3)),
               tuple(self.vel[i] + h * k1[1][i] for i in range(3)),
               self._qn(tuple(self.q[i] + h * k1[2][i] for i in range(4))),
               tuple(self.w[i] + h * k1[3][i] for i in range(3)),
               [max(0.0, self.rotor[i] + h * k1[4][i]) for i in range(5)])
        k2 = self.derivs(*mid, self.elevon, duty, v_pack)
        self.pos = tuple(self.pos[i] + dt * k2[0][i] for i in range(3))
        self.vel = tuple(self.vel[i] + dt * k2[1][i] for i in range(3))
        self.q = self._qn(tuple(self.q[i] + dt * k2[2][i] for i in range(4)))
        self.w = tuple(self.w[i] + dt * k2[3][i] for i in range(3))
        self.rotor = [max(0.0, self.rotor[i] + dt * k2[4][i]) for i in range(5)]
        self.last_accel = k2[5]
        self.i_batt = k2[6]
        self.soc_c += self.i_batt * dt
        self.v_batt = v_pack
        self.energy_wh += v_pack * self.i_batt * dt / 3600.0
        touching = k2[7]
        if touching and not self.on_ground:
            self.max_touch_vz = max(self.max_touch_vz, self.vel[2])
            if self.vel[2] > 3.0:
                self.crashed, self.crash_reason = True, f"hard landing {self.vel[2]:.1f} m/s"
        self.on_ground = touching
        R = rot(self.q)
        for p in (self.crash_pts if self.ground_on else ()):
            if self.pos[2] + mv(R, p)[2] > 0.005 and not self.crashed:
                self.crashed, self.crash_reason = True, "airframe touched ground (not on legs)"
        self.time += dt

    @staticmethod
    def _qn(q):
        n = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
        return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)

    # convenience
    def airspeed_body(self):
        R = rot(self.q)
        w = self.wind()
        return mtv(R, (self.vel[0] - w[0], self.vel[1] - w[1], self.vel[2] - w[2]))
