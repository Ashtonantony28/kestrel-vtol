"""M3/M4: fly scripted missions on real ArduPlane firmware (SITL) with the Kestrel physics.

  python -m kestrel.fly --mission basic [--wind 5 --gust 1.5 --instance 0 --speedup 10]

Mission 'basic': VTOL takeoff to 30 m -> transition -> 3 waypoints at 50 m -> back-transition -> VTOL land at home.
Returns/prints metrics JSON (sitl/runs/<name>.json, CSV state log alongside).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

from pymavlink import mavutil

from . import spec as S
from .sim import Dispersion
from .sitl_bridge import Bridge

ROOT = S.ROOT
ARDUPILOT = Path(os.environ.get("ARDUPILOT_DIR", str(Path.home() / "ardupilot")))
BIN = ARDUPILOT / "build" / "sitl" / "bin" / "arduplane"
PARM = ROOT / "sitl" / "kestrel.parm"
HOME = (-35.363261, 149.165230, 584.0, 0.0)
RUNS = ROOT / "sitl" / "runs"


def offset(lat, lon, dn, de):
    dlat = dn / 6378137.0
    dlon = de / (6378137.0 * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


MISSIONS = {
    # (command, north m, east m, alt m)
    "basic": [("TAKEOFF_VTOL", 0, 0, 30), ("WP", 250, 0, 50), ("WP", 250, 250, 50), ("WP", 0, 250, 50),
              ("WP", -150, 0, 40), ("LAND_VTOL", 0, 0, 0)],
    "long": [("TAKEOFF_VTOL", 0, 0, 30)] + [("WP", 300 * ((i // 2) % 2), 300 * (((i + 1) // 2) % 2), 60) for i in range(1, 9)]
            + [("WP", -150, 0, 40), ("LAND_VTOL", 0, 0, 0)],
    "hover": [("TAKEOFF_VTOL", 0, 0, 20), ("LOITER_TIME_VTOL", 0, 0, 20), ("LAND_VTOL", 0, 0, 0)],
    # endurance: 400 m racetrack laps until the battery failsafe sends it home (RTL -> VTOL land)
    "endurance": [("TAKEOFF_VTOL", 0, 0, 30)] + [("WP", n, e, 60) for _ in range(40) for n, e in ((400, 0), (400, 200), (0, 200), (0, 0))]
                 + [("LAND_VTOL", 0, 0, 0)],
}


class Run:
    def __init__(self, mission="basic", instance=0, speedup=10, wind=(0.0, 0.0, 0.0), gust=0.0, seed=0,
                 disp: Dispersion | None = None, name=None, timeout_s=900, rc_loss_at=None, extra_params=None, verbose=True):
        self.mission, self.inst, self.speedup = mission, instance, speedup
        self.wind, self.gust, self.seed, self.disp = wind, gust, seed, disp
        self.name = name or f"{mission}_i{instance}"
        self.timeout_s, self.rc_loss_at, self.extra = timeout_s, rc_loss_at, extra_params or {}
        self.verbose = verbose
        self.events = []

    def log(self, *a):
        if self.verbose:
            print(f"[{self.name}]", *a, flush=True)

    def _upload(self, m, items):
        lat0, lon0 = HOME[0], HOME[1]
        cmds = {"TAKEOFF_VTOL": mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, "WP": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                "LAND_VTOL": mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND, "LOITER_TIME_VTOL": mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME}
        seq = [("WP", 0, 0, 0)] + items   # item 0 = home
        m.mav.mission_count_send(m.target_system, m.target_component, len(seq), 0)
        sent = 0
        t0 = time.time()
        while sent < len(seq) and time.time() - t0 < 30:
            req = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=5)
            if req is None:
                continue
            i = req.seq
            c, dn, de, alt = seq[i]
            lat, lon = offset(lat0, lon0, dn, de)
            p1 = 20 if c == "LOITER_TIME_VTOL" else 0
            if c == "LOITER_TIME_VTOL":
                # loiter in VTOL: precede with DO_VTOL_TRANSITION is not needed when already hovering after takeoff
                pass
            m.mav.mission_item_int_send(m.target_system, m.target_component, i, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                        cmds[c], 0, 1, p1, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), alt, 0)
            sent = max(sent, i + 1)
        ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
        return ack is not None and ack.type == 0

    def set_param(self, m, name, value):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)

    def run(self):
        RUNS.mkdir(parents=True, exist_ok=True)
        wd = Path(tempfile.mkdtemp(prefix=f"kestrel_sitl_{self.inst}_"))
        bridge = Bridge(instance=self.inst, disp=self.disp, wind_ned=self.wind, gust_sigma=self.gust, seed=self.seed,
                        log_path=str(RUNS / f"{self.name}.csv")).start()
        home = ",".join(str(x) for x in HOME)
        parm = wd / "kestrel.parm"
        txt = PARM.read_text() + "".join(f"\n{k} {v}" for k, v in self.extra.items()) + "\n"
        parm.write_text(txt)
        cmd = [str(BIN), "--model", "JSON:127.0.0.1", "--speedup", str(self.speedup), "-I", str(self.inst),
               "--home", home, "--defaults", str(parm), "--wipe"]
        out = open(wd / "sitl.log", "w")
        proc = subprocess.Popen(cmd, cwd=wd, stdout=out, stderr=subprocess.STDOUT)
        res = {"name": self.name, "mission": self.mission, "wind": self.wind, "gust": self.gust, "ok": False}
        try:
            res.update(self._fly(5760 + 10 * self.inst, bridge))
        except Exception as e:  # noqa
            res["error"] = repr(e)
        finally:
            proc.terminate()
            try:
                proc.wait(5)
            except subprocess.TimeoutExpired:
                proc.kill()
            bridge.stop()
            out.close()
        res["physics"] = bridge.stats
        res["ok"] = bool(res.get("completed")) and not bridge.stats["crashed"] and res.get("land_error_m", 99) < 5.0
        (RUNS / f"{self.name}.json").write_text(json.dumps(res, indent=1, default=float))
        shutil.rmtree(wd, ignore_errors=True)
        return res

    def _fly(self, port, bridge):
        t_start = time.time()
        m = None
        while time.time() - t_start < 60:
            try:
                m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=255, autoreconnect=False)
                break
            except Exception:
                time.sleep(0.5)
        if m is None:
            return {"error": "no mavlink"}
        m.wait_heartbeat(timeout=60)
        m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.log("connected")
        items = MISSIONS[self.mission]
        # wait until the EKF uses GPS (mission storage and arming are ready by then)
        t0 = time.time()
        while time.time() - t0 < 120:
            st = m.recv_match(type="STATUSTEXT", blocking=True, timeout=2)
            if st and "is using GPS" in st.text:
                break
        ok = False
        for _ in range(5):
            if self._upload(m, items):
                ok = True
                break
            time.sleep(1)
        assert ok, "mission upload failed"
        # wait for EKF + GPS and arm in QLOITER (RC throttle low via override)
        m.mav.rc_channels_override_send(m.target_system, m.target_component, 1500, 1500, 1000, 1500, 0, 0, 0, 0)
        m.set_mode("QLOITER")
        armed = False
        t0 = time.time()
        while not armed and time.time() - t0 < 180:
            m.mav.rc_channels_override_send(m.target_system, m.target_component, 1500, 1500, 1000, 1500, 0, 0, 0, 0)
            m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                armed = True
            st = m.recv_match(type="STATUSTEXT", blocking=False)
            if st and "PreArm" in st.text and time.time() - t0 > 170:
                self.log(st.text)
        if not armed:
            return {"error": "could not arm"}
        self.log("armed")
        m.set_mode("AUTO")
        sim_t0 = bridge.v.time
        completed = False
        wp_reached = 0
        texts = []
        last_print = 0
        rc_lost = False
        alt_err_max = 0.0
        landed_pos = None
        disarm_seen = False
        while time.time() - t_start < self.timeout_s:
            msg = m.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            tp = msg.get_type()
            st_t = bridge.v.time - sim_t0
            if self.rc_loss_at is not None and not rc_lost and st_t > self.rc_loss_at:
                self.set_param(m, "SIM_RC_FAIL", 1)
                rc_lost = True
                texts.append((round(st_t, 1), "** RC LINK LOST (test) **"))
            if tp == "STATUSTEXT":
                texts.append((round(st_t, 1), msg.text))
                self.log(f"t={st_t:6.1f} {msg.text}")
            elif tp == "MISSION_ITEM_REACHED":
                wp_reached = max(wp_reached, msg.seq)
            elif tp == "HEARTBEAT" and msg.get_srcSystem() == m.target_system:
                if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and st_t > 10:
                    disarm_seen = True
            if bridge.stats["crashed"]:
                break
            if disarm_seen:
                v = bridge.v
                landed_pos = (v.pos[0], v.pos[1])
                completed = True
                break
            if self.verbose and time.time() - last_print > 10:
                last_print = time.time()
                v = bridge.v
                self.log(f"t={st_t:6.1f} pos n={v.pos[0]:7.1f} e={v.pos[1]:7.1f} alt={-v.pos[2]:5.1f} V={math.sqrt(sum(x*x for x in v.airspeed_body())):4.1f} batt={v.v_batt:5.2f}V")
        v = bridge.v
        res = {"completed": completed, "wp_reached": wp_reached, "sim_time_s": bridge.v.time - sim_t0,
               "statustext": texts, "energy_wh": v.energy_wh, "mah_used": v.soc_c / 3.6,
               "wall_s": time.time() - t_start}
        if landed_pos is not None:
            res["land_error_m"] = math.hypot(*landed_pos)
        return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission", default="basic", choices=list(MISSIONS))
    ap.add_argument("--wind", type=float, default=0.0, help="steady wind speed m/s (from the west)")
    ap.add_argument("--gust", type=float, default=0.0)
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--speedup", type=int, default=10)
    ap.add_argument("--name", default=None)
    a = ap.parse_args()
    r = Run(a.mission, a.instance, a.speedup, wind=(0.0, a.wind, 0.0), gust=a.gust, name=a.name).run()
    print(json.dumps({k: v for k, v in r.items() if k != "statustext"}, indent=1, default=float))
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
