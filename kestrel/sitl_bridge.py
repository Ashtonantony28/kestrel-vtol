"""M3: ArduPilot SITL <-> Kestrel 6-DOF bridge (SITL JSON backend, lockstep).

SITL sends binary servo packets to UDP 9002 (+10 per instance); for each packet we advance the physics by
1/frame_rate s (sub-stepped at >= 1.2 kHz) and answer with one JSON line of sensor-truth state. SITL then adds
its own sensor models (IMU noise, GPS, compass, baro).

Output mapping (spec/ArduPilot params, sitl/kestrel.parm):
  SERVO1 elevon left (77), SERVO2 elevon right (78): PWM > 1500 = trailing edge UP (ArduPlane mixer convention),
  +-500 us = +-elevon_max_deg.  SERVO3 pusher throttle (70). SERVO5-8 motors 1-4 (33-36). Throttle duty = (pwm-1000)/1000.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
import math
import socket
import struct
import threading
import time

from . import spec as S
from .sim import Dispersion, Vehicle, euler_from_q

PKT16 = struct.Struct("<HHI16H")
PKT32 = struct.Struct("<HHI32H")


class Bridge:
    def __init__(self, instance=0, disp: Dispersion | None = None, wind_ned=(0, 0, 0), gust_sigma=0.0, seed=0,
                 log_path=None, log_hz=25.0, rc_fail_at=None, airspeed_sensor=False):
        self.s = S.load()
        self.airspeed_sensor = airspeed_sensor   # send true pitot airspeed (option: ~$15 sensor, ARSPD_TYPE 100 in SITL)
        self.port = 9002 + 10 * instance
        self.disp = disp
        self.wind, self.gust, self.seed = wind_ned, gust_sigma, seed
        self.v = Vehicle(self.s, disp=disp, wind_ned=wind_ned, gust_sigma=gust_sigma, seed=seed)
        self.elev_max = math.radians(self.s["wing"]["elevon_max_deg"])
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", self.port))
        self.sock.settimeout(0.5)
        self.last_frame = None
        self.running = False
        self.log_path, self.log_dt = log_path, 1.0 / log_hz
        self.next_log = 0.0
        self.rows = []
        self.stats = {"max_tilt_deg": 0.0, "crashed": False, "crash_reason": "", "max_alt": 0.0, "frames": 0,
                      "energy_wh": 0.0, "max_touch_vz": 0.0}
        self.thread = None
        self.lock = threading.Lock()

    def decode(self, pwm):
        def duty(p):
            return min(1.0, max(0.0, (p - 1000) / 1000.0)) if p >= 900 else 0.0
        el = -(pwm[0] - 1500) / 500.0 * self.elev_max if pwm[0] > 0 else 0.0
        er = -(pwm[1] - 1500) / 500.0 * self.elev_max if pwm[1] > 0 else 0.0
        duties = [duty(pwm[4]), duty(pwm[5]), duty(pwm[6]), duty(pwm[7]), duty(pwm[2])]
        return duties, (el, er)

    def handle(self, data, addr):
        if len(data) == PKT16.size:
            magic, rate, count, *pwm = PKT16.unpack(data)
            if magic != 18458:
                return
        elif len(data) == PKT32.size:
            magic, rate, count, *pwm = PKT32.unpack(data)
            if magic != 29569:
                return
        else:
            return
        if self.last_frame is not None and count < self.last_frame:
            self.v = Vehicle(self.s, disp=self.disp, wind_ned=self.wind, gust_sigma=self.gust, seed=self.seed)  # SITL restarted
            self.next_log = 0.0
        if self.last_frame is not None and count == self.last_frame:
            self.reply(addr)   # duplicate: resend state
            return
        self.last_frame = count
        dt = 1.0 / max(rate, 1)
        n = max(1, int(math.ceil(dt * 1200)))
        duties, elev = self.decode(pwm)
        v = self.v
        for _ in range(n):
            v.step(dt / n, duties, elev)
        self.stats["frames"] += 1
        r, p, y = euler_from_q(v.q)
        tilt = math.degrees(math.acos(max(-1, min(1, math.cos(r) * math.cos(p)))))
        if not v.on_ground:
            self.stats["max_tilt_deg"] = max(self.stats["max_tilt_deg"], tilt)
        self.stats["max_alt"] = max(self.stats["max_alt"], -v.pos[2])
        if v.crashed and not self.stats["crashed"]:
            self.stats["crashed"], self.stats["crash_reason"] = True, f"{v.crash_reason} at t={v.time:.1f}s"
        self.stats["energy_wh"] = v.energy_wh
        self.stats["max_touch_vz"] = v.max_touch_vz
        if self.log_path and v.time >= self.next_log:
            self.next_log += self.log_dt
            vb = v.airspeed_body()
            V = math.sqrt(sum(x * x for x in vb))
            self.rows.append([round(v.time, 3), *[round(x, 3) for x in v.pos], *[round(x, 3) for x in v.vel],
                              round(math.degrees(r), 2), round(math.degrees(p), 2), round(math.degrees(y), 2),
                              round(V, 2), round(math.degrees(math.atan2(vb[2], vb[0])), 2) if V > 1 else 0.0,
                              *[round(d, 3) for d in duties], round(math.degrees(v.elevon[0]), 2), round(math.degrees(v.elevon[1]), 2),
                              round(v.v_batt, 2), round(v.i_batt, 2), int(v.on_ground), int(v.crashed)])
        self.reply(addr)

    def reply(self, addr):
        v = self.v
        msg = {
            "timestamp": v.time,
            "imu": {"gyro": list(v.w), "accel_body": list(v.last_accel)},
            "position": list(v.pos),
            "quaternion": list(v.q),
            "velocity": list(v.vel),
            "battery": {"voltage": v.v_batt, "current": v.i_batt},
        }
        if self.airspeed_sensor:
            msg["airspeed"] = max(0.0, v.airspeed_body()[0])   # pitot sees the forward component
        self.sock.sendto((json.dumps(msg) + "\n").encode(), addr)

    def loop(self):
        self.running = True
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            with self.lock:
                self.handle(data, addr)

    def start(self):
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        return self

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        self.sock.close()
        if self.log_path and self.rows:
            Path(self.log_path).parent.mkdir(parents=True, exist_ok=True)
            with open(self.log_path, "w", newline="") as f:
                w = csv.writer(f)
                # elevon columns are the actual surface angles (after servo lag and any stuck-servo fault)
                w.writerow(["t", "n", "e", "d", "vn", "ve", "vd", "roll", "pitch", "yaw", "airspeed", "alpha",
                            "m1", "m2", "m3", "m4", "pusher", "elev_l", "elev_r", "vbatt", "ibatt", "ground", "crashed"])
                w.writerows(self.rows)


if __name__ == "__main__":
    b = Bridge(log_path="sitl_bridge_log.csv").start()
    print(f"Kestrel physics listening on UDP {b.port}; Ctrl-C to stop")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        b.stop()
