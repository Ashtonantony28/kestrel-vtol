"""M4: robustness campaign on ArduPlane SITL + Kestrel physics.

  python -m kestrel.mc --suite wind|faults|endurance|monte_carlo [--n 60 --jobs 4]

Each scenario is a full mission on real firmware; results in sitl/runs/<suite>/summary.json.
"""
from __future__ import annotations

import argparse
import json
import math
import time
import random
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

from . import spec as S
from .fly import RUNS, Run
from .sim import Dispersion


_INST = None


def _init(counter):
    """Each worker process gets its own SITL instance number (ports 5760+10i, 9002+10i)."""
    global _INST
    with counter.get_lock():
        counter.value += 1
        _INST = counter.value


def _job(args):
    kw, _ = args
    inst = _INST
    disp = kw.pop("disp", None)
    d = Dispersion(**disp) if disp else None
    for attempt in range(3):
        r = Run(instance=inst, disp=d, verbose=False, **dict(kw)).run()
        # a SITL start-up failure (port race, connection reset before arming) is infrastructure, not flight
        if not (r.get("error") and r.get("physics", {}).get("frames", 0) <= 1):
            break
        time.sleep(3)
    r["disp"] = disp
    r["attempts"] = attempt + 1
    return r


def wind_suite():
    out = []
    for w in (0, 4, 7, 10):
        for g in ((0.0,) if w == 0 else (1.5,)):
            for hd in ((0,) if w == 0 else (90, 225)):
                wn, we = -w * math.cos(math.radians(hd)), -w * math.sin(math.radians(hd))   # wind FROM heading hd
                out.append({"mission": "basic", "wind": (wn, we, 0.0), "gust": g, "name": f"wind_{w}ms_from{hd}"})
    return out


def fault_suite():
    return [
        {"mission": "basic", "name": "pusher_fails_in_cruise", "disp": {"motor_fail": {4: 45.0}}},
        {"mission": "basic", "name": "rc_link_lost_in_cruise", "rc_loss_at": 50.0},
        {"mission": "basic", "name": "elevon_left_stuck_neutral", "disp": {"servo_fail": {0: (45.0, 0.0)}}},
        {"mission": "basic", "name": "cg_10mm_aft", "disp": {"cg_dx": -0.010}},
        {"mission": "basic", "name": "cg_10mm_forward", "disp": {"cg_dx": 0.010}},
        {"mission": "basic", "name": "heavy_15pct", "disp": {"mass": 1.15}},
        {"mission": "basic", "name": "weak_motors_minus10pct", "disp": {"thrust": (0.9, 0.9, 0.9, 0.9, 0.9)}},
        {"mission": "basic", "name": "draggy_plus30pct", "disp": {"drag": 1.3}},
        {"mission": "basic", "name": "warped_wing_roll", "disp": {"cl0_roll": 0.006}},
        {"mission": "hover", "name": "lift_motor1_fails_in_hover", "disp": {"motor_fail": {0: 15.0}}},
    ]


def endurance_suite():
    return [{"mission": "endurance", "name": "endurance_nominal", "timeout_s": 3000},
            {"mission": "endurance", "name": "endurance_pessimistic", "timeout_s": 3000,
             "disp": {"drag": 1.25, "mass": 1.08, "thrust": (0.95,) * 5, "batt_capacity": 0.9}}]


def mc_suite(n, seed=1):
    rng = random.Random(seed)
    out = []
    for i in range(n):
        w = rng.uniform(0, 7)
        hd = rng.uniform(0, 360)
        disp = {"mass": rng.gauss(1, 0.04), "cg_dx": rng.gauss(0, 0.006), "drag": rng.gauss(1, 0.12),
                "lift": rng.gauss(1, 0.05), "cm0": rng.gauss(0, 0.006), "cl0_roll": rng.gauss(0, 0.002),
                "thrust": tuple(rng.gauss(1, 0.03) for _ in range(5)), "batt_r": rng.uniform(0.8, 1.6)}
        out.append({"mission": "basic", "name": f"mc_{i:03d}", "seed": i, "gust": rng.uniform(0, 2),
                    "wind": (-w * math.cos(math.radians(hd)), -w * math.sin(math.radians(hd)), 0.0), "disp": disp})
    return out


def run_suite(name, jobs_list, jobs=4):
    d = RUNS / name
    d.mkdir(parents=True, exist_ok=True)
    results = []
    import multiprocessing as mp
    counter = mp.Value("i", 0)
    with ProcessPoolExecutor(max_workers=jobs, initializer=_init, initargs=(counter,)) as ex:
        futs = {}
        for i, kw in enumerate(jobs_list):
            kw = dict(kw)
            kw["name"] = f"{name}/{kw['name']}"
            futs[ex.submit(_job, (kw, None))] = kw["name"]
        for f in as_completed(futs):
            r = f.result()
            results.append(r)
            print(f"{r['name']:45s} ok={r['ok']} land_err={r.get('land_error_m', float('nan')):.2f} "
                  f"crash={r['physics']['crashed']} {r['physics']['crash_reason']} t={r.get('sim_time_s', 0):.0f}s "
                  f"{r.get('error', '')}", flush=True)
    results.sort(key=lambda r: r["name"])
    summ = {"suite": name, "n": len(results), "passed": sum(r["ok"] for r in results), "runs": results}
    (d / "summary.json").write_text(json.dumps(summ, indent=1, default=float))
    print(f"{name}: {summ['passed']}/{summ['n']} passed")
    return summ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", required=True, choices=["wind", "faults", "endurance", "monte_carlo"])
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--only", default=None, help="run only cases whose name contains this text")
    a = ap.parse_args()
    js = {"wind": wind_suite, "faults": fault_suite, "endurance": endurance_suite}.get(a.suite)
    jl = js() if js else mc_suite(a.n)
    if a.only:
        jl = [j for j in jl if a.only in j["name"]]
    run_suite(a.suite, jl, a.jobs)


if __name__ == "__main__":
    main()
