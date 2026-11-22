"""Summarise the SITL robustness suites (M4) into build/sitl_results.json and docs/img/*.png.

Verify: python -m kestrel.report   (after python -m kestrel.mc --suite wind|faults|endurance|monte_carlo)
"""
from __future__ import annotations

import json
import math
import statistics as st
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "sitl" / "runs"
IMG = ROOT / "docs" / "img"


def _load(suite):
    p = RUNS / suite / "summary.json"
    return json.loads(p.read_text()) if p.exists() else None


def _row(r):
    ph = r.get("physics", {})
    return {"name": r["name"].split("/")[-1], "ok": bool(r.get("ok")), "land_error_m": r.get("land_error_m"),
            "crash": ph.get("crash_reason", "") if ph.get("crashed") else "", "error": r.get("error", ""),
            "max_tilt_deg": round(ph.get("max_tilt_deg", 0.0), 1), "touchdown_mps": round(ph.get("max_touch_vz", 0.0), 2),
            "energy_wh": round(ph.get("energy_wh", 0.0), 2), "sim_time_s": round(r.get("sim_time_s", 0.0), 1)}


def _pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    k = (len(xs) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def main():
    IMG.mkdir(parents=True, exist_ok=True)
    out = {}
    for suite in ("wind", "faults", "endurance", "monte_carlo"):
        d = _load(suite)
        if not d:
            continue
        rows = [_row(r) for r in d["runs"]]
        out[suite] = {"n": len(rows), "passed": sum(r["ok"] for r in rows), "runs": rows}
    mc = out.get("monte_carlo")
    if mc:
        ok = [r for r in mc["runs"] if r["ok"]]
        le = [r["land_error_m"] for r in ok]
        td = [r["touchdown_mps"] for r in ok]
        en = [r["energy_wh"] for r in ok]
        mc["stats"] = {
            "pass_rate": mc["passed"] / mc["n"],
            "land_error_m": {"median": st.median(le), "p95": _pct(le, 0.95), "max": max(le)} if le else None,
            "within_2m": sum(x < 2.0 for x in le) / mc["n"],
            "touchdown_mps": {"median": st.median(td), "max": max(td)} if td else None,
            "mission_energy_wh": {"mean": st.mean(en), "p95": _pct(en, 0.95)} if en else None,
            "failures": [{k: r[k] for k in ("name", "crash", "error", "land_error_m")} for r in mc["runs"] if not r["ok"]],
        }
        fig, ax = plt.subplots(1, 3, figsize=(12, 3.4))
        ax[0].hist(le, bins=25, color="#3b6ea5")
        ax[0].set_xlabel("landing error from home (m)")
        ax[0].set_ylabel("runs")
        ax[1].hist(td, bins=25, color="#5a8f4e")
        ax[1].set_xlabel("touchdown sink rate (m/s)")
        ax[2].hist(en, bins=25, color="#b0703c")
        ax[2].set_xlabel("mission energy (Wh), 2.6 min mission")
        fig.suptitle(f"Kestrel Monte Carlo: {mc['passed']}/{mc['n']} missions passed (ArduPlane 4.7.3 SITL + own physics)")
        fig.tight_layout()
        fig.savefig(IMG / "monte_carlo.png", dpi=120)
    (ROOT / "build" / "sitl_results.json").write_text(json.dumps(out, indent=1))
    for s, v in out.items():
        print(f"{s:12s} {v['passed']}/{v['n']}")
    if mc:
        print(json.dumps(mc["stats"], indent=1, default=float))


if __name__ == "__main__":
    main()
