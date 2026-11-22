"""Plot a SITL run CSV (sitl/runs/<name>.csv) -> PNG with altitude, airspeed, attitude, actuators, battery."""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(path):
    with open(path) as f:
        r = csv.DictReader(f)
        rows = list(r)
    return {k: np.array([float(x[k]) for x in rows]) for k in rows[0]}


def plot(path, out=None, title=None):
    d = load(path)
    t = d["t"]
    fig, ax = plt.subplots(6, 1, figsize=(10, 13), sharex=True)
    ax[0].plot(t, -d["d"], label="altitude (m)"); ax[0].set_ylabel("alt m"); ax[0].grid(alpha=.3)
    ax[1].plot(t, d["airspeed"], label="airspeed"); ax[1].axhline(9.6, ls="--", c="r", lw=.8, label="trimmed stall 9.6")
    ax[1].axhline(12, ls=":", c="k", lw=.8, label="AIRSPEED_MIN 12"); ax[1].set_ylabel("m/s"); ax[1].legend(fontsize=7); ax[1].grid(alpha=.3)
    ax[2].plot(t, d["roll"], label="roll"); ax[2].plot(t, d["pitch"], label="pitch"); ax[2].plot(t, np.clip(d["alpha"], -30, 30), label="alpha (clipped)")
    ax[2].set_ylabel("deg"); ax[2].legend(fontsize=7); ax[2].grid(alpha=.3)
    for k in ("m1", "m2", "m3", "m4", "pusher"):
        ax[3].plot(t, d[k], label=k, lw=.8)
    ax[3].set_ylabel("duty"); ax[3].legend(fontsize=7, ncol=5); ax[3].grid(alpha=.3)
    ax[4].plot(t, d["elev_l"], label="elevon L (+TE down)"); ax[4].plot(t, d["elev_r"], label="elevon R")
    ax[4].set_ylabel("deg"); ax[4].legend(fontsize=7); ax[4].grid(alpha=.3)
    ax[5].plot(t, d["vbatt"], label="V"); a2 = ax[5].twinx(); a2.plot(t, d["ibatt"], c="tab:orange", lw=.7, label="A")
    ax[5].set_ylabel("V"); a2.set_ylabel("A"); ax[5].set_xlabel("sim time s"); ax[5].grid(alpha=.3)
    fig.suptitle(title or Path(path).stem)
    fig.tight_layout()
    out = out or str(Path(path).with_suffix(".png"))
    fig.savefig(out, dpi=80)
    return out


if __name__ == "__main__":
    print(plot(sys.argv[1]))
