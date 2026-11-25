"""Wiring diagram: off-the-shelf boards only (no custom PCB). Output assignments follow sitl/kestrel.parm.

Run: python -m kestrel.wiring      -> cad/drawings/wiring_diagram.pdf and .png
"""
from __future__ import annotations

import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from . import spec as S

OUT = S.ROOT / "cad" / "drawings"
INK, PWR, SIG, BUS, RF, NOTE = "#1d1d1f", "#c0392b", "#1f5fa8", "#2e8b57", "#8e44ad", "#555555"


def _validation():
    vp = S.ROOT / "build" / "validation.json"
    if not vp.exists():
        return {}
    v = json.loads(vp.read_text())
    return {c["check"]: c["value"] for c in (v if isinstance(v, list) else v.get("checks", []))}


def _val(val, prefix, default):
    for k, x in val.items():
        if k.startswith(prefix):
            return x
    return default


def build():
    s = S.load()
    val = _validation()
    tilt = s["lift_rotors"].get("yaw_tilt_deg", 0)
    i_lift = _val(val, "V4b lift", 15.5)
    i_push = _val(val, "V4e pusher", 14.9)
    i_peak = _val(val, "V4c", 69.0)
    thr_max = round(100 * s["propulsion"].get("pusher_thr_max", 1.0))
    c_rate = s["batteries"][s["propulsion"]["battery"]].get("c_rating", 30)

    fig = plt.figure(figsize=(16.54, 11.69))          # A3 landscape, 1 unit = 1 mm
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 420)
    ax.set_ylim(0, 297)
    ax.axis("off")
    ax.add_patch(plt.Rectangle((8, 8), 404, 281, fill=False, lw=1.0, ec=INK))
    halo = [pe.withStroke(linewidth=3.2, foreground="white")]   # a wire crossing another is drawn over a white gap

    def box(x, y, w, h, title, lines=(), fc="#ffffff"):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=2", fc=fc, ec=INK, lw=1.0, zorder=3))
        ax.text(x + w / 2, y + h - 4.0, title, ha="center", va="center", fontsize=7.8, weight="bold", color=INK, zorder=4)
        for k, ln in enumerate(lines):
            ax.text(x + 2.5, y + h - 9.4 - k * 4.0, ln, ha="left", va="center", fontsize=6.0, color=INK, zorder=4)

    def wire(pts, color, lw=1.3, ls="-", over=False):
        xs, ys = zip(*pts)
        ln, = ax.plot(xs, ys, color=color, lw=lw, ls=ls, solid_capstyle="round", zorder=2.5 if over else 2)
        if over:
            ln.set_path_effects(halo + [pe.Normal()])

    def label(x, y, text, color, ha="left", va="bottom", size=5.6):
        ax.text(x, y, text, fontsize=size, color=color, ha=ha, va=va, zorder=5,
                bbox=dict(boxstyle="square,pad=0.15", fc="white", ec="none"))

    ax.text(16, 281, "KESTREL QUADPLANE  —  WIRING DIAGRAM (aircraft and ground station)", fontsize=12, weight="bold")
    ax.text(16, 275.5, "Off-the-shelf boards only, no custom PCB. Outputs follow sitl/kestrel.parm; serial ports, receiver, "
            "airspeed bus and switches follow sitl/kestrel_hw.parm. Check every pad against the SpeedyBee F405 Wing Mini "
            "manual before soldering.", fontsize=6.6, color=NOTE)

    # ---------------- power: battery -> star point -> FC and three harnesses (no crossings) ----------------
    box(16, 214, 52, 34, "3S 2200 mAh LiPo", [f"{c_rate}C or higher", f"(peak draw {i_peak:.0f} A, V4c)", "XT60, 12.6 V full"], fc="#fdf1ef")
    box(86, 214, 46, 34, "Star point", ["XT60 pigtail + 14 AWG,", "soldered and heat-shrunk,", "in the pod"], fc="#fdf1ef")
    wire([(68, 231), (86, 231)], PWR)
    label(77, 232.5, "XT60", PWR, ha="center")
    wire([(132, 231), (150, 231)], PWR)
    label(141, 232.5, "VBAT", PWR, ha="center")

    ex, mx = 296, 352
    escs = [("ESC 1 (FR)", "M1 FR  CCW", 236, "right"), ("ESC 4 (RR)", "M4 RR  CW", 212, "right"),
            ("ESC 3 (FL)", "M3 FL  CW", 182, "left"), ("ESC 2 (RL)", "M2 RL  CCW", 158, "left")]
    for name, mname, y, side in escs:
        box(ex, y, 40, 18, name, ["30 A, " + side + " boom,", "in the prop wash"], fc="#fdf1ef")
        box(mx, y, 50, 18, mname, ["A2212 1000KV, 10x4.5", f"leans {tilt:.0f} deg outboard"], fc="#f7f7f7")
        wire([(336, y + 9), (mx, y + 9)], INK)
    box(ex, 124, 40, 18, "Pusher ESC", ["30 A, in the pod tail"], fc="#fdf1ef")
    box(mx, 124, 50, 18, "Pusher motor", ["A2212 1400KV, 8x6", "on the printed pylon"], fc="#f7f7f7")
    wire([(336, 133), (mx, 133)], INK)
    ax.text(344, 256, "3 phase leads", fontsize=5.6, color=INK, ha="center")

    # harnesses leave the star point upward and run along the top: right boom (outermost), left boom, pusher
    for xs, ytop, xv, stubs, lab in ((110, 268, 284, (249, 225), f"right boom harness, 14 AWG (2 ESCs, about {2 * i_lift:.0f} A)"),
                                     (118, 264, 278, (195, 171), f"left boom harness, 14 AWG (2 ESCs, about {2 * i_lift:.0f} A)"),
                                     (126, 260, 272, (137,), f"pusher feed, 16 AWG (about {i_push:.0f} A at THR_MAX {thr_max})")):
        wire([(xs, 248), (xs, ytop), (xv, ytop), (xv, min(stubs))], PWR)
        for yy in stubs:
            wire([(xv, yy), (ex, yy)], PWR)
        label(xs + 4, ytop + 0.6, lab, PWR)

    # ---------------- flight controller ----------------
    box(150, 110, 88, 138, "Flight controller", [
        "SpeedyBee F405 Wing Mini", "ArduPilot target SpeedyBeeF405WING", "ArduPlane 4.7.3, quadplane",
        "BATT_MONITOR 4: on-board V + I", "", "Servo rail powered by the FC's", "own BEC only (servos S1-S2)", "",
        "PWM timer groups:", "  S1-S2 TIM4  (elevons, 50 Hz)", "  S3 TIM3  (pusher)", "  S5-S7 TIM8, S8 TIM2 (lift, 490 Hz)",
        "No servo shares a timer with a", "lift motor (hwdef)."], fc="#eef4fb")
    outs = [("S5  SERVO5 33  M1 FR", 241), ("S8  SERVO8 36  M4 RR", 217), ("S7  SERVO7 35  M3 FL", 187),
            ("S6  SERVO6 34  M2 RL", 163), ("S3  SERVO3 70  pusher", 129)]
    for lab, y in outs:
        wire([(238, y), (ex, y)], SIG, lw=0.9, over=True)
        label(240, y + 0.6, lab, SIG, size=5.2)

    # servos
    box(150, 78, 40, 22, "Elevon servo L", ["SG90 9 g, both segments", "(torque rod, ADR 0006)"], fc="#f7f7f7")
    box(198, 78, 40, 22, "Elevon servo R", ["SG90 9 g, both segments", "(torque rod, ADR 0006)"], fc="#f7f7f7")
    wire([(170, 110), (170, 100)], SIG)
    wire([(218, 110), (218, 100)], SIG)
    label(172, 104, "S1  77", SIG, size=5.2)
    label(220, 104, "S2  78", SIG, size=5.2)

    # ---------------- peripherals, left column ----------------
    box(16, 166, 56, 32, "M10 GPS + compass", ["on a mast ahead of the cross spar", "GPS: 5 V, GND, TX, RX", "compass: SDA, SCL"], fc="#eef8f1")
    box(16, 120, 56, 34, "MS4525DO airspeed", ["pitot on the pod nose (ADR 0007)", "5 V, GND, SDA, SCL", "hoses: total -> P1,", "static -> P2"], fc="#eef8f1")
    box(16, 66, 56, 30, "ELRS 2.4 GHz receiver", ["CRSF, 5 V", "antennas out of the pod floor"], fc="#f5effa")
    wire([(72, 190), (150, 190)], SIG)
    label(76, 191, "GPS UART -> pads T3/R3 = SERIAL3 (SERIAL3_PROTOCOL 5); TX->RX, RX->TX", SIG, size=5.2)
    wire([(72, 174), (140, 174), (140, 137), (72, 137)], BUS)
    wire([(140, 156), (150, 156)], BUS)
    label(76, 175, "I2C1 (the only I2C bus): SDA, SCL, 5 V, GND", BUS, size=5.2)
    label(76, 138.5, "ARSPD_BUS 0 (default 1 finds nothing)", BUS, size=5.2)
    wire([(72, 84), (132, 84), (132, 128), (150, 128)], SIG)
    label(76, 85.5, "pads T1/R1 = SERIAL1 (protocol 23, RC_OPTIONS 8192)", SIG, size=5.2)
    ax.text(48, 61.5, "Not the SBUS pad: it has an inverter and cannot read CRSF.", fontsize=5.2, color=NOTE)

    # ---------------- ground station ----------------
    gy = 18
    ax.text(16, gy + 38, "GROUND STATION", fontsize=7.6, weight="bold", color=NOTE)
    box(16, gy, 56, 32, "ELRS receiver flashed as TX", ["CRSF in at 420000 baud", "(400000 on some builds)"], fc="#f5effa")
    box(100, gy, 62, 32, "ESP32 dev board", ["Bluepad32 sketch", "ground/kestrel_radio/", "USB power"], fc="#f7f7f7")
    box(186, gy, 56, 32, "Xbox Wireless Controller", ["Bluetooth", "(owned already)"], fc="#f7f7f7")
    wire([(100, gy + 22), (72, gy + 22)], SIG)
    label(86, gy + 23, "GPIO17 (TX2) -> RX", SIG, ha="center", size=5.2)
    wire([(100, gy + 9), (72, gy + 9)], SIG)
    label(86, gy + 3, "GPIO16 (RX2) <- TX, 1 kOhm", SIG, ha="center", size=5.2)
    wire([(186, gy + 16), (162, gy + 16)], RF, ls="--")
    label(174, gy + 17, "Bluetooth", RF, ha="center", size=5.2)
    wire([(44, gy + 32), (44, 66)], RF, ls="--")
    label(46, gy + 35, "2.4 GHz ELRS link", RF, size=5.2)
    ax.text(16, 11.5, "Optional, recommended for first flights: a stock ELRS handset in place of the Xbox bridge "
            "(same receiver, same parameters).", fontsize=5.4, color=NOTE)

    # ---------------- notes and legend ----------------
    notes = [
        "NOTES",
        "1. ESCs take signal and ground only from the FC. Pull the red (BEC) wire out of all five ESC signal plugs, so",
        "   five BECs are never paralleled with the FC's (bench item B19).",
        "2. Motor numbers and spin directions follow ADR 0001 (Q_FRAME_TYPE 1, quad X). Swap any two phase wires",
        "   to reverse. Check with ArduPilot motor test, props off (B5).",
        f"3. Currents (validation V4): lift motor {i_lift:.1f} A each at full throttle on a full pack, so about",
        f"   {2 * i_lift:.0f} A per boom harness; pusher {i_push:.1f} A at THR_MAX {thr_max}; pack peak {i_peak:.0f} A.",
        "4. Airspeed hoses must not kink. Blow into the pitot tip and check that airspeed rises (B16).",
        "5. Keep the GPS mast and receiver antennas away from the battery leads and the ESCs.",
        "6. Load sitl/kestrel.parm, then sitl/kestrel_hw.parm, and run B17 with the props off.",
    ]
    for k, n in enumerate(notes):
        ax.text(260, 108 - k * 4.4, n, fontsize=6.1 if k else 7.0, weight="bold" if k == 0 else "normal", color=INK)
    lx, ly = 260, 52
    for k, (c, t_, ls) in enumerate(((PWR, "battery power", "-"), (SIG, "signal (PWM / UART)", "-"), (BUS, "I2C bus", "-"),
                                     (RF, "radio link", "--"), (INK, "motor phases", "-"))):
        ax.plot([lx, lx + 10], [ly - k * 5, ly - k * 5], color=c, lw=1.4, ls=ls)
        ax.text(lx + 13, ly - k * 5, t_, fontsize=6.2, va="center")
    ax.text(404, 12, "Generated by python -m kestrel.wiring", fontsize=5.6, color=NOTE, ha="right")
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / "wiring_diagram.pdf")
    fig.savefig(OUT / "wiring_diagram.png", dpi=130)
    plt.close(fig)
    return OUT / "wiring_diagram.pdf"


if __name__ == "__main__":
    print("wrote", build())
