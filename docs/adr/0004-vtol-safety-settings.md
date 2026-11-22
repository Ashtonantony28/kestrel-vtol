# 0004: Safety settings found by SITL testing (wind, faults, endurance)

- Status: Accepted
- Date: 2026-11-22

## Context
The first parameter file flew calm missions well. The robustness suites (`python -m kestrel.mc`) then
exposed six failure modes. Each one is a decision that is easy to undo by mistake, so each is recorded here.

## Decision
| Finding in SITL | Change |
|---|---|
| In wind, the synthetic airspeed estimate dipped below `Q_ASSIST_SPEED 11` and kept restarting the transition | `Q_ASSIST_SPEED 10` (trimmed stall 9.6 m/s), `Q_ASSIST_DELAY 1.0` |
| At 10 m/s, a wing hovering side-on to the wind saturated a lift motor | `Q_WVANE_ENABLE 1`: in VTOL the nose yaws into the wind |
| A quad holding position into the wind pitches nose down. On a flying wing that makes **downforce**: alpha −27°, lift motors at 90 % | `Q_FWD_THR_USE 1`, `Q_FWD_THR_GAIN 1.5` (AUW / pusher thrust), `Q_FWD_PIT_LIM 3`: the pusher holds position instead |
| In gusty crosswind, quad yaw authority from prop drag alone saturated | lift motors tilted 4° about the boom (`lift_rotors.yaw_tilt_deg`, printed into the motor mount): about 3× yaw authority for −0.2 % lift |
| A dead pusher left the aircraft stuck mid-transition until the pack died | `Q_TRANS_FAIL 45`, `Q_TRANS_FAIL_ACT 1`: fly home as a quad. 20 s was too short; it fired in a 10 m/s crosswind |
| A stuck elevon let the wing sink slowly into the ground at cruise speed | `Q_ASSIST_ALT 15`: lift motors help below 15 m in plane modes |
| Under cruise load the pack held 10.5 V until about 96 % used, so the voltage warning came with about 1 minute left | `BATT_LOW_MAH 660` (RTL with 30 % left), `BATT_CRT_MAH 330` (QLAND); voltage thresholds kept only as a backstop |

The sim's battery now collapses past its rated capacity, instead of flying on at 3.0 V/cell. A flight
that runs the pack flat ends as a crash in the sim, as it would in reality.

## Not survivable, accepted
- **One lift motor failing in hover.** No quad can survive this. The mitigation is operational: hover low and briefly,
  and transition above 15 m (build/BENCH.md).

## Consequences
- **Verify:** `python -m kestrel.mc --suite wind|faults|endurance|monte_carlo`. Results are in docs/DESIGN-REVIEW.md.
- **Firmware updates:** rerun all four suites after any ArduPilot update or parameter change.
