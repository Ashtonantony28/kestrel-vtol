# 0008: Fixes from the external design review

- Status: Accepted
- Date: 2026-11-25
- Amends: ADR 0004 (assist speed), ADR 0005 (spar joint), ADR 0007 (cost ceiling). The pusher requirement
  "excess thrust >= 2" from the lean plan is replaced (see K4).

## Context
An outside review of the design (checked claim by claim before acting) found nine items, K1 to K9, plus
some smaller ones. The user asked for the confirmed and plausible ones to be fixed.

## Decision
- **K1 Radio, arming, failsafe.** New `sitl/kestrel_hw.parm`, loaded after `kestrel.parm` on the real board
  only. It sets up ELRS on SERIAL1 (USART1; the SBUS pad cannot read CRSF), GPS on SERIAL3, `ARSPD_BUS 0`,
  CH5 flight modes, CH7 arm switch, `INITIAL_MODE` QLOITER and the RC failsafe. Serial numbers come from the
  board's hwdef. SITL does not load it, because the harness drives modes over MAVLink.
  - `tests/test_params.py` checks every parameter name against ArduPlane 4.7.3's own list, because ArduPilot
    ignores an unknown name without any error.
  - The check found two dead names. `Q_A_ACCEL_Y_MAX` had been renamed, so every SITL run flew the default;
    the line is removed and the default kept. `ARMING_CHECK` is now `ARMING_SKIPCHK`.
  - Also fixed: ArduPilot's `ARSPD_BUS` default of 1 does not exist on this board, so the sensor would never
    have been found.
- **K2 Cut list.** The booms are cut to 850 mm, as in the CAD. The outer spars are 249 mm along the sweep, so
  they need a 1 m tube.
- **K3 Prop model.** Lift props get `prop_derate.lift_cp_scale 1.24`, which lowers the static figure of merit
  from 0.80 to 0.65. Hover rises from 90 W to 117 W. The pusher stays as tabulated: its cruise efficiency
  (about 0.6) matches the separate hand check, V2c. Bench item B3 replaces both with measured values.
- **K4 Pusher current.** On a fresh pack the 1400KV + 8x6 draws 17.5 A at full static throttle, against a
  15 A continuous rating.
  - `THR_MAX 85` holds it to 14.9 A. Sizing and validation now check this (V4e).
  - No A2212 pusher combination meets both 15 A and 2x excess thrust. The requirement becomes what 2x stood
    for, a climb rate: at least 1.5 m/s on the pusher alone at cruise speed. The design gets 1.9 m/s.
  - Peak pack draw is 69 A, so the pack must be 35C or better (a 30C 2200 mAh pack gives 66 A).
- **K5 Stall margin.**
  - `Q_ASSIST_SPEED` goes from 10 to 11.5 m/s and `Q_ASSIST_DELAY` from 1.0 to 0.5 s. The pitot removed the
    noise that forced the low setting.
  - `ROLL_LIMIT_DEG 40` keeps the banked stall speed (11.1 m/s) under both the assist speed and
    `AIRSPEED_MIN`.
  - The static margin target goes from 5.0 % to 5.5 %, so the 3 mm CG tolerance (1.1 % of MAC) still leaves
    at least 4 %.
- **K6 Rules.** The checklist now follows Transport Canada's rules for a small drone: registration, marking, a
  Basic certificate, and the Basic operating limits. It no longer refers to US rules.
- **K7 Harness.**
  - The BOM gains the power harness: XT60 pigtail, 14 AWG wire and ESC connectors.
  - The wiring diagram and bench item B19 cover BEC isolation and the PWM timer groups. No servo shares a
    timer with a lift motor.
- **K8 Spar joint.**
  - The printed boom clamp gains a 28 mm socket around the outer-spar root, so the outer panel's load goes
    into the clamp instead of through foam.
  - A new torsion check (V3g to V3i) treats the cross spar and the inboard wing box as one torsion path. The
    pultruded tube alone would twist 23° under full pitch command, so the wing box carries most of the
    torque, and the boom must be glued along the whole chord. Combined twist is 1.4°; bench item B18
    measures it.
- **K9 Radio.** A stock ELRS handset is listed as an optional item ($60–90) for first flights.
- **Smaller items.**
  - Motor tilt goes from 4° to 5°, because the derated props had left yaw authority 1 % short of 2x the
    winglet moment (V6c).
  - Printed-part masses now come from the exported volumes: 4 mounts, 4 legs, 2 clamps and the pylon,
    about 95 g with clamps and pylon included.
  - The boom bending mode is 25.5 Hz; the spec comment had said 37 Hz.
  - The "with reserve" endurance figure uses 64 % of pack energy (80 % usable x 80 %).
- **Cost ceiling.** `max_bom_usd` rises from $275 to $285 for the harness that had been missing, the 35C pack
  and the longer outer-spar tube.

## Consequences
- AUW rises from 1.065 to 1.091 kg. CG is 167 mm behind the root LE, and the battery moved 9 mm forward.
- Sizing endurance is 20.1 min (15.6 min with the 20 % reserve).
- The full SITL campaign was re-run with these settings. The results are in `docs/DESIGN-REVIEW.md` §9.
- The figures for prop derate, foamboard shear stiffness and allowable shear flow are estimates. Bench items
  B3 and B18 replace them with measured values.
