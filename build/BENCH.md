# Kestrel bench and first-flight checklist

The cloud work (M0–M4) proves the design *given the models*. These checks measure the real parts
and close that gap. Do them in order, and **with props off** unless a step says otherwise. Each item
names the model value it checks and what to do if the real value is off.

| # | Check | How | Pass | If it fails |
|---|---|---|---|---|
| B1 | Weigh every part | kitchen scale, fill `spec/vehicle.yaml` masses | AUW ≤ 1.10 kg (model 1.04) | re-run `python -m kestrel.balance` and `kestrel.sizing` |
| B2 | CG | balance on fingertips at the marked CG line (`cad/out/` template mark, 168 mm behind root LE) | level ±3 mm | slide the battery; never add tail weight blindly |
| B3 | Lift motor thrust | motor on a kitchen scale, prop **on**, 50 % and 100 % throttle, wattmeter | ≥ 6.4 N at 100 % (model 7.1); hover ≤ 4.2 A each | fit 1000KV + 10×4.5 from a different batch, or reduce AUW |
| B4 | Pusher static thrust | same rig | ≥ 6.0 N (model 6.95) | the 8×6 prop is the knob: 8×6 → 9×6 adds thrust and current, so re-check ESC amps |
| B5 | Motor direction and tilt | ArduPilot motor test, props off; look along each boom | 1 FR CCW, 2 RL CCW, 3 FL CW, 4 RR CW (ADR 0001). Each mount leans 4° so the prop's thrust pushes the arm the same way the prop's drag twists the airframe: FR and RR lean right (outboard on the right boom), FL and RL lean left (outboard on the left boom), i.e. all four lean outboard (ADR 0004) | swap any two motor wires; turn a mount 180° on its boom |
| B6 | Elevon direction | FBWA, tilt the aircraft by hand (it should push back) | right wing dipped by hand → right elevon goes down; nose dipped → both go up; right stick right → right elevon goes up | `SERVO1/2_REVERSED`, or swap 77/78 |
| B7 | Elevon throw | measure the trailing edge | ±20° (= ±27 mm at the elevon root) | servo horn hole |
| B8 | Battery sag | 10 A load, read voltage | ≥ 11.1 V on a full pack | the pack is tired; endurance is the first thing to suffer |
| B9 | Radio bridge | `ground/tests/run.sh` passes on a PC; then the real hardware: ELRS binds, Mission Planner radio calibration shows each axis moving the correct way (ground/README.md); turning the controller off triggers RC failsafe within 1.5 s | all pass | baud: set `kCrsfBaud` to 400000; pitch: `RC2_REVERSED 1` |
| B10 | Failsafe on the bench | props off, armed in QHOVER, controller off | the mode changes to RTL/QLAND within `FS_LONG_TIMEOUT` (5 s) | check `FS_LONG_ACTN`, `THR_FAILSAFE` |
| B11 | Vibration | first hover log | VIBE x/y/z < 30 m/s², no clipping | balance the props, soft-mount the FC |

## First flights (at a FRIA field or with Remote ID fitted, light wind < 4 m/s)

1. **Tethered hover, 1 m, QHOVER.** Check that `Q_M_THST_HOVER` learns to 0.45–0.55 (model 0.49).
2. **QLOITER hover, 5 m, 2 minutes.** Run QAUTOTUNE on roll and pitch only if oscillating.
3. **Hover segments stay low and short.** A single lift-motor failure in hover cannot be survived on a
   quad. SITL `lift_motor1_fails_in_hover` crashes, as expected. Keep transitions above 15 m and
   hovering below 10 m where possible.
4. **First transition:** AUTO, a straight 300 m leg into the wind, with `Q_ASSIST_SPEED 10` as the safety net.
5. **CRUISE handling:** check that `TRIM_THROTTLE` matches the 14 m/s pusher duty (model 64 %), then calibrate
   synthetic airspeed against GPS groundspeed on reciprocal legs.

Wind envelope demonstrated in SITL: 10 m/s mean + 1.5 m/s gusts (7/7 runs). The first flights are still for calm days.
