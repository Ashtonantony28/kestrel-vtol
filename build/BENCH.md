# Kestrel bench and first-flight checklist

The cloud work (M0–M4) proves the design *given the models*. These checks measure the real parts
and close that gap. Do them in order, and **with props off** unless a step says otherwise. Each item
names the model value it checks and what to do if the real value is off.

| # | Check | How | Pass | If it fails |
|---|---|---|---|---|
| B1 | Weigh every part | kitchen scale, fill `spec/vehicle.yaml` masses | AUW ≤ 1.12 kg (model 1.091) | re-run `python -m kestrel.balance` and `kestrel.sizing` |
| B2 | CG | balance on fingertips at the marked CG line (`cad/out/` template mark, 167 mm behind root LE; battery centre 19 mm behind the root LE) | level ±3 mm (3 mm = 1.1 % of MAC; the 5.5 % margin target keeps ≥ 4 %, design review K5) | slide the battery; never add tail weight blindly |
| B3 | Lift motor thrust | motor on a kitchen scale, prop **on**, 50 % and 100 % throttle, wattmeter | ≥ 5.8 N at 100 % (model 6.3 with the derated prop, `spec prop_derate`); hover ≤ 5.4 A each (model 4.9) | put the measured thrust and watts into `prop_derate.lift_cp_scale`, then re-run sizing, SITL and validate; if thrust is short, fit props from a different batch or reduce AUW |
| B4 | Pusher static thrust | same rig | at `THR_MAX 85` on a full pack: ≥ 5.9 N (model 6.6) and ≤ 15 A (motor continuous rating, design review K4) | over 15 A: lower `THR_MAX`; short of thrust: 9×6 adds thrust but also current, so re-check the 15 A limit |
| B5 | Motor direction and tilt | ArduPilot motor test, props off; look along each boom | 1 FR CCW, 2 RL CCW, 3 FL CW, 4 RR CW (ADR 0001). Each mount leans 5° so the prop's thrust pushes the arm the same way the prop's drag twists the airframe: FR and RR lean right (outboard on the right boom), FL and RL lean left (outboard on the left boom), i.e. all four lean outboard (ADR 0004) | swap any two motor wires; turn a mount 180° on its boom |
| B6 | Elevon direction | FBWA, tilt the aircraft by hand (it should push back) | right wing dipped by hand → right elevon goes down; nose dipped → both go up; right stick right → right elevon goes up | `SERVO1/2_REVERSED`, or swap 77/78 |
| B7 | Elevon throw | measure the trailing edge of both segments | ±20° (= ±27 mm at the elevon root); the outer segment follows the inner one within 1° (torque rod, ADR 0006); full down throw clears the boom | servo horn hole; re-glue the torque rod if the segments twist apart |
| B8 | Battery sag | 10 A load, read voltage | ≥ 11.1 V on a full pack | the pack is tired; endurance is the first thing to suffer |
| B9 | Radio bridge | `ground/tests/run.sh` passes on a PC; then the real hardware: ELRS binds, Mission Planner radio calibration shows each axis moving the correct way (ground/README.md); turning the controller off triggers RC failsafe within 1.5 s | all pass | baud: set `kCrsfBaud` to 400000; pitch: `RC2_REVERSED 1` |
| B10 | Failsafe on the bench | props off, armed in QHOVER, controller off | the mode changes to RTL/QLAND within `FS_LONG_TIMEOUT` (5 s) | check `FS_LONG_ACTN`, `THR_FAILSAFE` |
| B11 | Vibration | first hover log | VIBE x/y/z < 30 m/s², no clipping | balance the props, soft-mount the FC |
| B12 | Boom and spar hang test | aircraft upside down on a table, hang 2 kg (about 1.5× one boom's full-thrust load) from each motor mount in turn | no cracking sound, mount returns flat, tip moves ≤ 5 mm (validation V3d: 3.2 mm at full thrust) | re-glue the boom clamp to the cross spar; check the spar is fully inside the foam |
| B13 | Pack C rating | read the label | ≥ 35C for 2200 mAh (peak draw 69 A, validation V4c; a 30C pack is rated only 66 A) | buy a 35–40C pack |
| B15 | Build features from the interference audit | look before closing the hatch | each elevon in two segments around the boom with a 2 mm carbon torque rod along the hinge; battery strap through the two pod-floor slots; lift-motor leads in the channel on top of each leg; FC behind the battery, not on it | `python -m kestrel.interference` lists every slot and pocket the build needs (`ALLOWED`) |
| B16 | Airspeed sensor | props off, FC on USB, Mission Planner | blow gently into the pitot tip: airspeed rises, then returns to 0 ±1 m/s; hose not kinked; first flight: `ARSPD_AUTOCAL 1` for a few minutes of loiter, then save `ARSPD_RATIO` (ADR 0007) | swap the two hoses (pitot and static reversed); re-seat the I2C plug |
| B14 | Boom notch filter | first hover log, FFT in Mission Planner | boom bending peak near 25–29 Hz (validation V3e) and the 1P rotor peak near 71 Hz are attenuated | set `INS_HNTCH_ENABLE 1`, `INS_HNTCH_MODE 3` (ESC telemetry) or `1` (throttle), `INS_HNTCH_FREQ` to the measured hover peak |
| B17 | Hardware parameters | props off: load `sitl/kestrel.parm`, then `sitl/kestrel_hw.parm`, reboot | ELRS on SERIAL1 shows RC input; GPS on SERIAL3 gets a fix; `ARSPD_BUS 0` finds the MS4525 (without it the sensor is never found); the CH5 switch walks through QLOITER / QHOVER / CRUISE / AUTO / RTL / QLAND; LB + RB held 1.5 s arms and disarms | check each pad against the SpeedyBee manual; the SBUS pad does not take CRSF |
| B18 | Boom pitch stiffness | aircraft level on the bench, wing clamped at the pod; hang 1 kg from the front motor mount (about 1.5× the full-thrust pitch torque) | the boom pitches ≤ 2° (validation V3i: 1.4° from the cross spar plus the wing box; the tube alone would twist about 23°) | the boom must be glued to the wing underside along the whole chord and the clamp glued into its pocket; add a second glue line, then re-test |
| B19 | Power harness and BECs | props off, multimeter | the red wire is pulled from all five ESC signal plugs; only the FC's BEC powers the servo rail (5.0–5.3 V); servos on S1–S2 (timer 4) and pusher on S3 (timer 3) never share a timer with the lift motors on S5–S8 (timers 8 and 2) | pull the red lead; never move a servo onto S5–S8 |

## Before the first flight: Canadian rules (Transport Canada, CARs Part IX)

Kestrel weighs about 1.09 kg, so it is a *small drone* (250 g to 25 kg). Building it yourself changes nothing:
- **Register it** in the Drone Management Portal and **mark the registration number** on the airframe
  (permanent marker, label or engraving). Carry proof of registration when flying.
- **Pilot certificate, Basic operations:** pass the online Small Basic exam (35 questions, 90 min, 65 % to pass,
  minimum age 14). The certificate is included in the exam fee and does not expire. Carry it when flying.
- **Basic operations limits:** always in visual line of sight; more than 30 m horizontally from any person;
  uncontrolled airspace only; more than 5.6 km from the centre of a certified airport and 1.9 km from a heliport;
  at or below 122 m (400 ft) above ground.
- Flying closer than 30 m to people, or in controlled airspace, needs an Advanced certificate.
- Fees change every April 1 with inflation; check the portal for the current registration and exam fees.
- Fines for individuals go up to $5,000 for an unregistered or unmarked drone and $1,000 for flying without a certificate.

Source: Transport Canada, [Flying your drone safely and legally](https://tc.canada.ca/en/aviation/drone-safety/new-rules-drones-canada),
[Registering your drone](https://tc.canada.ca/en/aviation/drone-safety/registering-your-drone-overview),
[Basic operations](https://tc.canada.ca/en/aviation/drone-safety/learn-rules-you-fly-your-drone/drone-operation-categories-pilot-certificates/basic-operations) (pages dated 2025-11-04, read 2026-11-25).

## First flights (light wind < 4 m/s; a proven ELRS handset is recommended for these, BOM optional item)

1. **Tethered hover, 1 m, QHOVER.** Check that `Q_M_THST_HOVER` learns to 0.48–0.60 (model 0.53). If it settles above 0.58, the props are worse than the derate assumes: measure them (B3).
2. **QLOITER hover, 5 m, 2 minutes.** Run QAUTOTUNE on roll and pitch only if oscillating.
3. **Hover segments stay low and short.** A single lift-motor failure in hover cannot be survived on a
   quad. SITL `lift_motor1_fails_in_hover` crashes, as expected. Keep transitions above 15 m and
   hovering below 10 m where possible.
4. **First transition:** AUTO, a straight 300 m leg into the wind, with `Q_ASSIST_SPEED 11.5` as the safety net (lift motors help below 11.5 m/s; stall at the 40° bank limit is 11.1 m/s).
5. **CRUISE handling:** check that `TRIM_THROTTLE` matches the 14 m/s pusher duty (model 65 %), then fly a few minutes of loiter
   with `ARSPD_AUTOCAL 1` and save `ARSPD_RATIO` (B16).

Wind envelope demonstrated in SITL: 10 m/s mean + 1.5 m/s gusts (7/7 runs, after the fixed-wing landing approach, ADR 0008). The first flights are still for calm days.
