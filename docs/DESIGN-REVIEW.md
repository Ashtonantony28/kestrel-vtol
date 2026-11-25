# Kestrel design review (M0–M4 complete 2026-11-22; independent validation 2026-11-23)

**Verdict:** the lean v2 design closes. On paper and in simulation it meets every sizing target. It flies
ArduPlane 4.7.3 missions hands-off in winds up to 10 m/s. Since 2026-11-24 that needs the $14–22 airspeed sensor (ADR 0007); without it the limit is 7 m/s. It survives the
single failures that a quad can survive, and lands with a 20 % battery reserve after about 18 minutes.

The most useful result is the list of seven problems that the first parameter file had (ADR 0004).
All seven were found and fixed in simulation, at $0, before buying anything.

What simulation cannot prove is listed at the end. The bench checklist (`build/BENCH.md`) measures those items.

## 1. How it was tested

| Layer | Tool | Verify command | Result |
|---|---|---|---|
| Sizing, BOM | own models, datasheet-fitted motor/prop | `python -m kestrel.sizing` | 9/9 checks pass |
| Aero | AeroSandbox 4.2.10 AeroBuildup + NeuralFoil, trimmed alpha × elevon tables | `python -m kestrel.aero` | see §2 |
| CAD | build123d: clearances, mass properties, nesting, 1:1 templates | `python -m kestrel.cad` | 5 sheets, no collisions, mass −1.5 % vs budget |
| 6-DOF physics | own pure-Python model, analytic tests | `pytest -q` | 32 pass (+2 SITL) |
| Flight software | **real ArduPlane 4.7.3** in SITL, lockstep JSON link to our physics | `KESTREL_SITL=1 pytest -q tests/test_sitl.py` | hover and full mission pass |
| Robustness | wind, faults, endurance, Monte Carlo | `python -m kestrel.mc --suite …` then `python -m kestrel.report` | see §3 |
| Radio bridge | C++ core host tests, ASan/UBSan, sketch type-check | `ground/tests/run.sh` | 2,276 checks pass |
| Independent validation | vortex lattice, DATCOM, momentum theory, beam theory, time-step convergence, control authority | `python -m kestrel.validate` | 32/32 pass (§6) |

## 2. Design numbers (all derived from `spec/vehicle.yaml`)

| | Value | Target |
|---|---|---|
| All-up weight (CAD) | 1.055 kg | ≤ 1.10 kg |
| Hover T/W (full / empty pack) | 2.75 / 2.47 | ≥ 2.0 / 1.6 |
| Hover throttle, power | 49 %, 88 W | ≤ 60 % |
| Cruise | 14 m/s, L/D 6.6, 53 W, pusher 65 % | |
| Pusher excess thrust at cruise | 2.7× | ≥ 2 |
| Trimmed stall | 9.7 m/s (CLmax 0.67 trimmed) | ≤ 10.5 m/s |
| Static margin | 9.4 % at cruise; 5.1 % minimum over 2–15° | 4–16 % |
| Cruise trim | alpha 6.8°, elevon −1° | small elevon |
| Endurance, model | 21.2 min to empty, 16.7 min with 20 % reserve | ≥ 12 min |
| Endurance, SITL (mission flown until the reserve trigger, then RTL and land) | **17.2 min** nominal, **15.1 min** pessimistic (rerun 2026-11-23 after the spar fix; 18.3 / 15.5 before) (+25 % drag, +8 % mass, −5 % thrust, −10 % capacity), both landing with 20 % left | |
| Aircraft / total cost | $183–250 / $211–302 | ≤ $250 aircraft (was $240; ADR 0005) |

## 3. SITL results (final parameter file, `sitl/kestrel.parm`)

**Mission** ("basic"): VTOL takeoff to 30 m, transition, 4 waypoints over 250 m legs at 40–50 m, back-transition and VTOL landing at home.

| Suite | Result | Notes |
|---|---|---|
| Calm | pass, lands 0.03 m from home | `docs/img/mission_calm.png` |
| Wind, 0/4/7/10 m/s steady + 1.5 m/s gusts, two directions | **7/7**; worst landing 1.9 m | `docs/img/mission_wind7.png` |
| Same, with the optional $15 airspeed sensor | 7/7 | at the time the sensor was *not* needed; superseded by §8 and ADR 0007 |
| Dead pusher in cruise | pass, lands 0.03 m | the transition times out, and it flies home as a quad |
| RC link lost in cruise | pass | RTL, VTOL landing at home |
| Left elevon stuck in cruise | pass | the lift motors catch the slow sink below 15 m |
| CG ±10 mm, +15 % mass, −10 % thrust, +30 % drag, warped wing | 6/6 pass | |
| Lift motor 1 fails in hover | **crash, as expected** | no quad survives this; see the operational mitigation |
| Monte Carlo, 100 runs (wind 0–7 m/s, gusts 0–2 m/s, ±4 % mass, ±6 mm CG, ±12 % drag, lift/Cm/roll-trim errors, motor spread, battery resistance 0.8–1.6×) | **99/100 pass** | rerun 2026-11-23: landing error median 0.67 m, 95th percentile 1.95 m (completed runs), 94 of 100 within 2 m. Touchdown median 0.51 m/s, max 0.87 m/s. `docs/img/monte_carlo.png` |

The one Monte Carlo miss (rerun: mc_093, 1.75 m/s gusts) completed the mission without crashing but landed
5.3 m from the pad at 1.5 m/s, after tilting up to 44° in the descent. The first run's miss (mc_049) was the
same pattern: gusts on the wing during the last metres of descent, when the lift motors have little thrust
in reserve. It is an open item to look for in the first real landing logs (M6).

The plan's M4 target was "≥ 98 % complete and within 2 m". Both are now met: 99 % complete, and the 95th
percentile landing error is 1.95 m (the first run, before the re-balance, reached 2.1 m).

## 4. Problems found and fixed (details in ADR 0004)

1. In wind, the synthetic-airspeed estimate kept re-triggering the transition: assist speed lowered to 10, with a 1 s delay.
2. At 10 m/s, a wing hovering side-on saturated a lift motor: weathervaning turned on.
3. A flying wing leaning into the wind in hover makes downforce (alpha −27°): the pusher now holds position.
4. In gusty crosswind, yaw authority ran out: the lift motors now tilt 4° outboard, which roughly triples yaw authority.
   The motor-mount STL was updated to match.
5. A dead pusher meant the aircraft circled in a "transition" until the pack died: there is now a 45 s transition time-out, then QRTL.
6. A stuck elevon sank the wing into the ground at cruise speed: the lift motors now assist below 15 m.
7. The voltage-based low-battery warning came with about 1 minute left: the reserve is now capacity-based (RTL at 30 % left, QLAND at 15 %).

Along the way, the tests also caught bugs in our own tools:

- **Radio bridge:** the link watchdog's timeout was effectively 600 ms instead of 300.
- **Sim battery:** it never ran out.
- **Templates:** stale template sheets were left behind after a re-nest.
- **CSV logs:** they recorded the commanded elevon angle, not the actual one.

## 5. Known limits and open items

- **Wind:** the design is demonstrated to 10 m/s mean with 1.5 m/s gusts. Fly the first flights in under 4 m/s,
  and treat 7 m/s as the working limit until real logs agree with the sim.
- **Aerodynamics are modelled, not measured.**
  - NeuralFoil and AeroBuildup at Re ≈ 150k on a foam wing with a sharp-ish leading edge are approximate.
  - CLmax carries a 0.9 foam factor.
  - Clb (dihedral effect) is slightly positive. That is weak, but the autopilot copes in SITL.
  - B3/B4 thrust tests and M6 log comparison calibrate the model.
- **Not modelled:**
  - propwash over the wing during transition
  - ground effect
  - servo slop
  - flexing of the foam wing
  - GPS and compass errors (SITL's defaults are used)
- **The radio bridge has not been compiled against the real Bluepad32 core** (the sandbox could not install the
  Arduino toolchain), and the ELRS baud rate (400k or 420k) is unconfirmed. Both are bench item B9.
- **The M3 cross-check against SITL's built-in quadplane model was not done.** Running the same mission on that generic model would check the mission scripts, not this aircraft,
  so it adds little and is optional.
- **Single lift-motor failure in hover cannot be survived** (true of every quad). Hover low and briefly.

## 6. Independent validation (2026-11-23)

The design tools and the SITL suites share models, so they can agree and still be wrong. `kestrel/validate.py`
re-derives the key numbers by a second method or checks a physical limit the design tools never looked at.
All 32 checks pass; details in `build/validation.json`.

| Group | Method | Result |
|---|---|---|
| V1 aero | vortex lattice vs AeroBuildup vs DATCOM lift slope (3.63 / 3.95 / 3.44 /rad); neutral point three ways (201 / 204 / 193 mm); wing self-trims (Cm about NP +0.023) | 6/6 |
| V2 hand calcs | stall 9.7 m/s; hover power by momentum theory 105 W vs model 88 W (model implies FM×η 0.53); cruise power; endurance by hand and from the SITL flight (18.1 vs 21.2 min) | 5/5 |
| V3 structure | spar and boom stress at 4 g × 1.5 (129, 93, 67 MPa vs 600); boom tip deflection 3.6 mm; boom first mode 25.5–28.6 Hz; twist 0.72° | 10/10 |
| V4 limits | tip Mach 0.33 / 0.37; motor current 14 / 18.9 A; **peak pack current 65.3 A vs 66 A for a 30C pack**; empty-pack T/W 2.47 | 6/6 |
| V5 numerics | 1.2 kHz step within 0.006 % of a 4.8 kHz reference (hover and glide) | 2/2 |
| V6 control | roll 334 deg/s at half throw; elevon to trim at CLmax 31 % of throw; quad yaw moment 2.05× the winglet moment in a 7 m/s crosswind | 3/3 |

What validation changed:

1. **Spar layout (ADR 0005).** A straight quarter-chord spar cannot follow a 30° swept wing; it would exit the
   leading edge 0.14 m from the centreline. Replaced by an 8×6 mm cross spar at 0.18 m behind the root LE
   (booms clamp to it) and 6×4 mm outer spars along the quarter chord. Adds $6–10, so the aircraft ceiling
   moves from $240 to $250.
2. **Re-balance.** Minimum static margin 3.8 % → 5.1 %.
3. **Bench items** B12 (2 kg hang test per motor mount), B13 (pack ≥ 35C), B14 (harmonic notch for the boom mode).

## 7. Detailed CAD and interference audit (2026-11-24, ADR 0006)

The first 3D model was a sizing model and had real geometry faults. It was rebuilt as a 67-part build
assembly (`kestrel/cad_detail.py`) and audited with `python -m kestrel.interference`:

- every solid is valid and closed
- every pair of parts is intersected; each overlap above 1 mm³ is either a listed build feature (slot,
  pocket, hole, glue joint: 38 of them) or a failure
- each prop disc is swept at full diameter (minimum gap 25.5 mm, required 6 mm), and each elevon is
  rotated to ±20° about its hinge line

Faults found and fixed:

| Fault | Fix |
|---|---|
| Boom under the elevon blocks full down throw | elevon split around the boom, torque rod links the segments |
| Flight controller inside the battery and XT60 | FC moved behind the battery; GPS onto a mast; re-balanced |
| Winglets 5–23 mm off the tip rib | winglet placed on the twisted tip chord |
| Legs inside the motor mount saddle | legs 28 mm inboard of the motor |
| Motor leads through the legs | lead channel on top of each printed leg |
| Elevon horn into the fixed wing (swept hinge) | horn placed on the hinge station at the servo arm |
| Horn hits the torque rod | servo moved to y = 200 mm |
| Pusher prop hub inside the motor | prop 33.5 mm aft of the pylon plate |
| Battery strap through the pod floor | two strap slots in the floor (listed build feature) |
| Faceted polyline wing loft | spline wing sections |

Outputs: `docs/DIMENSIONS.md` (63 dimensions), `cad/drawings/kestrel_drawings.pdf` (three A3 sheets),
`cad/detail/interference.json`, and the browser viewer in `cad/viewer/`. `tests/test_detail.py` keeps the
audit green.

## 8. Airspeed sensor fitted (2026-11-24, ADR 0007)

The rerun after the elevon split showed that the sensorless synthetic airspeed read about 1.2 m/s low. The
aircraft flew fast (endurance 18.3 min, V2e failed), and in a 10 m/s beam wind it falsely tripped the
transition-fail fallback and flew home as a quad until the battery ran low (344 m and 707 m from the pad on
two runs). The user chose to fit an MS4525DO pitot: pitot on the pod nose, board on the pod floor, 10 g, battery
13 mm aft after re-balance. CAD audit: 70 parts, 0 conflicts.

Full SITL rerun with the sensor:

| Suite | Result |
|---|---|
| Wind 0–10 m/s, two directions | 7/7 (worst 2.1 m from the pad, 10 m/s beam wind) |
| Faults and model errors | 9/10 (lift motor out in hover crashes, as expected for a quad) |
| Endurance | 2/2; nominal 19.8 min flown, 20.1 min scaled to 80 % usable (sizing model 21.7, V2e passes) |
| Monte Carlo | 98/100; median 0.64 m, p95 2.3 m, 89 within 2 m; the two misses landed 7.1 m and 24.8 m off after a long VTOL approach in 6 m/s wind |

Validation: 32/32.

One harness flake: on the first run of `cg_10mm_forward` the autopilot's estimator stopped GPS aiding
just after take-off at 12 m. The AHRS fell back to DCM, and the aircraft crashed. A rerun of the same case
landed 0.03 m from the pad, and no other run in the campaign showed this. The harness arms as soon as one
EKF lane uses GPS. On the real aircraft, arming checks and a longer GPS settle before take-off cover this.
A harness change to wait for both lanes is an open item.
