# Kestrel: a low-budget foam quadplane VTOL, designed and flown in code first

Kestrel is a 1.0 m foamboard flying wing with four lift motors and a pusher (a "4+1 quadplane"),
flown by ArduPlane 4.7.3. It takes off and lands like a quadcopter, and cruises like a plane.

- **Cost:** about $197–272 for the aircraft and $225–324 in total, including an Xbox-controller radio
  bridge in place of a transmitter (`build/bom.yaml`).
- **Design:** sized, shaped, built in CAD, balanced and flown in simulation before any part is
  bought. The plan and the reasoning are in `PLAN.md` (kept in the project folder).

## Design at a glance (all from `spec/vehicle.yaml`)

| | |
|---|---|
| Wing | 1.00 m span, S5020 reflexed airfoil, 0.32 / 0.22 m chord, 30° sweep, −8° washout, winglets |
| Weight | 1.091 kg all-up, from CAD mass properties (budget 1.10 kg) |
| Lift | 4 × A2212 1000KV + 10×4.5, T/W 2.36, hover at 53 % throttle, 117 W (props derated to figure of merit 0.65, ADR 0008) |
| Cruise | pusher A2212 1400KV + 8×6 at `THR_MAX 85` (≤ 15 A), 14 m/s, L/D 6.9, 53 W, climb 1.8 m/s; MS4525DO pitot airspeed sensor (ADR 0007) |
| Stall | 10.0 m/s trimmed; 11.4 m/s at the 40° bank limit, under `Q_ASSIST_SPEED 11.5` |
| Stability | static margin 9.4 % at cruise, 5.5 % minimum across 2–15° alpha (≥ 4.4 % within the ±3 mm CG tolerance) |
| Endurance | 20.1 min modelled on a 3S 2200 pack (35C or higher), 15.6 min with a 20 % reserve (64 % of pack energy); SITL 19.5 min |
| Structure | 10×8 mm carbon booms clamped to an 8×6 mm cross spar; 6×4 mm outer spars along the swept quarter chord, their roots socketed in the clamps (ADR 0005, 0008) |
| Build | 5 sheets of 30×20 in foamboard, 1:1 templates on 40 Letter pages, 5 printed part types (lift motors tilted 5° outboard); STEP/STL in `cad/build/`, wiring in `cad/drawings/wiring_diagram.pdf` |
| Validated | 36/36 independent cross-checks (vortex lattice, DATCOM, momentum and beam theory, convergence, control authority) |
| Tested | ArduPlane SITL after the design-review fixes: wind to 10 m/s 7/7, faults 9/10, Monte Carlo 100/100 (92 within 2 m, worst 3.5 m), endurance 2/2. Detailed CAD: 70 parts, 0 interference conflicts (ADR 0006) |

## Verify (each milestone's command)

```bash
pip install -e .                       # Python 3.11+
pytest -q                              # 32 tests (+2 SITL flights with KESTREL_SITL=1)
python -m kestrel.sizing               # M0: sizing, trade study, BOM -> build/sizing.json
python -m kestrel.aero                 # M1: aero tables (AeroSandbox + NeuralFoil) -> aero/tables.json
python -m kestrel.cad                  # M1: CAD, clearances, mass props, cut templates, STLs -> cad/
python -m kestrel.balance              # M1: CG / battery / rotor placement, written back to the spec
python -m kestrel.fly --mission basic  # M3: real ArduPlane SITL flying our physics (needs ArduPilot built)
python -m kestrel.mc --suite wind      # M4: wind 0-10 m/s with gusts
python -m kestrel.mc --suite faults    # M4: failures and model errors
python -m kestrel.mc --suite monte_carlo --n 100  # M4: Monte Carlo (add --no-airspeed-sensor for the sensorless comparison)
python -m kestrel.report                # M4: summary -> build/sitl_results.json, docs/img/
python -m kestrel.validate              # independent cross-checks (32) -> build/validation.json
python -m kestrel.cad_detail            # detailed 67-part build assembly -> cad/detail/checks.json
python -m kestrel.interference          # pairwise interference + swept props/elevons, exit 1 on conflict (ADR 0006)
python -m kestrel.dimensions            # full dimension table -> build/dimensions.json, docs/DIMENSIONS.md
python -m kestrel.drawing               # 3 dimensioned A3 sheets -> cad/drawings/kestrel_drawings.pdf
python -m kestrel.export_viewer         # meshes, dimensions and audit for the browser 3D viewer -> cad/viewer/
ground/tests/run.sh                    # radio bridge: C++ host tests (+ASan/UBSan)
```

To set up SITL, check out ArduPilot tag `ArduPlane-stable` (4.7.3), run `./waf configure --board sitl && ./waf plane`,
and set `ARDUPILOT_DIR` (the default is `~/ardupilot`).

## Layout

| Path | What it is |
|---|---|
| `spec/vehicle.yaml` | single source of truth |
| `kestrel/` | sizing, aero, CAD, balance, 6-DOF sim, SITL bridge, mission runner, robustness suites |
| `sitl/kestrel.parm` | the ArduPlane parameter file you flash (flight tuning, tested in SITL) |
| `sitl/kestrel_hw.parm` | real-board settings loaded after it: serial ports, receiver, airspeed bus, mode and arm switches, RC failsafe |
| `cad/` | templates PDF/SVG, STLs, STEP, mass properties, checks; `cad/viewer/` is a three.js 3D viewer |
| `ground/` | Xbox → ESP32 → ELRS bridge (Arduino sketch, C++ core, tests) |
| `build/` | BOM, sizing output, bench and first-flight checklist (`BENCH.md`) |
| `docs/` | ADRs and the design review (`docs/DESIGN-REVIEW.md`) |
