# Kestrel: a low-budget foam quadplane VTOL, designed and flown in code first

Kestrel is a 1.0 m foamboard flying wing with four lift motors and a pusher (a "4+1 quadplane"),
flown by ArduPlane 4.7.3. It takes off and lands like a quadcopter, and cruises like a plane.

- **Cost:** about $175–240 for the aircraft and $205–290 in total, including an Xbox-controller radio
  bridge in place of a transmitter (`build/bom.yaml`).
- **Design:** sized, shaped, built in CAD, balanced and flown in simulation before any part is
  bought. The plan and the reasoning are in `PLAN.md` (kept in the project folder).

## Design at a glance (all from `spec/vehicle.yaml`)

| | |
|---|---|
| Wing | 1.00 m span, S5020 reflexed airfoil, 0.32 / 0.22 m chord, 30° sweep, −8° washout, winglets |
| Weight | 1.04 kg all-up, from CAD mass properties (budget 1.05 kg) |
| Lift | 4 × A2212 1000KV + 10×4.5, T/W 2.8, hover at 49 % throttle, 86 W |
| Cruise | pusher A2212 1400KV + 8×6, 14 m/s, L/D 6.7, 51 W |
| Stall | 9.6 m/s, trimmed |
| Stability | static margin 9.3 % at cruise, 5 % minimum across 2–15° alpha |
| Endurance | 21.7 min modelled on a 3S 2200 pack, 17.1 min with a 20 % reserve |
| Build | 5 sheets of 30×20 in foamboard, 1:1 templates on 40 Letter pages, 3 printed parts |

## Verify (each milestone's command)

```bash
pip install -e .                       # Python 3.11+
pytest -q                              # 30 tests: propulsion, sizing, aero, CAD, 6-DOF physics
python -m kestrel.sizing               # M0: sizing, trade study, BOM -> build/sizing.json
python -m kestrel.aero                 # M1: aero tables (AeroSandbox + NeuralFoil) -> aero/tables.json
python -m kestrel.cad                  # M1: CAD, clearances, mass props, cut templates, STLs -> cad/
python -m kestrel.balance              # M1: CG / battery / rotor placement, written back to the spec
python -m kestrel.fly --mission basic  # M3: real ArduPlane SITL flying our physics (needs ArduPilot built)
python -m kestrel.mc --suite wind      # M4: wind 0-10 m/s with gusts
python -m kestrel.mc --suite faults    # M4: failures and model errors
python -m kestrel.mc --suite monte_carlo --n 200  # M4: Monte Carlo
ground/tests/run.sh                    # radio bridge: C++ host tests (+ASan/UBSan)
```

To set up SITL, check out ArduPilot tag `ArduPlane-stable` (4.7.3), run `./waf configure --board sitl && ./waf plane`,
and set `ARDUPILOT_DIR` (the default is `~/ardupilot`).

## Layout

| Path | What it is |
|---|---|
| `spec/vehicle.yaml` | single source of truth |
| `kestrel/` | sizing, aero, CAD, balance, 6-DOF sim, SITL bridge, mission runner, robustness suites |
| `sitl/kestrel.parm` | the ArduPlane parameter file you flash |
| `cad/` | templates PDF/SVG, STLs, STEP, mass properties, checks |
| `ground/` | Xbox → ESP32 → ELRS bridge (Arduino sketch, C++ core, tests) |
| `build/` | BOM, sizing output, bench and first-flight checklist (`BENCH.md`) |
| `docs/` | ADRs and the design review (`docs/DESIGN-REVIEW.md`) |
