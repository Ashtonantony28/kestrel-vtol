# 0003: 1.0 m swept flying wing, S5020 airfoil, −8° washout, balanced on the minimum local static margin

- Status: Accepted
- Date: 2026-11-22

## Context
The lean v2 plan dropped the tail to save foam, servos and weight. A tailless wing must trim with
small elevon deflection and stay statically stable across its alpha range. NeuralFoil's pitching
moment is nonlinear at Re ≈ 150k, so one static margin at a single point misleads.

## Decision
- **Airfoil:** S5020 (reflexed).
- **Planform:** root chord 0.32 m, tip chord 0.22 m, 30° LE sweep, −8° tip twist, 0.10 m winglets.
- **Elevons:** 25 % chord over 25–95 % semispan.
- **Balance:** the CG is placed so the minimum *local* static margin (−dCm/dCL over 2–15° alpha) is
  5 %. This gives 9.3 % at cruise. `kestrel.balance` slides the battery and places the rotor
  centroid on the CG.
- **Polar:** stall and cruise use the *trimmed* polar, from an alpha × elevon grid.

## Alternatives rejected
- **NACA 2412 / Clark-Y:** needed about 10° of up-elevon to trim. That costs CLmax and drag.
- **Less washout:** the tips stall first, giving roll-off at stall.
- **0.9 m span:** trimmed stall above 10.5 m/s. Too fast for hand-held field work.

## Consequences
- **Results:** trimmed CLmax 0.67, stall 9.6 m/s, cruise 14 m/s at alpha 6.8° with −1° elevon, L/D 6.7.
- **Weak dihedral effect:** Clb is slightly positive. The winglets don't fully cure it, and the autopilot copes in SITL.
- **Verify:** `pytest -q tests/test_aero.py tests/test_sim.py`.
