# 0002: ArduPilot Plane on a stock wing FC, own code only on the ground and in simulation

- Status: Accepted
- Date: 2026-11-22

## Context
Transitions, Q_ASSIST, weathervaning, failsafes and the EKF are where quadplanes crash. Mature
open-source firmware already handles them. Writing our own firmware (as in neuroflight-fc) would
add months and real crash risk to a ~$200 build.

## Decision
- **Flight controller:** ArduPlane 4.7.3 (tag ArduPlane-stable) QuadPlane on a SpeedyBee F405 Wing Mini.
- **Our own work:**
  - the vehicle physics (`kestrel/sim.py`), which flies the real ArduPlane binary through the SITL JSON backend in lockstep
  - the parameter file (`sitl/kestrel.parm`)
  - the ground radio bridge (`ground/`)
- **Not used:** ArduPilot's built-in quadplane model. SITL runs our own physics, built from the aero tables and CAD mass properties.

## Consequences
- The parameter file flown in SITL is the file flashed on the aircraft. Bench and first-hover checks
  (build/BENCH.md) close the gap between the model and the real aircraft.
- Firmware updates are a reviewed change: rerun `python -m kestrel.mc --suite faults` and `--suite wind`.
