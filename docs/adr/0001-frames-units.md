# 0001: Frames, units and conventions

- Status: Accepted
- Date: 2026-11-22

## Context
Four tools touch the vehicle geometry: AeroSandbox, build123d, our 6-DOF model and ArduPilot's
JSON SITL backend. Each uses a different axis convention.

## Decision
- **Units:** SI everywhere (m, kg, s, rad internally; degrees only in YAML and reports).
- **Body frame:** FRD (x forward, y right, z down). The origin is the wing root leading edge.
- **World frame:** NED.
- **Attitude:** quaternion body→NED, scalar first. Euler angles are 3-2-1 and used for reporting only.
- **CAD (build123d) frame:** FLU, x forward from the nose. `cad.mass_properties` converts to FRD by
  flipping y and z and shifting by `fuselage.nose_x`.
- **Elevon sign:** positive = trailing edge down.
  - ArduPlane elevon PWM > 1500 means trailing edge up, so the bridge negates it.
  - AeroSandbox's antisymmetric deflection puts the right trailing edge down, so `aero.py` flips the sign of Clda and Cnda.
  - Roll command: da = (left − right) / 2.
- **Motor order and spin** follow ArduPilot Q_FRAME_TYPE 1 (X):
  - 1 FR CCW, 2 RL CCW, 3 FL CW, 4 RR CW.
  - A CCW prop's reaction torque yaws the airframe nose right (+r).

## Consequences
Sign conventions are pinned by tests: `test_sim.py::test_yaw_sign` and `::test_control_signs`.
A sign error would show up there before it reaches SITL.
