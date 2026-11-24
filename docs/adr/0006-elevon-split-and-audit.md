# 0006: Split elevons around the booms, and audit every CAD change for interference

- Status: Accepted
- Date: 2026-11-24

## Context
The first 3D model was a sizing model: parts were boxes and cylinders placed at their mass stations, and
nothing checked whether they fit together. A review of that model found real geometry faults, not just
cosmetic ones. The worst was that the 10 mm boom runs under the elevon at y = 0.25 m: at full down throw
the elevon would hit the boom, so roll authority on that side would be blocked. Others were the flight
controller sitting inside the battery and XT60, the winglets floating 5 to 23 mm off the tip, the legs
inside the motor mounts, and the pusher prop hub inside its motor.

## Decision
- **Detailed assembly.** `kestrel/cad_detail.py` builds the airframe as a build assembly (67 solids): spline
  wing sections, real motor, prop, servo and connector shapes, printed parts with fillets.
- **Interference audit.** `python -m kestrel.interference` intersects every pair of solids. An overlap above
  1 mm³ is either listed in `ALLOWED` with the build step that makes room for it (a slot, a pocket, a hole)
  or the audit fails. It also sweeps each prop disc (full diameter, 6 mm minimum gap) and each elevon at
  ±full throw. CAD is not shown or merged unless the audit passes.
- **Elevon split.** Each elevon is cut into two segments around the boom (`wing.elevon_gap`, 34 mm wide at
  y = 0.25 m). One SG90 drives both through a 2 mm carbon torque rod glued along the hinge line. The servo
  moved to y = 0.20 m so the horn clears the torque rod.
- **Aero model of the gap.** Adding the gap as extra wing cross-sections changed the AeroBuildup neutral
  point by 15 mm (201, 195, 186 mm for 2, 4 and 6 sections) while VLM stayed near 200 mm, so the shift is
  an artifact of the build-up method, not physics. The model keeps the four-section layout and scales
  elevon deflection by `k = 1 - gap / elevon span` (0.91).
- **Avionics relocation.** The flight controller moved behind the battery (x = -0.110 m, z = 0.040 m), and
  the GPS moved onto a mast ahead of the cross spar (x = -0.135 m). The aircraft was re-balanced.

## Consequences
- Audit result: 67 parts, 0 conflicts, 38 deliberate embeds, minimum prop gap 25.5 mm, both elevons clear
  at ±20°.
- Static margin is 9.2 % at cruise and 5.0 % minimum. The roll rate stays above the requirement with the
  reduced effectiveness.
- Build adds a 2 mm × 60 mm carbon rod per side (negligible cost), two strap slots in the pod floor and a
  lead channel in each printed leg (`build/BENCH.md`).
- OCC reports wrong volumes on the spline-section lofts, so `cad_detail.vol()` measures volume from the
  tessellation. Any new check that needs volume must use it.
