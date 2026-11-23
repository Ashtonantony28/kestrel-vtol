# 0005: Carbon spar layout for the swept wing

- Status: Accepted
- Date: 2026-11-23

## Context
The original layout had one straight carbon spar along the quarter chord. Independent validation
(`python -m kestrel.validate`, V3) checked the structure against the CAD and found that a straight spar
cannot follow a 30° swept quarter chord: across the span it leaves the foam and would exit the leading
edge about 0.14 m from the centreline. The design tools had never checked this, because they treated the
spar as a mass point.

## Decision
- **Cross spar:** 8×6 mm carbon tube, 0.50 m long, straight across the centre section at 0.18 m behind the
  root leading edge and z = 6 mm, where the foam has 9.8 mm of interior height. Both booms clamp to it, so
  hover thrust reaches the pod through carbon.
- **Outer spars:** 6×4 mm carbon tubes along the quarter chord from y = 0.25 m to 0.47 m, following the sweep.
- Parameters live in `spec/vehicle.yaml` (`structure:`). The CAD checks that each spar is 100 % inside the
  wing (`spar_inside_frac`).
- The aircraft was re-balanced (battery and rotor stations). Minimum static margin rose from 3.8 % to 5.1 %.
- The aircraft cost ceiling moved from $240 to $250 to pay for the extra tube ($6–10).

## Consequences
- Validation V3a/V3b: 129 MPa and 93 MPa at ultimate load against 600 MPa allowable.
- Boom first bending mode, cantilevered from the cross-spar clamp, is 25–29 Hz. It is clear of the rate loop
  and of hover 1P (71 Hz), but needs a harmonic notch (`build/BENCH.md` B14).
- Glue joints and the foam around the spars are not analysed; bench item B12 (2 kg hang test) covers them.
