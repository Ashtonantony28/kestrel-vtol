# 0007: Fit an airspeed sensor

- Status: Accepted
- Date: 2026-11-24
- Supersedes: the "no airspeed sensor" choice in the lean plan (PLAN.md v2). ADR 0004's assist-speed and
  transition-fail settings stay.

## Context
The lean plan left out the pitot to save about $15. ArduPlane then flies on a synthetic airspeed, estimated
from GPS ground speed and its wind estimate. The first SITL campaign passed every wind case without a
sensor, so the sensor was judged unnecessary.

The SITL rerun after the elevon split (ADR 0006) showed this was marginal:

- On straight legs the synthetic airspeed read about 1.2 m/s low. The aircraft flew at 15.3 m/s true when
  it was commanded 14 m/s, and drew 64 W instead of about 51 W. Endurance fell to 18.3 min, and validation
  V2e (SITL endurance within 15 % of the sizing model's 21.9 min) failed.
- In a 10 m/s beam wind with gusts, the low estimate kept starting lift-motor assist. That tripped the 45 s
  transition-fail timer, and the aircraft flew home as a quad against the wind until the battery ran low.
  It landed 344 m and then 707 m from home on two runs.
- With the SITL airspeed sensor fitted, the same beam-wind case landed 2.2 m from the pad, and endurance
  was 20.2 min (V2e passes).

## Decision
The user chose to fit the sensor (2026-11-24).

- **Hardware:** an MS4525DO I2C differential-pressure board with a 4 mm pitot-static tube and silicone hose.
  Generic clones cost $14–22; a Matek ASPD-4525 costs about $35.
- **Mounting:** the tube comes out of the pod nose on the centreline, 50 mm ahead of the foam, between
  the front prop discs (134 mm from the nearest disc). The board sits on a foam pad on the pod floor ahead of
  the battery. The total of 10 g moved the battery 13 mm aft at re-balance.
- **Parameters:** `ARSPD_TYPE 1`, `ARSPD_USE 1`. SITL overrides the type to 100, a simulated sensor fed
  with the true airspeed from our physics.
- **Simulation default:** `spec/vehicle.yaml` `airspeed_sensor.fitted` sets what SITL flies.
  `python -m kestrel.mc --no-airspeed-sensor` reruns any suite sensorless, into `<suite>_nopitot`.
- **Cost ceiling:** `limits.max_bom_usd` rises from $250 to $275.

## Consequences
- The sensor needs a pre-flight check (bench item B16): blow gently into the pitot and see the airspeed
  move, then calibrate `ARSPD_RATIO` with an automatic airspeed calibration flight.
- A blocked or iced pitot is a new failure mode. ArduPlane's airspeed health check falls back to synthetic
  airspeed, which is the configuration that flew every case up to 7 m/s wind.
- In hover the tube sits in still air ahead of the front props, so it does not affect the VTOL phases.
