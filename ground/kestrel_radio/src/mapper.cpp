#include "mapper.hpp"

#include <cmath>
#include <cstdlib>

namespace kestrel {

int Mapper::stick_us(int16_t v, bool invert) {
  float x = static_cast<float>(v) / 512.0f;
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;
  const float db = static_cast<float>(kDeadband) / 512.0f;
  if (std::fabs(x) < db) {
    x = 0.0f;
  } else {
    x = (x > 0 ? x - db : x + db) / (1.0f - db);
  }
  x = (1.0f - kExpo) * x + kExpo * x * x * x;
  if (invert) x = -x;
  return 1500 + static_cast<int>(std::lround(x * 500.0f));
}

int Mapper::mode_us(Mode m) {
  // FLTMODE1..6 bands: <1230, 1231-1360, 1361-1490, 1491-1620, 1621-1749, >1750 (ArduPilot)
  switch (m) {
    case Mode::QLoiter: return 1165;  // FLTMODE1 = QLOITER
    case Mode::QHover:  return 1295;  // FLTMODE2 = QHOVER
    case Mode::Cruise:  return 1425;  // FLTMODE3 = CRUISE
    case Mode::Auto:    return 1555;  // FLTMODE4 = AUTO
    case Mode::Rtl:     return 1685;  // FLTMODE5 = RTL (QRTL landing with Q_RTL_MODE=1)
    case Mode::QLand:   return 1815;  // FLTMODE6 = QLAND
  }
  return 1165;
}

Outputs Mapper::update(const GamepadState& g, uint32_t now_ms, const crsf::LinkStats* link, const crsf::Battery* batt) {
  Outputs o;
  // judge staleness from the sample's own timestamp: a stalled Bluetooth link keeps re-delivering the
  // last sample, so "connected" alone is not enough
  if (!g.connected || now_ms - g.t_ms > kLinkTimeoutMs) {
    o.send = false;          // controller lost: go silent -> ELRS failsafe -> ArduPilot RTL / QLAND
    holding_ = false;
    return o;
  }
  // modes: Y QLOITER, B QHOVER, A CRUISE, X RTL, d-pad up AUTO, d-pad down QLAND
  if (g.y) mode_ = Mode::QLoiter;
  if (g.b) mode_ = Mode::QHover;
  if (g.a) mode_ = Mode::Cruise;
  if (g.x) mode_ = Mode::Rtl;
  if (g.dpad_up) mode_ = Mode::Auto;
  if (g.dpad_down) mode_ = Mode::QLand;

  const int thr = stick_us(g.ly, true);     // stick up = more; centred = hold altitude in Q modes
  // arming toggle: hold LB+RB for kArmHoldMs with throttle stick fully down
  const bool combo = g.lb && g.rb && thr < 1100;
  if (combo) {
    if (!holding_) {
      holding_ = true;
      hold_start_ = now_ms;
    } else if (hold_start_ != 0 && now_ms - hold_start_ >= kArmHoldMs) {
      arm_ = !arm_;
      hold_start_ = 0;   // require release before toggling again
    }
  } else {
    holding_ = false;
  }

  o.us.fill(1500);
  o.us[0] = stick_us(g.rx, false);          // roll: right = right
  o.us[1] = stick_us(g.ry, false);          // pitch: stick pulled back (+y) = higher PWM = nose up (ArduPlane mode_fbwa.cpp: norm_input > 0 -> pitch up, RC2_REVERSED 0)
  o.us[2] = thr;
  o.us[3] = stick_us(g.lx, false);          // yaw
  o.us[4] = mode_us(mode_);
  o.us[6] = arm_ ? 1900 : 1100;             // RC7_OPTION 153 (ArmDisarm)
  o.send = true;
  const bool weak = link && link->valid && link->uplink_lq < 50;
  const bool low = batt && batt->valid && batt->voltage > 1.0f && batt->voltage < 10.6f;
  o.rumble = weak || low;
  return o;
}

}  // namespace kestrel
