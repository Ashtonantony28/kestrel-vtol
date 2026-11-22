// Xbox controller -> ArduPilot RC channels, with arming hold, mode buttons and a link watchdog.
// Pure C++17 (host-tested). The Arduino sketch fills GamepadState from Bluepad32 and calls Mapper::update().
#pragma once
#include <array>
#include <cstdint>

#include "crsf.hpp"

namespace kestrel {

struct GamepadState {
  bool connected = false;
  int16_t lx = 0, ly = 0, rx = 0, ry = 0;   // Bluepad32 range -512..511, +y = stick DOWN
  uint16_t lt = 0, rt = 0;                  // triggers 0..1023
  bool a = false, b = false, x = false, y = false, lb = false, rb = false;
  bool dpad_up = false, dpad_down = false;
  uint32_t t_ms = 0;                        // time of this sample
};

// ArduPilot channel order (RCMAP defaults): 1 roll, 2 pitch, 3 throttle, 4 yaw, 5 flight mode, 7 arm/disarm.
struct Outputs {
  std::array<int, crsf::kChannels> us{};
  bool send = false;          // false -> stop sending CRSF so the aircraft enters RC failsafe
  bool rumble = false;
};

enum class Mode : uint8_t { QLoiter, QHover, Cruise, Auto, Rtl, QLand };

class Mapper {
 public:
  static constexpr int kDeadband = 40;          // of 512
  static constexpr float kExpo = 0.3f;
  static constexpr uint32_t kArmHoldMs = 1500;  // both bumpers + throttle low
  static constexpr uint32_t kLinkTimeoutMs = 300;

  Outputs update(const GamepadState& g, uint32_t now_ms, const crsf::LinkStats* link = nullptr,
                 const crsf::Battery* batt = nullptr);
  Mode mode() const { return mode_; }
  bool armed_request() const { return arm_; }
  static int stick_us(int16_t v, bool invert);   // -512..511 -> 1000..2000 with deadband + expo
  static int mode_us(Mode m);                    // ArduPilot 6-position mode switch bands

 private:
  Mode mode_ = Mode::QLoiter;
  bool arm_ = false;
  uint32_t hold_start_ = 0;
  bool holding_ = false;
};

}  // namespace kestrel
