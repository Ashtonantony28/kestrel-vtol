// Host unit tests for the Kestrel ground radio bridge (CRSF framing + gamepad mapper).
// Run: ground/tests/run.sh   (g++ -std=c++17 -Wall -Wextra -Werror, plus ASan/UBSan)
#include <cstdio>
#include <cstring>
#include <vector>

#include "../kestrel_radio/src/crsf.hpp"
#include "../kestrel_radio/src/mapper.hpp"

using namespace kestrel;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond)                                                          \
  do {                                                                       \
    if (cond) {                                                              \
      ++g_pass;                                                              \
    } else {                                                                 \
      ++g_fail;                                                              \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
    }                                                                        \
  } while (0)

static void test_crc() {
  // CRC-8/DVB-S2 check value from the CRC catalogue: "123456789" -> 0xBC
  const char* s = "123456789";
  CHECK(crsf::crc8(reinterpret_cast<const uint8_t*>(s), 9) == 0xBC);
  CHECK(crsf::crc8(nullptr, 0) == 0x00);
}

static void test_ticks() {
  CHECK(crsf::us_to_ticks(1500) == crsf::kTicksMid);
  CHECK(crsf::us_to_ticks(1000) == 192);                // ExpressLRS/ArduPilot: 1000 us <-> 192, 2000 us <-> 1792
  CHECK(crsf::us_to_ticks(2000) == 1792);
  CHECK(crsf::ticks_to_us(192) == 1000 && crsf::ticks_to_us(1792) == 2000);
  CHECK(crsf::us_to_ticks(987) == crsf::kTicksMin);
  CHECK(crsf::us_to_ticks(2012) == crsf::kTicksMax);
  CHECK(crsf::us_to_ticks(500) == crsf::kTicksMin);    // clamped
  CHECK(crsf::us_to_ticks(2500) == crsf::kTicksMax);
  for (int us = 988; us <= 2012; us += 7) {
    const int back = crsf::ticks_to_us(crsf::us_to_ticks(us));
    CHECK(back >= us - 1 && back <= us + 1);             // 0.625 us/tick resolution
  }
}

static void test_pack_layout() {
  std::array<uint16_t, crsf::kChannels> ch{};
  for (std::size_t i = 0; i < ch.size(); ++i) ch[i] = static_cast<uint16_t>(172 + 97 * i);
  const auto f = crsf::pack_rc(ch);
  CHECK(f[0] == crsf::kAddrTransmitter);
  CHECK(f[1] == 24);                                     // type + 22 payload + crc
  CHECK(f[2] == crsf::kTypeRcChannels);
  CHECK(f[25] == crsf::crc8(&f[2], 23));
  // channel 0 occupies the low 11 bits little-endian: byte3 = low 8 bits, byte4 low 3 bits = high bits
  CHECK(f[3] == (ch[0] & 0xFF));
  CHECK((f[4] & 0x07) == (ch[0] >> 8));
  CHECK(((f[4] >> 3) | ((f[5] & 0x3F) << 5)) == ch[1]);
  const auto back = crsf::unpack_rc_payload(&f[3]);
  CHECK(back == ch);
  // all extremes
  ch.fill(crsf::kTicksMax);
  CHECK(crsf::unpack_rc_payload(&crsf::pack_rc(ch)[3]) == ch);
  ch.fill(0x7FF);
  CHECK(crsf::unpack_rc_payload(&crsf::pack_rc(ch)[3]) == ch);
}

static std::vector<uint8_t> frame(uint8_t addr, uint8_t type, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> f{addr, static_cast<uint8_t>(payload.size() + 2), type};
  f.insert(f.end(), payload.begin(), payload.end());
  f.push_back(crsf::crc8(&f[2], payload.size() + 1));
  return f;
}

static void test_parser() {
  crsf::Parser p;
  // link stats: rssi1, rssi2, lq, snr, ant, mode, power, d_rssi, d_lq, d_snr
  auto ls = frame(crsf::kAddrRadio, crsf::kTypeLinkStats, {60, 62, 87, static_cast<uint8_t>(-3), 0, 4, 2, 55, 99, 8});
  // battery 11.7 V, 8.4 A, 1234 mAh, 55 %
  auto bt = frame(crsf::kAddrRadio, crsf::kTypeBattery, {0x00, 117, 0x00, 84, 0x00, 0x04, 0xD2, 55});
  std::vector<uint8_t> stream{0x00, 0x13, 0x77};       // garbage before sync
  stream.insert(stream.end(), ls.begin(), ls.end());
  auto bad = bt;
  bad.back() ^= 0x5A;                                   // corrupted CRC
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), bt.begin(), bt.end());
  int good = 0;
  for (uint8_t b : stream) good += p.feed(b) ? 1 : 0;
  CHECK(good == 2);
  CHECK(p.frames_ok() == 2);
  CHECK(p.frames_bad() == 1);
  CHECK(p.link().valid && p.link().uplink_lq == 87 && p.link().uplink_snr == -3 && p.link().downlink_lq == 99);
  CHECK(p.battery().valid);
  CHECK(p.battery().voltage > 11.69f && p.battery().voltage < 11.71f);
  CHECK(p.battery().current > 8.39f && p.battery().current < 8.41f);
  CHECK(p.battery().used_mah == 1234 && p.battery().remaining == 55);
  // invalid length byte resyncs without consuming the next good frame
  crsf::Parser q;
  q.feed(crsf::kAddrRadio);
  q.feed(0xFF);
  for (uint8_t b : ls) q.feed(b);
  CHECK(q.frames_ok() == 1 && q.frames_bad() == 1);
}

static GamepadState pad(uint32_t t) {
  GamepadState g;
  g.connected = true;
  g.t_ms = t;
  return g;
}

static void test_mapper_sticks() {
  CHECK(Mapper::stick_us(0, false) == 1500);
  CHECK(Mapper::stick_us(39, false) == 1500);           // inside deadband
  CHECK(Mapper::stick_us(-39, false) == 1500);
  CHECK(Mapper::stick_us(512, false) == 2000);
  CHECK(Mapper::stick_us(-512, false) == 1000);
  CHECK(Mapper::stick_us(512, true) == 1000);
  CHECK(Mapper::stick_us(41, false) >= 1500 && Mapper::stick_us(41, false) <= 1502);  // continuous at edge
  // expo: half stick gives less than half throw, and the map is monotonic and odd
  const int half = Mapper::stick_us(256, false) - 1500;
  CHECK(half > 0 && half < 250);
  int prev = 0;
  for (int v = -512; v <= 511; ++v) {
    const int u = Mapper::stick_us(static_cast<int16_t>(v), false);
    CHECK(u >= prev);
    prev = u;
    CHECK(u - 1500 == -(Mapper::stick_us(static_cast<int16_t>(-v), false) - 1500) || v == -512);
  }
}

static void test_mapper_modes_and_channels() {
  Mapper m;
  auto g = pad(0);
  g.rx = 512;     // roll right
  g.ry = 512;     // stick pulled back -> nose up
  g.ly = -512;    // throttle stick up
  g.lx = -512;    // yaw left
  auto o = m.update(g, 0);
  CHECK(o.send);
  CHECK(o.us[0] == 2000 && o.us[1] == 2000 && o.us[2] == 2000 && o.us[3] == 1000);
  CHECK(o.us[4] == Mapper::mode_us(Mode::QLoiter));    // boots in QLOITER
  CHECK(o.us[6] == 1100);                               // disarmed
  for (int i : {5, 7, 8, 15}) CHECK(o.us[i] == 1500);
  struct { bool GamepadState::*btn; Mode m; } table[] = {
      {&GamepadState::b, Mode::QHover}, {&GamepadState::a, Mode::Cruise}, {&GamepadState::x, Mode::Rtl},
      {&GamepadState::dpad_up, Mode::Auto}, {&GamepadState::dpad_down, Mode::QLand}, {&GamepadState::y, Mode::QLoiter}};
  uint32_t t = 10;
  for (auto& e : table) {
    auto h = pad(t);
    h.*(e.btn) = true;
    m.update(h, t);
    CHECK(m.mode() == e.m);
    auto rel = m.update(pad(t + 5), t + 5);              // mode latches after release
    CHECK(m.mode() == e.m && rel.us[4] == Mapper::mode_us(e.m));
    t += 10;
  }
  // every mode PWM sits inside its ArduPilot FLTMODE band, >= 40 us from band edges
  const int edges[] = {1230, 1360, 1490, 1620, 1749};
  const Mode order[] = {Mode::QLoiter, Mode::QHover, Mode::Cruise, Mode::Auto, Mode::Rtl, Mode::QLand};
  for (int i = 0; i < 6; ++i) {
    const int us = Mapper::mode_us(order[i]);
    if (i > 0) CHECK(us > edges[i - 1] + 40);
    if (i < 5) CHECK(us < edges[i] - 40);
  }
}

static void test_mapper_arming() {
  Mapper m;
  auto hold = [](uint32_t t, bool thr_low) {
    auto g = pad(t);
    g.lb = g.rb = true;
    g.ly = thr_low ? 512 : 0;
    return g;
  };
  // throttle not low: never arms
  for (uint32_t t = 0; t <= 3000; t += 20) m.update(hold(t, false), t);
  CHECK(!m.armed_request());
  m.update(pad(3010), 3010);
  // short hold (1.0 s) does not arm
  for (uint32_t t = 4000; t <= 5000; t += 20) m.update(hold(t, true), t);
  CHECK(!m.armed_request());
  m.update(pad(5010), 5010);
  // 1.5 s hold arms exactly once, even if held for 5 s
  for (uint32_t t = 6000; t <= 11000; t += 20) m.update(hold(t, true), t);
  CHECK(m.armed_request());
  auto o = m.update(pad(11010), 11010);
  CHECK(o.us[6] == 1900);
  // second full hold disarms
  for (uint32_t t = 12000; t <= 13600; t += 20) m.update(hold(t, true), t);
  CHECK(!m.armed_request());
}

static void test_mapper_failsafe_and_rumble() {
  Mapper m;
  CHECK(m.update(pad(0), 0).send);
  // stale sample (Bluetooth stalled) for > 300 ms -> stop sending so ELRS/ArduPilot failsafe triggers
  auto stale = pad(0);
  CHECK(m.update(stale, 250).send);
  CHECK(!m.update(stale, 400).send);
  // disconnected
  GamepadState off;
  CHECK(!m.update(off, 500).send);
  // reconnect resumes
  CHECK(m.update(pad(600), 600).send);
  crsf::LinkStats ls;
  ls.valid = true;
  ls.uplink_lq = 90;
  crsf::Battery b;
  b.valid = true;
  b.voltage = 11.8f;
  CHECK(!m.update(pad(610), 610, &ls, &b).rumble);
  ls.uplink_lq = 40;
  CHECK(m.update(pad(620), 620, &ls, &b).rumble);
  ls.uplink_lq = 90;
  b.voltage = 10.4f;
  CHECK(m.update(pad(630), 630, &ls, &b).rumble);
  b.voltage = 0.0f;                                      // no sensor reading: no false alarm
  CHECK(!m.update(pad(640), 640, &ls, &b).rumble);
}

int main() {
  test_crc();
  test_ticks();
  test_pack_layout();
  test_parser();
  test_mapper_sticks();
  test_mapper_modes_and_channels();
  test_mapper_arming();
  test_mapper_failsafe_and_rumble();
  std::printf("%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
