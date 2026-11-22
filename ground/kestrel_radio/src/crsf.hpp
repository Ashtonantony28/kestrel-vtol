// CRSF (Crossfire / ExpressLRS serial protocol) framing for the Kestrel ground radio bridge.
// Pure C++17, no Arduino dependencies, so it is unit-tested on the host (tests/run.sh).
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace kestrel::crsf {

constexpr uint8_t kAddrTransmitter = 0xEE;   // handset -> TX module
constexpr uint8_t kAddrRadio = 0xEA;         // TX module -> handset (telemetry)
constexpr uint8_t kTypeRcChannels = 0x16;
constexpr uint8_t kTypeLinkStats = 0x14;
constexpr uint8_t kTypeBattery = 0x08;
constexpr std::size_t kChannels = 16;
constexpr std::size_t kRcFrameLen = 26;      // addr, len, type, 22 payload, crc
constexpr uint16_t kTicksMin = 172, kTicksMid = 992, kTicksMax = 1811;

uint8_t crc8(const uint8_t* data, std::size_t len);          // CRC-8/DVB-S2 (poly 0xD5)
uint16_t us_to_ticks(int us);                                 // 988..2012 us -> 172..1811
int ticks_to_us(uint16_t ticks);

// Pack 16 channels (CRSF ticks, 11 bit) into a complete RC_CHANNELS_PACKED frame.
std::array<uint8_t, kRcFrameLen> pack_rc(const std::array<uint16_t, kChannels>& ticks);
// Inverse of the payload packing (used by tests and by the parser).
std::array<uint16_t, kChannels> unpack_rc_payload(const uint8_t* payload22);

struct LinkStats { uint8_t uplink_rssi1 = 0, uplink_lq = 0; int8_t uplink_snr = 0; uint8_t downlink_lq = 0; bool valid = false; };
struct Battery { float voltage = 0, current = 0; uint32_t used_mah = 0; uint8_t remaining = 0; bool valid = false; };

// Byte-wise telemetry parser for frames coming back from the TX module.
class Parser {
 public:
  // Returns true when a complete, CRC-valid frame was consumed.
  bool feed(uint8_t b);
  const LinkStats& link() const { return link_; }
  const Battery& battery() const { return batt_; }
  uint32_t frames_ok() const { return ok_; }
  uint32_t frames_bad() const { return bad_; }
 private:
  void dispatch();
  std::array<uint8_t, 64> buf_{};
  std::size_t pos_ = 0;
  LinkStats link_{};
  Battery batt_{};
  uint32_t ok_ = 0, bad_ = 0;
};

}  // namespace kestrel::crsf
