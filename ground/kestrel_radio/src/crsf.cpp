#include "crsf.hpp"

namespace kestrel::crsf {

uint8_t crc8(const uint8_t* data, std::size_t len) {
  uint8_t crc = 0;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

uint16_t us_to_ticks(int us) {
  long t = static_cast<long>(us - 1500) * 8 / 5 + kTicksMid;
  if (t < kTicksMin) t = kTicksMin;
  if (t > kTicksMax) t = kTicksMax;
  return static_cast<uint16_t>(t);
}

int ticks_to_us(uint16_t ticks) { return (static_cast<int>(ticks) - kTicksMid) * 5 / 8 + 1500; }

std::array<uint8_t, kRcFrameLen> pack_rc(const std::array<uint16_t, kChannels>& ticks) {
  std::array<uint8_t, kRcFrameLen> f{};
  f[0] = kAddrTransmitter;
  f[1] = 24;  // type + 22 payload + crc
  f[2] = kTypeRcChannels;
  uint32_t acc = 0;
  int bits = 0;
  std::size_t o = 3;
  for (std::size_t i = 0; i < kChannels; ++i) {
    acc |= static_cast<uint32_t>(ticks[i] & 0x7FF) << bits;
    bits += 11;
    while (bits >= 8) {
      f[o++] = static_cast<uint8_t>(acc & 0xFF);
      acc >>= 8;
      bits -= 8;
    }
  }
  f[25] = crc8(&f[2], 23);
  return f;
}

std::array<uint16_t, kChannels> unpack_rc_payload(const uint8_t* p) {
  std::array<uint16_t, kChannels> ch{};
  uint32_t acc = 0;
  int bits = 0;
  std::size_t i = 0, c = 0;
  while (c < kChannels) {
    while (bits < 11) {
      acc |= static_cast<uint32_t>(p[i++]) << bits;
      bits += 8;
    }
    ch[c++] = static_cast<uint16_t>(acc & 0x7FF);
    acc >>= 11;
    bits -= 11;
  }
  return ch;
}

bool Parser::feed(uint8_t b) {
  if (pos_ == 0) {
    if (b != kAddrRadio && b != 0xC8 && b != kAddrTransmitter) return false;  // sync on known addresses
  }
  buf_[pos_++] = b;
  if (pos_ == 2 && (buf_[1] < 2 || buf_[1] > 62)) {  // invalid length -> resync
    pos_ = 0;
    ++bad_;
    return false;
  }
  if (pos_ >= 2 && pos_ == static_cast<std::size_t>(buf_[1]) + 2) {
    const std::size_t n = buf_[1];
    const bool good = crc8(&buf_[2], n - 1) == buf_[n + 1];
    if (good) {
      ++ok_;
      dispatch();
    } else {
      ++bad_;
    }
    pos_ = 0;
    return good;
  }
  return false;
}

void Parser::dispatch() {
  const uint8_t type = buf_[2];
  const uint8_t* p = &buf_[3];
  if (type == kTypeLinkStats && buf_[1] >= 12) {
    link_.uplink_rssi1 = p[0];
    link_.uplink_lq = p[2];
    link_.uplink_snr = static_cast<int8_t>(p[3]);
    link_.downlink_lq = p[8];
    link_.valid = true;
  } else if (type == kTypeBattery && buf_[1] >= 10) {
    batt_.voltage = static_cast<float>((p[0] << 8) | p[1]) / 10.0f;   // big-endian, 0.1 V
    batt_.current = static_cast<float>((p[2] << 8) | p[3]) / 10.0f;   // 0.1 A
    batt_.used_mah = (static_cast<uint32_t>(p[4]) << 16) | (p[5] << 8) | p[6];
    batt_.remaining = p[7];
    batt_.valid = true;
  }
}

}  // namespace kestrel::crsf
