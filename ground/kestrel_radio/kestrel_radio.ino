// Kestrel ground radio bridge: Xbox controller (Bluetooth) -> ESP32 (Bluepad32) -> CRSF -> ELRS TX.
//
// Board: "ESP32 Dev Module" from the Bluepad32 Arduino core (esp32_bluepad32 board package).
// The protocol and stick logic live in src/ and are unit-tested on a PC: ground/tests/run.sh.
// Wiring (see ground/README.md): ESP32 GPIO17 (TX2) -> ELRS module RX pad, GPIO16 (RX2) <- ELRS TX pad
// (through 1 kOhm), 5 V and GND shared. The ELRS module runs "RX-as-TX" / TX firmware.
#include <Bluepad32.h>

#include "src/crsf.hpp"
#include "src/mapper.hpp"

using kestrel::GamepadState;
using kestrel::Mapper;
namespace crsf = kestrel::crsf;

// --- configuration (VERIFY on the bench, build/BENCH.md B9) ---
static constexpr uint32_t kCrsfBaud = 420000;     // must equal the ELRS module's serial baud (400000 on some builds)
static constexpr int kPinTx = 17, kPinRx = 16;
static constexpr uint32_t kFramePeriodUs = 4000;  // 250 Hz RC frames (ELRS packet rate 250 Hz)
static constexpr int kLedPin = 2;

static ControllerPtr g_ctl = nullptr;
static Mapper g_mapper;
static crsf::Parser g_parser;
static GamepadState g_pad;
static uint32_t g_last_frame_us = 0;
static uint32_t g_last_rumble_ms = 0;

static void onConnected(ControllerPtr ctl) {
  if (g_ctl == nullptr && ctl->isGamepad()) {
    g_ctl = ctl;
    Serial.printf("controller connected: %s\n", ctl->getModelName().c_str());
    // Only one controller may ever drive the aircraft: stop accepting new pairings while one is connected.
    BP32.enableNewBluetoothConnections(false);
  }
}

static void onDisconnected(ControllerPtr ctl) {
  if (ctl == g_ctl) {
    g_ctl = nullptr;
    g_pad.connected = false;
    Serial.println("controller disconnected -> CRSF stopped -> aircraft failsafe");
    BP32.enableNewBluetoothConnections(true);
  }
}

static void readPad(uint32_t now_ms) {
  if (g_ctl == nullptr || !g_ctl->isConnected()) {
    g_pad.connected = false;
    return;
  }
  if (!g_ctl->hasData()) return;  // keep the old sample; its timestamp ages and Mapper times it out
  g_pad.connected = true;
  g_pad.lx = static_cast<int16_t>(g_ctl->axisX());
  g_pad.ly = static_cast<int16_t>(g_ctl->axisY());
  g_pad.rx = static_cast<int16_t>(g_ctl->axisRX());
  g_pad.ry = static_cast<int16_t>(g_ctl->axisRY());
  g_pad.lt = static_cast<uint16_t>(g_ctl->brake());
  g_pad.rt = static_cast<uint16_t>(g_ctl->throttle());
  g_pad.a = g_ctl->a();
  g_pad.b = g_ctl->b();
  g_pad.x = g_ctl->x();
  g_pad.y = g_ctl->y();
  g_pad.lb = g_ctl->l1();
  g_pad.rb = g_ctl->r1();
  const uint8_t dp = g_ctl->dpad();
  g_pad.dpad_up = (dp & DPAD_UP) != 0;
  g_pad.dpad_down = (dp & DPAD_DOWN) != 0;
  g_pad.t_ms = now_ms;
}

void setup() {
  Serial.begin(115200);
  pinMode(kLedPin, OUTPUT);
  Serial2.begin(kCrsfBaud, SERIAL_8N1, kPinRx, kPinTx);
  BP32.setup(&onConnected, &onDisconnected);
  BP32.enableVirtualDevice(false);
  Serial.println("Kestrel radio bridge ready: hold the Xbox pair button");
}

void loop() {
  const uint32_t now_ms = millis();
  if (BP32.update()) readPad(now_ms);
  if (g_ctl == nullptr) g_pad.connected = false;

  while (Serial2.available() > 0) g_parser.feed(static_cast<uint8_t>(Serial2.read()));

  const uint32_t now_us = micros();
  if (now_us - g_last_frame_us < kFramePeriodUs) return;
  g_last_frame_us = now_us;

  const kestrel::Outputs out = g_mapper.update(g_pad, now_ms, &g_parser.link(), &g_parser.battery());
  digitalWrite(kLedPin, out.send ? HIGH : LOW);
  if (!out.send) return;  // silence on purpose: ELRS stops sending, the aircraft's RC failsafe takes over

  std::array<uint16_t, crsf::kChannels> ticks{};
  for (size_t i = 0; i < crsf::kChannels; ++i) ticks[i] = crsf::us_to_ticks(out.us[i]);
  const auto frame = crsf::pack_rc(ticks);
  Serial2.write(frame.data(), frame.size());

  if (out.rumble && g_ctl != nullptr && now_ms - g_last_rumble_ms > 2000) {
    g_ctl->playDualRumble(0, 250, 0x80, 0x40);  // weak link or low battery: buzz every 2 s
    g_last_rumble_ms = now_ms;
  }
}
