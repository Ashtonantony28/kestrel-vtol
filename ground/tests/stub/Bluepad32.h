// Minimal host stub of the Arduino + Bluepad32 API surface used by kestrel_radio.ino.
// Only checks that the sketch type-checks; it does not prove the real library matches (bench item B9).
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define SERIAL_8N1 0
#define DPAD_UP 0x01
#define DPAD_DOWN 0x02
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
struct HardwareSerial {
  void begin(unsigned long, int = 0, int = -1, int = -1) {}
  int available() { return 0; }
  int read() { return -1; }
  size_t write(const uint8_t*, size_t n) { return n; }
  template <typename... A> void printf(const char*, A...) {}
  void println(const char*) {}
};
inline HardwareSerial Serial, Serial2;
struct Controller {
  bool isGamepad() const { return true; }
  bool isConnected() const { return true; }
  bool hasData() const { return true; }
  std::string getModelName() const { return "stub"; }
  int32_t axisX() const { return 0; }
  int32_t axisY() const { return 0; }
  int32_t axisRX() const { return 0; }
  int32_t axisRY() const { return 0; }
  int32_t brake() const { return 0; }
  int32_t throttle() const { return 0; }
  bool a() const { return false; }
  bool b() const { return false; }
  bool x() const { return false; }
  bool y() const { return false; }
  bool l1() const { return false; }
  bool r1() const { return false; }
  uint8_t dpad() const { return 0; }
  void playDualRumble(uint16_t, uint16_t, uint8_t, uint8_t) {}
};
using ControllerPtr = Controller*;
struct Bluepad32 {
  void setup(void (*)(ControllerPtr), void (*)(ControllerPtr)) {}
  bool update() { return true; }
  void enableNewBluetoothConnections(bool) {}
  void enableVirtualDevice(bool) {}
};
inline Bluepad32 BP32;
