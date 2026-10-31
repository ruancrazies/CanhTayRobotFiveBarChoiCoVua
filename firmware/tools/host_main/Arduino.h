// HOST-ONLY stand-in for Arduino.h. It does two jobs:
//  1. records the few pin and Serial calls src/main.cpp makes, in order, so boot_check.cpp can test the
//     boot sequence and a command session without hardware;
//  2. defines the macros and global names the REAL Arduino.h / binary.h / pins_arduino.h of Arduino-ESP32
//     2.0.x define (bit, min, max, abs, round, sq, constrain, radians, degrees, word, HIGH, LOW, INPUT,
//     OUTPUT, DEFAULT, B0..B7, PI, F(), byte, SDA, SCL, SS, TX, RX, A0, T0, ...), so a project identifier
//     that collides with one breaks the HOST build instead of only the ESP32 build.
//
// run_host_tests.sh force-includes this header (-include) into every host translation unit, exactly as
// every file of an Arduino build sees Arduino.h first. The list is written from memory of the Arduino-ESP32
// 2.0.x sources (they could not be downloaded here), and min/max/abs/round are defined as macros even
// though the real C++ core mostly avoids that: the stub is stricter on purpose. Add a name here whenever
// a build on the real core shows a collision this list missed.
//
// Not part of any PlatformIO build.
#ifndef HOST_ARDUINO_STUB_H
#define HOST_ARDUINO_STUB_H

// The standard headers first, as Arduino.h does: the macros below would break them if they came later.
#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <map>
#include <string>
#include <utility>
#include <vector>

// ---- Arduino.h ----------------------------------------------------------------------------------
#define HIGH 0x1
#define LOW 0x0

#define INPUT 0x01
#define OUTPUT 0x03
#define PULLUP 0x04
#define INPUT_PULLUP 0x05
#define PULLDOWN 0x08
#define INPUT_PULLDOWN 0x09
#define OPEN_DRAIN 0x10
#define OUTPUT_OPEN_DRAIN 0x12
#define ANALOG 0xC0

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define SERIAL 0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03
#define ONLOW 0x04
#define ONHIGH 0x05
#define ONLOW_WE 0x0C
#define ONHIGH_WE 0x0D

#define DEFAULT 1
#define EXTERNAL 0

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x)-0.5))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x) * (x))

#define sei()
#define cli()
#define interrupts() sei()
#define noInterrupts() cli()
#define clockCyclesPerMicrosecond() (240L)
#define clockCyclesToMicroseconds(a) ((a) / clockCyclesPerMicrosecond())
#define microsecondsToClockCycles(a) ((a)*clockCyclesPerMicrosecond())

#define lowByte(w) ((uint8_t)((w)&0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))
#define _BV(b) (1UL << (b))
#define word(...) makeWord(__VA_ARGS__)

#define F(string_literal) (string_literal)
#define PSTR(s) (s)
#define NOT_A_PIN -1
#define NOT_AN_INTERRUPT -1
// esp_attr.h (pulled in by the real Arduino.h): code placed in internal RAM, so it runs while the flash cache is off.
#define IRAM_ATTR
#define digitalPinToInterrupt(p) (((p) < 40) ? (p) : -1)

typedef bool boolean;
typedef uint8_t byte;
typedef unsigned int word;  // legal next to the function-like macro word(...): it expands only before "("

// ---- binary.h -----------------------------------------------------------------------------------
#define B0 0
#define B1 1
#define B2 2
#define B3 3
#define B4 4
#define B5 5
#define B6 6
#define B7 7
#define B00000000 0
#define B00000001 1
#define B11111111 255

// ---- pins_arduino.h (esp32dev / ESP32 Dev Module) -------------------------------------------------
static const uint8_t TX = 1;
static const uint8_t RX = 3;
static const uint8_t SDA = 21;
static const uint8_t SCL = 22;
static const uint8_t SS = 5;
static const uint8_t MOSI = 23;
static const uint8_t MISO = 19;
static const uint8_t SCK = 18;
static const uint8_t A0 = 36;
static const uint8_t A3 = 39;
static const uint8_t A4 = 32;
static const uint8_t A5 = 33;
static const uint8_t A6 = 34;
static const uint8_t A7 = 35;
static const uint8_t A10 = 4;
static const uint8_t A18 = 25;
static const uint8_t A19 = 26;
static const uint8_t T0 = 4;
static const uint8_t T9 = 32;
static const uint8_t DAC1 = 25;
static const uint8_t DAC2 = 26;
static const uint8_t LED_BUILTIN = 2;

// ---- the recording part --------------------------------------------------------------------------
namespace host_arduino {
inline std::vector<std::string>& log() { static std::vector<std::string> events; return events; }
inline uint32_t& clock_ms() { static uint32_t now = 0; return now; }
// The level of every pin: what a test sets for an input, and what digitalWrite() last wrote for an output.
inline std::map<int, int>& pin_levels() { static std::map<int, int> levels; return levels; }
// The interrupt service routines attachInterrupt() was given, by pin, and the mode each was attached with.
inline std::map<int, void (*)()>& isr_table() { static std::map<int, void (*)()> table; return table; }
inline std::map<int, int>& isr_modes() { static std::map<int, int> modes; return modes; }
}  // namespace host_arduino

inline void digitalWrite(uint8_t pin, uint8_t value) {
  host_arduino::pin_levels()[pin] = value;
  host_arduino::log().push_back("write " + std::to_string(pin) + " " + std::to_string(value));
}
inline int digitalRead(uint8_t pin) {
  const auto found = host_arduino::pin_levels().find(pin);
  return found == host_arduino::pin_levels().end() ? LOW : found->second;
}
inline void pinMode(uint8_t pin, uint8_t mode) {
  host_arduino::log().push_back("mode " + std::to_string(pin) + (mode == OUTPUT ? " output" : mode == INPUT ? " input" : " other"));
}
inline void attachInterrupt(uint8_t interrupt, void (*isr)(void), int mode) {
  host_arduino::isr_table()[interrupt] = isr;
  host_arduino::isr_modes()[interrupt] = mode;
  host_arduino::log().push_back("isr " + std::to_string(interrupt) + " " + std::to_string(mode));
}
// LEDC, Arduino-ESP32 2.0.x API.
inline uint32_t ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits) {
  host_arduino::log().push_back("ledc.setup " + std::to_string(channel) + " " + std::to_string(freq) + " " + std::to_string(resolution_bits));
  return freq;
}
inline void ledcAttachPin(uint8_t pin, uint8_t channel) {
  host_arduino::log().push_back("ledc.attach " + std::to_string(pin) + " " + std::to_string(channel));
}
inline void ledcWrite(uint8_t channel, uint32_t duty) {
  host_arduino::log().push_back("ledc.write " + std::to_string(channel) + " " + std::to_string(duty));
}
inline uint32_t millis() { return host_arduino::clock_ms(); }

class FakeSerial {
 public:
  void setRxBufferSize(size_t size) { host_arduino::log().push_back("serial.rxbuf " + std::to_string(size)); }
  void begin(unsigned long baud) { host_arduino::log().push_back("serial.begin " + std::to_string(baud)); }
  int available() { return static_cast<int>(rx.size()); }
  int read() {
    if (rx.empty()) return -1;
    const int next = rx.front();
    rx.pop_front();
    return next;
  }
  size_t write(uint8_t value) { tx.push_back(static_cast<char>(value)); return 1; }
  size_t write(const uint8_t* data, size_t size) { tx.append(reinterpret_cast<const char*>(data), size); return size; }

  void type(const std::string& text) { for (char c : text) rx.push_back(static_cast<unsigned char>(c)); }

  std::deque<int> rx;  // bytes the PC "sent"
  std::string tx;      // bytes the ESP32 sent
};

inline FakeSerial Serial;

#endif  // HOST_ARDUINO_STUB_H
