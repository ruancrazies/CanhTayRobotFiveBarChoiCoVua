// HOST-ONLY stub of the few Arduino calls src/main.cpp uses. It records every pin and Serial call, in
// order, so tools/host_main/boot_check.cpp can check the boot sequence and a command session without
// hardware. It is not part of any PlatformIO build.
#ifndef HOST_ARDUINO_STUB_H
#define HOST_ARDUINO_STUB_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#define HIGH 1
#define LOW 0
#define OUTPUT 1

namespace host_arduino {
inline std::vector<std::string>& log() { static std::vector<std::string> events; return events; }
inline uint32_t& clock_ms() { static uint32_t now = 0; return now; }
}  // namespace host_arduino

inline void digitalWrite(uint8_t pin, uint8_t value) {
  host_arduino::log().push_back("write " + std::to_string(pin) + " " + std::to_string(value));
}
inline void pinMode(uint8_t pin, uint8_t mode) {
  host_arduino::log().push_back("mode " + std::to_string(pin) + (mode == OUTPUT ? " output" : " other"));
}
inline uint32_t millis() { return host_arduino::clock_ms(); }

class FakeSerial {
 public:
  void setRxBufferSize(size_t size) { host_arduino::log().push_back("serial.rxbuf " + std::to_string(size)); }
  void begin(unsigned long baud) { host_arduino::log().push_back("serial.begin " + std::to_string(baud)); }
  int available() { return static_cast<int>(rx.size()); }
  int read() {
    if (rx.empty()) return -1;
    const int byte = rx.front();
    rx.pop_front();
    return byte;
  }
  size_t write(uint8_t byte) { tx.push_back(static_cast<char>(byte)); return 1; }
  size_t write(const uint8_t* data, size_t size) { tx.append(reinterpret_cast<const char*>(data), size); return size; }

  void type(const std::string& text) { for (unsigned char c : text) rx.push_back(c); }

  std::deque<int> rx;  // bytes the PC "sent"
  std::string tx;      // bytes the ESP32 sent
};

inline FakeSerial Serial;

#endif  // HOST_ARDUINO_STUB_H
