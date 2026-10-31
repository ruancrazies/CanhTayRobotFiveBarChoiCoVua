// HOST-ONLY stand-in for FastAccelStepper 0.33.14 (the version pinned in platformio.ini). It declares ONLY the calls
// src/main.cpp is allowed to make, with the signatures of the real src/FastAccelStepper.h of that tag, and records
// them in host_arduino::log() so tools/host_main/boot_check.cpp can check the sequence.
//
// It deliberately does NOT declare FastAccelStepper::setEnablePin(): in 0.33.14 that call drives the enable pin to
// the DISABLED level the moment it is made (PIN_OUTPUT(pin, LOW) for low_active_enables_stepper = false), which would
// release a joint at boot. The ENA pins belong to the firmware (SPEC 4.3, 10.2), so main.cpp must not hand them to
// the library; with this stub on the include path such a call does not even compile on the host.
//
// This is a model written from the real header, not the library: whether the real library behaves like it can only
// be seen on the ESP32. Not part of any PlatformIO build.
#ifndef HOST_FAST_ACCEL_STEPPER_STUB_H
#define HOST_FAST_ACCEL_STEPPER_STUB_H

#include <stdint.h>

#include <string>

#include "Arduino.h"

enum class MoveResultCode : int8_t {
  OK = 0,
  ErrorNoDirectionPin = -1,
  ErrorSpeedIsUndefined = -2,
  ErrorAccelerationIsUndefined = -3,
};
#define MOVE_OK MoveResultCode::OK

enum class FasDriver : uint8_t { MCPWM_PCNT = 0, RMT = 1, DONT_CARE = 2 };
#define DRIVER_DONT_CARE FasDriver::DONT_CARE

namespace host_fas {
// What the test controls: a stepper "runs" from moveTo until finish() or forceStop().
struct State {
  int32_t position[2] = {0, 0};
  int32_t target[2] = {0, 0};
  bool running[2] = {false, false};
  uint32_t speed_hz[2] = {0, 0};
  int32_t acceleration[2] = {0, 0};
  bool refuse_moves = false;  // moveTo answers an error
};
inline State& state() {
  static State s;
  return s;
}
// The running steppers arrive at their targets.
inline void finish() {
  State& s = state();
  for (int i = 0; i < 2; ++i) {
    if (s.running[i]) s.position[i] = s.target[i];
    s.running[i] = false;
  }
}
}  // namespace host_fas

class FastAccelStepper {
 public:
  explicit FastAccelStepper(uint8_t index, uint8_t step_pin) : index_(index), step_pin_(step_pin) {}

  void setDirectionPin(uint8_t dirPin, bool dirHighCountsUp = true, uint16_t dir_change_delay_us = 0) {
    host_arduino::log().push_back("fas" + std::to_string(index_) + ".dirpin " + std::to_string(dirPin) + (dirHighCountsUp ? " high-counts-up" : " low-counts-up") +
                                  " delay " + std::to_string(dir_change_delay_us));
  }
  void setAutoEnable(bool auto_enable) {
    host_arduino::log().push_back("fas" + std::to_string(index_) + ".autoenable " + (auto_enable ? "1" : "0"));
  }
  int8_t setSpeedInHz(uint32_t speed_hz) {
    host_fas::state().speed_hz[index_] = speed_hz;
    host_arduino::log().push_back("fas" + std::to_string(index_) + ".speed " + std::to_string(speed_hz));
    return 0;
  }
  int8_t setAcceleration(int32_t step_s_s) {
    host_fas::state().acceleration[index_] = step_s_s;
    host_arduino::log().push_back("fas" + std::to_string(index_) + ".accel " + std::to_string(step_s_s));
    return step_s_s > 0 ? 0 : -1;
  }
  MoveResultCode moveTo(int32_t position, bool blocking = false) {
    (void)blocking;
    host_arduino::log().push_back("fas" + std::to_string(index_) + ".moveto " + std::to_string(position));
    if (host_fas::state().refuse_moves) return MoveResultCode::ErrorSpeedIsUndefined;
    host_fas::state().target[index_] = position;
    host_fas::state().running[index_] = position != host_fas::state().position[index_];
    return MoveResultCode::OK;
  }
  int32_t getCurrentPosition() { return host_fas::state().position[index_]; }
  bool isRunning() { return host_fas::state().running[index_]; }
  void forceStop() {
    host_arduino::log().push_back("fas" + std::to_string(index_) + ".forcestop");
    host_fas::state().running[index_] = false;
  }

 private:
  uint8_t index_;
  uint8_t step_pin_;
};

class FastAccelStepperEngine {
 public:
  void init() { host_arduino::log().push_back("fas.init"); }
  FastAccelStepper* stepperConnectToPin(uint8_t step_pin, FasDriver driver_type = DRIVER_DONT_CARE) {
    (void)driver_type;
    host_arduino::log().push_back("fas" + std::to_string(connected_) + ".connect " + std::to_string(step_pin));
    steppers_[connected_] = FastAccelStepper(connected_, step_pin);
    return &steppers_[connected_++];
  }

 private:
  FastAccelStepper steppers_[2] = {FastAccelStepper(0, 0), FastAccelStepper(1, 0)};
  uint8_t connected_ = 0;
};

#endif  // HOST_FAST_ACCEL_STEPPER_STUB_H
