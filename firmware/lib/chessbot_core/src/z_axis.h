// The Z axis of docs/SPEC.md sections 4.3, 5 and 10.2 as plain logic with no hardware access: the two limit switches
// with debounce, the TOP / BOTTOM / MID state, the ramped and eased PWM duty, the per-move timeout, and the LIMIT_Z
// rules. The processor reads the switches, calls poll(), and gives the result to the board; src/main.cpp only turns a
// ZDrive and a duty into L298 pin levels and one LEDC channel.
//
// Z is a DC motor with only two end positions. A limit switch is normally closed: it reads "actuated" when it is
// pressed OR when its wire is broken, so a cut wire stops Z (fail-safe). Everything here treats "actuated" as true.
#ifndef CHESSBOT_Z_AXIS_H
#define CHESSBOT_Z_AXIS_H

#include <stdint.h>

#include "project_headers.h"

namespace chessbot {

// What the L298 is told. kBrake is electric braking: ENA fully on with IN1 equal to IN2 (the motor terminals are
// shorted). kCoast is "off": every pin LOW, the motor is free (the boot state, SPEC 10.2). The duty only matters for
// kRaise and kLower; the core reports 100 for kBrake and 0 for kCoast.
enum class ZDrive : uint8_t { kCoast, kRaise, kLower, kBrake };

enum class ZLimit : uint8_t { kTop, kBottom };

// True when a motor driven as ``drive`` must stop at once for these RAW (not debounced) switch readings: it reached the
// limit it was driving towards, or both switches read actuated (a fault: the carriage cannot be at both ends).
// Header-only and trivial on purpose: the limit-switch interrupt calls it, so it must be inlined into IRAM code and
// must touch nothing but its arguments.
inline __attribute__((always_inline)) bool z_must_stop_now(ZDrive drive, bool top_actuated, bool bottom_actuated) {
  if (drive != ZDrive::kRaise && drive != ZDrive::kLower) {
    return false;
  }
  if (top_actuated && bottom_actuated) {
    return true;
  }
  return drive == ZDrive::kRaise ? top_actuated : bottom_actuated;
}

// Every number of the Z axis (from the generated config.h in the firmware, written by hand in the host tests).
struct ZConfig {
  uint32_t full_travel_ms;         // electrical.z_axis.full_travel_ms; 0 = not measured: ZUP and ZDOWN are refused (error:2)
  float timeout_factor;            // electrical.z_axis.timeout_factor
  uint8_t debounce_ms;             // electrical.limit_switches.debounce_ms
  uint8_t duty_start_percent;      // duty at the start of a move
  uint8_t duty_peak_percent;       // duty at the end of the ramp
  uint16_t duty_ramp_ms;           // time of the ramp from start to peak
  uint8_t duty_eased_percent;      // duty near the end of the travel
  uint8_t ease_after_travel_percent;  // the duty drops to the eased value after this share of full_travel_ms

  static ZConfig from_project();

  bool configured() const { return full_travel_ms != 0; }
  // The per-move timeout: timeout_factor times the full-travel time.
  uint32_t timeout_ms() const;
  // The duty of a move that has been running for ``elapsed_ms``: a linear ramp from the start to the peak duty, then the
  // eased duty once the move has run ease_after_travel_percent of the full-travel time (that rule wins over the ramp).
  uint8_t duty_percent(uint32_t elapsed_ms) const;
};

// The duty in percent as LEDC ticks at ``resolution_bits``: 0 gives 0, 100 gives the full scale.
uint32_t z_duty_to_ticks(uint8_t duty_percent, uint8_t resolution_bits);
// The full scale of the LEDC channel (the duty of 100 %): the value the brake writes.
uint32_t z_full_scale_ticks(uint8_t resolution_bits);

// A debounced switch: the stable value changes only after the raw value has differed from it for debounce_ms in a row.
// Power-up value: not actuated, so the Z state is MID until a switch has been seen actuated for the whole window.
class LimitDebouncer {
 public:
  explicit LimitDebouncer(uint32_t debounce_ms) : debounce_ms_(debounce_ms), stable_(false), pending_(false), since_ms_(0) {}

  bool update(bool raw_actuated, uint32_t now_ms);
  bool stable() const { return stable_; }

 private:
  uint32_t debounce_ms_;
  bool stable_;
  bool pending_;
  uint32_t since_ms_;
};

enum class ZEvent : uint8_t {
  kNone,
  kDone,     // the move reached its limit switch (debounced): send done
  kTimeout,  // the move ran out of time: Z is braked, raise !alarm:Z_TIMEOUT
  kLimitZ,   // both switches actuated, or Z left TOP without a command: Z is braked, emergency stop, !alarm:LIMIT_Z (once, until reset())
};

class ZAxis {
 public:
  explicit ZAxis(const ZConfig& config = ZConfig::from_project());

  enum class StartResult : uint8_t { kStarted, kAlreadyThere, kNotConfigured };

  // ZUP (``toward_top``) or ZDOWN. Z already at the target limit (debounced) starts nothing: kAlreadyThere, and the caller
  // answers ok and done at once. A move that is running is replaced: the new one starts from the beginning of its ramp.
  StartResult start(bool toward_top, uint32_t now_ms);
  // ZSTOP, ESTOP, RESET and the heartbeat: stop and brake at once; a pending done is cancelled. Always leaves the brake on.
  void stop();
  // RESET: stop, and forget a latched LIMIT_Z so the next fault is reported again.
  void reset();
  // Bring the logic up to ``now_ms`` with the RAW switch readings (true = actuated). Call it from every loop pass.
  ZEvent poll(uint32_t now_ms, bool top_actuated, bool bottom_actuated);

  // TOP / BOTTOM / MID from the debounced switches. MID at power-up, while moving, and when both read actuated.
  protocol::ZState state() const;
  bool moving() const { return moving_; }
  ZDrive drive() const { return drive_; }
  uint8_t duty_percent() const;
  const ZConfig& config() const { return config_; }

 private:
  void brake_now() {
    drive_ = ZDrive::kBrake;
    moving_ = false;
  }

  ZConfig config_;
  LimitDebouncer top_;
  LimitDebouncer bottom_;
  ZDrive drive_;
  bool moving_;
  bool toward_top_;     // the direction of the move that is (or was last) running
  bool limit_reached_;  // the raw reading of the target switch has stopped the motor; waiting for the debounce
  bool limit_z_latched_;  // LIMIT_Z was raised and has not been acknowledged by reset(): it is reported once, not on every change after it
  bool top_was_stable_;
  // A ZDOWN lets the top switch release, and the debounced release can arrive after the move has already ended (ZSTOP, ESTOP,
  // timeout right after the start): the release stays expected for one debounce window after the last lowering move.
  bool release_expected_;
  bool release_grace_running_;
  uint32_t release_grace_since_ms_;
  uint32_t started_ms_;
  uint8_t duty_;
};

}  // namespace chessbot

#endif  // CHESSBOT_Z_AXIS_H
