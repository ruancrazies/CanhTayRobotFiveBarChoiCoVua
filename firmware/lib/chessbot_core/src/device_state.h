// The state flags of docs/SPEC.md section 10.2: homed, ESTOP, heartbeat watchdog, drivers enabled, and
// the latched alarms. Plain data and rules, no hardware access (the processor applies the pins).
#ifndef CHESSBOT_DEVICE_STATE_H
#define CHESSBOT_DEVICE_STATE_H

#include <stdint.h>

#include "config.h"
#include "protocol.h"

namespace chessbot {

// Heartbeat (SPEC 5.3, 10.2). Armed by the first PING; a gap longer than the timeout expires it once.
// Time is a free-running unsigned millisecond counter that may wrap.
class HeartbeatWatchdog {
 public:
  explicit HeartbeatWatchdog(uint32_t timeout_ms = config::kHeartbeatTimeoutMs) : timeout_ms_(timeout_ms), last_ms_(0), armed_(false) {}

  void feed(uint32_t now_ms) {
    armed_ = true;
    last_ms_ = now_ms;
  }
  bool armed() const { return armed_; }
  uint32_t timeout_ms() const { return timeout_ms_; }

  // True exactly once when an armed watchdog has gone longer than the timeout without a feed(); it is
  // disarmed by that, until the next feed().
  bool poll(uint32_t now_ms) {
    if (armed_ && static_cast<uint32_t>(now_ms - last_ms_) > timeout_ms_) {
      armed_ = false;
      return true;
    }
    return false;
  }

 private:
  uint32_t timeout_ms_;
  uint32_t last_ms_;
  bool armed_;
};

class DeviceState {
 public:
  // Power-up: not homed, no ESTOP, no alarm, drivers ENABLED (the ENA pins float or are driven HIGH at
  // boot so the motors hold position; SPEC 10.2).
  DeviceState() : homed_z_(false), homed_joints_(false), estop_(false), drivers_enabled_(true), alarms_(0) {}

  // Homed means Z and both joints are homed (SPEC 10.2: every MOVE is error:4 until then).
  bool homed() const { return homed_z_ && homed_joints_; }
  bool homed_z() const { return homed_z_; }
  bool homed_joints() const { return homed_joints_; }
  void set_homed_z(bool value) { homed_z_ = value; }
  void set_homed_joints(bool value) { homed_joints_ = value; }

  bool estop() const { return estop_; }
  bool drivers_enabled() const { return drivers_enabled_; }
  void set_drivers_enabled(bool value) { drivers_enabled_ = value; }

  bool alarm_latched(protocol::AlarmCode alarm) const { return (alarms_ & bit(alarm)) != 0; }
  bool any_alarm() const { return alarms_ != 0; }

  // ESTOP (command or button): set the flag, latch ESTOP, homing becomes mandatory. The drivers stay
  // enabled (SPEC 10.2). Idempotent.
  void enter_estop() {
    estop_ = true;
    latch(protocol::AlarmCode::kESTOP);
    clear_homed();
  }

  // Heartbeat timeout (SPEC 10.2): latch HB_TIMEOUT, homing becomes mandatory. NOT an ESTOP, and the
  // drivers stay enabled.
  void enter_heartbeat_timeout() {
    latch(protocol::AlarmCode::kHB_TIMEOUT);
    clear_homed();
  }

  // Any other alarm: latch it and make homing mandatory (SPEC 5.3).
  void raise_alarm(protocol::AlarmCode alarm) {
    latch(alarm);
    clear_homed();
  }

  // RESET: clear ESTOP and every alarm; homing is mandatory afterwards. Leaves the drivers as they are.
  void reset() {
    estop_ = false;
    alarms_ = 0;
    clear_homed();
  }

 private:
  static uint8_t bit(protocol::AlarmCode alarm) { return static_cast<uint8_t>(1U << static_cast<uint8_t>(alarm)); }
  void latch(protocol::AlarmCode alarm) { alarms_ = static_cast<uint8_t>(alarms_ | bit(alarm)); }
  void clear_homed() {
    homed_z_ = false;
    homed_joints_ = false;
  }

  bool homed_z_;
  bool homed_joints_;
  bool estop_;
  bool drivers_enabled_;
  uint8_t alarms_;  // one bit per protocol::AlarmCode (at most 8 alarm codes fit)
};

static_assert(protocol::kAlarmCount <= 8, "alarms_ is a uint8_t bit mask: widen it before declaring a ninth alarm");

}  // namespace chessbot

#endif  // CHESSBOT_DEVICE_STATE_H
