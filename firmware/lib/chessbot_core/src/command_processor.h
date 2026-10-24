// Command dispatch for the hardware-independent core. It owns the state flags and the command queue,
// decides every reply, and reaches the hardware only through the two small interfaces below, so host
// tests run it with fakes.
//
// Stage 4a implements INFO, PING, ENABLE, DISABLE, ESTOP and RESET. The other commands are parsed (a
// malformed one is error:2) and then answered error:1 ("not implemented yet") until their stage.
//
// Order of checks for a line (same order as the PC simulator, desktop_app/hardware/esp32_simulator.py):
//   1. blank line: no reply                     2. unknown command: error:1
//   3. bad parameter: error:2                   4. in ESTOP and not an ESTOP-safe command: error:5
//   5. the command itself (a stub: error:1)
#ifndef CHESSBOT_COMMAND_PROCESSOR_H
#define CHESSBOT_COMMAND_PROCESSOR_H

#include <stdint.h>

#include "command_parser.h"
#include "command_queue.h"
#include "device_state.h"
#include "protocol.h"

namespace chessbot {

// Where replies go. The implementation adds the line terminator.
class ResponseSink {
 public:
  virtual ~ResponseSink() {}
  virtual void write_line(const char* line) = 0;
};

// The few things the core asks the hardware to do.
class HardwareOps {
 public:
  virtual ~HardwareOps() {}
  // Drive both TB6600 ENA pins (true = driver enabled).
  virtual void set_drivers_enabled(bool enabled) = 0;
  // Stop every motion output at once: joint pulses and the Z motor. Must NEVER touch the TB6600 ENA pins
  // (SPEC 10.2: the drivers stay enabled in an ESTOP or a heartbeat timeout).
  virtual void halt_motion_outputs() = 0;
};

class CommandProcessor {
 public:
  CommandProcessor(ResponseSink* sink, HardwareOps* hardware);

  // One framed line (no terminator). ``now_ms`` is the free-running millisecond clock.
  void handle_line(const char* line, uint32_t now_ms);
  // The framer reported an unusable line (too long, not printable ASCII): error:1.
  void handle_bad_frame();

  // Heartbeat watchdog step: on expiry stop motion, keep the drivers enabled, latch HB_TIMEOUT, make
  // homing mandatory and send !alarm:HB_TIMEOUT. Stage 4a's main loop does NOT call this yet.
  void tick(uint32_t now_ms);

  // Put a command in the motion queue. A full queue answers error:3 and returns false (the command is
  // NOT queued); on success it returns true and sends nothing (the caller sends ok). Used by MOVE from
  // stage 4b on.
  bool try_enqueue(const Command& command);

  const DeviceState& state() const { return state_; }
  const HeartbeatWatchdog& watchdog() const { return watchdog_; }
  const CommandQueue& queue() const { return queue_; }

 private:
  void send_ok();
  void send_error(protocol::ErrorCode code);
  void send_alarm(protocol::AlarmCode code);
  void send_line(bool formatted, const char* line);

  void do_info();
  void do_ping(uint32_t now_ms);
  void do_enable(bool enable);
  void do_estop();
  void do_reset();

  static bool allowed_in_estop(protocol::Cmd command);

  ResponseSink* sink_;
  HardwareOps* hardware_;
  DeviceState state_;
  HeartbeatWatchdog watchdog_;
  CommandQueue queue_;
};

}  // namespace chessbot

#endif  // CHESSBOT_COMMAND_PROCESSOR_H
