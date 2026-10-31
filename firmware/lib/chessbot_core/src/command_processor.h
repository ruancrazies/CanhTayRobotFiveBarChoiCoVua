// Command dispatch for the hardware-independent core. It owns the state flags and the command queue,
// decides every reply, and reaches the hardware only through the two small interfaces below, so host
// tests run it with fakes.
//
// Stage 4a implements INFO, PING, ENABLE, DISABLE, ESTOP and RESET. Stage 4b-1 (S1) adds the joint commands MOVE,
// ACCEL, WAIT and POS. HOME, ZUP, ZDOWN, ZSTOP and GRIP are parsed (a malformed one is error:2) and then answered
// error:1 ("not implemented yet") until their stage; HOME already passes through the gates below.
//
// Order of checks for a line (SPEC 5.2, "Order of checks"; CB_CHECK_ORDER in protocol.h; the same order as the
// PC simulator, desktop_app/hardware/esp32_simulator.py). The first check that fails decides the reply:
//   0. blank line: no reply                     1. unknown command: error:1
//   2. bad parameter: error:2 (also: a MOVE outside the joint soft limits or with F above the speed ceiling, an ACCEL
//      above the acceleration ceiling)             3. in ESTOP and not accepted there: error:5
//   4. needs the joint drivers and they are disabled: error:6 (MOVE; HOME with no flag or with AB; not HOME Z)
//   5. needs a homed arm: error:4 (MOVE)        6. the motion queue is full: error:3 (MOVE)
//   7. the command itself (a stub in 4a: error:1)
// "Z not at TOP" (MOVE, HOME AB) belongs to check 5 but needs the Z state, which exists from stage 4b-2 (S2). With
// motion.bench_move_without_homing the homed part of check 5 is skipped for MOVE (BENCH ONLY).
// While the ESTOP flag is set only POS, INFO, PING, ZSTOP, ESTOP, RESET and DISABLE are accepted (SPEC 5.1;
// declared once as CB_ESTOP_ACCEPTED in protocol.h). ENABLE is refused with error:5 until RESET.
//
// DISABLE (SPEC 5.1, "DISABLE rules") stops the joint pulses, clears the whole motion queue, drives the ENA pins
// to the disabled level and clears the homed flags. ESTOP, RESET, a heartbeat timeout (and, from 4b, a new HOME)
// clear the queue through the same clear_motion_queue(), which is also where a pending WAIT loses its done
// (SPEC 5.1, "Cancelled done"): one rule, one place.
#ifndef CHESSBOT_COMMAND_PROCESSOR_H
#define CHESSBOT_COMMAND_PROCESSOR_H

#include <stdint.h>

#include "command_parser.h"
#include "command_queue.h"
#include "device_state.h"
#include "motion.h"
#include "project_headers.h"

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
  // Stop the joint pulses only; Z is left alone (DISABLE does not stop a running Z move or a Z-only HOME).
  // Must not touch the ENA pins either: DISABLE drives those itself, after the pulses have stopped.
  virtual void stop_joint_pulses() = 0;
  // Start both joints towards the absolute step counts of ``move`` (only the joints with ``runs`` set), each with its
  // own speed and acceleration. Returns at once; the joints keep running until they arrive or are stopped. Returns
  // false when the board could not start the move (it must then have stopped both joints); the core treats that as a
  // fault and does an ESTOP, so a move that never ran is never reported as finished.
  virtual bool start_joint_move(const JointMove& move) = 0;
  // True while either joint still has steps to issue.
  virtual bool joints_running() = 0;
  // Steps issued so far on ``joint`` (kJointAlpha or kJointBeta), counted from power-up; it follows the joint while it
  // runs and equals the target once it has arrived.
  virtual int32_t joint_steps_issued(uint8_t joint) = 0;
};

// Defined by the unit tests only, to preset the device state (stage 4a has no HOME that could set the homed
// flags). Nothing in the firmware uses it.
struct CommandProcessorTestPeer;

class CommandProcessor {
 public:
  CommandProcessor(ResponseSink* sink, HardwareOps* hardware, const MotionConfig& motion = MotionConfig::from_project());

  // One framed line (no terminator). ``now_ms`` is the free-running millisecond clock.
  void handle_line(const char* line, uint32_t now_ms);
  // The framer reported an unusable line (too long, not printable ASCII): error:1.
  void handle_bad_frame();

  // Heartbeat watchdog step: on expiry stop motion, keep the drivers enabled, latch HB_TIMEOUT, make
  // homing mandatory and send !alarm:HB_TIMEOUT. Stage 4a's main loop does NOT call this yet.
  void tick(uint32_t now_ms);

  // The main loop calls this between lines: a finished MOVE leaves the queue, the next one starts, and the done of a
  // WAIT is sent once the queue has drained. The running MOVE stays in the queue until it has finished, so it counts
  // toward the depth (SPEC 5.2).
  void poll_motion();

  // Put a MOVE in the motion queue together with the ACCEL that is current now. A full queue answers error:3 and
  // returns false (the command is NOT queued); on success it returns true and sends nothing (the caller sends ok).
  bool try_enqueue(const Command& command);

  const DeviceState& state() const { return state_; }
  const HeartbeatWatchdog& watchdog() const { return watchdog_; }
  const CommandQueue& queue() const { return queue_; }

 private:
  friend struct CommandProcessorTestPeer;

  void send_ok();
  void send_error(protocol::ErrorCode code);
  void send_alarm(protocol::AlarmCode code);
  void send_line(bool formatted, const char* line);

  // The one place that empties the motion queue (DISABLE, ESTOP, RESET, heartbeat timeout; a new HOME from 4b).
  void clear_motion_queue();
  // The checks 4 to 6 of the order above for a parsed command; true when it may go on to run. Sends the error
  // itself when it refuses.
  bool passes_gates(const Command& command);

  // Check 2 beyond the parser: the joint soft limits, the speed and acceleration ceilings. Sends error:2 and returns
  // false when the command is out of range.
  bool in_range(const Command& command);
  // Start the oldest queued MOVE on the joints; false when it needed no motion (it is dropped from the queue).
  bool start_next_move();
  double joint_angle_deg(uint8_t joint, int32_t steps) const;
  void send_done();

  void do_info();
  void do_move(const Command& command);
  void do_accel(const Command& command);
  void do_wait();
  void do_pos();
  void do_ping(uint32_t now_ms);
  void do_enable(bool enable);
  void do_estop();
  void do_reset();

  ResponseSink* sink_;
  HardwareOps* hardware_;
  MotionConfig motion_;
  DeviceState state_;
  HeartbeatWatchdog watchdog_;
  CommandQueue queue_;
  double accel_deg_s2_;                      // the ACCEL that is current; before the first ACCEL the ceiling
  double commanded_deg_[kJointCount];        // target angle of the MOVE that started last (0 before the first)
  bool move_running_;                        // the oldest queued MOVE is on the joints
  uint16_t waits_pending_;                   // WAITs that were acknowledged and still owe their done
};

}  // namespace chessbot

#endif  // CHESSBOT_COMMAND_PROCESSOR_H
