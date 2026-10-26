#include "command_processor.h"

#include "project_headers.h"
#include "response_format.h"

namespace chessbot {

using protocol::AlarmCode;
using protocol::Cmd;
using protocol::ErrorCode;

CommandProcessor::CommandProcessor(ResponseSink* sink, HardwareOps* hardware) : sink_(sink), hardware_(hardware) {}

void CommandProcessor::send_line(bool formatted, const char* line) {
  if (formatted) {
    sink_->write_line(line);
  }
}

void CommandProcessor::send_ok() {
  char line[config::kMaxResponseBytes + 1];
  send_line(format_ok(line, sizeof(line)), line);
}

void CommandProcessor::send_error(ErrorCode code) {
  char line[config::kMaxResponseBytes + 1];
  send_line(format_error(line, sizeof(line), code), line);
}

void CommandProcessor::send_alarm(AlarmCode code) {
  char line[config::kMaxResponseBytes + 1];
  send_line(format_alarm(line, sizeof(line), code), line);
}

void CommandProcessor::handle_bad_frame() { send_error(ErrorCode::kUNRECOGNIZED); }

void CommandProcessor::handle_line(const char* line, uint32_t now_ms) {
  Command command;
  switch (parse_command(line, &command)) {
    case ParseStatus::kEmpty:
      return;
    case ParseStatus::kUnrecognized:
      send_error(ErrorCode::kUNRECOGNIZED);
      return;
    case ParseStatus::kBadParameter:
      send_error(ErrorCode::kBAD_PARAMETER);
      return;
    case ParseStatus::kOk:
      break;
  }

  // The ESTOP set is declared once, in protocol.h (CB_ESTOP_ACCEPTED, SPEC 5.1): ENABLE is not in it.
  if (state_.estop() && !protocol::accepted_in_estop(command.id)) {
    send_error(ErrorCode::kIN_ESTOP);
    return;
  }
  if (!passes_gates(command)) {
    return;
  }

  switch (command.id) {
    case Cmd::kINFO:
      do_info();
      return;
    case Cmd::kPING:
      do_ping(now_ms);
      return;
    case Cmd::kENABLE:
      do_enable(true);
      return;
    case Cmd::kDISABLE:
      do_enable(false);
      return;
    case Cmd::kESTOP:
      do_estop();
      return;
    case Cmd::kRESET:
      do_reset();
      return;
    // Not implemented in stage 4a: parsed above, answered error:1 until their stage.
    case Cmd::kHOME:
    case Cmd::kMOVE:
    case Cmd::kZUP:
    case Cmd::kZDOWN:
    case Cmd::kZSTOP:
    case Cmd::kACCEL:
    case Cmd::kGRIP:
    case Cmd::kWAIT:
    case Cmd::kPOS:
    case Cmd::kCount:
      send_error(ErrorCode::kUNRECOGNIZED);
      return;
  }
}

void CommandProcessor::clear_motion_queue() {
  // Stage 4b: a WAIT that is pending is cancelled here, with no done (SPEC 5.1, "Cancelled done"). There is no
  // WAIT in 4a, so the queue is all there is to clear.
  queue_.clear();
}

// Checks 4 to 6 of the order in command_processor.h, in that order. Only MOVE and HOME have gates; every other
// command passes. HOME that moves only Z (HOME Z, without AB) needs no joint driver and is not gated.
bool CommandProcessor::passes_gates(const Command& command) {
  const bool moves_joints = command.id == Cmd::kMOVE || (command.id == Cmd::kHOME && (command.has("AB") || !command.has("Z")));
  if (moves_joints && !state_.drivers_enabled()) {
    send_error(ErrorCode::kDRIVERS_DISABLED);
    return false;
  }
  if (command.id == Cmd::kMOVE) {
    if (!state_.homed()) {
      send_error(ErrorCode::kNOT_HOMED);
      return false;
    }
    if (queue_.full()) {
      send_error(ErrorCode::kQUEUE_FULL);
      return false;
    }
  }
  return true;
}

void CommandProcessor::do_info() {
  char line[config::kMaxResponseBytes + 1];
  if (format_info(line, sizeof(line), config::kFirmwareVersion)) {
    sink_->write_line(line);
    send_ok();
  }
}

void CommandProcessor::do_ping(uint32_t now_ms) {
  watchdog_.feed(now_ms);
  send_ok();
}

// ENABLE only sets the pins and the flag. DISABLE (SPEC 5.1, "DISABLE rules"), in this order: stop the joint pulses,
// clear the whole queue, drive the ENA pins to the disabled level, clear the homed flags (set_drivers_enabled).
// Z is not touched, so a Z-only HOME and a running ZUP / ZDOWN carry on (stage 4b).
void CommandProcessor::do_enable(bool enable) {
  if (!enable) {
    hardware_->stop_joint_pulses();
    clear_motion_queue();
  }
  hardware_->set_drivers_enabled(enable);
  state_.set_drivers_enabled(enable);
  send_ok();
}

// SPEC 10.2: stop the motion outputs, keep the drivers enabled, set the ESTOP flag. Answered by the
// alarm alone (no ok). Repeating it while already in ESTOP is accepted and harmless: the same steps run
// again and the alarm is sent again (SPEC 5.1).
void CommandProcessor::do_estop() {
  hardware_->halt_motion_outputs();
  clear_motion_queue();
  state_.enter_estop();
  send_alarm(AlarmCode::kESTOP);
}

void CommandProcessor::do_reset() {
  clear_motion_queue();
  state_.reset();
  send_ok();
}

void CommandProcessor::tick(uint32_t now_ms) {
  if (!watchdog_.poll(now_ms)) {
    return;
  }
  hardware_->halt_motion_outputs();  // the drivers stay enabled
  clear_motion_queue();
  state_.enter_heartbeat_timeout();
  send_alarm(AlarmCode::kHB_TIMEOUT);
}

bool CommandProcessor::try_enqueue(const Command& command) {
  if (!queue_.push(command)) {
    send_error(ErrorCode::kQUEUE_FULL);
    return false;
  }
  return true;
}

}  // namespace chessbot
