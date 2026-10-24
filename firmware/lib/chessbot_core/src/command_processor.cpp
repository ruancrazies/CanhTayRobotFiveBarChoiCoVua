#include "command_processor.h"

#include "config.h"
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

// Commands that still work in an ESTOP (same set as the PC simulator). Everything else is error:5.
bool CommandProcessor::allowed_in_estop(Cmd command) {
  switch (command) {
    case Cmd::kPING:
    case Cmd::kPOS:
    case Cmd::kINFO:
    case Cmd::kRESET:
    case Cmd::kESTOP:
    case Cmd::kZSTOP:
      return true;
    case Cmd::kHOME:
    case Cmd::kMOVE:
    case Cmd::kZUP:
    case Cmd::kZDOWN:
    case Cmd::kACCEL:
    case Cmd::kGRIP:
    case Cmd::kENABLE:
    case Cmd::kDISABLE:
    case Cmd::kWAIT:
    case Cmd::kCount:
      return false;
  }
  return false;
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

  if (state_.estop() && !allowed_in_estop(command.id)) {
    send_error(ErrorCode::kIN_ESTOP);
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

void CommandProcessor::do_enable(bool enable) {
  hardware_->set_drivers_enabled(enable);
  state_.set_drivers_enabled(enable);
  send_ok();
}

// SPEC 10.2: stop the motion outputs, keep the drivers enabled, set the ESTOP flag. Answered by the
// alarm alone (no ok).
void CommandProcessor::do_estop() {
  hardware_->halt_motion_outputs();
  queue_.clear();
  state_.enter_estop();
  send_alarm(AlarmCode::kESTOP);
}

void CommandProcessor::do_reset() {
  queue_.clear();
  state_.reset();
  send_ok();
}

void CommandProcessor::tick(uint32_t now_ms) {
  if (!watchdog_.poll(now_ms)) {
    return;
  }
  hardware_->halt_motion_outputs();  // the drivers stay enabled
  queue_.clear();
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
