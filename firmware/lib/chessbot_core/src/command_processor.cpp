#include "command_processor.h"

#include "project_headers.h"
#include "response_format.h"

namespace chessbot {

using protocol::AlarmCode;
using protocol::Cmd;
using protocol::ErrorCode;

CommandProcessor::CommandProcessor(ResponseSink* sink, HardwareOps* hardware, const MotionConfig& motion, const ZConfig& z)
    : sink_(sink),
      hardware_(hardware),
      motion_(motion),
      z_axis_(z),
      z_applied_drive_(ZDrive::kCoast),  // the boot state of the pins: every L298 pin LOW (SPEC 10.2)
      z_applied_duty_(0),
      accel_deg_s2_(motion.max_accel_deg_s2),
      commanded_deg_{0, 0},
      move_running_(false),
      waits_pending_(0) {}

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

void CommandProcessor::send_done() {
  char line[config::kMaxResponseBytes + 1];
  send_line(format_done(line, sizeof(line)), line);
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

  if (!in_range(command)) {
    return;
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
    case Cmd::kMOVE:
      do_move(command);
      return;
    case Cmd::kACCEL:
      do_accel(command);
      return;
    case Cmd::kWAIT:
      do_wait();
      return;
    case Cmd::kPOS:
      do_pos();
      return;
    // Not implemented yet: parsed above, answered error:1 until their stage (HOME and GRIP later).
    case Cmd::kZUP:
      do_z_move(true, now_ms);
      return;
    case Cmd::kZDOWN:
      do_z_move(false, now_ms);
      return;
    case Cmd::kZSTOP:
      do_zstop();
      return;
    case Cmd::kHOME:
    case Cmd::kGRIP:
    case Cmd::kCount:
      send_error(ErrorCode::kUNRECOGNIZED);
      return;
  }
}

void CommandProcessor::clear_motion_queue() {
  // A WAIT that is pending is cancelled here, with no done (SPEC 5.1, "Cancelled done"); the running MOVE is discarded
  // with the rest of the queue (the caller has already stopped the pulses).
  queue_.clear();
  move_running_ = false;
  waits_pending_ = 0;
}

// Check 2 beyond the parser (SPEC 5.2): values that are well-formed but outside what the robot allows.
bool CommandProcessor::in_range(const Command& command) {
  bool fine = true;
  if (command.id == Cmd::kMOVE) {
    fine = motion_.within_limits(command.value("A"), command.value("B")) && motion_.speed_allowed(command.value("F")) &&
           motion_.accel_allowed(accel_deg_s2_);  // an undecided acceleration ceiling (null) refuses every MOVE
  } else if (command.id == Cmd::kACCEL) {
    fine = motion_.accel_allowed(command.value("S"));
  }
  if (!fine) {
    send_error(ErrorCode::kBAD_PARAMETER);
  }
  return fine;
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
    // Check 4: not homed (the bench setting skips this part only), or Z not at TOP (the joints must not move with Z down).
    if ((!state_.homed() && !motion_.bench_move_without_homing) || z_axis_.state() != protocol::ZState::kTOP) {
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

void CommandProcessor::do_move(const Command& command) {
  if (try_enqueue(command)) {
    send_ok();
  }
}

void CommandProcessor::do_accel(const Command& command) {
  accel_deg_s2_ = command.value("S");
  send_ok();
}

// ok now, done when the queue has drained (immediately when it is empty). The done is lost when the queue is cleared
// first: clear_motion_queue() forgets the WAITs.
void CommandProcessor::do_wait() {
  send_ok();
  if (queue_.empty()) {
    send_done();
  } else {
    ++waits_pending_;
  }
}

// The angles are where the pulses issued so far have taken each joint; once a joint has arrived they are exactly the
// angles that were asked for (not rounded to a whole step). The sensor angles do not exist yet (stage 4b-4): 0.
void CommandProcessor::do_pos() {
  int32_t steps[kJointCount];
  for (uint8_t joint = 0; joint < kJointCount; ++joint) {
    steps[joint] = hardware_->joint_steps_issued(joint);
  }
  PosReport report;
  report.a_deg = joint_angle_deg(kJointAlpha, steps[kJointAlpha]);
  report.b_deg = joint_angle_deg(kJointBeta, steps[kJointBeta]);
  report.sensor_a_deg = 0;
  report.sensor_b_deg = 0;
  report.steps_a = steps[kJointAlpha];
  report.steps_b = steps[kJointBeta];
  report.z_state = protocol::name_of(z_axis_.state());
  char line[config::kMaxResponseBytes + 1];
  if (format_pos(line, sizeof(line), report)) {
    sink_->write_line(line);
    send_ok();
  }
}

double CommandProcessor::joint_angle_deg(uint8_t joint, int32_t steps) const {
  return steps == angle_to_steps(motion_, commanded_deg_[joint]) ? commanded_deg_[joint] : steps_to_angle(motion_, steps);
}

void CommandProcessor::poll_motion() {
  for (;;) {
    if (move_running_) {
      if (hardware_->joints_running()) {
        return;
      }
      Command finished;
      queue_.pop(&finished);
      move_running_ = false;
    }
    if (queue_.empty()) {
      break;
    }
    if (start_next_move()) {
      return;
    }
  }
  while (waits_pending_ > 0) {  // the queue has drained: every pending WAIT gets its done
    --waits_pending_;
    send_done();
  }
}

bool CommandProcessor::start_next_move() {
  Command command;
  double accel_deg_s2 = 0;
  if (!queue_.peek(&command, &accel_deg_s2)) {
    return false;
  }
  int32_t current[kJointCount];
  for (uint8_t joint = 0; joint < kJointCount; ++joint) {
    current[joint] = hardware_->joint_steps_issued(joint);
  }
  const double alpha_deg = command.value("A");
  const double beta_deg = command.value("B");
  const JointMove move = plan_joint_move(motion_, current, alpha_deg, beta_deg, command.value("F"), accel_deg_s2);
  commanded_deg_[kJointAlpha] = alpha_deg;
  commanded_deg_[kJointBeta] = beta_deg;
  if (!move.any()) {  // already there: nothing to run
    Command dropped;
    queue_.pop(&dropped);
    return false;
  }
  if (!hardware_->start_joint_move(move)) {
    do_estop();  // the joints could not be started: fail safe (stop, clear the queue, latch ESTOP, !alarm:ESTOP)
    return true;
  }
  move_running_ = true;
  return true;
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
  z_axis_.stop();
  apply_z_drive();
  clear_motion_queue();
  state_.enter_estop();
  send_alarm(AlarmCode::kESTOP);
}

void CommandProcessor::do_reset() {
  hardware_->stop_joint_pulses();  // a running MOVE does not survive a RESET (SPEC 5.1, "Cancelled done")
  z_axis_.reset();                 // Z is braked too, and a latched LIMIT_Z is acknowledged
  apply_z_drive();
  clear_motion_queue();
  state_.reset();
  send_ok();
}

void CommandProcessor::tick(uint32_t now_ms) {
  if (!watchdog_.poll(now_ms)) {
    return;
  }
  hardware_->halt_motion_outputs();  // the drivers stay enabled
  z_axis_.stop();
  apply_z_drive();
  clear_motion_queue();
  state_.enter_heartbeat_timeout();
  send_alarm(AlarmCode::kHB_TIMEOUT);
}

// ZUP / ZDOWN: ok, then done when the limit switch has been reached (at once when Z is already there). A Z move that is
// running is replaced. The joint drivers play no part: Z is a DC motor. An unmeasured travel time (null in settings.yaml)
// refuses the move, because the timeout cannot be computed.
void CommandProcessor::do_z_move(bool toward_top, uint32_t now_ms) {
  const ZAxis::StartResult result = z_axis_.start(toward_top, now_ms);
  if (result == ZAxis::StartResult::kNotConfigured) {
    send_error(ErrorCode::kBAD_PARAMETER);
    return;
  }
  apply_z_drive();
  send_ok();
  if (result == ZAxis::StartResult::kAlreadyThere) {
    send_done();
  }
}

// ZSTOP: stop and brake Z at once; the done of a ZUP / ZDOWN that was running is cancelled. A HOME that is waiting for
// Z (stage 4b-3) will be aborted here too.
void CommandProcessor::do_zstop() {
  z_axis_.stop();
  apply_z_drive();
  send_ok();
}

void CommandProcessor::apply_z_drive() {
  const ZDrive drive = z_axis_.drive();
  const uint8_t duty = z_axis_.duty_percent();
  if (drive != z_applied_drive_ || duty != z_applied_duty_) {
    z_applied_drive_ = drive;
    z_applied_duty_ = duty;
    hardware_->drive_z(drive, duty);
  }
}

void CommandProcessor::emergency_stop_for(AlarmCode alarm) {
  hardware_->halt_motion_outputs();
  z_axis_.stop();
  apply_z_drive();
  clear_motion_queue();
  state_.trip(alarm);
  send_alarm(alarm);
}

void CommandProcessor::poll_z(uint32_t now_ms) {
  const ZEvent event = z_axis_.poll(now_ms, hardware_->limit_actuated(ZLimit::kTop), hardware_->limit_actuated(ZLimit::kBottom));
  apply_z_drive();
  switch (event) {
    case ZEvent::kNone:
      return;
    case ZEvent::kDone:
      send_done();
      return;
    case ZEvent::kTimeout:  // Z is braked; homing is mandatory (SPEC 5.3)
      state_.raise_alarm(AlarmCode::kZ_TIMEOUT);
      send_alarm(AlarmCode::kZ_TIMEOUT);
      return;
    case ZEvent::kLimitZ:
      emergency_stop_for(AlarmCode::kLIMIT_Z);
      return;
  }
}

bool CommandProcessor::try_enqueue(const Command& command) {
  if (!queue_.push(command, accel_deg_s2_)) {
    send_error(ErrorCode::kQUEUE_FULL);
    return false;
  }
  return true;
}

}  // namespace chessbot
