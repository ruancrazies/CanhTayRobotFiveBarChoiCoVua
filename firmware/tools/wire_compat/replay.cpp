// HOST-ONLY replay of command lines through the real firmware core (lib/chessbot_core), for
// tools/wire_compat/wire_compat.py. It is not part of any PlatformIO build.
//
// stdin: one command line per input line. The line "#NEW" starts a fresh device (power-up state); "#NEW bench"
// starts one with motion.bench_move_without_homing on. The line "@finish" is not a command: the (faked) joints
// arrive at the targets they were started towards, Z arrives at the switch it drives to (unless "@zstall" was sent), time
// runs on by FINISH_MS, and the core is polled (a WAIT may owe its done, a Z move its done or its timeout).
// The line "@zstall" makes the Z motor turn without ever reaching a switch (a stuck carriage). "@both" presses both limit
// switches (the debounce window passes) and "@clear" puts the carriage back on the top switch alone.
// stdout, per input line:
//   > <the line as given>
//   < <one line per reply the core wrote, in order>
//   = drivers=<0|1> estop=<0|1> homed=<0|1>      (the core's state after the line)
// The first output line is "FW <firmware version>", so the Python side can give the simulator the same one.
//
// The clock starts at 0 and runs on by 100 ms per command line (nothing here exercises the heartbeat). After every
// command line the core is polled (poll_motion, poll_z), as loop() does, so a queued MOVE starts at once; it then runs
// until "@finish". The carriage of a new device rests on the top switch, settled for the debounce window, so Z reads TOP
// like the simulator's initial Z does.
#include <stdio.h>
#include <string.h>

#include "chessbot_core.h"
#include "config.h"

namespace {

class PrintSink : public chessbot::ResponseSink {
 public:
  void write_line(const char* line) override { printf("< %s\n", line); }
};

// Joints that move only when told: a started move keeps running until finish(); the steps issued stay where they
// were until then (POS is compared after "@finish" only).
class FakeHardware : public chessbot::HardwareOps {
 public:
  // Z: the carriage rests on the top switch. Driving it away releases that switch at once; it reaches the other switch
  // on the next arrive_z() unless it is stuck.
  bool top_switch = true;
  bool bottom_switch = false;
  bool z_stuck = false;
  chessbot::ZDrive z_drive = chessbot::ZDrive::kCoast;
  void drive_z(chessbot::ZDrive drive, uint8_t) override {
    z_drive = drive;
    if (drive == chessbot::ZDrive::kLower) top_switch = false;
    if (drive == chessbot::ZDrive::kRaise) bottom_switch = false;
  }
  bool limit_actuated(chessbot::ZLimit limit) override { return limit == chessbot::ZLimit::kTop ? top_switch : bottom_switch; }
  void arrive_z() {
    if (z_stuck) return;
    if (z_drive == chessbot::ZDrive::kLower) bottom_switch = true;
    if (z_drive == chessbot::ZDrive::kRaise) top_switch = true;
  }

  void set_drivers_enabled(bool) override {}
  void halt_motion_outputs() override { stop_joint_pulses(); }  // (the core brakes Z itself through drive_z)
  void stop_joint_pulses() override { running_ = false; }
  bool start_joint_move(const chessbot::JointMove& move) override {
    for (uint8_t joint = 0; joint < chessbot::kJointCount; ++joint) {
      if (move.runs[joint]) {
        target_[joint] = move.target_steps[joint];
      }
    }
    running_ = true;
    return true;
  }
  bool joints_running() override { return running_; }
  int32_t joint_steps_issued(uint8_t joint) override { return steps_[joint]; }
  void finish() {
    if (running_) {
      for (uint8_t joint = 0; joint < chessbot::kJointCount; ++joint) {
        steps_[joint] = target_[joint];
      }
    }
    running_ = false;
  }

 private:
  bool running_ = false;
  int32_t steps_[chessbot::kJointCount] = {0, 0};
  int32_t target_[chessbot::kJointCount] = {0, 0};
};

constexpr uint32_t kLineMs = 100;     // the clock advance of one command line (more than the debounce window)
constexpr uint32_t kFinishStepMs = 100;
constexpr int kFinishSteps = 100;     // "@finish" lets 10 s pass in steps: more than the Z timeout (1.5 x the full travel)

chessbot::MotionConfig motion_config(bool bench) {
  chessbot::MotionConfig motion = chessbot::MotionConfig::from_project();
  motion.bench_move_without_homing = bench;
  return motion;
}

}  // namespace

int main() {
  PrintSink sink;
  FakeHardware* hardware = new FakeHardware();
  bool bench = false;
  chessbot::CommandProcessor* processor = new chessbot::CommandProcessor(&sink, hardware, motion_config(bench));
  printf("FW %s\n", chessbot::config::kFirmwareVersion);
  uint32_t now = 0;
  auto settle = [&]() {  // the debounce window passes with the top switch pressed
    processor->poll_z(now);
    now += kLineMs;
    processor->poll_z(now);
  };
  settle();

  char buffer[512];
  while (fgets(buffer, sizeof(buffer), stdin) != nullptr) {
    size_t length = strlen(buffer);
    while (length > 0 && (buffer[length - 1] == '\n' || buffer[length - 1] == '\r')) {
      buffer[--length] = '\0';
    }
    if (strncmp(buffer, "#NEW", 4) == 0) {
      delete processor;
      delete hardware;
      hardware = new FakeHardware();
      bench = strcmp(buffer, "#NEW bench") == 0;
      processor = new chessbot::CommandProcessor(&sink, hardware, motion_config(bench));
      now = 0;
      settle();
      printf("%s\n", buffer);
      continue;
    }
    printf("> %s\n", buffer);
    if (strcmp(buffer, "@finish") == 0) {
      for (int step = 0; step < kFinishSteps; ++step) {  // every queued MOVE runs to its end, as loop() would let them
        hardware->finish();
        hardware->arrive_z();
        now += kFinishStepMs;
        processor->poll_motion();
        processor->poll_z(now);
      }
    } else if (strcmp(buffer, "@zstall") == 0) {
      hardware->z_stuck = true;
    } else if (strcmp(buffer, "@both") == 0 || strcmp(buffer, "@clear") == 0) {
      const bool both = strcmp(buffer, "@both") == 0;
      hardware->top_switch = true;
      hardware->bottom_switch = both;
      for (int step = 0; step < 2; ++step) {  // the debounce window passes
        now += kLineMs;
        processor->poll_z(now);
      }
    } else {
      now += kLineMs;
      processor->handle_line(buffer, now);
      processor->poll_motion();
      processor->poll_z(now);
    }
    const chessbot::DeviceState& state = processor->state();
    printf("= drivers=%d estop=%d homed=%d\n", state.drivers_enabled() ? 1 : 0, state.estop() ? 1 : 0, state.homed() ? 1 : 0);
  }
  delete processor;
  delete hardware;
  return 0;
}
