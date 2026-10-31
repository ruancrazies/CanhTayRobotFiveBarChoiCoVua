// HOST-ONLY replay of command lines through the real firmware core (lib/chessbot_core), for
// tools/wire_compat/wire_compat.py. It is not part of any PlatformIO build.
//
// stdin: one command line per input line. The line "#NEW" starts a fresh device (power-up state); "#NEW bench"
// starts one with motion.bench_move_without_homing on. The line "@finish" is not a command: the (faked) joints
// arrive at the targets they were started towards, and the core is polled (a WAIT may owe its done).
// stdout, per input line:
//   > <the line as given>
//   < <one line per reply the core wrote, in order>
//   = drivers=<0|1> estop=<0|1> homed=<0|1>      (the core's state after the line)
// The first output line is "FW <firmware version>", so the Python side can give the simulator the same one.
//
// Time is a constant 0: nothing here exercises the heartbeat. After every command line the core is polled
// (poll_motion), as loop() does, so a queued MOVE starts at once; it then runs until "@finish".
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
  void set_drivers_enabled(bool) override {}
  void halt_motion_outputs() override { stop_joint_pulses(); }
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
      printf("%s\n", buffer);
      continue;
    }
    printf("> %s\n", buffer);
    if (strcmp(buffer, "@finish") == 0) {
      do {  // every queued MOVE runs to its end, as loop() would let them
        hardware->finish();
        processor->poll_motion();
      } while (hardware->joints_running());
    } else {
      processor->handle_line(buffer, 0);
      processor->poll_motion();
    }
    const chessbot::DeviceState& state = processor->state();
    printf("= drivers=%d estop=%d homed=%d\n", state.drivers_enabled() ? 1 : 0, state.estop() ? 1 : 0, state.homed() ? 1 : 0);
  }
  delete processor;
  delete hardware;
  return 0;
}
