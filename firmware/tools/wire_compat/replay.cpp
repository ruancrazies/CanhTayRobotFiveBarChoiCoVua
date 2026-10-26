// HOST-ONLY replay of command lines through the real firmware core (lib/chessbot_core), for
// tools/wire_compat/wire_compat.py. It is not part of any PlatformIO build.
//
// stdin: one command line per input line. The line "#NEW" starts a fresh device (power-up state).
// stdout, per input line:
//   > <the line as given>
//   < <one line per reply the core wrote, in order>
//   = drivers=<0|1> estop=<0|1> homed=<0|1>      (the core's state after the line)
// The first output line is "FW <firmware version>", so the Python side can give the simulator the same one.
//
// Time is a constant 0: nothing here exercises the heartbeat, and the core is never ticked.
#include <stdio.h>
#include <string.h>

#include "chessbot_core.h"
#include "config.h"

namespace {

class PrintSink : public chessbot::ResponseSink {
 public:
  void write_line(const char* line) override { printf("< %s\n", line); }
};

class NullHardware : public chessbot::HardwareOps {
 public:
  void set_drivers_enabled(bool) override {}
  void halt_motion_outputs() override {}
  void stop_joint_pulses() override {}
};

}  // namespace

int main() {
  PrintSink sink;
  NullHardware hardware;
  chessbot::CommandProcessor* processor = new chessbot::CommandProcessor(&sink, &hardware);
  printf("FW %s\n", chessbot::config::kFirmwareVersion);

  char buffer[512];
  while (fgets(buffer, sizeof(buffer), stdin) != nullptr) {
    size_t length = strlen(buffer);
    while (length > 0 && (buffer[length - 1] == '\n' || buffer[length - 1] == '\r')) {
      buffer[--length] = '\0';
    }
    if (strcmp(buffer, "#NEW") == 0) {
      delete processor;
      processor = new chessbot::CommandProcessor(&sink, &hardware);
      printf("#NEW\n");
      continue;
    }
    printf("> %s\n", buffer);
    processor->handle_line(buffer, 0);
    const chessbot::DeviceState& state = processor->state();
    printf("= drivers=%d estop=%d homed=%d\n", state.drivers_enabled() ? 1 : 0, state.estop() ? 1 : 0, state.homed() ? 1 : 0);
  }
  delete processor;
  return 0;
}
