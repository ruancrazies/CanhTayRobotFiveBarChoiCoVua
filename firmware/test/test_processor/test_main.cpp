// CommandProcessor with a fake sink and fake hardware: every reply of stage 4a, SPEC 10.2 boot / ESTOP
// behaviour, the stubbed commands, the heartbeat step, the queue-full error, and the driver rules of the phase 4a
// follow-up (error:6, the order of checks of SPEC 5.2, what DISABLE does to the queue).
//
// Stage 4b-1 made MOVE, ACCEL, WAIT and POS real; their behaviour is tested in test/test_motion. The Bench below uses
// a motion configuration WITHOUT soft limits, so the tiny angles of these gate tests (A=1 B=2) are in range.
#include <string>
#include <unity.h>
#include <vector>

#include "command_processor.h"
#include "config.h"
#include "protocol.h"

using chessbot::CommandProcessor;
namespace protocol = chessbot::protocol;

// CommandProcessor declares this a friend. Stage 4a has no HOME that could set the homed flags, so the test
// presets them here to see what the commands do to a homed device.
namespace chessbot {
struct CommandProcessorTestPeer {
  static DeviceState& state(CommandProcessor& processor) { return processor.state_; }
};
}  // namespace chessbot

void setUp(void) {}
void tearDown(void) {}

namespace {

struct FakeSink : chessbot::ResponseSink {
  std::vector<std::string> lines;
  void write_line(const char* line) override { lines.push_back(line); }
};

// The project's motion settings, but with no soft limit: these tests are about the gates, not about the limits.
chessbot::MotionConfig gate_test_config() {
  chessbot::MotionConfig config = chessbot::MotionConfig::from_project();
  config.limits_configured = false;
  config.bench_move_without_homing = false;
  return config;
}

struct FakeHardware : chessbot::HardwareOps {
  std::vector<bool> driver_calls;  // every set_drivers_enabled(...) argument, in order
  std::vector<std::string> events;  // every hardware call, in order: "pulses", "halt", "drivers=0", "drivers=1"
  int halts = 0;
  int joint_stops = 0;
  void set_drivers_enabled(bool enabled) override {
    driver_calls.push_back(enabled);
    events.push_back(enabled ? "drivers=1" : "drivers=0");
  }
  void halt_motion_outputs() override {
    ++halts;
    events.push_back("halt");
  }
  void stop_joint_pulses() override {
    ++joint_stops;
    events.push_back("pulses");
  }
  bool start_joint_move(const chessbot::JointMove&) override { return true; }
  bool joints_running() override { return false; }
  int32_t joint_steps_issued(uint8_t) override { return 0; }
  // Z: the carriage sits on the top switch unless a test moves it; every drive_z() call is recorded.
  bool top_switch = true;
  bool bottom_switch = false;
  std::vector<std::string> z_calls;
  void drive_z(chessbot::ZDrive drive, uint8_t duty_percent) override {
    static const char* const names[] = {"coast", "raise", "lower", "brake"};
    z_calls.push_back(std::string(names[static_cast<int>(drive)]) + ":" + std::to_string(duty_percent));
  }
  bool limit_actuated(chessbot::ZLimit limit) override { return limit == chessbot::ZLimit::kTop ? top_switch : bottom_switch; }
};

struct Bench {
  FakeSink sink;
  FakeHardware hardware;
  CommandProcessor processor;
  uint32_t now = 0;
  Bench() : processor(&sink, &hardware, gate_test_config()) {
    processor.poll_z(0);  // let the debounce see the top switch: Z reads TOP
    processor.poll_z(20);
  }

  // Send one line; return the replies it produced.
  std::vector<std::string> send(const char* line) {
    sink.lines.clear();
    processor.handle_line(line, now);
    return sink.lines;
  }
};

bool same(const std::vector<std::string>& got, std::initializer_list<const char*> want) {
  if (got.size() != want.size()) {
    return false;
  }
  size_t i = 0;
  for (const char* expected : want) {
    if (got[i++] != expected) {
      return false;
    }
  }
  return true;
}

void expect(const std::vector<std::string>& got, std::initializer_list<const char*> want, const char* what) {
  if (!same(got, want)) {
    std::string message = std::string(what) + ": got [";
    for (const std::string& line : got) {
      message += line + "|";
    }
    message += "]";
    TEST_FAIL_MESSAGE(message.c_str());
  }
}

}  // namespace

// ---- boot ----------------------------------------------------------------------------------------

void test_a_new_processor_touches_no_hardware_and_says_nothing() {
  Bench bench;
  TEST_ASSERT_TRUE(bench.hardware.driver_calls.empty());
  TEST_ASSERT_EQUAL_INT(0, bench.hardware.halts);
  TEST_ASSERT_TRUE(bench.sink.lines.empty());
  TEST_ASSERT_TRUE(bench.processor.state().drivers_enabled());  // matches the boot pins (SPEC 10.2)
  TEST_ASSERT_TRUE(!bench.processor.state().estop());
  TEST_ASSERT_TRUE(!bench.processor.state().homed());
}

// ---- the six implemented commands ----------------------------------------------------------------

void test_info() {
  Bench bench;
  const std::string info = std::string("INFO FW=") + chessbot::config::kFirmwareVersion + " PROTO=" + std::to_string(protocol::kProtocolVersion);
  expect(bench.send("INFO"), {info.c_str(), "ok"}, "INFO");
}

void test_ping_answers_ok_and_arms_the_watchdog() {
  Bench bench;
  TEST_ASSERT_TRUE(!bench.processor.watchdog().armed());
  expect(bench.send("PING"), {"ok"}, "PING");
  TEST_ASSERT_TRUE(bench.processor.watchdog().armed());
}

void test_disable_then_enable() {
  Bench bench;
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE");
  TEST_ASSERT_TRUE(!bench.processor.state().drivers_enabled());
  expect(bench.send("ENABLE"), {"ok"}, "ENABLE");
  TEST_ASSERT_TRUE(bench.processor.state().drivers_enabled());
  TEST_ASSERT_EQUAL_INT(2, bench.hardware.driver_calls.size());
  TEST_ASSERT_TRUE(!bench.hardware.driver_calls[0]);
  TEST_ASSERT_TRUE(bench.hardware.driver_calls[1]);
}

void test_estop_is_answered_by_the_alarm_alone() {
  Bench bench;
  expect(bench.send("ESTOP"), {"!alarm:ESTOP"}, "ESTOP");
}

void test_estop_stops_motion_sets_the_flag_and_keeps_the_drivers_enabled() {
  Bench bench;
  bench.send("ESTOP");
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.halts);
  TEST_ASSERT_TRUE(bench.processor.state().estop());
  TEST_ASSERT_TRUE(!bench.processor.state().homed());
  TEST_ASSERT_TRUE(bench.hardware.driver_calls.empty());  // the ENA pins are never touched (SPEC 10.2)
  TEST_ASSERT_TRUE(bench.processor.state().drivers_enabled());
}

void test_estop_is_idempotent_and_answers_the_alarm_again() {
  Bench bench;
  bench.send("ESTOP");
  expect(bench.send("ESTOP"), {"!alarm:ESTOP"}, "second ESTOP");
  TEST_ASSERT_TRUE(bench.processor.state().estop());
}

void test_estop_empties_the_queue() {
  Bench bench;
  chessbot::Command move(protocol::Cmd::kMOVE);
  TEST_ASSERT_TRUE(bench.processor.try_enqueue(move));
  bench.send("ESTOP");
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
}

void test_in_estop_the_motion_commands_and_enable_are_error_5() {
  Bench bench;
  bench.send("ESTOP");
  const char* refused[] = {"HOME", "HOME Z", "MOVE A=1 B=2 F=3", "ZUP", "ZDOWN", "ACCEL S=1", "GRIP S=90", "ENABLE", "WAIT"};
  for (const char* line : refused) {
    expect(bench.send(line), {"error:5"}, line);
  }
  TEST_ASSERT_TRUE(bench.hardware.driver_calls.empty());  // the refused ENABLE did not reach the pins
}

// ---- the ESTOP rules (owner decisions, SPEC 5.1 / 10.2) ------------------------------------------
// The two lists below are typed out on purpose: they are NOT taken from protocol.h, which the code under test
// uses, so the test cannot agree with the code by construction. tests/test_protocol_declaration.py ties
// protocol.h to the SPEC text and to the Python declaration.

namespace {

struct EstopCase {
  const char* name;  // the command word
  const char* line;  // a valid line for it
};

const EstopCase kAcceptedInEstop[] = {
    {"ZSTOP", "ZSTOP"}, {"DISABLE", "DISABLE"}, {"POS", "POS"}, {"INFO", "INFO"},
    {"PING", "PING"},   {"ESTOP", "ESTOP"},     {"RESET", "RESET"},
};
const EstopCase kRefusedInEstop[] = {
    {"HOME", "HOME"},         {"MOVE", "MOVE A=1 B=2 F=3"}, {"ZUP", "ZUP"},       {"ZDOWN", "ZDOWN"},
    {"ACCEL", "ACCEL S=1"},   {"GRIP", "GRIP S=90"},        {"ENABLE", "ENABLE"}, {"WAIT", "WAIT"},
};

bool listed(const EstopCase* cases, size_t count, const char* name) {
  for (size_t i = 0; i < count; ++i) {
    if (std::string(cases[i].name) == name) return true;
  }
  return false;
}

}  // namespace

void test_the_two_estop_lists_cover_every_command_of_the_protocol_exactly_once() {
  const size_t accepted = sizeof(kAcceptedInEstop) / sizeof(kAcceptedInEstop[0]);
  const size_t refused = sizeof(kRefusedInEstop) / sizeof(kRefusedInEstop[0]);
  TEST_ASSERT_EQUAL_INT(protocol::kCommandCount, accepted + refused);
  for (uint8_t c = 0; c < protocol::kCommandCount; ++c) {
    const char* name = protocol::kCommands[c].name;
    const bool in_accepted = listed(kAcceptedInEstop, accepted, name);
    const bool in_refused = listed(kRefusedInEstop, refused, name);
    if (in_accepted == in_refused) TEST_FAIL_MESSAGE(name);  // in both lists, or in neither
  }
}

void test_in_estop_every_command_outside_the_set_is_error_5() {
  for (const EstopCase& item : kRefusedInEstop) {
    Bench bench;
    bench.send("ESTOP");
    expect(bench.send(item.line), {"error:5"}, item.line);
  }
}

void test_in_estop_every_command_in_the_set_is_accepted_and_none_is_error_5() {
  const std::string info = std::string("INFO FW=") + chessbot::config::kFirmwareVersion + " PROTO=" + std::to_string(protocol::kProtocolVersion);
  for (const EstopCase& item : kAcceptedInEstop) {
    Bench bench;
    bench.send("ESTOP");
    const std::vector<std::string> reply = bench.send(item.line);
    for (const std::string& line : reply) {
      if (line == "error:5") TEST_FAIL_MESSAGE(item.line);
    }
    const std::string name = item.name;
    if (name == "PING" || name == "DISABLE" || name == "RESET") {
      expect(reply, {"ok"}, item.line);
    } else if (name == "ESTOP") {
      expect(reply, {"!alarm:ESTOP"}, item.line);
    } else if (name == "INFO") {
      expect(reply, {info.c_str(), "ok"}, item.line);
    } else if (name == "POS") {
      expect(reply, {"POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=TOP", "ok"}, item.line);  // implemented in stage 4b-1 (S1)
    } else {
      expect(reply, {"ok"}, item.line);  // ZSTOP: implemented in stage 4b-1 S2, accepted in an ESTOP
    }
  }
}

void test_disable_is_accepted_in_estop_and_reaches_the_pins() {
  Bench bench;
  bench.send("ESTOP");
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE in ESTOP");
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.driver_calls.size());
  TEST_ASSERT_TRUE(!bench.hardware.driver_calls[0]);
  TEST_ASSERT_TRUE(!bench.processor.state().drivers_enabled());
  TEST_ASSERT_TRUE(bench.processor.state().estop());  // DISABLE does not leave the ESTOP
}

void test_enable_is_refused_in_estop_and_accepted_again_after_reset() {
  Bench bench;
  bench.send("DISABLE");
  bench.send("ESTOP");
  expect(bench.send("ENABLE"), {"error:5"}, "ENABLE in ESTOP");
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.driver_calls.size());  // only the first DISABLE: ENABLE never reached the pins
  TEST_ASSERT_TRUE(!bench.processor.state().drivers_enabled());
  expect(bench.send("RESET"), {"ok"}, "RESET");
  TEST_ASSERT_TRUE(!bench.processor.state().estop());
  expect(bench.send("ENABLE"), {"ok"}, "ENABLE after RESET");
  TEST_ASSERT_EQUAL_INT(2, bench.hardware.driver_calls.size());
  TEST_ASSERT_TRUE(bench.hardware.driver_calls[1]);
  TEST_ASSERT_TRUE(bench.processor.state().drivers_enabled());
}

void test_a_repeated_estop_is_accepted_and_harmless() {
  Bench bench;
  bench.send("DISABLE");
  expect(bench.send("ESTOP"), {"!alarm:ESTOP"}, "first ESTOP");
  for (int i = 0; i < 3; ++i) {
    expect(bench.send("ESTOP"), {"!alarm:ESTOP"}, "repeated ESTOP");
  }
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.driver_calls.size());  // the ENA pins were not touched again
  TEST_ASSERT_TRUE(!bench.processor.state().drivers_enabled());  // and the drivers did not come back
  TEST_ASSERT_TRUE(bench.processor.state().estop());
  TEST_ASSERT_TRUE(bench.processor.state().alarm_latched(protocol::AlarmCode::kESTOP));
  TEST_ASSERT_TRUE(!bench.processor.state().alarm_latched(protocol::AlarmCode::kHB_TIMEOUT));
  TEST_ASSERT_TRUE(!bench.processor.state().homed());
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
  expect(bench.send("RESET"), {"ok"}, "one RESET is enough");
  TEST_ASSERT_TRUE(!bench.processor.state().estop());
  TEST_ASSERT_TRUE(!bench.processor.state().any_alarm());
}

void test_disable_clears_the_homed_flag_and_enable_does_not_restore_it() {
  Bench bench;
  chessbot::DeviceState& state = chessbot::CommandProcessorTestPeer::state(bench.processor);
  state.set_homed_z(true);
  state.set_homed_joints(true);
  TEST_ASSERT_TRUE(bench.processor.state().homed());
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE");
  TEST_ASSERT_TRUE(!bench.processor.state().homed_z());
  TEST_ASSERT_TRUE(!bench.processor.state().homed_joints());
  expect(bench.send("ENABLE"), {"ok"}, "ENABLE");
  TEST_ASSERT_TRUE(!bench.processor.state().homed());  // only a new homing sets it again
}

void test_commands_other_than_disable_leave_the_homed_flag_alone() {
  Bench bench;
  chessbot::DeviceState& state = chessbot::CommandProcessorTestPeer::state(bench.processor);
  state.set_homed_z(true);
  state.set_homed_joints(true);
  bench.send("PING");
  bench.send("INFO");
  bench.send("ENABLE");  // the drivers are already enabled at power-up
  bench.send("FOO");
  TEST_ASSERT_TRUE(bench.processor.state().homed());
}

void test_disable_in_estop_keeps_homing_mandatory() {
  Bench bench;
  chessbot::DeviceState& state = chessbot::CommandProcessorTestPeer::state(bench.processor);
  state.set_homed_z(true);
  state.set_homed_joints(true);
  bench.send("ESTOP");
  bench.send("DISABLE");
  bench.send("RESET");
  TEST_ASSERT_TRUE(!bench.processor.state().homed());
  bench.send("ENABLE");
  TEST_ASSERT_TRUE(!bench.processor.state().homed());
}

void test_in_estop_ping_info_and_estop_still_work() {
  Bench bench;
  bench.send("ESTOP");
  expect(bench.send("PING"), {"ok"}, "PING in ESTOP");
  const std::string info = std::string("INFO FW=") + chessbot::config::kFirmwareVersion + " PROTO=" + std::to_string(protocol::kProtocolVersion);
  expect(bench.send("INFO"), {info.c_str(), "ok"}, "INFO in ESTOP");
  expect(bench.send("ESTOP"), {"!alarm:ESTOP"}, "ESTOP in ESTOP");
}

void test_in_estop_parse_errors_come_before_error_5() {
  // Same order as the PC simulator: unknown (1) and bad parameter (2) are decided before the ESTOP gate.
  Bench bench;
  bench.send("ESTOP");
  expect(bench.send("FOO"), {"error:1"}, "unknown in ESTOP");
  expect(bench.send("MOVE A=1"), {"error:2"}, "bad parameter in ESTOP");
}

void test_in_estop_zstop_and_pos_are_implemented_and_answered() {
  Bench bench;
  bench.send("ESTOP");
  expect(bench.send("ZSTOP"), {"ok"}, "ZSTOP in ESTOP");
  expect(bench.send("POS"), {"POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=TOP", "ok"}, "POS in ESTOP");
}

void test_reset_clears_estop_and_makes_homing_mandatory() {
  Bench bench;
  bench.send("ESTOP");
  expect(bench.send("RESET"), {"ok"}, "RESET");
  TEST_ASSERT_TRUE(!bench.processor.state().estop());
  TEST_ASSERT_TRUE(!bench.processor.state().any_alarm());
  TEST_ASSERT_TRUE(!bench.processor.state().homed());
  expect(bench.send("ENABLE"), {"ok"}, "ENABLE after RESET");
}

void test_reset_without_an_estop_is_ok() {
  Bench bench;
  expect(bench.send("RESET"), {"ok"}, "RESET");
}

void test_reset_empties_the_queue() {
  // The PC simulator's RESET also drops whatever motion was queued.
  Bench bench;
  chessbot::Command move(protocol::Cmd::kMOVE);
  TEST_ASSERT_TRUE(bench.processor.try_enqueue(move));
  expect(bench.send("RESET"), {"ok"}, "RESET");
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
}

void test_reset_does_not_touch_the_drivers() {
  Bench bench;
  bench.send("DISABLE");
  bench.send("ESTOP");
  bench.send("RESET");
  TEST_ASSERT_TRUE(!bench.processor.state().drivers_enabled());
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.driver_calls.size());  // only the DISABLE
}

// ---- parse errors --------------------------------------------------------------------------------

void test_unknown_command_is_error_1() {
  Bench bench;
  expect(bench.send("FOO"), {"error:1"}, "FOO");
  expect(bench.send("ping"), {"error:1"}, "ping");
}

void test_bad_parameters_are_error_2() {
  Bench bench;
  expect(bench.send("PING x"), {"error:2"}, "PING x");
  expect(bench.send("ENABLE now"), {"error:2"}, "ENABLE now");
  expect(bench.send("MOVE A=1 B=2"), {"error:2"}, "MOVE without F");
  expect(bench.send("GRIP S=181"), {"error:2"}, "GRIP out of range");
}

void test_blank_lines_get_no_reply() {
  Bench bench;
  TEST_ASSERT_TRUE(bench.send("").empty());
  TEST_ASSERT_TRUE(bench.send("   ").empty());
}

void test_a_bad_frame_is_error_1() {
  Bench bench;
  bench.sink.lines.clear();
  bench.processor.handle_bad_frame();
  expect(bench.sink.lines, {"error:1"}, "bad frame");
}

// ---- stubs ---------------------------------------------------------------------------------------

void test_every_not_yet_implemented_command_parses_and_answers_error_1() {
  Bench bench;
  // MOVE, ACCEL, WAIT and POS are real since S1 (test/test_motion), ZUP, ZDOWN and ZSTOP since S2 (test/test_zcommands).
  const char* stubs[] = {"HOME", "HOME Z", "HOME AB", "GRIP S=90"};
  for (const char* line : stubs) {
    expect(bench.send(line), {"error:1"}, line);
  }
  // A stub does nothing: no pin, no flag, no queue entry.
  TEST_ASSERT_TRUE(bench.hardware.driver_calls.empty());
  TEST_ASSERT_EQUAL_INT(0, bench.hardware.halts);
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
  TEST_ASSERT_TRUE(!bench.processor.state().homed());
  TEST_ASSERT_TRUE(!bench.processor.watchdog().armed());
}

void test_the_stubs_are_exactly_the_commands_that_are_not_yet_implemented() {
  // Guards the report: a command that is neither implemented nor answered error:1 would be a surprise.
  // MOVE is "implemented" for this test: with the drivers enabled and the arm not homed the gates answer error:4,
  // never the error:1 of a stub.
  const char* implemented[] = {"INFO", "PING", "ENABLE", "DISABLE", "ESTOP", "RESET", "MOVE", "ACCEL", "WAIT", "POS", "ZUP", "ZDOWN", "ZSTOP"};
  Bench bench;
  for (uint8_t c = 0; c < protocol::kCommandCount; ++c) {
    const std::string name = protocol::kCommands[c].name;
    bool is_implemented = false;
    for (const char* item : implemented) {
      is_implemented = is_implemented || name == item;
    }
    std::string line = name;
    for (uint8_t i = 0; i < protocol::kParamCount; ++i) {
      const protocol::ParamSpec& p = protocol::kParams[i];
      if (static_cast<uint8_t>(p.command) == c && p.required) {
        line += std::string(" ") + p.name + "=1";
      }
    }
    const std::vector<std::string> reply = bench.send(line.c_str());
    const bool is_error_1 = reply.size() == 1 && reply[0] == "error:1";
    if (is_implemented == is_error_1) {
      TEST_FAIL_MESSAGE(line.c_str());
    }
    bench.send("RESET");  // leave ESTOP, if this line caused one
  }
}


// ---- driver rules: error:6, the order of checks, DISABLE and the queue (SPEC 5.1, 5.2) -----------------------
// Everything below is typed out here and NOT taken from protocol.h or from the code under test.

namespace {

chessbot::DeviceState& peer_state(Bench& bench) { return chessbot::CommandProcessorTestPeer::state(bench.processor); }

void fill_the_queue(Bench& bench) {
  chessbot::Command move(protocol::Cmd::kMOVE);
  for (int i = 0; i < chessbot::config::kQueueDepth; ++i) {
    TEST_ASSERT_TRUE(bench.processor.try_enqueue(move));
  }
}

}  // namespace

void test_move_with_the_drivers_disabled_is_error_6_and_not_queued() {
  Bench bench;
  bench.send("DISABLE");
  expect(bench.send("MOVE A=1 B=2 F=3"), {"error:6"}, "MOVE after DISABLE");
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
}

void test_move_with_the_drivers_enabled_and_not_homed_is_error_4() {
  Bench bench;
  expect(bench.send("MOVE A=1 B=2 F=3"), {"error:4"}, "MOVE not homed");
}

void test_move_on_a_homed_arm_with_room_passes_the_gates_and_is_queued() {
  Bench bench;
  peer_state(bench).set_homed_z(true);
  peer_state(bench).set_homed_joints(true);
  expect(bench.send("MOVE A=1 B=2 F=3"), {"ok"}, "MOVE homed, room");
  TEST_ASSERT_EQUAL_INT(1, bench.processor.queue().size());
}

void test_move_with_a_full_queue_is_error_3_and_not_queued() {
  Bench bench;
  peer_state(bench).set_homed_z(true);
  peer_state(bench).set_homed_joints(true);
  fill_the_queue(bench);
  bench.sink.lines.clear();
  expect(bench.send("MOVE A=1 B=2 F=3"), {"error:3"}, "MOVE full queue");
  TEST_ASSERT_EQUAL_INT(chessbot::config::kQueueDepth, bench.processor.queue().size());
}

void test_a_home_that_moves_the_joints_is_error_6_while_the_drivers_are_disabled() {
  const char* lines[] = {"HOME", "HOME AB", "HOME Z AB", "HOME AB Z"};
  for (const char* line : lines) {
    Bench bench;
    bench.send("DISABLE");
    expect(bench.send(line), {"error:6"}, line);
  }
}

void test_home_z_is_not_gated_by_the_drivers_and_never_enables_them() {
  Bench bench;
  bench.send("DISABLE");
  expect(bench.send("HOME Z"), {"error:1"}, "HOME Z with the drivers disabled");  // the stub: it got past the gates
  TEST_ASSERT_TRUE(!bench.processor.state().drivers_enabled());
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.driver_calls.size());  // only the DISABLE touched the pins
}

void test_home_never_enables_the_drivers_even_when_it_is_refused() {
  Bench bench;
  bench.send("DISABLE");
  bench.send("HOME");
  bench.send("HOME AB");
  TEST_ASSERT_TRUE(!bench.processor.state().drivers_enabled());
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.driver_calls.size());
}

void test_the_order_of_checks_error_1_2_then_5_then_6_then_4_then_3() {
  // 1 and 2 beat the ESTOP check and the drivers check
  {
    Bench bench;
    bench.send("DISABLE");
    bench.send("ESTOP");
    expect(bench.send("FOO"), {"error:1"}, "unknown in ESTOP with the drivers disabled");
    expect(bench.send("MOVE A=1"), {"error:2"}, "bad parameter in ESTOP with the drivers disabled");
    expect(bench.send("HOME x"), {"error:2"}, "bad HOME parameter");
    expect(bench.send("MOVE A=1 B=2 F=3"), {"error:5"}, "5 beats 6 (MOVE)");
    expect(bench.send("HOME"), {"error:5"}, "5 beats 6 (HOME)");
  }
  // 2 beats 6
  {
    Bench bench;
    bench.send("DISABLE");
    expect(bench.send("MOVE A=1 B=2"), {"error:2"}, "bad parameter with the drivers disabled");
  }
  // 6 beats 4: disabled and not homed
  {
    Bench bench;
    bench.send("DISABLE");
    expect(bench.send("MOVE A=1 B=2 F=3"), {"error:6"}, "6 beats 4");
  }
  // 6 beats 4 and 3 even in the state a bug could leave behind: disabled, homed, queue full
  {
    Bench bench;
    fill_the_queue(bench);
    peer_state(bench).set_drivers_enabled(false);  // the flag only: DISABLE itself would clear the queue
    peer_state(bench).set_homed_z(true);
    peer_state(bench).set_homed_joints(true);
    bench.sink.lines.clear();
    expect(bench.send("MOVE A=1 B=2 F=3"), {"error:6"}, "6 beats 3");
  }
  // 4 beats 3: not homed and the queue is full
  {
    Bench bench;
    fill_the_queue(bench);
    bench.sink.lines.clear();
    expect(bench.send("MOVE A=1 B=2 F=3"), {"error:4"}, "4 beats 3");
  }
}

void test_every_combination_of_estop_drivers_homed_and_queue_matches_the_oracle() {
  const char* lines[] = {"MOVE A=1 B=2 F=3", "HOME", "HOME Z", "HOME AB", "HOME Z AB"};
  for (int bits = 0; bits < 16; ++bits) {
    const bool estop = (bits & 1) != 0;
    const bool drivers = (bits & 2) != 0;
    const bool homed = (bits & 4) != 0;
    const bool full = (bits & 8) != 0;
    for (const char* line : lines) {
      Bench bench;
      if (!drivers) peer_state(bench).set_drivers_enabled(false);
      if (estop) peer_state(bench).enter_estop();
      if (homed) {
        peer_state(bench).set_homed_z(true);
        peer_state(bench).set_homed_joints(true);
      }
      if (full) fill_the_queue(bench);
      const std::string text = line;
      const bool is_move = text == "MOVE A=1 B=2 F=3";
      const bool needs_drivers = is_move || text == "HOME" || text == "HOME AB" || text == "HOME Z AB";
      const char* want = is_move ? "ok" : "error:1";  // passed every gate: a MOVE is queued (S1), HOME is still a stub
      if (estop) {
        want = "error:5";
      } else if (needs_drivers && !drivers) {
        want = "error:6";
      } else if (is_move && !homed) {
        want = "error:4";
      } else if (is_move && full) {
        want = "error:3";
      }
      const std::vector<std::string> got = bench.send(line);
      if (!same(got, {want})) {
        const std::string message = text + " estop=" + std::to_string(estop) + " drivers=" + std::to_string(drivers) +
                                    " homed=" + std::to_string(homed) + " full=" + std::to_string(full) + " want " + want +
                                    " got " + (got.empty() ? "nothing" : got[0]);
        TEST_FAIL_MESSAGE(message.c_str());
      }
    }
  }
}

void test_disable_stops_the_joint_pulses_then_the_pins_and_leaves_z_alone() {
  Bench bench;
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE");
  TEST_ASSERT_EQUAL_INT(2, bench.hardware.events.size());
  TEST_ASSERT_EQUAL_STRING("pulses", bench.hardware.events[0].c_str());   // pulses first ...
  TEST_ASSERT_EQUAL_STRING("drivers=0", bench.hardware.events[1].c_str());  // ... then the ENA pins
  TEST_ASSERT_EQUAL_INT(0, bench.hardware.halts);  // halt_motion_outputs would stop Z too: DISABLE must not call it
}

void test_enable_does_not_stop_anything() {
  Bench bench;
  bench.send("ENABLE");
  TEST_ASSERT_EQUAL_INT(0, bench.hardware.joint_stops);
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.events.size());
}

void test_disable_clears_the_whole_queue() {
  Bench bench;
  fill_the_queue(bench);
  TEST_ASSERT_EQUAL_INT(chessbot::config::kQueueDepth, bench.processor.queue().size());
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE with a full queue");
  TEST_ASSERT_TRUE(bench.processor.queue().empty());  // discarded, not drained
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.joint_stops);
}

void test_disable_in_an_estop_also_stops_the_pulses_and_clears_the_queue() {
  Bench bench;
  bench.send("ESTOP");
  fill_the_queue(bench);  // a queue that survived (it cannot in practice): DISABLE must still empty it
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE in ESTOP");
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.joint_stops);
}

void test_every_way_of_clearing_the_queue_empties_it() {
  // DISABLE, ESTOP, RESET and the heartbeat timeout go through the same clear_motion_queue(): the rule for a pending
  // WAIT (cancelled, no done) is one rule, in one place.
  const char* lines[] = {"DISABLE", "ESTOP", "RESET"};
  for (const char* line : lines) {
    Bench bench;
    fill_the_queue(bench);
    bench.send(line);
    if (!bench.processor.queue().empty()) TEST_FAIL_MESSAGE(line);
  }
  Bench bench;
  bench.send("PING");
  fill_the_queue(bench);
  bench.processor.tick(chessbot::config::kHeartbeatTimeoutMs + 1);
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
}

void test_disable_sends_no_done_and_nothing_else() {
  // 4a has no WAIT or HOME to cancel; this pins that DISABLE answers exactly "ok" (no done, no alarm).
  Bench bench;
  fill_the_queue(bench);
  bench.sink.lines.clear();
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE");
  TEST_ASSERT_TRUE(bench.processor.state().alarm_latched(protocol::AlarmCode::kESTOP) == false);
}

// ---- heartbeat step ------------------------------------------------------------------------------

void test_no_heartbeat_alarm_before_the_first_ping() {
  Bench bench;
  bench.processor.tick(10u * chessbot::config::kHeartbeatTimeoutMs);
  TEST_ASSERT_TRUE(bench.sink.lines.empty());
}

void test_heartbeat_timeout_stops_motion_keeps_the_drivers_and_sends_the_alarm_once() {
  Bench bench;
  bench.now = 500;
  bench.send("PING");
  bench.sink.lines.clear();
  bench.processor.tick(bench.now + chessbot::config::kHeartbeatTimeoutMs);
  TEST_ASSERT_TRUE(bench.sink.lines.empty());  // not yet
  bench.processor.tick(bench.now + chessbot::config::kHeartbeatTimeoutMs + 1);
  expect(bench.sink.lines, {"!alarm:HB_TIMEOUT"}, "tick after the timeout");
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.halts);
  TEST_ASSERT_TRUE(bench.hardware.driver_calls.empty());
  TEST_ASSERT_TRUE(bench.processor.state().drivers_enabled());
  TEST_ASSERT_TRUE(!bench.processor.state().estop());
  TEST_ASSERT_TRUE(!bench.processor.state().homed());
  bench.sink.lines.clear();
  bench.processor.tick(bench.now + 10u * chessbot::config::kHeartbeatTimeoutMs);
  TEST_ASSERT_TRUE(bench.sink.lines.empty());  // once
}

void test_a_ping_after_a_heartbeat_timeout_rearms_the_watchdog() {
  Bench bench;
  bench.send("PING");
  bench.processor.tick(chessbot::config::kHeartbeatTimeoutMs + 1);
  bench.now = chessbot::config::kHeartbeatTimeoutMs + 100;
  expect(bench.send("PING"), {"ok"}, "PING after the timeout");
  TEST_ASSERT_TRUE(bench.processor.watchdog().armed());
}

void test_commands_still_work_after_a_heartbeat_timeout() {
  // HB_TIMEOUT is not an ESTOP: only homing is mandatory.
  Bench bench;
  bench.send("PING");
  bench.processor.tick(chessbot::config::kHeartbeatTimeoutMs + 1);
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE after HB_TIMEOUT");
}

// ---- queue full ----------------------------------------------------------------------------------

void test_a_full_queue_answers_error_3_and_does_not_queue() {
  Bench bench;
  chessbot::Command move(protocol::Cmd::kMOVE);
  for (int i = 0; i < chessbot::config::kQueueDepth; ++i) {
    TEST_ASSERT_TRUE(bench.processor.try_enqueue(move));
  }
  TEST_ASSERT_TRUE(bench.sink.lines.empty());  // success sends nothing: the caller sends ok
  TEST_ASSERT_TRUE(!bench.processor.try_enqueue(move));
  expect(bench.sink.lines, {"error:3"}, "queue full");
  TEST_ASSERT_EQUAL_INT(chessbot::config::kQueueDepth, bench.processor.queue().size());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_a_new_processor_touches_no_hardware_and_says_nothing);
  RUN_TEST(test_info);
  RUN_TEST(test_ping_answers_ok_and_arms_the_watchdog);
  RUN_TEST(test_disable_then_enable);
  RUN_TEST(test_estop_is_answered_by_the_alarm_alone);
  RUN_TEST(test_estop_stops_motion_sets_the_flag_and_keeps_the_drivers_enabled);
  RUN_TEST(test_estop_is_idempotent_and_answers_the_alarm_again);
  RUN_TEST(test_estop_empties_the_queue);
  RUN_TEST(test_in_estop_the_motion_commands_and_enable_are_error_5);
  RUN_TEST(test_the_two_estop_lists_cover_every_command_of_the_protocol_exactly_once);
  RUN_TEST(test_in_estop_every_command_outside_the_set_is_error_5);
  RUN_TEST(test_in_estop_every_command_in_the_set_is_accepted_and_none_is_error_5);
  RUN_TEST(test_disable_is_accepted_in_estop_and_reaches_the_pins);
  RUN_TEST(test_enable_is_refused_in_estop_and_accepted_again_after_reset);
  RUN_TEST(test_a_repeated_estop_is_accepted_and_harmless);
  RUN_TEST(test_disable_clears_the_homed_flag_and_enable_does_not_restore_it);
  RUN_TEST(test_commands_other_than_disable_leave_the_homed_flag_alone);
  RUN_TEST(test_disable_in_estop_keeps_homing_mandatory);
  RUN_TEST(test_in_estop_ping_info_and_estop_still_work);
  RUN_TEST(test_in_estop_parse_errors_come_before_error_5);
  RUN_TEST(test_in_estop_zstop_and_pos_are_implemented_and_answered);
  RUN_TEST(test_reset_clears_estop_and_makes_homing_mandatory);
  RUN_TEST(test_reset_without_an_estop_is_ok);
  RUN_TEST(test_reset_empties_the_queue);
  RUN_TEST(test_reset_does_not_touch_the_drivers);
  RUN_TEST(test_unknown_command_is_error_1);
  RUN_TEST(test_bad_parameters_are_error_2);
  RUN_TEST(test_blank_lines_get_no_reply);
  RUN_TEST(test_a_bad_frame_is_error_1);
  RUN_TEST(test_every_not_yet_implemented_command_parses_and_answers_error_1);
  RUN_TEST(test_the_stubs_are_exactly_the_commands_that_are_not_yet_implemented);
  RUN_TEST(test_no_heartbeat_alarm_before_the_first_ping);
  RUN_TEST(test_heartbeat_timeout_stops_motion_keeps_the_drivers_and_sends_the_alarm_once);
  RUN_TEST(test_a_ping_after_a_heartbeat_timeout_rearms_the_watchdog);
  RUN_TEST(test_commands_still_work_after_a_heartbeat_timeout);
  RUN_TEST(test_a_full_queue_answers_error_3_and_does_not_queue);
  RUN_TEST(test_move_with_the_drivers_disabled_is_error_6_and_not_queued);
  RUN_TEST(test_move_with_the_drivers_enabled_and_not_homed_is_error_4);
  RUN_TEST(test_move_on_a_homed_arm_with_room_passes_the_gates_and_is_queued);
  RUN_TEST(test_move_with_a_full_queue_is_error_3_and_not_queued);
  RUN_TEST(test_a_home_that_moves_the_joints_is_error_6_while_the_drivers_are_disabled);
  RUN_TEST(test_home_z_is_not_gated_by_the_drivers_and_never_enables_them);
  RUN_TEST(test_home_never_enables_the_drivers_even_when_it_is_refused);
  RUN_TEST(test_the_order_of_checks_error_1_2_then_5_then_6_then_4_then_3);
  RUN_TEST(test_every_combination_of_estop_drivers_homed_and_queue_matches_the_oracle);
  RUN_TEST(test_disable_stops_the_joint_pulses_then_the_pins_and_leaves_z_alone);
  RUN_TEST(test_enable_does_not_stop_anything);
  RUN_TEST(test_disable_clears_the_whole_queue);
  RUN_TEST(test_disable_in_an_estop_also_stops_the_pulses_and_clears_the_queue);
  RUN_TEST(test_every_way_of_clearing_the_queue_empties_it);
  RUN_TEST(test_disable_sends_no_done_and_nothing_else);
  return UNITY_END();
}
