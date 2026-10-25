// CommandProcessor with a fake sink and fake hardware: every reply of stage 4a, SPEC 10.2 boot / ESTOP
// behaviour, the stubbed commands, the heartbeat step and the queue-full error.
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

struct FakeHardware : chessbot::HardwareOps {
  std::vector<bool> driver_calls;  // every set_drivers_enabled(...) argument, in order
  int halts = 0;
  void set_drivers_enabled(bool enabled) override { driver_calls.push_back(enabled); }
  void halt_motion_outputs() override { ++halts; }
};

struct Bench {
  FakeSink sink;
  FakeHardware hardware;
  CommandProcessor processor;
  uint32_t now = 0;
  Bench() : processor(&sink, &hardware) {}

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
    } else {
      // ZSTOP and POS are in the set but are stubs in stage 4a: they get past the ESTOP check and answer error:1.
      expect(reply, {"error:1"}, item.line);
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

void test_in_estop_the_stubbed_safe_commands_are_still_error_1() {
  // ZSTOP and POS are allowed in an ESTOP by the PC simulator but are not implemented until a later stage.
  Bench bench;
  bench.send("ESTOP");
  expect(bench.send("ZSTOP"), {"error:1"}, "ZSTOP in ESTOP");
  expect(bench.send("POS"), {"error:1"}, "POS in ESTOP");
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
  const char* stubs[] = {"HOME", "HOME Z", "HOME AB", "MOVE A=1 B=2 F=3", "ZUP", "ZDOWN", "ZSTOP", "ACCEL S=1", "GRIP S=90", "WAIT", "POS"};
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

void test_the_stubs_are_exactly_the_commands_that_are_not_stage_4a() {
  // Guards the report: a command that is neither stage 4a nor answered error:1 would be a surprise.
  const char* implemented[] = {"INFO", "PING", "ENABLE", "DISABLE", "ESTOP", "RESET"};
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
  RUN_TEST(test_in_estop_the_stubbed_safe_commands_are_still_error_1);
  RUN_TEST(test_reset_clears_estop_and_makes_homing_mandatory);
  RUN_TEST(test_reset_without_an_estop_is_ok);
  RUN_TEST(test_reset_empties_the_queue);
  RUN_TEST(test_reset_does_not_touch_the_drivers);
  RUN_TEST(test_unknown_command_is_error_1);
  RUN_TEST(test_bad_parameters_are_error_2);
  RUN_TEST(test_blank_lines_get_no_reply);
  RUN_TEST(test_a_bad_frame_is_error_1);
  RUN_TEST(test_every_not_yet_implemented_command_parses_and_answers_error_1);
  RUN_TEST(test_the_stubs_are_exactly_the_commands_that_are_not_stage_4a);
  RUN_TEST(test_no_heartbeat_alarm_before_the_first_ping);
  RUN_TEST(test_heartbeat_timeout_stops_motion_keeps_the_drivers_and_sends_the_alarm_once);
  RUN_TEST(test_a_ping_after_a_heartbeat_timeout_rearms_the_watchdog);
  RUN_TEST(test_commands_still_work_after_a_heartbeat_timeout);
  RUN_TEST(test_a_full_queue_answers_error_3_and_does_not_queue);
  return UNITY_END();
}
