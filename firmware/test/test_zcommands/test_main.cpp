// Stage 4b-1, S2: ZUP, ZDOWN and ZSTOP through the command processor, the Z alarms, and what ESTOP, DISABLE, RESET and
// the heartbeat do to Z (docs/SPEC.md 5.1, 5.2, 10.2). The limit switches are fake pins the test sets; time is a counter.
#include <string>
#include <unity.h>
#include <vector>

#include "command_processor.h"
#include "config.h"
#include "motion.h"
#include "protocol.h"
#include "z_axis.h"

using chessbot::CommandProcessor;
using chessbot::MotionConfig;
using chessbot::ZConfig;

namespace chessbot {
struct CommandProcessorTestPeer {
  static void set_homed(CommandProcessor& processor) {
    processor.state_.set_homed_z(true);
    processor.state_.set_homed_joints(true);
  }
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
  bool top = false, bottom = false;  // the RAW switches
  std::vector<std::string> events;   // "drivers=0", "halt", "pulses", "raise:40", "brake:100", ...
  bool running = false;
  void set_drivers_enabled(bool enabled) override { events.push_back(enabled ? "drivers=1" : "drivers=0"); }
  void halt_motion_outputs() override {
    events.push_back("halt");
    running = false;
  }
  void stop_joint_pulses() override {
    events.push_back("pulses");
    running = false;
  }
  bool start_joint_move(const chessbot::JointMove&) override {
    running = true;
    return true;
  }
  bool joints_running() override { return running; }
  int32_t joint_steps_issued(uint8_t) override { return 0; }
  void drive_z(chessbot::ZDrive drive, uint8_t duty_percent) override {
    static const char* const names[] = {"coast", "raise", "lower", "brake"};
    events.push_back(std::string(names[static_cast<int>(drive)]) + ":" + std::to_string(duty_percent));
  }
  bool limit_actuated(chessbot::ZLimit limit) override { return limit == chessbot::ZLimit::kTop ? top : bottom; }

  int count(const std::string& event) const {
    int n = 0;
    for (const std::string& e : events) n += e == event;
    return n;
  }
  std::string last_z() const {
    for (size_t i = events.size(); i-- > 0;) {
      const std::string& e = events[i];
      if (e.compare(0, 6, "raise:") == 0 || e.compare(0, 6, "lower:") == 0 || e.compare(0, 6, "brake:") == 0 || e.compare(0, 6, "coast:") == 0) return e;
    }
    return "";
  }
};

// 5000 ms full travel, timeout 7500 ms, 8 ms debounce, duty 40 -> 70 over 500 ms, 40 after 4000 ms.
ZConfig z_config(uint32_t full_travel_ms = 5000) {
  ZConfig config;
  config.full_travel_ms = full_travel_ms;
  config.timeout_factor = 1.5f;
  config.debounce_ms = 8;
  config.duty_start_percent = 40;
  config.duty_peak_percent = 70;
  config.duty_ramp_ms = 500;
  config.duty_eased_percent = 40;
  config.ease_after_travel_percent = 80;
  return config;
}

MotionConfig motion_config(bool bench = false) {
  MotionConfig config = MotionConfig::from_project();
  config.limits_configured = false;
  config.bench_move_without_homing = bench;
  return config;
}

struct Bench {
  FakeSink sink;
  FakeHardware hardware;
  CommandProcessor processor;
  uint32_t now = 100;

  explicit Bench(bool settle_at_top = true, bool bench_flag = false, const ZConfig& z = z_config())
      : processor(&sink, &hardware, motion_config(bench_flag), z) {
    if (settle_at_top) {
      hardware.top = true;
      run(20);
      sink.lines.clear();
    }
  }

  std::vector<std::string> send(const char* line) {
    sink.lines.clear();
    processor.handle_line(line, now);
    return sink.lines;
  }
  // The main loop for ``ms`` milliseconds; returns what it sent.
  std::vector<std::string> run(uint32_t ms) {
    sink.lines.clear();
    std::vector<std::string> sent;
    for (uint32_t t = 0; t < ms; ++t) {
      ++now;
      processor.poll_motion();
      processor.poll_z(now);
      sent.insert(sent.end(), sink.lines.begin(), sink.lines.end());
      sink.lines.clear();
    }
    return sent;
  }
  // The carriage leaves the top switch, travels, and presses the bottom one (or the other way round).
  void travel_to_bottom(uint32_t travel_ms = 2000) {
    hardware.top = false;
    run(travel_ms);
    hardware.bottom = true;
  }
  void travel_to_top(uint32_t travel_ms = 2000) {
    hardware.bottom = false;
    run(travel_ms);
    hardware.top = true;
  }
};

std::string joined(const std::vector<std::string>& lines) {
  std::string text;
  for (const std::string& line : lines) text += line + "|";
  return text;
}

void expect(const std::vector<std::string>& got, std::initializer_list<const char*> want, const char* what) {
  std::string wanted;
  for (const char* line : want) wanted += std::string(line) + "|";
  if (joined(got) != wanted) {
    const std::string message = std::string(what) + ": got [" + joined(got) + "] want [" + wanted + "]";
    TEST_FAIL_MESSAGE(message.c_str());
  }
}

// ---- boot -----------------------------------------------------------------------------------------------------------

void test_z_is_mid_at_boot_until_a_limit_switch_says_otherwise() {
  Bench bench(false);
  expect(bench.send("POS"), {"POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=MID", "ok"}, "boot");
  bench.hardware.top = true;  // the switch is pressed, but not for the whole debounce window yet
  bench.run(7);
  expect(bench.send("POS"), {"POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=MID", "ok"}, "7 ms");
  bench.run(2);
  expect(bench.send("POS"), {"POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=TOP", "ok"}, "debounced");
}

void test_nothing_is_driven_at_boot_or_by_polling_an_idle_axis() {
  Bench bench(false);
  bench.run(1000);
  TEST_ASSERT_TRUE(bench.hardware.events.empty());  // the pins keep their boot levels: no drive_z call
}

// ---- ZDOWN / ZUP ----------------------------------------------------------------------------------------------------

void test_zdown_is_ok_then_done_when_the_bottom_switch_is_reached() {
  Bench bench;
  expect(bench.send("ZDOWN"), {"ok"}, "ZDOWN");
  TEST_ASSERT_EQUAL_STRING("lower:40", bench.hardware.last_z().c_str());  // starts at the start duty
  bench.hardware.top = false;
  TEST_ASSERT_TRUE(bench.run(2000).empty());  // no done while travelling
  TEST_ASSERT_EQUAL_STRING("lower:70", bench.hardware.last_z().c_str());  // ramped
  bench.hardware.bottom = true;
  expect(bench.run(20), {"done"}, "arrived");
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());  // electric brake: ENA full, IN1 == IN2
  expect(bench.send("POS"), {"POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=BOTTOM", "ok"}, "POS");
  TEST_ASSERT_TRUE(bench.run(10000).empty());  // done exactly once, no alarm later
}

void test_zup_is_ok_then_done_when_the_top_switch_is_reached() {
  Bench bench;
  bench.send("ZDOWN");
  bench.travel_to_bottom();
  bench.run(20);
  bench.sink.lines.clear();
  expect(bench.send("ZUP"), {"ok"}, "ZUP");
  TEST_ASSERT_EQUAL_STRING("raise:40", bench.hardware.last_z().c_str());
  bench.travel_to_top();
  expect(bench.run(20), {"done"}, "arrived");
  expect(bench.send("POS"), {"POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=TOP", "ok"}, "POS");
}

void test_the_duty_eases_near_the_end_of_the_travel() {
  Bench bench;
  bench.send("ZDOWN");
  bench.hardware.top = false;
  bench.run(4100);
  TEST_ASSERT_EQUAL_STRING("lower:40", bench.hardware.last_z().c_str());
}

void test_zup_at_top_and_zdown_at_bottom_are_ok_and_done_at_once_and_drive_nothing() {
  Bench bench;
  expect(bench.send("ZUP"), {"ok", "done"}, "ZUP at TOP");
  TEST_ASSERT_TRUE(bench.hardware.events.empty());
  bench.send("ZDOWN");
  bench.travel_to_bottom();
  bench.run(20);
  const size_t before = bench.hardware.events.size();
  expect(bench.send("ZDOWN"), {"ok", "done"}, "ZDOWN at BOTTOM");
  TEST_ASSERT_EQUAL_UINT32(before, bench.hardware.events.size());
}

void test_z_needs_no_joint_driver() {
  Bench bench;
  bench.send("DISABLE");
  expect(bench.send("ZDOWN"), {"ok"}, "ZDOWN with the drivers disabled");
  TEST_ASSERT_EQUAL_STRING("lower:40", bench.hardware.last_z().c_str());
}

void test_a_z_move_is_not_refused_when_the_arm_is_not_homed() {
  Bench bench;
  expect(bench.send("ZDOWN"), {"ok"}, "ZDOWN");  // Z moves are what homing itself is made of
}

void test_a_new_z_move_replaces_the_running_one() {
  Bench bench;
  bench.send("ZDOWN");
  bench.hardware.top = false;
  bench.run(1000);
  expect(bench.send("ZUP"), {"ok"}, "ZUP replaces ZDOWN");
  TEST_ASSERT_EQUAL_STRING("raise:40", bench.hardware.last_z().c_str());  // the ramp starts again
  bench.hardware.top = true;
  expect(bench.run(20), {"done"}, "only the ZUP is reported");
}

void test_zdown_replaced_by_zup_while_still_on_the_top_switch_just_brakes() {
  Bench bench;
  bench.send("ZDOWN");
  bench.run(5);  // the carriage has not left the top switch yet
  expect(bench.send("ZUP"), {"ok", "done"}, "ZUP");
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
}

void test_an_unmeasured_travel_time_refuses_z_moves_with_error_2() {
  Bench bench(true, false, z_config(0));
  expect(bench.send("ZUP"), {"error:2"}, "ZUP");
  expect(bench.send("ZDOWN"), {"error:2"}, "ZDOWN");
  TEST_ASSERT_TRUE(bench.hardware.events.empty());
}

// ---- ZSTOP ----------------------------------------------------------------------------------------------------------

void test_zstop_brakes_answers_ok_and_cancels_the_done() {
  Bench bench;
  bench.send("ZDOWN");
  bench.hardware.top = false;
  bench.run(1000);
  expect(bench.send("ZSTOP"), {"ok"}, "ZSTOP");
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
  bench.hardware.bottom = true;  // it coasted on to the switch
  TEST_ASSERT_TRUE(bench.run(10000).empty());  // no done, no timeout
}

void test_zstop_when_nothing_moves_still_brakes() {
  Bench bench;
  expect(bench.send("ZSTOP"), {"ok"}, "ZSTOP");
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
}

// ---- timeout --------------------------------------------------------------------------------------------------------

void test_a_z_move_that_never_arrives_raises_z_timeout_once_and_brakes() {
  Bench bench;
  bench.send("ZDOWN");
  bench.hardware.top = false;
  TEST_ASSERT_TRUE(bench.run(7499).empty());
  expect(bench.run(1), {"!alarm:Z_TIMEOUT"}, "timeout");
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
  TEST_ASSERT_TRUE(bench.processor.state().alarm_latched(chessbot::protocol::AlarmCode::kZ_TIMEOUT));
  TEST_ASSERT_TRUE(bench.run(20000).empty());
}

void test_z_timeout_makes_homing_mandatory_but_is_not_an_estop() {
  Bench bench;
  chessbot::CommandProcessorTestPeer::set_homed(bench.processor);
  bench.send("ZDOWN");
  bench.hardware.top = false;
  bench.run(7500);
  TEST_ASSERT_FALSE(bench.processor.state().homed());
  TEST_ASSERT_FALSE(bench.processor.state().estop());
  expect(bench.send("PING"), {"ok"}, "still answering");
}

// ---- LIMIT_Z --------------------------------------------------------------------------------------------------------

void test_both_switches_actuated_is_limit_z_and_an_emergency_stop() {
  Bench bench;
  chessbot::CommandProcessorTestPeer::set_homed(bench.processor);
  bench.send("MOVE A=90 B=45 F=20");  // a running MOVE and a pending WAIT
  bench.send("WAIT");
  bench.hardware.events.clear();
  bench.hardware.bottom = true;  // the top switch is still pressed
  expect(bench.run(20), {"!alarm:LIMIT_Z"}, "LIMIT_Z, and nothing else");
  TEST_ASSERT_TRUE(bench.processor.state().estop());
  TEST_ASSERT_TRUE(bench.processor.state().alarm_latched(chessbot::protocol::AlarmCode::kLIMIT_Z));
  TEST_ASSERT_FALSE(bench.processor.state().homed());
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.count("halt"));  // the joints are stopped at once
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
  TEST_ASSERT_EQUAL_INT(0, bench.hardware.count("drivers=0"));  // the drivers stay enabled
  bench.hardware.running = false;
  TEST_ASSERT_TRUE(bench.run(100).empty());  // the WAIT's done was cancelled with the queue
  expect(bench.send("MOVE A=90 B=45 F=20"), {"error:5"}, "ESTOP flag is set");
  expect(bench.send("RESET"), {"ok"}, "RESET");
  TEST_ASSERT_FALSE(bench.processor.state().estop());
}

void test_the_top_switch_releasing_without_a_command_is_limit_z() {
  Bench bench;
  bench.hardware.top = false;
  expect(bench.run(20), {"!alarm:LIMIT_Z"}, "released at TOP");
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
}

void test_the_top_switch_releasing_for_a_zdown_is_no_alarm() {
  Bench bench;
  bench.send("ZDOWN");
  bench.hardware.top = false;
  TEST_ASSERT_TRUE(bench.run(100).empty());
}

void test_a_broken_wire_reads_as_an_actuated_switch_and_stops_a_move_at_once() {
  Bench bench;
  bench.send("ZDOWN");
  bench.hardware.top = false;
  bench.run(500);
  bench.hardware.bottom = true;  // the bottom wire breaks: an NC contact reads like a pressed switch
  bench.run(1);
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
}

// ---- ESTOP, DISABLE, RESET, heartbeat -------------------------------------------------------------------------------

void test_estop_brakes_z_and_cancels_the_done() {
  Bench bench;
  bench.send("ZDOWN");
  bench.hardware.top = false;
  bench.run(500);
  expect(bench.send("ESTOP"), {"!alarm:ESTOP"}, "ESTOP");
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
  bench.hardware.bottom = true;
  TEST_ASSERT_TRUE(bench.run(10000).empty());
}

void test_z_moves_are_error_5_in_an_estop_but_zstop_is_accepted() {
  Bench bench;
  bench.send("ESTOP");
  const size_t events = bench.hardware.events.size();
  expect(bench.send("ZUP"), {"error:5"}, "ZUP");
  expect(bench.send("ZDOWN"), {"error:5"}, "ZDOWN");
  TEST_ASSERT_EQUAL_UINT32(events, bench.hardware.events.size());  // a refusal drives nothing
  expect(bench.send("ZSTOP"), {"ok"}, "ZSTOP");
}

void test_disable_does_not_stop_a_running_z_move() {
  Bench bench;
  bench.send("ZDOWN");
  bench.hardware.top = false;
  bench.run(500);
  const int brakes = bench.hardware.count("brake:100");
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE");
  TEST_ASSERT_EQUAL_INT(brakes, bench.hardware.count("brake:100"));
  TEST_ASSERT_EQUAL_STRING("lower:70", bench.hardware.last_z().c_str());  // still running
  bench.hardware.bottom = true;
  expect(bench.run(20), {"done"}, "DISABLE never cancels the done of Z");
}

void test_reset_brakes_z() {
  Bench bench;
  bench.send("ZDOWN");
  bench.hardware.top = false;
  bench.run(500);
  bench.send("RESET");
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
  bench.hardware.bottom = true;
  TEST_ASSERT_TRUE(bench.run(1000).empty());
}

void test_a_heartbeat_timeout_brakes_z() {
  Bench bench;
  bench.send("PING");
  bench.send("ZDOWN");
  bench.hardware.top = false;
  bench.run(500);
  bench.processor.tick(bench.now + 5000);
  TEST_ASSERT_EQUAL_STRING("brake:100", bench.hardware.last_z().c_str());
}

// ---- MOVE needs Z at TOP --------------------------------------------------------------------------------------------

void test_move_is_error_4_unless_z_is_at_top_even_when_homed() {
  Bench bench;
  chessbot::CommandProcessorTestPeer::set_homed(bench.processor);
  expect(bench.send("MOVE A=90 B=45 F=20"), {"ok"}, "Z at TOP");
  bench.send("ESTOP");
  bench.send("RESET");
  chessbot::CommandProcessorTestPeer::set_homed(bench.processor);
  bench.send("ZDOWN");
  bench.travel_to_bottom();
  bench.run(20);
  expect(bench.send("MOVE A=90 B=45 F=20"), {"error:4"}, "Z at BOTTOM");
}

void test_move_is_error_4_while_z_is_in_the_middle_and_at_boot() {
  Bench boot(false);
  chessbot::CommandProcessorTestPeer::set_homed(boot.processor);
  expect(boot.send("MOVE A=90 B=45 F=20"), {"error:4"}, "Z unknown at boot");
  Bench mid;
  chessbot::CommandProcessorTestPeer::set_homed(mid.processor);
  mid.send("ZDOWN");
  mid.hardware.top = false;
  mid.run(500);
  expect(mid.send("MOVE A=90 B=45 F=20"), {"error:4"}, "Z in the middle");
}

void test_the_bench_setting_does_not_skip_the_z_check() {
  Bench bench(true, true);
  expect(bench.send("MOVE A=90 B=45 F=20"), {"ok"}, "bench, Z at TOP");
  Bench boot(false, true);
  expect(boot.send("MOVE A=90 B=45 F=20"), {"error:4"}, "bench, Z unknown");
}

void test_error_6_still_comes_before_error_4_for_z() {
  Bench bench;
  bench.send("ZDOWN");
  bench.travel_to_bottom();
  bench.run(20);
  bench.send("DISABLE");
  expect(bench.send("MOVE A=90 B=45 F=20"), {"error:6"}, "drivers disabled first");
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_z_is_mid_at_boot_until_a_limit_switch_says_otherwise);
  RUN_TEST(test_nothing_is_driven_at_boot_or_by_polling_an_idle_axis);
  RUN_TEST(test_zdown_is_ok_then_done_when_the_bottom_switch_is_reached);
  RUN_TEST(test_zup_is_ok_then_done_when_the_top_switch_is_reached);
  RUN_TEST(test_the_duty_eases_near_the_end_of_the_travel);
  RUN_TEST(test_zup_at_top_and_zdown_at_bottom_are_ok_and_done_at_once_and_drive_nothing);
  RUN_TEST(test_z_needs_no_joint_driver);
  RUN_TEST(test_a_z_move_is_not_refused_when_the_arm_is_not_homed);
  RUN_TEST(test_a_new_z_move_replaces_the_running_one);
  RUN_TEST(test_zdown_replaced_by_zup_while_still_on_the_top_switch_just_brakes);
  RUN_TEST(test_an_unmeasured_travel_time_refuses_z_moves_with_error_2);
  RUN_TEST(test_zstop_brakes_answers_ok_and_cancels_the_done);
  RUN_TEST(test_zstop_when_nothing_moves_still_brakes);
  RUN_TEST(test_a_z_move_that_never_arrives_raises_z_timeout_once_and_brakes);
  RUN_TEST(test_z_timeout_makes_homing_mandatory_but_is_not_an_estop);
  RUN_TEST(test_both_switches_actuated_is_limit_z_and_an_emergency_stop);
  RUN_TEST(test_the_top_switch_releasing_without_a_command_is_limit_z);
  RUN_TEST(test_the_top_switch_releasing_for_a_zdown_is_no_alarm);
  RUN_TEST(test_a_broken_wire_reads_as_an_actuated_switch_and_stops_a_move_at_once);
  RUN_TEST(test_estop_brakes_z_and_cancels_the_done);
  RUN_TEST(test_z_moves_are_error_5_in_an_estop_but_zstop_is_accepted);
  RUN_TEST(test_disable_does_not_stop_a_running_z_move);
  RUN_TEST(test_reset_brakes_z);
  RUN_TEST(test_a_heartbeat_timeout_brakes_z);
  RUN_TEST(test_move_is_error_4_unless_z_is_at_top_even_when_homed);
  RUN_TEST(test_move_is_error_4_while_z_is_in_the_middle_and_at_boot);
  RUN_TEST(test_the_bench_setting_does_not_skip_the_z_check);
  RUN_TEST(test_error_6_still_comes_before_error_4_for_z);
  return UNITY_END();
}
