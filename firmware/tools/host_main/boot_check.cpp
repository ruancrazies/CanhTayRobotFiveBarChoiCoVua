// Host check of src/main.cpp (stages 4a and 4b-1 S1) against a recording Arduino stub and a recording model of
// FastAccelStepper (tools/host_main/FastAccelStepper.h):
//  * SPEC 10.2 boot: the first thing setup() does is drive the two TB6600 ENA pins HIGH and the three Z
//    motor pins LOW, before anything else; nothing is sent at boot; the joint steppers are connected right after,
//    with the DIR polarity and the direction-change delay of settings.yaml, auto-enable OFF, and the library is never
//    given an ENA pin;
//  * a command session: the exact bytes sent back and the ENA pin writes of ENABLE / DISABLE / ESTOP;
//  * the glue: a MOVE through the real core reaches the library as speed, acceleration and an absolute target per
//    joint, and DISABLE / ESTOP / RESET stop both joints without touching an ENA pin.
// Built by tools/run_host_tests.sh together with the real src/main.cpp and lib/chessbot_core.
#include <unity.h>

#include <string>
#include <vector>

#include "Arduino.h"
#include "../../src/main.cpp"

void setUp(void) {}
void tearDown(void) {}

namespace {

using chessbot::pins::kL298Ena;
using chessbot::pins::kL298In1;
using chessbot::pins::kL298In2;
using chessbot::pins::kTb6600Ena1;
using chessbot::pins::kTb6600Ena2;

std::string w(uint8_t pin, int level) { return "write " + std::to_string(pin) + " " + std::to_string(level); }
std::string m(uint8_t pin) { return "mode " + std::to_string(pin) + " output"; }

size_t index_of(const std::vector<std::string>& log, const std::string& event) {
  for (size_t i = 0; i < log.size(); ++i) {
    if (log[i] == event) return i;
  }
  return log.size();
}

// setup() runs once for the whole file: it is the one boot.
std::vector<std::string> g_boot_log;
bool g_booted = false;
void boot_once() {
  if (!g_booted) {
    setup();
    g_booted = true;
    g_boot_log = host_arduino::log();
  }
}

}  // namespace

void test_the_first_calls_of_setup_are_the_pin_levels_and_serial_comes_after() {
  boot_once();
  // Five pins (two TB6600 ENA, three L298), each "level then output": ten events before any Serial or library call.
  const size_t pin_events = 5 * 2;
  TEST_ASSERT_TRUE(g_boot_log.size() >= pin_events);
  for (size_t i = 0; i < pin_events; ++i) {
    TEST_ASSERT_TRUE(g_boot_log[i].rfind("serial", 0) != 0);
    TEST_ASSERT_TRUE(g_boot_log[i].rfind("fas", 0) != 0);
  }
  const size_t begin = index_of(g_boot_log, "serial.begin " + std::to_string(chessbot::config::kBaudRate));
  TEST_ASSERT_TRUE(begin < g_boot_log.size());
  TEST_ASSERT_TRUE(begin >= pin_events);
}

void test_tb6600_ena_pins_are_driven_high_and_z_motor_pins_low() {
  boot_once();
  const int enabled = chessbot::config::kTb6600EnaLevelEnabled ? HIGH : LOW;
  const int z_idle = chessbot::config::kZMotorPinsBootLevel ? HIGH : LOW;
  TEST_ASSERT_EQUAL_INT(HIGH, enabled);  // SPEC 4.3: HIGH = driver enabled
  TEST_ASSERT_EQUAL_INT(LOW, z_idle);
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena1, HIGH).c_str(), g_boot_log[0].c_str());
  TEST_ASSERT_EQUAL_STRING(m(kTb6600Ena1).c_str(), g_boot_log[1].c_str());
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena2, HIGH).c_str(), g_boot_log[2].c_str());
  TEST_ASSERT_EQUAL_STRING(m(kTb6600Ena2).c_str(), g_boot_log[3].c_str());
  TEST_ASSERT_EQUAL_STRING(w(kL298Ena, LOW).c_str(), g_boot_log[4].c_str());
  TEST_ASSERT_EQUAL_STRING(m(kL298Ena).c_str(), g_boot_log[5].c_str());
  TEST_ASSERT_EQUAL_STRING(w(kL298In1, LOW).c_str(), g_boot_log[6].c_str());
  TEST_ASSERT_EQUAL_STRING(m(kL298In1).c_str(), g_boot_log[7].c_str());
  TEST_ASSERT_EQUAL_STRING(w(kL298In2, LOW).c_str(), g_boot_log[8].c_str());
  TEST_ASSERT_EQUAL_STRING(m(kL298In2).c_str(), g_boot_log[9].c_str());
}

void test_each_pin_level_is_written_before_the_pin_becomes_an_output() {
  boot_once();
  const uint8_t pins[] = {kTb6600Ena1, kTb6600Ena2, kL298Ena, kL298In1, kL298In2};
  for (uint8_t pin : pins) {
    const size_t mode = index_of(g_boot_log, m(pin));
    TEST_ASSERT_TRUE(mode > 0 && mode < g_boot_log.size());
    TEST_ASSERT_TRUE(g_boot_log[mode - 1].rfind("write " + std::to_string(pin) + " ", 0) == 0);
  }
}

void test_serial_starts_at_the_configured_baud_with_the_configured_rx_buffer() {
  boot_once();
  const size_t rx = index_of(g_boot_log, "serial.rxbuf " + std::to_string(chessbot::config::kSerialRxBufferBytes));
  const size_t begin = index_of(g_boot_log, "serial.begin " + std::to_string(chessbot::config::kBaudRate));
  TEST_ASSERT_TRUE(rx < g_boot_log.size());
  TEST_ASSERT_TRUE(rx < begin);  // the buffer size must be set before begin()
}

void test_the_joint_steppers_are_connected_right_after_the_pin_levels_and_never_get_an_ena_pin() {
  boot_once();
  const std::string dir_up1 = chessbot::config::kJoint1DirHighCountsUp ? " high-counts-up" : " low-counts-up";
  const std::string dir_up2 = chessbot::config::kJoint2DirHighCountsUp ? " high-counts-up" : " low-counts-up";
  const std::string delay = " delay " + std::to_string(chessbot::config::kDirectionChangeDelayUs);
  const std::vector<std::string> expected = {
      "fas.init",
      "fas0.connect " + std::to_string(chessbot::pins::kTb6600Pul1),
      "fas0.dirpin " + std::to_string(chessbot::pins::kTb6600Dir1) + dir_up1 + delay,
      "fas0.autoenable 0",
      "fas1.connect " + std::to_string(chessbot::pins::kTb6600Pul2),
      "fas1.dirpin " + std::to_string(chessbot::pins::kTb6600Dir2) + dir_up2 + delay,
      "fas1.autoenable 0",
  };
  std::vector<std::string> library_calls;
  for (const std::string& event : g_boot_log) {
    if (event.rfind("fas", 0) == 0) library_calls.push_back(event);
  }
  TEST_ASSERT_EQUAL_INT(expected.size(), library_calls.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    TEST_ASSERT_EQUAL_STRING(expected[i].c_str(), library_calls[i].c_str());
  }
  TEST_ASSERT_TRUE(index_of(g_boot_log, "fas.init") >= 10);  // after the five pins (two events each)
  TEST_ASSERT_TRUE(index_of(g_boot_log, "fas.init") < index_of(g_boot_log, "serial.begin " + std::to_string(chessbot::config::kBaudRate)));
}

void test_nothing_is_sent_at_boot_and_only_the_five_pins_and_the_library_are_touched() {
  boot_once();
  TEST_ASSERT_TRUE(Serial.tx.empty());
  for (const std::string& event : g_boot_log) {
    if (event.rfind("serial", 0) == 0 || event.rfind("fas", 0) == 0) continue;
    bool known = false;
    for (uint8_t pin : {kTb6600Ena1, kTb6600Ena2, kL298Ena, kL298In1, kL298In2}) {
      known = known || event.find(" " + std::to_string(pin) + " ") != std::string::npos || event == m(pin);
    }
    if (!known) TEST_FAIL_MESSAGE(event.c_str());
  }
}

void test_a_session_gets_the_exact_replies() {
  boot_once();
  host_arduino::log().clear();
  Serial.tx.clear();
  Serial.type("INFO\nPING\r\nENABLE\nDISABLE\nENABLE\nESTOP\nMOVE A=100 B=50 F=10\nENABLE\nDISABLE\nESTOP\nPING\nRESET\nENABLE\nFOO\nPING x\nPOS\n\n   \n");
  loop();
  const std::string expected =
      std::string("INFO FW=") + chessbot::config::kFirmwareVersion + " PROTO=" + std::to_string(chessbot::protocol::kProtocolVersion) + "\n"
      "ok\n"          // INFO
      "ok\n"          // PING
      "ok\n"          // ENABLE
      "ok\n"          // DISABLE
      "ok\n"          // ENABLE
      "!alarm:ESTOP\n"
      "error:5\n"     // MOVE in ESTOP
      "error:5\n"     // ENABLE in ESTOP
      "ok\n"          // DISABLE in ESTOP
      "!alarm:ESTOP\n"  // a repeated ESTOP in ESTOP: accepted, answered again
      "ok\n"          // PING in ESTOP
      "ok\n"          // RESET
      "ok\n"          // ENABLE after RESET
      "error:1\n"     // FOO
      "error:2\n"     // PING x
      "POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=MID\n"
      "ok\n";         // POS
  TEST_ASSERT_EQUAL_STRING(expected.c_str(), Serial.tx.c_str());
  TEST_ASSERT_TRUE(Serial.tx.find('\r') == std::string::npos);  // LF only
}

void test_the_ena_pins_follow_enable_and_disable_and_estop_never_touches_them() {
  boot_once();
  host_arduino::log().clear();
  Serial.type("DISABLE\n");
  loop();
  // The joint pulses stop first (SPEC 5.1, "DISABLE rules"), then both ENA pins go to the disabled level.
  const std::vector<std::string>& disable_log = host_arduino::log();
  TEST_ASSERT_EQUAL_INT(6, disable_log.size());
  TEST_ASSERT_EQUAL_STRING("fas0.forcestop", disable_log.at(0).c_str());
  TEST_ASSERT_EQUAL_STRING("fas1.forcestop", disable_log.at(1).c_str());
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena1, LOW).c_str(), disable_log.at(2).c_str());
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena2, LOW).c_str(), disable_log.at(4).c_str());
  host_arduino::log().clear();
  Serial.type("ENABLE\n");
  loop();
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena1, HIGH).c_str(), host_arduino::log().at(0).c_str());
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena2, HIGH).c_str(), host_arduino::log().at(2).c_str());
  host_arduino::log().clear();
  Serial.type("ESTOP\n");
  loop();
  for (const std::string& event : host_arduino::log()) {
    TEST_ASSERT_TRUE(event.find(" " + std::to_string(kTb6600Ena1) + " ") == std::string::npos);
    TEST_ASSERT_TRUE(event.find(" " + std::to_string(kTb6600Ena2) + " ") == std::string::npos);
  }
  Serial.type("RESET\n");
  loop();
}

void test_in_an_estop_disable_drives_the_ena_pins_low_and_enable_does_not_touch_them() {
  boot_once();
  Serial.type("ENABLE\nESTOP\n");  // a known start: drivers enabled, in an ESTOP
  loop();
  host_arduino::log().clear();
  Serial.tx.clear();
  Serial.type("ENABLE\n");  // refused: error:5 and not one pin write
  loop();
  TEST_ASSERT_EQUAL_STRING("error:5\n", Serial.tx.c_str());
  TEST_ASSERT_TRUE(host_arduino::log().empty());
  Serial.tx.clear();
  Serial.type("DISABLE\n");  // accepted in an ESTOP: both ENA pins LOW
  loop();
  TEST_ASSERT_EQUAL_STRING("ok\n", Serial.tx.c_str());
  TEST_ASSERT_EQUAL_STRING("fas0.forcestop", host_arduino::log().at(0).c_str());  // pulses first
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena1, LOW).c_str(), host_arduino::log().at(2).c_str());
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena2, LOW).c_str(), host_arduino::log().at(4).c_str());
  host_arduino::log().clear();
  Serial.tx.clear();
  Serial.type("ESTOP\n");  // repeated: the alarm again and no pin write, the drivers stay disabled
  loop();
  TEST_ASSERT_EQUAL_STRING("!alarm:ESTOP\n", Serial.tx.c_str());
  for (const std::string& event : host_arduino::log()) {
    TEST_ASSERT_TRUE(event.find(" " + std::to_string(kTb6600Ena1) + " ") == std::string::npos);
    TEST_ASSERT_TRUE(event.find(" " + std::to_string(kTb6600Ena2) + " ") == std::string::npos);
  }
  host_arduino::log().clear();
  Serial.tx.clear();
  Serial.type("RESET\nENABLE\n");  // ENABLE works again after RESET, and leaves the device enabled for later tests
  loop();
  TEST_ASSERT_EQUAL_STRING("ok\nok\n", Serial.tx.c_str());
  TEST_ASSERT_TRUE(index_of(host_arduino::log(), w(kTb6600Ena1, HIGH)) < host_arduino::log().size());
  TEST_ASSERT_TRUE(index_of(host_arduino::log(), w(kTb6600Ena2, HIGH)) < host_arduino::log().size());
}

void test_an_overlong_line_gets_one_error_1_and_the_next_line_still_works() {
  boot_once();
  Serial.tx.clear();
  Serial.type(std::string(chessbot::config::kMaxLineBytes + 10, 'X') + "\nPING\n");
  loop();
  TEST_ASSERT_EQUAL_STRING("error:1\nok\n", Serial.tx.c_str());
}

void test_a_line_split_across_loop_calls_is_reassembled() {
  boot_once();
  Serial.tx.clear();
  Serial.type("PI");
  loop();
  TEST_ASSERT_TRUE(Serial.tx.empty());
  Serial.type("NG\n");
  loop();
  TEST_ASSERT_EQUAL_STRING("ok\n", Serial.tx.c_str());
}

void test_no_heartbeat_alarm_is_sent_yet() {
  boot_once();
  Serial.tx.clear();
  Serial.type("PING\n");
  loop();
  Serial.tx.clear();
  host_arduino::clock_ms() += 100u * chessbot::config::kHeartbeatTimeoutMs;
  loop();
  TEST_ASSERT_TRUE(Serial.tx.empty());
}

// The driver rules of the phase 4a follow-up through the real main.cpp, the real core and the real BoardHardware: the
// exact replies of MOVE and HOME while the drivers are disabled / enabled, and that a refusal writes no pin.
void test_the_driver_gates_answer_through_main_and_write_no_pin() {
  boot_once();
  Serial.type("ENABLE\nRESET\n");  // a known start: drivers enabled, no ESTOP
  loop();
  Serial.tx.clear();
  Serial.type("MOVE A=100 B=50 F=10\nHOME\nHOME Z\nHOME AB\nDISABLE\n");
  loop();
  // enabled and not homed: MOVE is error:4; the HOME forms pass the gates and are the 4a stub (error:1)
  TEST_ASSERT_EQUAL_STRING("error:4\nerror:1\nerror:1\nerror:1\nok\n", Serial.tx.c_str());
  host_arduino::log().clear();
  Serial.tx.clear();
  Serial.type("MOVE A=100 B=50 F=10\nHOME\nHOME AB\nHOME Z AB\nHOME Z\nMOVE A=1\nFOO\n");
  loop();
  // disabled: error:6 for MOVE and for a HOME that moves the joints; HOME Z passes (stub); parse errors come first
  TEST_ASSERT_EQUAL_STRING("error:6\nerror:6\nerror:6\nerror:6\nerror:1\nerror:2\nerror:1\n", Serial.tx.c_str());
  TEST_ASSERT_TRUE(host_arduino::log().empty());  // no refusal touched a pin, and HOME did not enable the drivers
  Serial.tx.clear();
  Serial.type("ESTOP\nMOVE A=100 B=50 F=10\nHOME\nHOME Z\n");
  loop();
  TEST_ASSERT_EQUAL_STRING("!alarm:ESTOP\nerror:5\nerror:5\nerror:5\n", Serial.tx.c_str());  // 5 comes before 6
  Serial.type("RESET\nENABLE\n");  // leave the device enabled for later tests
  loop();
}

// ---- the glue: BoardHardware and the library ------------------------------------------------------------------

namespace {

// A second processor on the REAL BoardHardware, with the bench setting on (the project's default is off, and without
// a HOME no MOVE could ever be accepted through the real g_processor).
chessbot::MotionConfig bench_motion() {
  chessbot::MotionConfig config = chessbot::MotionConfig::from_project();
  config.bench_move_without_homing = true;
  return config;
}

struct GlueBench {
  SerialSink sink;
  chessbot::CommandProcessor processor;
  GlueBench() : processor(&sink, &g_hardware, bench_motion()) {
    host_fas::state() = host_fas::State();
    host_arduino::log().clear();
    Serial.tx.clear();
  }
  std::string send(const char* line) {
    Serial.tx.clear();
    processor.handle_line(line, 0);
    return Serial.tx;
  }
};

bool logged(const std::string& event) { return index_of(host_arduino::log(), event) < host_arduino::log().size(); }

}  // namespace

void test_a_move_reaches_the_library_as_speed_acceleration_and_an_absolute_target_per_joint() {
  boot_once();
  GlueBench bench;
  TEST_ASSERT_EQUAL_STRING("ok\n", bench.send("MOVE A=90 B=45 F=20").c_str());
  TEST_ASSERT_TRUE(host_arduino::log().empty());  // the line only queues; the main loop starts it
  bench.processor.poll_motion();
  const chessbot::MotionConfig config = bench_motion();
  const long target_a = lround(90.0 * config.steps_per_degree());
  const long target_b = lround(45.0 * config.steps_per_degree());
  TEST_ASSERT_TRUE(logged("fas0.moveto " + std::to_string(target_a)));
  TEST_ASSERT_TRUE(logged("fas1.moveto " + std::to_string(target_b)));
  TEST_ASSERT_TRUE(host_fas::state().speed_hz[0] > host_fas::state().speed_hz[1]);  // beta has the shorter way: slower
  TEST_ASSERT_TRUE(host_fas::state().acceleration[0] > 0 && host_fas::state().acceleration[1] > 0);
  TEST_ASSERT_TRUE(g_hardware.joints_running());
  // The library calls come in the order speed, acceleration, moveTo for each joint (moveTo uses the other two).
  const size_t speed0 = index_of(host_arduino::log(), "fas0.speed " + std::to_string(host_fas::state().speed_hz[0]));
  const size_t accel0 = index_of(host_arduino::log(), "fas0.accel " + std::to_string(host_fas::state().acceleration[0]));
  const size_t move0 = index_of(host_arduino::log(), "fas0.moveto " + std::to_string(target_a));
  TEST_ASSERT_TRUE(speed0 < accel0 && accel0 < move0);
  host_fas::finish();
  bench.processor.poll_motion();
  TEST_ASSERT_EQUAL_STRING("POS A=90 B=45 SA=0 SB=0 NA=800 NB=400 Z=MID\nok\n", bench.send("POS").c_str());
  host_fas::state() = host_fas::State();  // leave the positions at zero for the other tests
}

void test_wait_answers_done_through_the_real_hardware_once_the_joints_arrived() {
  boot_once();
  GlueBench bench;
  bench.send("MOVE A=90 B=45 F=20");
  TEST_ASSERT_EQUAL_STRING("ok\n", bench.send("WAIT").c_str());
  bench.processor.poll_motion();
  Serial.tx.clear();
  bench.processor.poll_motion();
  TEST_ASSERT_TRUE(Serial.tx.empty());  // still running
  host_fas::finish();
  bench.processor.poll_motion();
  TEST_ASSERT_EQUAL_STRING("done\n", Serial.tx.c_str());
  host_fas::state() = host_fas::State();
}

void test_disable_estop_and_reset_stop_both_joints_and_never_touch_an_ena_pin_except_disable() {
  boot_once();
  const char* lines[] = {"DISABLE", "ESTOP", "RESET"};
  for (const char* line : lines) {
    GlueBench bench;
    bench.send("MOVE A=90 B=45 F=20");
    bench.processor.poll_motion();
    host_arduino::log().clear();
    bench.send(line);
    TEST_ASSERT_TRUE(logged("fas0.forcestop"));
    TEST_ASSERT_TRUE(logged("fas1.forcestop"));
    TEST_ASSERT_TRUE(!g_hardware.joints_running());
    const bool ena_written = logged(w(kTb6600Ena1, LOW)) || logged(w(kTb6600Ena1, HIGH));
    TEST_ASSERT_EQUAL_INT(std::string(line) == "DISABLE", ena_written);
    bench.send("RESET");
    bench.send("ENABLE");  // leave the device enabled for later tests
    host_fas::state() = host_fas::State();
  }
}

void test_a_move_the_library_refuses_is_an_estop_and_both_joints_are_stopped() {
  boot_once();
  GlueBench bench;
  host_fas::state().refuse_moves = true;
  bench.send("MOVE A=90 B=45 F=20");
  Serial.tx.clear();
  bench.processor.poll_motion();
  TEST_ASSERT_EQUAL_STRING("!alarm:ESTOP\n", Serial.tx.c_str());
  TEST_ASSERT_TRUE(logged("fas0.forcestop") && logged("fas1.forcestop"));
  TEST_ASSERT_TRUE(!g_hardware.joints_running());
  bench.send("RESET");
  host_fas::state() = host_fas::State();
}

void test_the_main_loop_polls_the_motion_queue() {
  boot_once();
  // g_processor itself cannot be given a MOVE (not homed, bench setting off), but its loop must still reach poll_motion:
  // a WAIT on an empty queue is answered at once by the line handler, never by the loop, so nothing extra is sent.
  Serial.tx.clear();
  Serial.type("WAIT\n");
  loop();
  TEST_ASSERT_EQUAL_STRING("ok\ndone\n", Serial.tx.c_str());
}

void test_the_project_default_is_the_spec_behaviour_a_move_needs_homing() {
  TEST_ASSERT_TRUE(!chessbot::config::kBenchMoveWithoutHoming);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_the_first_calls_of_setup_are_the_pin_levels_and_serial_comes_after);
  RUN_TEST(test_tb6600_ena_pins_are_driven_high_and_z_motor_pins_low);
  RUN_TEST(test_each_pin_level_is_written_before_the_pin_becomes_an_output);
  RUN_TEST(test_serial_starts_at_the_configured_baud_with_the_configured_rx_buffer);
  RUN_TEST(test_the_joint_steppers_are_connected_right_after_the_pin_levels_and_never_get_an_ena_pin);
  RUN_TEST(test_nothing_is_sent_at_boot_and_only_the_five_pins_and_the_library_are_touched);
  RUN_TEST(test_a_session_gets_the_exact_replies);
  RUN_TEST(test_the_ena_pins_follow_enable_and_disable_and_estop_never_touches_them);
  RUN_TEST(test_in_an_estop_disable_drives_the_ena_pins_low_and_enable_does_not_touch_them);
  RUN_TEST(test_an_overlong_line_gets_one_error_1_and_the_next_line_still_works);
  RUN_TEST(test_a_line_split_across_loop_calls_is_reassembled);
  RUN_TEST(test_no_heartbeat_alarm_is_sent_yet);
  RUN_TEST(test_the_driver_gates_answer_through_main_and_write_no_pin);
  RUN_TEST(test_a_move_reaches_the_library_as_speed_acceleration_and_an_absolute_target_per_joint);
  RUN_TEST(test_wait_answers_done_through_the_real_hardware_once_the_joints_arrived);
  RUN_TEST(test_disable_estop_and_reset_stop_both_joints_and_never_touch_an_ena_pin_except_disable);
  RUN_TEST(test_a_move_the_library_refuses_is_an_estop_and_both_joints_are_stopped);
  RUN_TEST(test_the_main_loop_polls_the_motion_queue);
  RUN_TEST(test_the_project_default_is_the_spec_behaviour_a_move_needs_homing);
  return UNITY_END();
}
