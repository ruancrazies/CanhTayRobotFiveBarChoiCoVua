// Host check of src/main.cpp (stage 4a) against a recording Arduino stub:
//  * SPEC 10.2 boot: the first thing setup() does is drive the two TB6600 ENA pins HIGH and the three Z
//    motor pins LOW, before Serial starts; nothing is sent at boot; no other pin is ever touched;
//  * a command session: the exact bytes sent back and the ENA pin writes of ENABLE / DISABLE / ESTOP.
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
  // Five pins (two TB6600 ENA, three L298), each "level then output": ten events before any Serial call.
  const size_t pin_events = 5 * 2;
  TEST_ASSERT_TRUE(g_boot_log.size() >= pin_events);
  for (size_t i = 0; i < pin_events; ++i) {
    TEST_ASSERT_TRUE(g_boot_log[i].rfind("serial", 0) != 0);
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

void test_nothing_is_sent_at_boot_and_only_the_five_pins_are_touched() {
  boot_once();
  TEST_ASSERT_TRUE(Serial.tx.empty());
  for (const std::string& event : g_boot_log) {
    if (event.rfind("serial", 0) == 0) continue;
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
  Serial.type("INFO\nPING\r\nENABLE\nDISABLE\nENABLE\nESTOP\nMOVE A=1 B=2 F=3\nENABLE\nDISABLE\nESTOP\nPING\nRESET\nENABLE\nFOO\nPING x\nPOS\n\n   \n");
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
      "error:1\n";    // POS: not implemented in 4a
  TEST_ASSERT_EQUAL_STRING(expected.c_str(), Serial.tx.c_str());
  TEST_ASSERT_TRUE(Serial.tx.find('\r') == std::string::npos);  // LF only
}

void test_the_ena_pins_follow_enable_and_disable_and_estop_never_touches_them() {
  boot_once();
  host_arduino::log().clear();
  Serial.type("DISABLE\n");
  loop();
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena1, LOW).c_str(), host_arduino::log().at(0).c_str());
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena2, LOW).c_str(), host_arduino::log().at(2).c_str());
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
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena1, LOW).c_str(), host_arduino::log().at(0).c_str());
  TEST_ASSERT_EQUAL_STRING(w(kTb6600Ena2, LOW).c_str(), host_arduino::log().at(2).c_str());
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

void test_no_heartbeat_alarm_is_sent_in_stage_4a() {
  boot_once();
  Serial.tx.clear();
  Serial.type("PING\n");
  loop();
  Serial.tx.clear();
  host_arduino::clock_ms() += 100u * chessbot::config::kHeartbeatTimeoutMs;
  loop();
  TEST_ASSERT_TRUE(Serial.tx.empty());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_the_first_calls_of_setup_are_the_pin_levels_and_serial_comes_after);
  RUN_TEST(test_tb6600_ena_pins_are_driven_high_and_z_motor_pins_low);
  RUN_TEST(test_each_pin_level_is_written_before_the_pin_becomes_an_output);
  RUN_TEST(test_serial_starts_at_the_configured_baud_with_the_configured_rx_buffer);
  RUN_TEST(test_nothing_is_sent_at_boot_and_only_the_five_pins_are_touched);
  RUN_TEST(test_a_session_gets_the_exact_replies);
  RUN_TEST(test_the_ena_pins_follow_enable_and_disable_and_estop_never_touches_them);
  RUN_TEST(test_in_an_estop_disable_drives_the_ena_pins_low_and_enable_does_not_touch_them);
  RUN_TEST(test_an_overlong_line_gets_one_error_1_and_the_next_line_still_works);
  RUN_TEST(test_a_line_split_across_loop_calls_is_reassembled);
  RUN_TEST(test_no_heartbeat_alarm_is_sent_in_stage_4a);
  return UNITY_END();
}
