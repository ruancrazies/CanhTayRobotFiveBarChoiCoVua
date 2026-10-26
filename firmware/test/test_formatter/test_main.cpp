// Response formatter: the exact lines of docs/SPEC.md section 5.2.
#include <string>
#include <unity.h>

#include "config.h"
#include "protocol.h"
#include "response_format.h"

using chessbot::LineBuilder;
namespace protocol = chessbot::protocol;

void setUp(void) {}
void tearDown(void) {}

void test_ok_and_done() {
  char line[16];
  TEST_ASSERT_TRUE(chessbot::format_ok(line, sizeof(line)));
  TEST_ASSERT_EQUAL_STRING("ok", line);
  TEST_ASSERT_TRUE(chessbot::format_done(line, sizeof(line)));
  TEST_ASSERT_EQUAL_STRING("done", line);
}

void test_every_error_code() {
  const struct {
    protocol::ErrorCode code;
    const char* text;
  } cases[] = {
      {protocol::ErrorCode::kUNRECOGNIZED, "error:1"}, {protocol::ErrorCode::kBAD_PARAMETER, "error:2"},
      {protocol::ErrorCode::kQUEUE_FULL, "error:3"},   {protocol::ErrorCode::kNOT_HOMED, "error:4"},
      {protocol::ErrorCode::kIN_ESTOP, "error:5"},
      {protocol::ErrorCode::kDRIVERS_DISABLED, "error:6"},
  };
  char line[16];
  for (const auto& c : cases) {
    TEST_ASSERT_TRUE(chessbot::format_error(line, sizeof(line), c.code));
    TEST_ASSERT_EQUAL_STRING(c.text, line);
  }
}

void test_every_alarm_code() {
  const char* expected[] = {"!alarm:ESTOP", "!alarm:LIMIT_Z", "!alarm:Z_TIMEOUT", "!alarm:SENSOR_FAULT", "!alarm:STEP_LOSS", "!alarm:HB_TIMEOUT"};
  TEST_ASSERT_EQUAL_INT(sizeof(expected) / sizeof(expected[0]), protocol::kAlarmCount);
  char line[32];
  for (uint8_t i = 0; i < protocol::kAlarmCount; ++i) {
    TEST_ASSERT_TRUE(chessbot::format_alarm(line, sizeof(line), static_cast<protocol::AlarmCode>(i)));
    TEST_ASSERT_EQUAL_STRING(expected[i], line);
  }
}

void test_info_line() {
  char line[chessbot::config::kMaxResponseBytes + 1];
  TEST_ASSERT_TRUE(chessbot::format_info(line, sizeof(line), chessbot::config::kFirmwareVersion));
  const std::string expected = std::string("INFO FW=") + chessbot::config::kFirmwareVersion + " PROTO=" + std::to_string(protocol::kProtocolVersion);
  TEST_ASSERT_EQUAL_STRING(expected.c_str(), line);
}

void test_the_firmware_version_is_one_plain_token() {
  const std::string version = chessbot::config::kFirmwareVersion;
  TEST_ASSERT_TRUE(!version.empty());
  TEST_ASSERT_TRUE(version.find(' ') == std::string::npos);
  TEST_ASSERT_TRUE(version.find('=') == std::string::npos);
}

void test_every_reply_fits_the_response_buffer() {
  char line[chessbot::config::kMaxResponseBytes + 1];
  TEST_ASSERT_TRUE(chessbot::format_info(line, sizeof(line), chessbot::config::kFirmwareVersion));
  for (uint8_t i = 0; i < protocol::kAlarmCount; ++i) {
    TEST_ASSERT_TRUE(chessbot::format_alarm(line, sizeof(line), static_cast<protocol::AlarmCode>(i)));
  }
}

void test_a_line_that_does_not_fit_fails_and_leaves_an_empty_string() {
  char line[4];
  TEST_ASSERT_TRUE(!chessbot::format_done(line, sizeof(line)));  // "done" needs 5
  TEST_ASSERT_EQUAL_STRING("", line);
  char exact[5];
  TEST_ASSERT_TRUE(chessbot::format_done(exact, sizeof(exact)));
  TEST_ASSERT_EQUAL_STRING("done", exact);
  char none[1];
  TEST_ASSERT_TRUE(!chessbot::format_ok(none, 0));
}

void test_line_builder_integers() {
  char buffer[32];
  LineBuilder builder(buffer, sizeof(buffer));
  builder.add_int(0).add_char(' ').add_int(-7).add_char(' ').add_int(1234567);
  TEST_ASSERT_TRUE(builder.ok());
  TEST_ASSERT_EQUAL_STRING("0 -7 1234567", builder.c_str());
}

void test_line_builder_fields() {
  char buffer[32];
  LineBuilder builder(buffer, sizeof(buffer));
  builder.add("X").add_text_field("K", "v").add_int_field("N", -3);
  TEST_ASSERT_EQUAL_STRING("X K=v N=-3", builder.c_str());
}

void test_line_builder_stays_failed_after_an_overflow() {
  char buffer[6];
  LineBuilder builder(buffer, sizeof(buffer));
  builder.add("abc").add("defg").add("x");
  TEST_ASSERT_TRUE(!builder.ok());
  TEST_ASSERT_EQUAL_STRING("", builder.c_str());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_ok_and_done);
  RUN_TEST(test_every_error_code);
  RUN_TEST(test_every_alarm_code);
  RUN_TEST(test_info_line);
  RUN_TEST(test_the_firmware_version_is_one_plain_token);
  RUN_TEST(test_every_reply_fits_the_response_buffer);
  RUN_TEST(test_a_line_that_does_not_fit_fails_and_leaves_an_empty_string);
  RUN_TEST(test_line_builder_integers);
  RUN_TEST(test_line_builder_fields);
  RUN_TEST(test_line_builder_stays_failed_after_an_overflow);
  return UNITY_END();
}
