// parse_command: the same grammar and error classes as decode_command() in desktop_app/hardware/protocol.py,
// driven by the declaration in protocol.h.
#include <string>
#include <unity.h>

#include "command_parser.h"
#include "protocol.h"

using chessbot::Command;
using chessbot::ParseStatus;
using chessbot::parse_command;
namespace protocol = chessbot::protocol;

void setUp(void) {}
void tearDown(void) {}

namespace {

ParseStatus parse(const char* line, Command* out = nullptr) {
  Command scratch;
  return parse_command(line, out != nullptr ? out : &scratch);
}

void expect_ok(const char* line) {
  if (parse(line) != ParseStatus::kOk) {
    TEST_FAIL_MESSAGE(line);
  }
}

void expect_bad_parameter(const char* line) {
  if (parse(line) != ParseStatus::kBadParameter) {
    TEST_FAIL_MESSAGE(line);
  }
}

}  // namespace

void test_every_declared_command_parses_with_its_required_parameters() {
  for (uint8_t c = 0; c < protocol::kCommandCount; ++c) {
    std::string line = protocol::kCommands[c].name;
    for (uint8_t i = 0; i < protocol::kParamCount; ++i) {
      const protocol::ParamSpec& p = protocol::kParams[i];
      if (static_cast<uint8_t>(p.command) == c && p.required) {
        line += std::string(" ") + p.name + "=1";
      }
    }
    Command command;
    TEST_ASSERT_TRUE(parse_command(line.c_str(), &command) == ParseStatus::kOk);
    TEST_ASSERT_EQUAL_INT(c, static_cast<int>(command.id));
  }
}

void test_each_required_parameter_is_needed() {
  for (uint8_t c = 0; c < protocol::kCommandCount; ++c) {
    for (uint8_t skipped = 0; skipped < protocol::kParamCount; ++skipped) {
      const protocol::ParamSpec& omitted = protocol::kParams[skipped];
      if (static_cast<uint8_t>(omitted.command) != c || !omitted.required) {
        continue;
      }
      std::string line = protocol::kCommands[c].name;
      for (uint8_t i = 0; i < protocol::kParamCount; ++i) {
        const protocol::ParamSpec& p = protocol::kParams[i];
        if (static_cast<uint8_t>(p.command) == c && p.required && i != skipped) {
          line += std::string(" ") + p.name + "=1";
        }
      }
      if (parse(line.c_str()) != ParseStatus::kBadParameter) {
        TEST_FAIL_MESSAGE(line.c_str());
      }
    }
  }
}

void test_no_command_takes_more_parameters_than_the_slots() {
  uint8_t most = 0;
  for (uint8_t c = 0; c < protocol::kCommandCount; ++c) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < protocol::kParamCount; ++i) {
      if (static_cast<uint8_t>(protocol::kParams[i].command) == c) {
        ++count;
      }
    }
    if (count > most) {
      most = count;
    }
  }
  TEST_ASSERT_EQUAL_INT(protocol::kMaxParamsPerCommand, most);
}

void test_move_values_are_read() {
  Command command;
  TEST_ASSERT_TRUE(parse("MOVE A=135.4 B=44.6 F=60", &command) == ParseStatus::kOk);
  TEST_ASSERT_TRUE(command.id == protocol::Cmd::kMOVE);
  TEST_ASSERT_TRUE(command.has("A") && command.has("B") && command.has("F"));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 135.4, command.value("A"));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 44.6, command.value("B"));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 60.0, command.value("F"));
}

void test_parameter_order_does_not_matter_and_extra_spaces_are_fine() {
  Command command;
  TEST_ASSERT_TRUE(parse("  MOVE   F=60  A=1   B=-2  ", &command) == ParseStatus::kOk);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, command.value("A"));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -2.0, command.value("B"));
}

void test_flags() {
  Command command;
  TEST_ASSERT_TRUE(parse("HOME", &command) == ParseStatus::kOk);
  TEST_ASSERT_TRUE(!command.has("Z") && !command.has("AB"));
  TEST_ASSERT_TRUE(parse("HOME Z AB", &command) == ParseStatus::kOk);
  TEST_ASSERT_TRUE(command.has("Z") && command.has("AB"));
  TEST_ASSERT_TRUE(parse("HOME AB", &command) == ParseStatus::kOk);
  TEST_ASSERT_TRUE(!command.has("Z") && command.has("AB"));
}

void test_flag_misuse_is_error_2() {
  expect_bad_parameter("HOME Z=1");
  expect_bad_parameter("HOME Z Z");
  expect_bad_parameter("HOME X");
}

void test_unknown_and_duplicate_parameters_are_error_2() {
  expect_bad_parameter("MOVE A=1 B=2 F=3 C=4");
  expect_bad_parameter("MOVE A=1 A=2 B=3 F=4");
  expect_bad_parameter("PING x");
  expect_bad_parameter("PING =1");
  expect_bad_parameter("ENABLE A=1");
  expect_bad_parameter("MOVE a=1 B=2 F=3");  // parameter names are case sensitive
}

void test_a_float_parameter_needs_a_value() {
  expect_bad_parameter("MOVE A B=2 F=3");
  expect_bad_parameter("MOVE A= B=2 F=3");
}

void test_ranges() {
  expect_ok("GRIP S=0");
  expect_ok("GRIP S=180");
  expect_ok("GRIP S=90.5");
  expect_bad_parameter("GRIP S=180.001");
  expect_bad_parameter("GRIP S=-0.1");
  expect_ok("MOVE A=1 B=2 F=0.001");
  expect_bad_parameter("MOVE A=1 B=2 F=0");
  expect_bad_parameter("MOVE A=1 B=2 F=-1");
  expect_ok("ACCEL S=1");
  expect_bad_parameter("ACCEL S=0");
  expect_ok("MOVE A=-720 B=720 F=1");  // angles carry no range here; joint limits are checked later
}

void test_the_number_grammar_is_strict() {
  expect_ok("MOVE A=.5 B=5. F=1");
  expect_ok("MOVE A=+5 B=-0 F=1");
  expect_ok("MOVE A=-0.25 B=+.75 F=1");
  const char* bad[] = {"1e3", "nan", "inf", "-inf", "+", "-", ".", "1.2.3", "--1", "1-2", "1+", "0x10", "1_0", "1,5", "abc"};
  for (const char* text : bad) {
    const std::string line = std::string("MOVE A=") + text + " B=2 F=3";
    if (parse(line.c_str()) != ParseStatus::kBadParameter) {
      TEST_FAIL_MESSAGE(line.c_str());
    }
  }
}

void test_unknown_command_names_are_error_1_and_case_sensitive() {
  TEST_ASSERT_TRUE(parse("FOO") == ParseStatus::kUnrecognized);
  TEST_ASSERT_TRUE(parse("ping") == ParseStatus::kUnrecognized);
  TEST_ASSERT_TRUE(parse("PINGX") == ParseStatus::kUnrecognized);
  TEST_ASSERT_TRUE(parse("PIN") == ParseStatus::kUnrecognized);
  TEST_ASSERT_TRUE(parse("FOO A=1") == ParseStatus::kUnrecognized);  // the name decides, not the parameters
  TEST_ASSERT_TRUE(parse("=1") == ParseStatus::kUnrecognized);
}

void test_blank_lines_are_ignored() {
  TEST_ASSERT_TRUE(parse("") == ParseStatus::kEmpty);
  TEST_ASSERT_TRUE(parse("    ") == ParseStatus::kEmpty);
  TEST_ASSERT_TRUE(parse(" \t ") == ParseStatus::kEmpty);
}

void test_find_command() {
  protocol::Cmd id;
  TEST_ASSERT_TRUE(chessbot::find_command("ESTOP", 5, &id));
  TEST_ASSERT_TRUE(id == protocol::Cmd::kESTOP);
  TEST_ASSERT_TRUE(!chessbot::find_command("ESTOP", 4, &id));
  TEST_ASSERT_TRUE(!chessbot::find_command("", 0, &id));
}

void test_a_failed_parse_leaves_the_output_untouched() {
  Command command(protocol::Cmd::kPING);
  TEST_ASSERT_TRUE(parse_command("MOVE A=1", &command) == ParseStatus::kBadParameter);
  TEST_ASSERT_TRUE(command.id == protocol::Cmd::kPING);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_every_declared_command_parses_with_its_required_parameters);
  RUN_TEST(test_each_required_parameter_is_needed);
  RUN_TEST(test_no_command_takes_more_parameters_than_the_slots);
  RUN_TEST(test_move_values_are_read);
  RUN_TEST(test_parameter_order_does_not_matter_and_extra_spaces_are_fine);
  RUN_TEST(test_flags);
  RUN_TEST(test_flag_misuse_is_error_2);
  RUN_TEST(test_unknown_and_duplicate_parameters_are_error_2);
  RUN_TEST(test_a_float_parameter_needs_a_value);
  RUN_TEST(test_ranges);
  RUN_TEST(test_the_number_grammar_is_strict);
  RUN_TEST(test_unknown_command_names_are_error_1_and_case_sensitive);
  RUN_TEST(test_blank_lines_are_ignored);
  RUN_TEST(test_find_command);
  RUN_TEST(test_a_failed_parse_leaves_the_output_untouched);
  return UNITY_END();
}
