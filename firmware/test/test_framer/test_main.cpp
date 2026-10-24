// LineFramer: the same framing rules as the PC side (desktop_app/hardware/serial_link.py LineFramer).
#include <string>
#include <unity.h>

#include "config.h"
#include "line_framer.h"

using chessbot::LineFramer;
using Result = chessbot::LineFramer::Result;

void setUp(void) {}
void tearDown(void) {}

namespace {

struct Outcome {
  int lines = 0;
  int errors = 0;
  std::string last_line;
};

Outcome feed_all(LineFramer& framer, const std::string& bytes) {
  Outcome outcome;
  for (unsigned char byte : bytes) {
    const Result result = framer.feed(byte);
    if (result == Result::kLine) {
      ++outcome.lines;
      outcome.last_line = framer.text();
    } else if (result == Result::kError) {
      ++outcome.errors;
    }
  }
  return outcome;
}

}  // namespace

void test_a_line_ends_at_newline() {
  LineFramer framer;
  TEST_ASSERT_TRUE(framer.feed('P') == Result::kNone);
  TEST_ASSERT_TRUE(framer.feed('I') == Result::kNone);
  TEST_ASSERT_TRUE(framer.feed('N') == Result::kNone);
  TEST_ASSERT_TRUE(framer.feed('G') == Result::kNone);
  TEST_ASSERT_TRUE(framer.feed('\n') == Result::kLine);
  TEST_ASSERT_EQUAL_STRING("PING", framer.text());
}

void test_crlf_is_tolerated_and_any_trailing_cr_is_dropped() {
  LineFramer framer;
  Outcome a = feed_all(framer, "PING\r\n");
  TEST_ASSERT_EQUAL_INT(1, a.lines);
  TEST_ASSERT_EQUAL_STRING("PING", a.last_line.c_str());
  Outcome b = feed_all(framer, "INFO\r\r\n");
  TEST_ASSERT_EQUAL_INT(1, b.lines);
  TEST_ASSERT_EQUAL_STRING("INFO", b.last_line.c_str());
}

void test_two_lines_in_one_stream() {
  LineFramer framer;
  Outcome outcome = feed_all(framer, "PING\nINFO\n");
  TEST_ASSERT_EQUAL_INT(2, outcome.lines);
  TEST_ASSERT_EQUAL_STRING("INFO", outcome.last_line.c_str());
  TEST_ASSERT_EQUAL_INT(0, outcome.errors);
}

void test_empty_lines_are_dropped_without_a_result() {
  LineFramer framer;
  Outcome outcome = feed_all(framer, "\n\r\n\r\r\n");
  TEST_ASSERT_EQUAL_INT(0, outcome.lines);
  TEST_ASSERT_EQUAL_INT(0, outcome.errors);
}

void test_a_blank_but_not_empty_line_is_passed_on() {
  // "   " is printable, so it is a line; the parser then ignores it (command_parser tests).
  LineFramer framer;
  Outcome outcome = feed_all(framer, "   \n");
  TEST_ASSERT_EQUAL_INT(1, outcome.lines);
  TEST_ASSERT_EQUAL_STRING("   ", outcome.last_line.c_str());
}

void test_a_tab_is_an_error() {
  LineFramer framer;
  Outcome outcome = feed_all(framer, "PI\tNG\n");
  TEST_ASSERT_EQUAL_INT(0, outcome.lines);
  TEST_ASSERT_EQUAL_INT(1, outcome.errors);
}

void test_non_ascii_del_nul_and_inner_cr_are_errors() {
  const std::string bad[] = {std::string("PI\xC3NG\n"), std::string("PI\x7F" "NG\n"), std::string("PI\0NG\n", 6), std::string("PI\rNG\n")};
  for (const std::string& line : bad) {
    LineFramer framer;
    Outcome outcome = feed_all(framer, line);
    TEST_ASSERT_EQUAL_INT(0, outcome.lines);
    TEST_ASSERT_EQUAL_INT(1, outcome.errors);
  }
}

void test_the_framer_recovers_after_an_error() {
  LineFramer framer;
  Outcome outcome = feed_all(framer, "PI\tNG\nPING\n");
  TEST_ASSERT_EQUAL_INT(1, outcome.errors);
  TEST_ASSERT_EQUAL_INT(1, outcome.lines);
  TEST_ASSERT_EQUAL_STRING("PING", outcome.last_line.c_str());
}

void test_a_line_of_exactly_the_maximum_length_is_accepted() {
  LineFramer framer;
  const std::string text(chessbot::config::kMaxLineBytes, 'X');
  Outcome outcome = feed_all(framer, text + "\n");
  TEST_ASSERT_EQUAL_INT(1, outcome.lines);
  TEST_ASSERT_EQUAL_INT(0, outcome.errors);
  TEST_ASSERT_EQUAL_INT(chessbot::config::kMaxLineBytes, outcome.last_line.size());
}

void test_a_longer_line_is_one_error_and_the_rest_of_it_is_discarded() {
  LineFramer framer;
  const std::string text(chessbot::config::kMaxLineBytes + 50, 'X');
  Outcome outcome = feed_all(framer, text + "\nPING\n");
  TEST_ASSERT_EQUAL_INT(1, outcome.errors);
  TEST_ASSERT_EQUAL_INT(1, outcome.lines);
  TEST_ASSERT_EQUAL_STRING("PING", outcome.last_line.c_str());
}

void test_the_error_comes_at_the_first_byte_over_the_limit() {
  LineFramer framer;
  for (unsigned i = 0; i < chessbot::config::kMaxLineBytes; ++i) {
    TEST_ASSERT_TRUE(framer.feed('X') == Result::kNone);
  }
  TEST_ASSERT_TRUE(framer.feed('X') == Result::kError);
  TEST_ASSERT_TRUE(framer.feed('X') == Result::kNone);  // discarding
  TEST_ASSERT_TRUE(framer.feed('\n') == Result::kNone);  // the discarded line ends silently
}

void test_the_carriage_return_counts_toward_the_limit_like_the_pc_framer() {
  // PC framer: the buffer (CR included) is checked before the CR is stripped.
  LineFramer framer;
  const std::string text(chessbot::config::kMaxLineBytes, 'X');
  Outcome outcome = feed_all(framer, text + "\r\n");
  TEST_ASSERT_EQUAL_INT(1, outcome.errors);
  TEST_ASSERT_EQUAL_INT(0, outcome.lines);
}

void test_reset_drops_a_partial_line_and_a_discard() {
  LineFramer framer;
  feed_all(framer, "PIN");
  framer.reset();
  Outcome outcome = feed_all(framer, "G\n");
  TEST_ASSERT_EQUAL_STRING("G", outcome.last_line.c_str());

  const std::string too_long(chessbot::config::kMaxLineBytes + 1, 'X');
  feed_all(framer, too_long);
  framer.reset();
  Outcome after = feed_all(framer, "PING\n");
  TEST_ASSERT_EQUAL_INT(1, after.lines);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_a_line_ends_at_newline);
  RUN_TEST(test_crlf_is_tolerated_and_any_trailing_cr_is_dropped);
  RUN_TEST(test_two_lines_in_one_stream);
  RUN_TEST(test_empty_lines_are_dropped_without_a_result);
  RUN_TEST(test_a_blank_but_not_empty_line_is_passed_on);
  RUN_TEST(test_a_tab_is_an_error);
  RUN_TEST(test_non_ascii_del_nul_and_inner_cr_are_errors);
  RUN_TEST(test_the_framer_recovers_after_an_error);
  RUN_TEST(test_a_line_of_exactly_the_maximum_length_is_accepted);
  RUN_TEST(test_a_longer_line_is_one_error_and_the_rest_of_it_is_discarded);
  RUN_TEST(test_the_error_comes_at_the_first_byte_over_the_limit);
  RUN_TEST(test_the_carriage_return_counts_toward_the_limit_like_the_pc_framer);
  RUN_TEST(test_reset_drops_a_partial_line_and_a_discard);
  return UNITY_END();
}
