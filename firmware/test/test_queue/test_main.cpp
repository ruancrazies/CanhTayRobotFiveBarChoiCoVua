// CommandQueue: bounded FIFO; a push into a full queue stores nothing (SPEC 5.2: error:3).
#include <unity.h>

#include "command_queue.h"
#include "config.h"
#include "protocol.h"

using chessbot::Command;
using chessbot::CommandQueue;
namespace protocol = chessbot::protocol;

void setUp(void) {}
void tearDown(void) {}

namespace {
Command move_with_a(double a) {
  Command command(protocol::Cmd::kMOVE);
  command.present[0] = true;
  command.values[0] = a;
  return command;
}
}  // namespace

void test_a_new_queue_is_empty_and_the_capacity_is_the_configured_depth() {
  CommandQueue queue;
  TEST_ASSERT_TRUE(queue.empty());
  TEST_ASSERT_TRUE(!queue.full());
  TEST_ASSERT_EQUAL_INT(0, queue.size());
  TEST_ASSERT_EQUAL_INT(chessbot::config::kQueueDepth, CommandQueue::capacity());
}

void test_it_accepts_exactly_the_configured_depth_then_refuses() {
  CommandQueue queue;
  for (int i = 0; i < chessbot::config::kQueueDepth; ++i) {
    TEST_ASSERT_TRUE(queue.push(move_with_a(i)));
  }
  TEST_ASSERT_TRUE(queue.full());
  TEST_ASSERT_TRUE(!queue.push(move_with_a(99)));
  TEST_ASSERT_EQUAL_INT(chessbot::config::kQueueDepth, queue.size());
}

void test_a_refused_command_is_not_queued() {
  CommandQueue queue;
  for (int i = 0; i < chessbot::config::kQueueDepth; ++i) {
    queue.push(move_with_a(i));
  }
  queue.push(move_with_a(99));
  Command out;
  for (int i = 0; i < chessbot::config::kQueueDepth; ++i) {
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, i, out.values[0]);
  }
  TEST_ASSERT_TRUE(!queue.pop(&out));  // the 99 never went in
}

void test_first_in_first_out_across_wrap_around() {
  CommandQueue queue;
  Command out;
  int next_in = 0;
  int next_out = 0;
  for (int round = 0; round < 5 * chessbot::config::kQueueDepth; ++round) {
    TEST_ASSERT_TRUE(queue.push(move_with_a(next_in++)));
    TEST_ASSERT_TRUE(queue.push(move_with_a(next_in++)));
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, next_out++, out.values[0]);
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, next_out++, out.values[0]);
  }
  TEST_ASSERT_TRUE(queue.empty());
}

void test_a_slot_frees_up_after_a_pop() {
  CommandQueue queue;
  for (int i = 0; i < chessbot::config::kQueueDepth; ++i) {
    queue.push(move_with_a(i));
  }
  Command out;
  TEST_ASSERT_TRUE(queue.pop(&out));
  TEST_ASSERT_TRUE(queue.push(move_with_a(100)));
  TEST_ASSERT_TRUE(queue.full());
}

void test_pop_on_an_empty_queue_fails_and_changes_nothing() {
  CommandQueue queue;
  Command out(protocol::Cmd::kHOME);
  TEST_ASSERT_TRUE(!queue.pop(&out));
  TEST_ASSERT_TRUE(out.id == protocol::Cmd::kHOME);
}

void test_clear_empties_the_queue() {
  CommandQueue queue;
  queue.push(move_with_a(1));
  queue.push(move_with_a(2));
  queue.clear();
  TEST_ASSERT_TRUE(queue.empty());
  TEST_ASSERT_TRUE(queue.push(move_with_a(3)));
  Command out;
  TEST_ASSERT_TRUE(queue.pop(&out));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 3.0, out.values[0]);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_a_new_queue_is_empty_and_the_capacity_is_the_configured_depth);
  RUN_TEST(test_it_accepts_exactly_the_configured_depth_then_refuses);
  RUN_TEST(test_a_refused_command_is_not_queued);
  RUN_TEST(test_first_in_first_out_across_wrap_around);
  RUN_TEST(test_a_slot_frees_up_after_a_pop);
  RUN_TEST(test_pop_on_an_empty_queue_fails_and_changes_nothing);
  RUN_TEST(test_clear_empties_the_queue);
  return UNITY_END();
}
