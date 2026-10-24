// DeviceState and HeartbeatWatchdog: the state flags of SPEC 10.2.
#include <unity.h>

#include "config.h"
#include "device_state.h"
#include "protocol.h"

using chessbot::DeviceState;
using chessbot::HeartbeatWatchdog;
using protocol_alarm = chessbot::protocol::AlarmCode;

void setUp(void) {}
void tearDown(void) {}

void test_power_up_state() {
  DeviceState state;
  TEST_ASSERT_TRUE(!state.homed());
  TEST_ASSERT_TRUE(!state.estop());
  TEST_ASSERT_TRUE(!state.any_alarm());
  TEST_ASSERT_TRUE(state.drivers_enabled());  // SPEC 10.2: enabled at boot so the motors hold position
}

void test_homed_needs_z_and_the_joints() {
  DeviceState state;
  state.set_homed_z(true);
  TEST_ASSERT_TRUE(!state.homed());
  state.set_homed_joints(true);
  TEST_ASSERT_TRUE(state.homed());
  state.set_homed_z(false);
  TEST_ASSERT_TRUE(!state.homed());
}

void test_estop_sets_the_flag_latches_the_alarm_clears_homing_and_keeps_the_drivers() {
  DeviceState state;
  state.set_homed_z(true);
  state.set_homed_joints(true);
  state.enter_estop();
  TEST_ASSERT_TRUE(state.estop());
  TEST_ASSERT_TRUE(state.alarm_latched(protocol_alarm::kESTOP));
  TEST_ASSERT_TRUE(!state.homed_z() && !state.homed_joints());
  TEST_ASSERT_TRUE(state.drivers_enabled());
}

void test_estop_is_idempotent() {
  DeviceState state;
  state.enter_estop();
  state.enter_estop();
  TEST_ASSERT_TRUE(state.estop());
  TEST_ASSERT_TRUE(state.alarm_latched(protocol_alarm::kESTOP));
}

void test_estop_does_not_re_enable_drivers_that_the_user_disabled() {
  DeviceState state;
  state.set_drivers_enabled(false);
  state.enter_estop();
  TEST_ASSERT_TRUE(!state.drivers_enabled());  // only ENABLE / DISABLE change it
}

void test_heartbeat_timeout_is_not_an_estop_and_keeps_the_drivers() {
  DeviceState state;
  state.set_homed_z(true);
  state.set_homed_joints(true);
  state.enter_heartbeat_timeout();
  TEST_ASSERT_TRUE(!state.estop());
  TEST_ASSERT_TRUE(state.alarm_latched(protocol_alarm::kHB_TIMEOUT));
  TEST_ASSERT_TRUE(!state.homed());
  TEST_ASSERT_TRUE(state.drivers_enabled());
}

void test_every_alarm_makes_homing_mandatory() {
  for (uint8_t i = 0; i < chessbot::protocol::kAlarmCount; ++i) {
    DeviceState state;
    state.set_homed_z(true);
    state.set_homed_joints(true);
    state.raise_alarm(static_cast<protocol_alarm>(i));
    TEST_ASSERT_TRUE(state.alarm_latched(static_cast<protocol_alarm>(i)));
    TEST_ASSERT_TRUE(!state.homed());
  }
}

void test_alarms_are_independent_bits() {
  DeviceState state;
  state.raise_alarm(protocol_alarm::kLIMIT_Z);
  TEST_ASSERT_TRUE(state.alarm_latched(protocol_alarm::kLIMIT_Z));
  TEST_ASSERT_TRUE(!state.alarm_latched(protocol_alarm::kESTOP));
  TEST_ASSERT_TRUE(!state.alarm_latched(protocol_alarm::kHB_TIMEOUT));
}

void test_reset_clears_estop_and_alarms_and_requires_homing_but_leaves_the_drivers() {
  DeviceState state;
  state.set_drivers_enabled(false);
  state.set_homed_z(true);
  state.set_homed_joints(true);
  state.enter_estop();
  state.raise_alarm(protocol_alarm::kZ_TIMEOUT);
  state.reset();
  TEST_ASSERT_TRUE(!state.estop());
  TEST_ASSERT_TRUE(!state.any_alarm());
  TEST_ASSERT_TRUE(!state.homed());
  TEST_ASSERT_TRUE(!state.drivers_enabled());
}

void test_reset_alone_makes_homing_mandatory() {
  // RESET must clear homing by itself, not rely on an earlier ESTOP having done it (SPEC 5.1: "homing is
  // mandatory afterwards").
  DeviceState state;
  state.set_homed_z(true);
  state.set_homed_joints(true);
  state.reset();
  TEST_ASSERT_TRUE(!state.homed_z());
  TEST_ASSERT_TRUE(!state.homed_joints());
}

void test_watchdog_is_not_armed_until_the_first_feed() {
  HeartbeatWatchdog dog;
  TEST_ASSERT_TRUE(!dog.armed());
  TEST_ASSERT_TRUE(!dog.poll(0));
  TEST_ASSERT_TRUE(!dog.poll(0xFFFFFFFFu));  // never expires while unarmed
}

void test_watchdog_uses_the_configured_timeout() {
  HeartbeatWatchdog dog;
  TEST_ASSERT_EQUAL_UINT32(chessbot::config::kHeartbeatTimeoutMs, dog.timeout_ms());
}

void test_watchdog_expires_strictly_after_the_timeout_and_only_once() {
  HeartbeatWatchdog dog;
  const uint32_t t0 = 1000;
  dog.feed(t0);
  TEST_ASSERT_TRUE(dog.armed());
  TEST_ASSERT_TRUE(!dog.poll(t0 + chessbot::config::kHeartbeatTimeoutMs));  // exactly the timeout: still fine
  TEST_ASSERT_TRUE(dog.poll(t0 + chessbot::config::kHeartbeatTimeoutMs + 1));
  TEST_ASSERT_TRUE(!dog.armed());
  TEST_ASSERT_TRUE(!dog.poll(t0 + 10 * chessbot::config::kHeartbeatTimeoutMs));  // once
}

void test_a_feed_pushes_the_deadline_and_rearms_after_an_expiry() {
  HeartbeatWatchdog dog;
  dog.feed(0);
  dog.feed(chessbot::config::kHeartbeatTimeoutMs);
  TEST_ASSERT_TRUE(!dog.poll(2 * chessbot::config::kHeartbeatTimeoutMs));
  TEST_ASSERT_TRUE(dog.poll(2 * chessbot::config::kHeartbeatTimeoutMs + 1));
  dog.feed(5 * chessbot::config::kHeartbeatTimeoutMs);
  TEST_ASSERT_TRUE(dog.armed());
  TEST_ASSERT_TRUE(!dog.poll(5 * chessbot::config::kHeartbeatTimeoutMs));
}

void test_watchdog_survives_the_millisecond_counter_wrapping() {
  HeartbeatWatchdog dog;
  const uint32_t before_wrap = 0xFFFFFFFFu - 10;
  dog.feed(before_wrap);
  // The counter wraps; elapsed time is still the timeout, not ~4 billion.
  TEST_ASSERT_TRUE(!dog.poll(before_wrap + chessbot::config::kHeartbeatTimeoutMs));
  TEST_ASSERT_TRUE(dog.poll(before_wrap + chessbot::config::kHeartbeatTimeoutMs + 1));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_power_up_state);
  RUN_TEST(test_homed_needs_z_and_the_joints);
  RUN_TEST(test_estop_sets_the_flag_latches_the_alarm_clears_homing_and_keeps_the_drivers);
  RUN_TEST(test_estop_is_idempotent);
  RUN_TEST(test_estop_does_not_re_enable_drivers_that_the_user_disabled);
  RUN_TEST(test_heartbeat_timeout_is_not_an_estop_and_keeps_the_drivers);
  RUN_TEST(test_every_alarm_makes_homing_mandatory);
  RUN_TEST(test_alarms_are_independent_bits);
  RUN_TEST(test_reset_clears_estop_and_alarms_and_requires_homing_but_leaves_the_drivers);
  RUN_TEST(test_reset_alone_makes_homing_mandatory);
  RUN_TEST(test_watchdog_is_not_armed_until_the_first_feed);
  RUN_TEST(test_watchdog_uses_the_configured_timeout);
  RUN_TEST(test_watchdog_expires_strictly_after_the_timeout_and_only_once);
  RUN_TEST(test_a_feed_pushes_the_deadline_and_rearms_after_an_expiry);
  RUN_TEST(test_watchdog_survives_the_millisecond_counter_wrapping);
  return UNITY_END();
}
