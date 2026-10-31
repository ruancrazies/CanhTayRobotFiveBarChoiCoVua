// Stage 4b-1, S2: the Z axis logic of the hardware-independent core (docs/SPEC.md 4.3, 5.1, 10.2).
//
// Debounce, the TOP / BOTTOM / MID state, the ramped and eased duty, the per-move timeout, the limit rules (stop at the
// target limit, LIMIT_Z for both switches or an unexpected release of TOP) and the interrupt's stop predicate. Every
// number is an explicit test value; time is a counter the test advances by hand.
#include <unity.h>

#include "protocol.h"
#include "z_axis.h"

using chessbot::LimitDebouncer;
using chessbot::ZAxis;
using chessbot::ZConfig;
using chessbot::ZDrive;
using chessbot::ZEvent;
using chessbot::protocol::ZState;

void setUp(void) {}
void tearDown(void) {}

namespace {

// 5000 ms full travel, timeout 1.5x = 7500 ms, 8 ms debounce, duty 40 -> 70 over 500 ms, eased to 40 after 80 % (4000 ms).
ZConfig test_config() {
  ZConfig config;
  config.full_travel_ms = 5000;
  config.timeout_factor = 1.5f;
  config.debounce_ms = 8;
  config.duty_start_percent = 40;
  config.duty_peak_percent = 70;
  config.duty_ramp_ms = 500;
  config.duty_eased_percent = 40;
  config.ease_after_travel_percent = 80;
  return config;
}

// A Z axis with fake switches and a clock. run() polls every ``step`` ms until ``until``.
struct Rig {
  ZAxis z;
  uint32_t now = 1000;
  bool top = false;
  bool bottom = false;
  ZEvent last = ZEvent::kNone;
  int done = 0, timeout = 0, limit_z = 0;

  Rig() : z(test_config()) {}
  explicit Rig(const ZConfig& config) : z(config) {}

  ZEvent poll() {
    const ZEvent event = z.poll(now, top, bottom);
    last = event;
    done += event == ZEvent::kDone;
    timeout += event == ZEvent::kTimeout;
    limit_z += event == ZEvent::kLimitZ;
    return event;
  }
  void run(uint32_t duration_ms, uint32_t step_ms = 1) {
    for (uint32_t t = 0; t < duration_ms; t += step_ms) {
      now += step_ms;
      poll();
    }
  }
  // The carriage sits on the top switch for good (settled for the debounce window).
  void at_top() {
    top = true;
    bottom = false;
    run(20);
  }
  void at_bottom() {
    top = false;
    bottom = true;
    run(20);
  }
  void mid() {
    top = false;
    bottom = false;
    run(20);
  }
};

// ---- the interrupt's predicate ----------------------------------------------------------------------------------

void test_the_interrupt_stops_a_raise_at_the_top_switch_only() {
  TEST_ASSERT_TRUE(chessbot::z_must_stop_now(ZDrive::kRaise, true, false));
  TEST_ASSERT_FALSE(chessbot::z_must_stop_now(ZDrive::kRaise, false, false));
  TEST_ASSERT_FALSE(chessbot::z_must_stop_now(ZDrive::kRaise, false, true));  // leaving the bottom switch is normal
}

void test_the_interrupt_stops_a_lower_at_the_bottom_switch_only() {
  TEST_ASSERT_TRUE(chessbot::z_must_stop_now(ZDrive::kLower, false, true));
  TEST_ASSERT_FALSE(chessbot::z_must_stop_now(ZDrive::kLower, false, false));
  TEST_ASSERT_FALSE(chessbot::z_must_stop_now(ZDrive::kLower, true, false));
}

void test_the_interrupt_stops_either_direction_when_both_switches_are_actuated() {
  TEST_ASSERT_TRUE(chessbot::z_must_stop_now(ZDrive::kRaise, true, true));
  TEST_ASSERT_TRUE(chessbot::z_must_stop_now(ZDrive::kLower, true, true));
}

void test_the_interrupt_has_nothing_to_stop_when_the_motor_is_not_driven() {
  for (ZDrive drive : {ZDrive::kCoast, ZDrive::kBrake}) {
    TEST_ASSERT_FALSE(chessbot::z_must_stop_now(drive, true, true));
  }
}

// ---- debounce -----------------------------------------------------------------------------------------------------

void test_a_switch_is_not_actuated_at_power_up() {
  LimitDebouncer switch_(8);
  TEST_ASSERT_FALSE(switch_.stable());
}

void test_a_change_is_taken_only_after_the_whole_window() {
  LimitDebouncer switch_(8);
  TEST_ASSERT_FALSE(switch_.update(true, 100));
  TEST_ASSERT_FALSE(switch_.update(true, 107));  // 7 ms
  TEST_ASSERT_TRUE(switch_.update(true, 108));   // 8 ms
  TEST_ASSERT_TRUE(switch_.stable());
}

void test_a_bounce_restarts_the_window() {
  LimitDebouncer switch_(8);
  switch_.update(true, 100);
  switch_.update(false, 104);  // bounced back: the pending change is forgotten
  switch_.update(true, 105);
  TEST_ASSERT_FALSE(switch_.update(true, 112));  // only 7 ms since 105
  TEST_ASSERT_TRUE(switch_.update(true, 113));
}

void test_release_is_debounced_too() {
  LimitDebouncer switch_(8);
  switch_.update(true, 0);
  switch_.update(true, 8);
  TEST_ASSERT_TRUE(switch_.stable());
  TEST_ASSERT_TRUE(switch_.update(false, 20));
  TEST_ASSERT_FALSE(switch_.update(false, 28));
}

void test_the_debounce_survives_the_millisecond_counter_wrapping() {
  LimitDebouncer switch_(8);
  switch_.update(true, 0xFFFFFFFCU);
  TEST_ASSERT_FALSE(switch_.update(true, 2));  // 6 ms later, across the wrap
  TEST_ASSERT_TRUE(switch_.update(true, 4));   // 8 ms later
}

// ---- state --------------------------------------------------------------------------------------------------------

void test_the_state_is_mid_at_boot_even_before_any_switch_has_been_seen() {
  Rig rig;
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kMID), static_cast<int>(rig.z.state()));
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kCoast), static_cast<int>(rig.z.drive()));  // Z is off at boot
}

void test_an_actuated_top_switch_makes_the_state_top_only_after_the_debounce() {
  Rig rig;
  rig.top = true;
  rig.run(7);
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kMID), static_cast<int>(rig.z.state()));
  rig.run(2);
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kTOP), static_cast<int>(rig.z.state()));
}

void test_the_bottom_switch_makes_the_state_bottom_and_none_makes_it_mid() {
  Rig rig;
  rig.at_bottom();
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kBOTTOM), static_cast<int>(rig.z.state()));
  rig.mid();
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kMID), static_cast<int>(rig.z.state()));
}

// ---- duty ---------------------------------------------------------------------------------------------------------

void test_the_duty_ramps_linearly_from_start_to_peak() {
  const ZConfig config = test_config();
  TEST_ASSERT_EQUAL_UINT8(40, config.duty_percent(0));
  TEST_ASSERT_EQUAL_UINT8(55, config.duty_percent(250));  // halfway
  TEST_ASSERT_EQUAL_UINT8(70, config.duty_percent(500));
  TEST_ASSERT_EQUAL_UINT8(70, config.duty_percent(3999));
}

void test_the_duty_is_eased_after_the_share_of_the_travel_time() {
  const ZConfig config = test_config();
  TEST_ASSERT_EQUAL_UINT8(70, config.duty_percent(3999));
  TEST_ASSERT_EQUAL_UINT8(40, config.duty_percent(4000));  // 80 % of 5000 ms
  TEST_ASSERT_EQUAL_UINT8(40, config.duty_percent(7000));
}

void test_a_zero_ramp_goes_straight_to_the_peak() {
  ZConfig config = test_config();
  config.duty_ramp_ms = 0;
  TEST_ASSERT_EQUAL_UINT8(70, config.duty_percent(0));
}

void test_the_duty_is_never_above_the_peak_or_below_the_lower_of_start_and_eased() {
  const ZConfig config = test_config();
  for (uint32_t t = 0; t < 8000; t += 7) {
    const uint8_t duty = config.duty_percent(t);
    TEST_ASSERT_TRUE(duty >= 40);
    TEST_ASSERT_TRUE(duty <= 70);
  }
}

void test_the_timeout_is_the_factor_times_the_full_travel_time() {
  TEST_ASSERT_EQUAL_UINT32(7500, test_config().timeout_ms());
}

void test_duty_ticks() {
  TEST_ASSERT_EQUAL_UINT32(0, chessbot::z_duty_to_ticks(0, 8));
  TEST_ASSERT_EQUAL_UINT32(255, chessbot::z_duty_to_ticks(100, 8));
  TEST_ASSERT_EQUAL_UINT32(102, chessbot::z_duty_to_ticks(40, 8));  // 40 % of 255
  TEST_ASSERT_EQUAL_UINT32(65535, chessbot::z_duty_to_ticks(100, 16));
  TEST_ASSERT_EQUAL_UINT32(255, chessbot::z_full_scale_ticks(8));
  TEST_ASSERT_EQUAL_UINT32(1023, chessbot::z_full_scale_ticks(10));
}

// ---- a move -------------------------------------------------------------------------------------------------------

void test_zdown_from_top_runs_lowers_and_is_done_at_the_bottom_switch() {
  Rig rig;
  rig.at_top();
  TEST_ASSERT_EQUAL(static_cast<int>(ZAxis::StartResult::kStarted), static_cast<int>(rig.z.start(false, rig.now)));
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kLower), static_cast<int>(rig.z.drive()));
  TEST_ASSERT_EQUAL_UINT8(40, rig.z.duty_percent());
  rig.top = false;  // the carriage leaves the top switch: expected, no alarm
  rig.run(2000);
  TEST_ASSERT_TRUE(rig.z.moving());
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kMID), static_cast<int>(rig.z.state()));
  TEST_ASSERT_EQUAL_UINT8(70, rig.z.duty_percent());
  rig.bottom = true;
  rig.run(1);
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kBrake), static_cast<int>(rig.z.drive()));  // braked at once on the raw reading
  TEST_ASSERT_EQUAL(0, rig.done);  // but done only after the debounce
  rig.run(10);
  TEST_ASSERT_EQUAL(1, rig.done);
  TEST_ASSERT_FALSE(rig.z.moving());
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kBOTTOM), static_cast<int>(rig.z.state()));
  TEST_ASSERT_EQUAL(0, rig.limit_z + rig.timeout);
  rig.run(50);
  TEST_ASSERT_EQUAL(1, rig.done);  // exactly once
}

void test_zup_from_bottom_is_done_at_the_top_switch() {
  Rig rig;
  rig.at_bottom();
  TEST_ASSERT_EQUAL(static_cast<int>(ZAxis::StartResult::kStarted), static_cast<int>(rig.z.start(true, rig.now)));
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kRaise), static_cast<int>(rig.z.drive()));
  rig.bottom = false;
  rig.run(1500);
  rig.top = true;
  rig.run(12);
  TEST_ASSERT_EQUAL(1, rig.done);
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kTOP), static_cast<int>(rig.z.state()));
}

void test_a_move_towards_the_limit_it_is_already_at_starts_nothing() {
  Rig rig;
  rig.at_top();
  TEST_ASSERT_EQUAL(static_cast<int>(ZAxis::StartResult::kAlreadyThere), static_cast<int>(rig.z.start(true, rig.now)));
  TEST_ASSERT_FALSE(rig.z.moving());
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kCoast), static_cast<int>(rig.z.drive()));  // the motor was never driven
  rig.at_bottom();
  TEST_ASSERT_EQUAL(static_cast<int>(ZAxis::StartResult::kAlreadyThere), static_cast<int>(rig.z.start(false, rig.now)));
}

void test_a_move_from_the_middle_runs() {
  Rig rig;
  rig.mid();
  TEST_ASSERT_EQUAL(static_cast<int>(ZAxis::StartResult::kStarted), static_cast<int>(rig.z.start(true, rig.now)));
  TEST_ASSERT_TRUE(rig.z.moving());
}

void test_without_a_measured_travel_time_a_move_is_refused() {
  ZConfig config = test_config();
  config.full_travel_ms = 0;
  Rig rig(config);
  TEST_ASSERT_EQUAL(static_cast<int>(ZAxis::StartResult::kNotConfigured), static_cast<int>(rig.z.start(false, rig.now)));
  TEST_ASSERT_FALSE(rig.z.moving());
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kCoast), static_cast<int>(rig.z.drive()));
}

void test_the_duty_follows_the_ramp_then_eases_while_moving() {
  Rig rig;
  rig.at_top();
  rig.z.start(false, rig.now);
  rig.top = false;
  rig.run(250);
  TEST_ASSERT_EQUAL_UINT8(55, rig.z.duty_percent());
  rig.run(1000);
  TEST_ASSERT_EQUAL_UINT8(70, rig.z.duty_percent());
  rig.run(3000);  // 4250 ms into the move
  TEST_ASSERT_EQUAL_UINT8(40, rig.z.duty_percent());
}

void test_a_brake_is_full_duty_and_coast_is_zero() {
  Rig rig;
  rig.mid();
  rig.z.start(true, rig.now);
  rig.z.stop();
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kBrake), static_cast<int>(rig.z.drive()));
  TEST_ASSERT_EQUAL_UINT8(100, rig.z.duty_percent());
  Rig fresh;
  TEST_ASSERT_EQUAL_UINT8(0, fresh.z.duty_percent());
}

// ---- timeout ------------------------------------------------------------------------------------------------------

void test_a_move_that_never_reaches_a_switch_times_out_at_the_factor_times_the_travel_time() {
  Rig rig;
  rig.at_top();
  rig.z.start(false, rig.now);
  rig.top = false;
  rig.run(7499);
  TEST_ASSERT_EQUAL(0, rig.timeout);
  TEST_ASSERT_TRUE(rig.z.moving());
  rig.run(1);
  TEST_ASSERT_EQUAL(1, rig.timeout);
  TEST_ASSERT_FALSE(rig.z.moving());
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kBrake), static_cast<int>(rig.z.drive()));
  TEST_ASSERT_EQUAL(0, rig.done);
  rig.run(20000);
  TEST_ASSERT_EQUAL(1, rig.timeout);  // once
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kMID), static_cast<int>(rig.z.state()));
}

void test_the_timeout_starts_at_the_command_not_at_boot() {
  Rig rig;
  rig.mid();
  rig.run(20000);
  rig.z.start(true, rig.now);
  rig.run(7000);
  TEST_ASSERT_EQUAL(0, rig.timeout);
}

void test_a_glitch_that_stops_the_motor_but_is_not_confirmed_ends_in_the_timeout_alarm() {
  Rig rig;
  rig.mid();
  rig.z.start(true, rig.now);
  rig.run(100);
  rig.top = true;
  rig.run(3);  // 3 ms of "actuated": the motor is braked on the raw reading
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kBrake), static_cast<int>(rig.z.drive()));
  rig.top = false;
  rig.run(20000);
  TEST_ASSERT_EQUAL(0, rig.done);  // never reported done: the switch was not really reached
  TEST_ASSERT_EQUAL(1, rig.timeout);  // the PC gets a defined answer
}

// ---- stop / replace -----------------------------------------------------------------------------------------------

void test_zstop_brakes_and_cancels_the_done() {
  Rig rig;
  rig.mid();
  rig.z.start(true, rig.now);
  rig.run(500);
  rig.z.stop();
  TEST_ASSERT_FALSE(rig.z.moving());
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kBrake), static_cast<int>(rig.z.drive()));
  rig.top = true;  // even if the switch is reached afterwards (coasting), no done is owed
  rig.run(30);
  TEST_ASSERT_EQUAL(0, rig.done);
  rig.run(20000);
  TEST_ASSERT_EQUAL(0, rig.timeout);
}

void test_a_new_move_replaces_the_running_one_and_restarts_the_ramp_and_the_timeout() {
  Rig rig;
  rig.mid();
  rig.z.start(true, rig.now);
  rig.run(7000);
  rig.z.start(false, rig.now);  // reversed
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kLower), static_cast<int>(rig.z.drive()));
  TEST_ASSERT_EQUAL_UINT8(40, rig.z.duty_percent());
  rig.run(1000);
  TEST_ASSERT_EQUAL(0, rig.timeout);  // 7000 ms of the first move do not count against the second
  rig.run(6600);
  TEST_ASSERT_EQUAL(1, rig.timeout);
}

// ---- LIMIT_Z ------------------------------------------------------------------------------------------------------

void test_both_switches_actuated_is_limit_z_once_and_brakes() {
  Rig rig;
  rig.mid();
  rig.top = true;
  rig.bottom = true;
  rig.run(7);
  TEST_ASSERT_EQUAL(0, rig.limit_z);
  rig.run(2);
  TEST_ASSERT_EQUAL(1, rig.limit_z);
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kBrake), static_cast<int>(rig.z.drive()));
  rig.run(100);
  TEST_ASSERT_EQUAL(1, rig.limit_z);  // not repeated while the condition lasts
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kMID), static_cast<int>(rig.z.state()));
}

void test_both_switches_again_after_a_reset_is_a_new_limit_z() {
  Rig rig;
  rig.mid();
  rig.top = rig.bottom = true;
  rig.run(20);
  TEST_ASSERT_EQUAL(1, rig.limit_z);
  rig.mid();
  rig.z.reset();  // RESET acknowledges the alarm
  rig.top = rig.bottom = true;
  rig.run(20);
  TEST_ASSERT_EQUAL(2, rig.limit_z);
}

void test_the_releases_after_a_limit_z_are_not_new_alarms_until_reset() {
  Rig rig;
  rig.mid();
  rig.top = rig.bottom = true;
  rig.run(20);
  rig.mid();  // both released: the top switch "released at TOP" must not raise a second alarm
  rig.run(100);
  TEST_ASSERT_EQUAL(1, rig.limit_z);
}

void test_both_switches_while_moving_is_limit_z_and_no_done() {
  Rig rig;
  rig.mid();
  rig.z.start(true, rig.now);
  rig.top = true;
  rig.bottom = true;
  rig.run(20);
  TEST_ASSERT_EQUAL(1, rig.limit_z);
  TEST_ASSERT_EQUAL(0, rig.done);
  TEST_ASSERT_FALSE(rig.z.moving());
}

void test_the_top_switch_releasing_at_top_without_a_command_is_limit_z() {
  Rig rig;
  rig.at_top();
  rig.top = false;
  rig.run(7);
  TEST_ASSERT_EQUAL(0, rig.limit_z);
  rig.run(2);
  TEST_ASSERT_EQUAL(1, rig.limit_z);
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kBrake), static_cast<int>(rig.z.drive()));
}

void test_the_top_switch_releasing_during_a_zdown_is_normal() {
  Rig rig;
  rig.at_top();
  rig.z.start(false, rig.now);
  rig.top = false;
  rig.run(50);
  TEST_ASSERT_EQUAL(0, rig.limit_z);
  TEST_ASSERT_TRUE(rig.z.moving());
}

void test_the_top_switch_releasing_after_a_zstop_is_limit_z() {
  // Z stopped at TOP by ZSTOP and the carriage then drifts off the switch: nobody commanded that.
  Rig rig;
  rig.at_top();
  rig.z.stop();
  rig.top = false;
  rig.run(20);
  TEST_ASSERT_EQUAL(1, rig.limit_z);
}

void test_a_zstop_right_after_a_zdown_does_not_turn_the_debounced_release_into_limit_z() {
  // ZDOWN, the carriage leaves the switch, ZSTOP 3 ms later: the debounced release arrives after the move has ended.
  Rig rig;
  rig.at_top();
  rig.z.start(false, rig.now);
  rig.top = false;
  rig.run(3);
  rig.z.stop();
  rig.run(50);
  TEST_ASSERT_EQUAL(0, rig.limit_z);
  TEST_ASSERT_EQUAL(static_cast<int>(ZState::kMID), static_cast<int>(rig.z.state()));
}

void test_a_zdown_and_a_zstop_in_the_same_loop_pass_do_not_either() {
  Rig rig;
  rig.at_top();
  rig.z.start(false, rig.now);
  rig.z.stop();  // no poll in between
  rig.top = false;
  rig.run(50);
  TEST_ASSERT_EQUAL(0, rig.limit_z);
}

void test_the_release_is_expected_for_one_debounce_window_only() {
  // ZDOWN, ZSTOP at once, but the carriage never left the switch. A release much later is nobody's command.
  Rig rig;
  rig.at_top();
  rig.z.start(false, rig.now);
  rig.z.stop();
  rig.run(100);
  rig.top = false;
  rig.run(20);
  TEST_ASSERT_EQUAL(1, rig.limit_z);
}

void test_the_bottom_switch_releasing_is_never_an_alarm() {
  Rig rig;
  rig.at_bottom();
  rig.bottom = false;
  rig.run(50);
  TEST_ASSERT_EQUAL(0, rig.limit_z);
}

void test_a_broken_top_wire_reads_actuated_and_stops_a_raise_at_once() {
  Rig rig;
  rig.mid();
  rig.z.start(true, rig.now);
  rig.top = true;  // the wire breaks: NC contact reads like a pressed switch
  rig.run(1);
  TEST_ASSERT_EQUAL(static_cast<int>(ZDrive::kBrake), static_cast<int>(rig.z.drive()));
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_the_interrupt_stops_a_raise_at_the_top_switch_only);
  RUN_TEST(test_the_interrupt_stops_a_lower_at_the_bottom_switch_only);
  RUN_TEST(test_the_interrupt_stops_either_direction_when_both_switches_are_actuated);
  RUN_TEST(test_the_interrupt_has_nothing_to_stop_when_the_motor_is_not_driven);
  RUN_TEST(test_a_switch_is_not_actuated_at_power_up);
  RUN_TEST(test_a_change_is_taken_only_after_the_whole_window);
  RUN_TEST(test_a_bounce_restarts_the_window);
  RUN_TEST(test_release_is_debounced_too);
  RUN_TEST(test_the_debounce_survives_the_millisecond_counter_wrapping);
  RUN_TEST(test_the_state_is_mid_at_boot_even_before_any_switch_has_been_seen);
  RUN_TEST(test_an_actuated_top_switch_makes_the_state_top_only_after_the_debounce);
  RUN_TEST(test_the_bottom_switch_makes_the_state_bottom_and_none_makes_it_mid);
  RUN_TEST(test_the_duty_ramps_linearly_from_start_to_peak);
  RUN_TEST(test_the_duty_is_eased_after_the_share_of_the_travel_time);
  RUN_TEST(test_a_zero_ramp_goes_straight_to_the_peak);
  RUN_TEST(test_the_duty_is_never_above_the_peak_or_below_the_lower_of_start_and_eased);
  RUN_TEST(test_the_timeout_is_the_factor_times_the_full_travel_time);
  RUN_TEST(test_duty_ticks);
  RUN_TEST(test_zdown_from_top_runs_lowers_and_is_done_at_the_bottom_switch);
  RUN_TEST(test_zup_from_bottom_is_done_at_the_top_switch);
  RUN_TEST(test_a_move_towards_the_limit_it_is_already_at_starts_nothing);
  RUN_TEST(test_a_move_from_the_middle_runs);
  RUN_TEST(test_without_a_measured_travel_time_a_move_is_refused);
  RUN_TEST(test_the_duty_follows_the_ramp_then_eases_while_moving);
  RUN_TEST(test_a_brake_is_full_duty_and_coast_is_zero);
  RUN_TEST(test_a_move_that_never_reaches_a_switch_times_out_at_the_factor_times_the_travel_time);
  RUN_TEST(test_the_timeout_starts_at_the_command_not_at_boot);
  RUN_TEST(test_a_glitch_that_stops_the_motor_but_is_not_confirmed_ends_in_the_timeout_alarm);
  RUN_TEST(test_zstop_brakes_and_cancels_the_done);
  RUN_TEST(test_a_new_move_replaces_the_running_one_and_restarts_the_ramp_and_the_timeout);
  RUN_TEST(test_both_switches_actuated_is_limit_z_once_and_brakes);
  RUN_TEST(test_both_switches_again_after_a_reset_is_a_new_limit_z);
  RUN_TEST(test_the_releases_after_a_limit_z_are_not_new_alarms_until_reset);
  RUN_TEST(test_both_switches_while_moving_is_limit_z_and_no_done);
  RUN_TEST(test_the_top_switch_releasing_at_top_without_a_command_is_limit_z);
  RUN_TEST(test_the_top_switch_releasing_during_a_zdown_is_normal);
  RUN_TEST(test_the_top_switch_releasing_after_a_zstop_is_limit_z);
  RUN_TEST(test_a_zstop_right_after_a_zdown_does_not_turn_the_debounced_release_into_limit_z);
  RUN_TEST(test_a_zdown_and_a_zstop_in_the_same_loop_pass_do_not_either);
  RUN_TEST(test_the_release_is_expected_for_one_debounce_window_only);
  RUN_TEST(test_the_bottom_switch_releasing_is_never_an_alarm);
  RUN_TEST(test_a_broken_top_wire_reads_actuated_and_stops_a_raise_at_once);
  return UNITY_END();
}
