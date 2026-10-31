// Stage 4b-1, S1: joint motion in the hardware-independent core (docs/SPEC.md 5, 6.1, 10.2).
//
// Steps conversion, soft limits, speed and acceleration ceilings, the plan that makes both joints finish together,
// the motion queue (the running MOVE counts toward its depth), the gate order for MOVE, ACCEL, WAIT, POS, and what
// DISABLE, ESTOP and RESET do to a running move. Every number is an explicit test value: nothing here depends on
// the assumed values of settings.yaml except the two tests that say so.
#include <math.h>

#include <string>
#include <unity.h>
#include <vector>

#include "command_processor.h"
#include "config.h"
#include "motion.h"
#include "protocol.h"

using chessbot::CommandProcessor;
using chessbot::JointMove;
using chessbot::MotionConfig;

// The processor declares this a friend; stage 4b-1 has no HOME yet, so the test presets the homed flags itself.
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

// A fully explicit configuration: 200 * 16 * 1 = 3200 steps per turn, 8.888... steps per degree.
MotionConfig test_config() {
  MotionConfig config;
  config.full_steps_per_rev = 200;
  config.microstepping = 16;
  config.gear_ratio = 1.0;
  config.max_speed_deg_s = 30.0;
  config.max_accel_deg_s2 = 60.0;
  config.limits_configured = true;
  config.alpha_min_deg = 60.0;
  config.alpha_max_deg = 200.0;
  config.beta_min_deg = -20.0;
  config.beta_max_deg = 115.0;
  config.bench_move_without_homing = false;
  return config;
}

constexpr double kStepsPerDeg = 3200.0 / 360.0;

struct FakeSink : chessbot::ResponseSink {
  std::vector<std::string> lines;
  void write_line(const char* line) override { lines.push_back(line); }
};

// Joints that move only when the test says so: start_joint_move records the move and "runs" until finish() or a stop.
struct FakeHardware : chessbot::HardwareOps {
  std::vector<JointMove> starts;
  std::vector<std::string> events;
  bool running = false;
  int32_t steps[2] = {0, 0};
  int32_t target[2] = {0, 0};

  void set_drivers_enabled(bool enabled) override { events.push_back(enabled ? "drivers=1" : "drivers=0"); }
  void halt_motion_outputs() override {
    events.push_back("halt");
    running = false;
  }
  void stop_joint_pulses() override {
    events.push_back("pulses");
    running = false;
  }
  bool fail_next_start = false;
  bool start_joint_move(const JointMove& move) override {
    if (fail_next_start) {
      events.push_back("start failed");
      return false;
    }
    starts.push_back(move);
    events.push_back("start");
    for (int i = 0; i < 2; ++i) {
      target[i] = move.runs[i] ? move.target_steps[i] : steps[i];
    }
    running = move.any();
    return true;
  }
  bool joints_running() override { return running; }
  int32_t joint_steps_issued(uint8_t joint) override { return steps[joint]; }
  // Z: the carriage sits on the top switch unless a test moves it; every drive_z() call is recorded.
  bool top_switch = true;
  bool bottom_switch = false;
  std::vector<std::string> z_calls;
  void drive_z(chessbot::ZDrive drive, uint8_t duty_percent) override {
    static const char* const names[] = {"coast", "raise", "lower", "brake"};
    z_calls.push_back(std::string(names[static_cast<int>(drive)]) + ":" + std::to_string(duty_percent));
  }
  bool limit_actuated(chessbot::ZLimit limit) override { return limit == chessbot::ZLimit::kTop ? top_switch : bottom_switch; }

  // The joints arrive.
  void finish() {
    steps[0] = target[0];
    steps[1] = target[1];
    running = false;
  }
  // The joints are part way there: a fraction of the way, in steps.
  void part_way(double fraction, int32_t from_a, int32_t from_b) {
    steps[0] = from_a + static_cast<int32_t>(lround(fraction * (target[0] - from_a)));
    steps[1] = from_b + static_cast<int32_t>(lround(fraction * (target[1] - from_b)));
  }
};

struct Bench {
  FakeSink sink;
  FakeHardware hardware;
  CommandProcessor processor;
  uint32_t now = 0;

  explicit Bench(const MotionConfig& config = test_config()) : processor(&sink, &hardware, config) {
    processor.poll_z(0);  // let the debounce see the top switch: Z reads TOP
    processor.poll_z(20);
  }

  std::vector<std::string> send(const char* line) {
    sink.lines.clear();
    processor.handle_line(line, now);
    return sink.lines;
  }
  // The main loop between two lines; returns what it sent (the done of a WAIT).
  std::vector<std::string> poll() {
    sink.lines.clear();
    processor.poll_motion();
    return sink.lines;
  }
};

std::string joined(const std::vector<std::string>& lines) {
  std::string text;
  for (const std::string& line : lines) {
    text += line + "|";
  }
  return text;
}

void expect(const std::vector<std::string>& got, std::initializer_list<const char*> want, const char* what) {
  std::string wanted;
  for (const char* line : want) {
    wanted += std::string(line) + "|";
  }
  if (joined(got) != wanted) {
    const std::string message = std::string(what) + ": got [" + joined(got) + "] want [" + wanted + "]";
    TEST_FAIL_MESSAGE(message.c_str());
  }
}

MotionConfig bench_config() {
  MotionConfig config = test_config();
  config.bench_move_without_homing = true;
  return config;
}

}  // namespace

// ---- steps conversion (SPEC 6.1: N = 200 * f * i * angle / 360) ------------------------------------

void test_steps_per_degree_follows_the_drive_settings() {
  MotionConfig config = test_config();
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, kStepsPerDeg, config.steps_per_degree());
  config.gear_ratio = 2.0;  // a 2:1 gear doubles the steps
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2 * kStepsPerDeg, config.steps_per_degree());
  config.gear_ratio = 1.0;
  config.microstepping = 8;
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, kStepsPerDeg / 2, config.steps_per_degree());
}

void test_angles_convert_to_whole_steps() {
  const MotionConfig config = test_config();
  TEST_ASSERT_EQUAL_INT32(0, chessbot::angle_to_steps(config, 0));
  TEST_ASSERT_EQUAL_INT32(3200, chessbot::angle_to_steps(config, 360));
  TEST_ASSERT_EQUAL_INT32(800, chessbot::angle_to_steps(config, 90));
  TEST_ASSERT_EQUAL_INT32(-400, chessbot::angle_to_steps(config, -45));
  TEST_ASSERT_EQUAL_INT32(1204, chessbot::angle_to_steps(config, 135.4));  // 1203.56 rounds up
  TEST_ASSERT_EQUAL_INT32(1, chessbot::angle_to_steps(config, 0.0563));    // just over half a step
  TEST_ASSERT_EQUAL_INT32(0, chessbot::angle_to_steps(config, 0.0561));    // just under half a step
}

void test_steps_convert_back_to_angles() {
  const MotionConfig config = test_config();
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 90.0, chessbot::steps_to_angle(config, 800));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -45.0, chessbot::steps_to_angle(config, -400));
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 135.45, chessbot::steps_to_angle(config, 1204));
}

// ---- soft limits and ceilings -----------------------------------------------------------------------

void test_joint_limits_are_inclusive() {
  const MotionConfig config = test_config();
  TEST_ASSERT_TRUE(config.within_limits(60, -20));
  TEST_ASSERT_TRUE(config.within_limits(200, 115));
  TEST_ASSERT_TRUE(!config.within_limits(59.999, 0));
  TEST_ASSERT_TRUE(!config.within_limits(200.001, 0));
  TEST_ASSERT_TRUE(!config.within_limits(100, -20.001));
  TEST_ASSERT_TRUE(!config.within_limits(100, 115.001));
}

void test_without_configured_limits_every_angle_is_allowed() {
  MotionConfig config = test_config();
  config.limits_configured = false;
  TEST_ASSERT_TRUE(config.within_limits(-1000, 1000));
}

void test_the_speed_and_acceleration_ceilings_are_inclusive_and_zero_means_not_decided() {
  MotionConfig config = test_config();
  TEST_ASSERT_TRUE(config.speed_allowed(30.0));
  TEST_ASSERT_TRUE(!config.speed_allowed(30.001));
  TEST_ASSERT_TRUE(config.accel_allowed(60.0));
  TEST_ASSERT_TRUE(!config.accel_allowed(60.001));
  config.max_speed_deg_s = 0;  // null in settings.yaml
  config.max_accel_deg_s2 = 0;
  TEST_ASSERT_TRUE(!config.speed_allowed(1.0));
  TEST_ASSERT_TRUE(!config.accel_allowed(1.0));
}

void test_the_project_configuration_is_the_generated_one() {
  const MotionConfig config = MotionConfig::from_project();
  TEST_ASSERT_EQUAL_UINT16(chessbot::config::kFullStepsPerRev, config.full_steps_per_rev);
  TEST_ASSERT_EQUAL_UINT16(chessbot::config::kMicrostepping, config.microstepping);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, chessbot::config::kGearRatio, config.gear_ratio);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, chessbot::config::kMaxSpeedDegPerS, config.max_speed_deg_s);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, chessbot::config::kMaxAccelDegPerS2, config.max_accel_deg_s2);
  TEST_ASSERT_EQUAL(chessbot::config::kJointLimitsConfigured, config.limits_configured);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, chessbot::config::kJointAlphaMaxDeg, config.alpha_max_deg);
  TEST_ASSERT_EQUAL(chessbot::config::kBenchMoveWithoutHoming, config.bench_move_without_homing);
  TEST_ASSERT_TRUE(config.steps_per_degree() > 0);
}

// ---- the plan: both joints finish together -------------------------------------------------------------

// Time of a trapezoid (or triangle) move of ``steps`` at ``speed`` with ``accel``: the same model FastAccelStepper runs.
double move_time_s(double steps, double speed, double accel) {
  const double ramp_steps = speed * speed / accel;  // up and down together
  if (steps >= ramp_steps) {
    return steps / speed + speed / accel;
  }
  return 2.0 * sqrt(steps / accel);
}

void test_the_longer_joint_runs_at_f_and_the_shorter_one_proportionally_slower() {
  const MotionConfig config = test_config();
  const int32_t from[2] = {0, 0};
  const JointMove move = chessbot::plan_joint_move(config, from, 90.0, 45.0, 20.0, 40.0);
  TEST_ASSERT_TRUE(move.runs[0] && move.runs[1]);
  TEST_ASSERT_EQUAL_INT32(800, move.target_steps[0]);
  TEST_ASSERT_EQUAL_INT32(400, move.target_steps[1]);
  TEST_ASSERT_UINT32_WITHIN(1, static_cast<uint32_t>(lround(20.0 * kStepsPerDeg)), move.speed_hz[0]);
  TEST_ASSERT_UINT32_WITHIN(1, static_cast<uint32_t>(lround(10.0 * kStepsPerDeg)), move.speed_hz[1]);
  TEST_ASSERT_UINT32_WITHIN(1, static_cast<uint32_t>(lround(40.0 * kStepsPerDeg)), move.accel_steps_s2[0]);
  TEST_ASSERT_UINT32_WITHIN(1, static_cast<uint32_t>(lround(20.0 * kStepsPerDeg)), move.accel_steps_s2[1]);
}

void test_both_joints_take_the_same_time_for_unequal_distances() {
  const MotionConfig config = test_config();
  const int32_t from[2] = {100, -50};
  const JointMove move = chessbot::plan_joint_move(config, from, 120.0, 17.3, 25.0, 55.0);
  const double steps_a = fabs(static_cast<double>(move.target_steps[0] - from[0]));
  const double steps_b = fabs(static_cast<double>(move.target_steps[1] - from[1]));
  const double time_a = move_time_s(steps_a, move.speed_hz[0], move.accel_steps_s2[0]);
  const double time_b = move_time_s(steps_b, move.speed_hz[1], move.accel_steps_s2[1]);
  TEST_ASSERT_DOUBLE_WITHIN(0.02 * time_a, time_a, time_b);  // within rounding of whole steps per second
}

void test_a_triangle_profile_still_finishes_together() {
  const MotionConfig config = test_config();
  const int32_t from[2] = {0, 0};
  const JointMove move = chessbot::plan_joint_move(config, from, 4.0, 1.0, 30.0, 60.0);  // too short to reach F
  const double time_a = move_time_s(fabs(static_cast<double>(move.target_steps[0])), move.speed_hz[0], move.accel_steps_s2[0]);
  const double time_b = move_time_s(fabs(static_cast<double>(move.target_steps[1])), move.speed_hz[1], move.accel_steps_s2[1]);
  TEST_ASSERT_DOUBLE_WITHIN(0.05 * time_a, time_a, time_b);
}

void test_a_joint_already_at_its_target_does_not_run() {
  const MotionConfig config = test_config();
  const int32_t from[2] = {800, 0};
  const JointMove move = chessbot::plan_joint_move(config, from, 90.0, 45.0, 20.0, 40.0);
  TEST_ASSERT_TRUE(!move.runs[0]);
  TEST_ASSERT_TRUE(move.runs[1]);
  TEST_ASSERT_EQUAL_INT32(800, move.target_steps[0]);
}

void test_a_move_to_where_both_joints_are_does_nothing() {
  const MotionConfig config = test_config();
  const int32_t from[2] = {800, 400};
  TEST_ASSERT_TRUE(!chessbot::plan_joint_move(config, from, 90.0, 45.0, 20.0, 40.0).any());
}

void test_a_tiny_move_never_plans_a_zero_speed_or_acceleration() {
  const MotionConfig config = test_config();
  const int32_t from[2] = {0, 0};
  const JointMove move = chessbot::plan_joint_move(config, from, 90.0, 0.2, 20.0, 40.0);  // beta: 2 steps against 800
  TEST_ASSERT_TRUE(move.runs[1]);
  TEST_ASSERT_TRUE(move.speed_hz[1] >= 1);
  TEST_ASSERT_TRUE(move.accel_steps_s2[1] >= 1);
}

void test_a_negative_move_plans_the_same_speeds() {
  const MotionConfig config = test_config();
  const int32_t from[2] = {800, 400};
  const JointMove move = chessbot::plan_joint_move(config, from, 0.0, 0.0, 20.0, 40.0);
  TEST_ASSERT_EQUAL_INT32(0, move.target_steps[0]);
  TEST_ASSERT_UINT32_WITHIN(1, static_cast<uint32_t>(lround(20.0 * kStepsPerDeg)), move.speed_hz[0]);
}

// ---- MOVE through the processor: gate order (SPEC 5.2: 1, 2, 5, 6, 4, 3) ---------------------------------

void test_move_on_a_homed_arm_is_ok_and_starts_both_joints() {
  Bench bench;
  chessbot::CommandProcessorTestPeer::set_homed(bench.processor);
  expect(bench.send("MOVE A=90 B=45 F=20"), {"ok"}, "MOVE");
  TEST_ASSERT_EQUAL_INT(0, bench.hardware.starts.size());  // started by the main loop, not by the line
  bench.poll();
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.starts.size());
  TEST_ASSERT_EQUAL_INT32(800, bench.hardware.starts[0].target_steps[0]);
  TEST_ASSERT_EQUAL_INT32(400, bench.hardware.starts[0].target_steps[1]);
}

void test_move_without_homing_is_error_4_unless_the_bench_setting_is_on() {
  Bench strict;
  expect(strict.send("MOVE A=90 B=45 F=20"), {"error:4"}, "not homed");
  TEST_ASSERT_TRUE(strict.processor.queue().empty());
  Bench bench(bench_config());
  expect(bench.send("MOVE A=90 B=45 F=20"), {"ok"}, "bench: not homed is allowed");
  bench.poll();
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.starts.size());
}

void test_the_bench_setting_does_not_skip_the_driver_check() {
  Bench bench(bench_config());
  bench.send("DISABLE");
  expect(bench.send("MOVE A=90 B=45 F=20"), {"error:6"}, "bench, drivers disabled");
}

void test_the_bench_setting_does_not_skip_the_estop_check() {
  Bench bench(bench_config());
  bench.send("ESTOP");
  expect(bench.send("MOVE A=90 B=45 F=20"), {"error:5"}, "bench, ESTOP");
}

void test_the_bench_setting_does_not_skip_the_queue_check() {
  Bench bench(bench_config());
  for (int i = 0; i < chessbot::config::kQueueDepth; ++i) {
    expect(bench.send("MOVE A=90 B=45 F=20"), {"ok"}, "fill");
  }
  expect(bench.send("MOVE A=90 B=45 F=20"), {"error:3"}, "bench, full queue");
}

void test_a_joint_limit_is_error_2() {
  Bench bench(bench_config());
  expect(bench.send("MOVE A=59.9 B=45 F=20"), {"error:2"}, "alpha below");
  expect(bench.send("MOVE A=200.1 B=45 F=20"), {"error:2"}, "alpha above");
  expect(bench.send("MOVE A=90 B=-20.1 F=20"), {"error:2"}, "beta below");
  expect(bench.send("MOVE A=90 B=115.1 F=20"), {"error:2"}, "beta above");
  expect(bench.send("MOVE A=60 B=-20 F=20"), {"ok"}, "both limits exactly");
  TEST_ASSERT_EQUAL_INT(1, bench.processor.queue().size());  // only the last was queued
}

void test_without_configured_limits_no_joint_limit_is_invented() {
  MotionConfig config = bench_config();
  config.limits_configured = false;
  Bench bench(config);
  expect(bench.send("MOVE A=-1000 B=1000 F=20"), {"ok"}, "no limits");
}

void test_f_above_the_ceiling_is_error_2_and_the_ceiling_itself_is_fine() {
  Bench bench(bench_config());
  expect(bench.send("MOVE A=90 B=45 F=30.001"), {"error:2"}, "F above max");
  expect(bench.send("MOVE A=90 B=45 F=30"), {"ok"}, "F at max");
}

void test_an_undecided_speed_ceiling_refuses_every_move() {
  MotionConfig config = bench_config();
  config.max_speed_deg_s = 0;
  Bench bench(config);
  expect(bench.send("MOVE A=90 B=45 F=1"), {"error:2"}, "max speed null");
}

void test_an_undecided_acceleration_ceiling_refuses_every_move_and_accel() {
  MotionConfig config = bench_config();
  config.max_accel_deg_s2 = 0;
  Bench bench(config);
  expect(bench.send("MOVE A=90 B=45 F=1"), {"error:2"}, "max accel null: MOVE");
  expect(bench.send("ACCEL S=1"), {"error:2"}, "max accel null: ACCEL");
}

void test_error_2_for_limits_comes_before_error_5_6_4_3() {
  Bench bench;  // not homed
  bench.send("DISABLE");
  bench.send("ESTOP");
  expect(bench.send("MOVE A=1 B=45 F=20"), {"error:2"}, "limit beats ESTOP, drivers, homed");
  expect(bench.send("MOVE A=90 B=45 F=99"), {"error:2"}, "F beats ESTOP, drivers, homed");
}

void test_the_gate_order_for_an_in_range_move_is_5_then_6_then_4_then_3() {
  Bench bench;
  bench.send("ESTOP");
  bench.send("DISABLE");
  expect(bench.send("MOVE A=90 B=45 F=20"), {"error:5"}, "ESTOP first");
  bench.send("RESET");
  expect(bench.send("MOVE A=90 B=45 F=20"), {"error:6"}, "then drivers");
  bench.send("ENABLE");
  expect(bench.send("MOVE A=90 B=45 F=20"), {"error:4"}, "then homed");
}

// ---- the queue: depth, running move counts, error:3 --------------------------------------------------------

void test_the_running_move_counts_toward_the_queue_depth() {
  Bench bench(bench_config());
  const int depth = chessbot::config::kQueueDepth;
  expect(bench.send("MOVE A=90 B=45 F=20"), {"ok"}, "first");
  bench.poll();  // it starts: it is running, and it still counts
  for (int i = 1; i < depth; ++i) {
    expect(bench.send("MOVE A=91 B=46 F=20"), {"ok"}, "queued behind it");
  }
  TEST_ASSERT_EQUAL_INT(depth, bench.processor.queue().size());
  expect(bench.send("MOVE A=92 B=47 F=20"), {"error:3"}, "no room");
  TEST_ASSERT_EQUAL_INT(depth, bench.processor.queue().size());  // the refused MOVE was not queued
}

void test_a_finished_move_frees_a_slot() {
  Bench bench(bench_config());
  for (int i = 0; i < chessbot::config::kQueueDepth; ++i) {
    bench.send("MOVE A=90 B=45 F=20");
  }
  bench.poll();
  expect(bench.send("MOVE A=92 B=47 F=20"), {"error:3"}, "full");
  bench.hardware.finish();
  bench.poll();  // the first is done; the second starts
  expect(bench.send("MOVE A=92 B=47 F=20"), {"ok"}, "a slot is free");
}

void test_the_next_move_starts_only_after_the_joints_stopped() {
  Bench bench(bench_config());
  bench.send("MOVE A=90 B=45 F=20");
  bench.send("MOVE A=100 B=50 F=20");
  bench.poll();
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.starts.size());
  bench.poll();  // still running
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.starts.size());
  bench.hardware.finish();
  bench.poll();
  TEST_ASSERT_EQUAL_INT(2, bench.hardware.starts.size());
  TEST_ASSERT_EQUAL_INT32(889, bench.hardware.starts[1].target_steps[0]);  // 100 deg
  TEST_ASSERT_EQUAL_INT(1, bench.processor.queue().size());
}

void test_moves_run_in_the_order_they_were_sent() {
  Bench bench(bench_config());
  bench.send("MOVE A=70 B=0 F=20");
  bench.send("MOVE A=80 B=0 F=20");
  bench.send("MOVE A=90 B=0 F=20");
  for (int i = 0; i < 3; ++i) {
    bench.poll();
    bench.hardware.finish();
  }
  bench.poll();
  TEST_ASSERT_EQUAL_INT(3, bench.hardware.starts.size());
  TEST_ASSERT_EQUAL_INT32(622, bench.hardware.starts[0].target_steps[0]);
  TEST_ASSERT_EQUAL_INT32(711, bench.hardware.starts[1].target_steps[0]);
  TEST_ASSERT_EQUAL_INT32(800, bench.hardware.starts[2].target_steps[0]);
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
}

void test_a_move_to_the_place_the_joints_already_are_finishes_without_starting_anything() {
  Bench bench(bench_config());
  bench.send("MOVE A=60 B=0 F=20");  // (0,0) steps are alpha 0 deg: this one has a way to go...
  bench.poll();
  bench.hardware.finish();
  bench.poll();
  bench.send("MOVE A=60 B=0 F=20");  // ...this one has none
  bench.poll();
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.starts.size());
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
}

// ---- ACCEL --------------------------------------------------------------------------------------------

void test_accel_is_ok_and_above_the_ceiling_is_error_2() {
  Bench bench;
  expect(bench.send("ACCEL S=40"), {"ok"}, "ACCEL");
  expect(bench.send("ACCEL S=60"), {"ok"}, "ACCEL at the ceiling");
  expect(bench.send("ACCEL S=60.001"), {"error:2"}, "ACCEL above");
}

void test_accel_needs_no_homing_and_no_drivers() {
  Bench bench;
  bench.send("DISABLE");
  expect(bench.send("ACCEL S=40"), {"ok"}, "ACCEL, drivers disabled, not homed");
}

void test_accel_is_refused_in_an_estop() {
  Bench bench;
  bench.send("ESTOP");
  expect(bench.send("ACCEL S=40"), {"error:5"}, "ACCEL in ESTOP");
}

void test_a_move_uses_the_accel_that_was_current_when_it_was_queued() {
  Bench bench(bench_config());
  bench.send("ACCEL S=10");
  bench.send("MOVE A=90 B=45 F=20");
  bench.send("ACCEL S=50");  // applies to the following segments only
  bench.send("MOVE A=100 B=45 F=20");
  bench.poll();
  bench.hardware.finish();
  bench.poll();
  TEST_ASSERT_EQUAL_INT(2, bench.hardware.starts.size());
  TEST_ASSERT_UINT32_WITHIN(1, static_cast<uint32_t>(lround(10.0 * kStepsPerDeg)), bench.hardware.starts[0].accel_steps_s2[0]);
  TEST_ASSERT_UINT32_WITHIN(1, static_cast<uint32_t>(lround(50.0 * kStepsPerDeg)), bench.hardware.starts[1].accel_steps_s2[0]);
}

void test_before_any_accel_the_ceiling_is_used() {
  Bench bench(bench_config());
  expect(bench.send("MOVE A=90 B=45 F=20"), {"ok"}, "MOVE");
  bench.poll();
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.starts.size());
  TEST_ASSERT_UINT32_WITHIN(1, static_cast<uint32_t>(lround(60.0 * kStepsPerDeg)), bench.hardware.starts[0].accel_steps_s2[0]);
}

// ---- WAIT -----------------------------------------------------------------------------------------------

void test_wait_on_an_empty_queue_is_ok_then_done_at_once() {
  Bench bench;
  expect(bench.send("WAIT"), {"ok", "done"}, "WAIT, nothing queued");
}

void test_wait_sends_its_done_only_when_the_queue_has_drained() {
  Bench bench(bench_config());
  bench.send("MOVE A=90 B=45 F=20");
  bench.send("MOVE A=100 B=45 F=20");
  expect(bench.send("WAIT"), {"ok"}, "WAIT is acknowledged");
  bench.poll();
  expect(bench.poll(), {}, "running: no done");
  bench.hardware.finish();
  expect(bench.poll(), {}, "the second move starts: no done");
  bench.hardware.finish();
  expect(bench.poll(), {"done"}, "drained: done");
  expect(bench.poll(), {}, "the done is sent once");
}

void test_two_waits_each_get_a_done() {
  Bench bench(bench_config());
  bench.send("MOVE A=90 B=45 F=20");
  bench.send("WAIT");
  bench.send("WAIT");
  bench.poll();
  bench.hardware.finish();
  expect(bench.poll(), {"done", "done"}, "two dones");
}

void test_a_wait_works_in_every_state_that_accepts_it_but_not_in_an_estop() {
  Bench bench;
  bench.send("ESTOP");
  expect(bench.send("WAIT"), {"error:5"}, "WAIT in ESTOP");
}

// ---- stopping: DISABLE, ESTOP, RESET -------------------------------------------------------------------

void test_disable_stops_the_pulses_discards_the_queue_and_cancels_a_pending_wait() {
  Bench bench(bench_config());
  bench.send("MOVE A=90 B=45 F=20");
  bench.send("MOVE A=100 B=45 F=20");
  bench.send("WAIT");
  bench.poll();
  bench.hardware.events.clear();
  expect(bench.send("DISABLE"), {"ok"}, "DISABLE");
  TEST_ASSERT_EQUAL_STRING("pulses", bench.hardware.events[0].c_str());  // pulses stop first, then the pins
  TEST_ASSERT_EQUAL_STRING("drivers=0", bench.hardware.events[1].c_str());
  TEST_ASSERT_TRUE(!bench.hardware.running);
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
  bench.hardware.finish();
  expect(bench.poll(), {}, "the WAIT's done is cancelled");
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.starts.size());  // the second MOVE never started
}

void test_estop_stops_a_running_move_and_cancels_the_wait() {
  Bench bench(bench_config());
  bench.send("MOVE A=90 B=45 F=20");
  bench.send("WAIT");
  bench.poll();
  expect(bench.send("ESTOP"), {"!alarm:ESTOP"}, "ESTOP");
  TEST_ASSERT_TRUE(!bench.hardware.running);
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
  expect(bench.poll(), {}, "no done");
}

void test_reset_stops_a_running_move_and_cancels_the_wait() {
  Bench bench(bench_config());
  bench.send("MOVE A=90 B=45 F=20");
  bench.send("WAIT");
  bench.poll();
  bench.hardware.events.clear();
  expect(bench.send("RESET"), {"ok"}, "RESET");
  TEST_ASSERT_TRUE(!bench.hardware.running);
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
  expect(bench.poll(), {}, "no done");
}

void test_a_heartbeat_timeout_stops_a_running_move_keeps_the_drivers_and_cancels_the_wait() {
  Bench bench(bench_config());
  bench.send("PING");
  bench.send("MOVE A=90 B=45 F=20");
  bench.send("WAIT");
  bench.poll();
  bench.hardware.events.clear();
  bench.sink.lines.clear();
  bench.processor.tick(bench.now + chessbot::config::kHeartbeatTimeoutMs + 1);
  expect(bench.sink.lines, {"!alarm:HB_TIMEOUT"}, "heartbeat");
  TEST_ASSERT_TRUE(!bench.hardware.running);
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
  TEST_ASSERT_TRUE(bench.processor.state().drivers_enabled());
  TEST_ASSERT_EQUAL_STRING("halt", bench.hardware.events[0].c_str());  // no "drivers=" event: the ENA pins stay as they are
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.events.size());
  expect(bench.poll(), {}, "no done");
}

void test_a_move_the_board_cannot_start_is_an_estop_and_never_a_finished_move() {
  Bench bench(bench_config());
  bench.hardware.fail_next_start = true;
  bench.send("MOVE A=90 B=45 F=20");
  bench.send("WAIT");
  expect(bench.poll(), {"!alarm:ESTOP"}, "the start failed");
  TEST_ASSERT_TRUE(bench.processor.state().estop());
  TEST_ASSERT_TRUE(bench.processor.queue().empty());
  expect(bench.poll(), {}, "no done for a move that never ran");
}

void test_a_wait_after_the_clear_finds_an_empty_queue() {
  Bench bench(bench_config());
  bench.send("MOVE A=90 B=45 F=20");
  bench.poll();
  bench.send("DISABLE");
  bench.send("ENABLE");
  expect(bench.send("WAIT"), {"ok", "done"}, "WAIT after DISABLE");
}

void test_nothing_starts_after_a_clear_even_if_the_queue_had_more() {
  Bench bench(bench_config());
  for (int i = 0; i < 4; ++i) {
    bench.send("MOVE A=90 B=45 F=20");
  }
  bench.poll();
  bench.send("DISABLE");
  bench.hardware.finish();
  bench.poll();
  bench.poll();
  TEST_ASSERT_EQUAL_INT(1, bench.hardware.starts.size());
}

// ---- POS ----------------------------------------------------------------------------------------------------

void test_pos_at_power_up_reports_zero_steps_and_the_z_state_of_the_switches() {
  Bench bench;
  expect(bench.send("POS"), {"POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=TOP", "ok"}, "POS at boot");
}

void test_pos_after_a_move_reports_the_commanded_angles_and_the_steps_issued() {
  Bench bench(bench_config());
  bench.send("MOVE A=135.4 B=44.6 F=20");
  bench.poll();
  bench.hardware.finish();
  bench.poll();
  expect(bench.send("POS"), {"POS A=135.4 B=44.6 SA=0 SB=0 NA=1204 NB=396 Z=TOP", "ok"}, "POS after a move");
}

void test_pos_while_a_move_runs_follows_the_steps_issued_so_far() {
  Bench bench(bench_config());
  bench.send("MOVE A=90 B=0 F=20");
  bench.poll();
  bench.hardware.part_way(0.5, 0, 0);  // 400 of 800 steps: 45 degrees
  expect(bench.send("POS"), {"POS A=45 B=0 SA=0 SB=0 NA=400 NB=0 Z=TOP", "ok"}, "POS half way");
}

void test_pos_after_a_stop_short_of_the_target_reports_where_the_joints_stopped() {
  Bench bench(bench_config());
  bench.send("MOVE A=90 B=0 F=20");
  bench.poll();
  bench.hardware.part_way(0.25, 0, 0);  // 200 steps: 22.5 degrees
  bench.send("DISABLE");
  expect(bench.send("POS"), {"POS A=22.5 B=0 SA=0 SB=0 NA=200 NB=0 Z=TOP", "ok"}, "POS after DISABLE");
}

void test_pos_is_accepted_in_an_estop_and_never_needs_the_drivers_or_homing() {
  Bench bench;
  bench.send("ESTOP");
  bench.send("DISABLE");
  expect(bench.send("POS"), {"POS A=0 B=0 SA=0 SB=0 NA=0 NB=0 Z=TOP", "ok"}, "POS in ESTOP, drivers disabled");
}

void test_a_negative_angle_is_printed_with_its_sign_and_trailing_zeros_are_dropped() {
  MotionConfig config = bench_config();
  config.alpha_min_deg = -200;
  Bench bench(config);
  bench.send("MOVE A=-90 B=-20 F=20");
  bench.poll();
  bench.hardware.finish();
  expect(bench.send("POS"), {"POS A=-90 B=-20 SA=0 SB=0 NA=-800 NB=-178 Z=TOP", "ok"}, "negative angles");
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_steps_per_degree_follows_the_drive_settings);
  RUN_TEST(test_angles_convert_to_whole_steps);
  RUN_TEST(test_steps_convert_back_to_angles);
  RUN_TEST(test_joint_limits_are_inclusive);
  RUN_TEST(test_without_configured_limits_every_angle_is_allowed);
  RUN_TEST(test_the_speed_and_acceleration_ceilings_are_inclusive_and_zero_means_not_decided);
  RUN_TEST(test_the_project_configuration_is_the_generated_one);
  RUN_TEST(test_the_longer_joint_runs_at_f_and_the_shorter_one_proportionally_slower);
  RUN_TEST(test_both_joints_take_the_same_time_for_unequal_distances);
  RUN_TEST(test_a_triangle_profile_still_finishes_together);
  RUN_TEST(test_a_joint_already_at_its_target_does_not_run);
  RUN_TEST(test_a_move_to_where_both_joints_are_does_nothing);
  RUN_TEST(test_a_tiny_move_never_plans_a_zero_speed_or_acceleration);
  RUN_TEST(test_a_negative_move_plans_the_same_speeds);
  RUN_TEST(test_move_on_a_homed_arm_is_ok_and_starts_both_joints);
  RUN_TEST(test_move_without_homing_is_error_4_unless_the_bench_setting_is_on);
  RUN_TEST(test_the_bench_setting_does_not_skip_the_driver_check);
  RUN_TEST(test_the_bench_setting_does_not_skip_the_estop_check);
  RUN_TEST(test_the_bench_setting_does_not_skip_the_queue_check);
  RUN_TEST(test_a_joint_limit_is_error_2);
  RUN_TEST(test_without_configured_limits_no_joint_limit_is_invented);
  RUN_TEST(test_f_above_the_ceiling_is_error_2_and_the_ceiling_itself_is_fine);
  RUN_TEST(test_an_undecided_speed_ceiling_refuses_every_move);
  RUN_TEST(test_an_undecided_acceleration_ceiling_refuses_every_move_and_accel);
  RUN_TEST(test_error_2_for_limits_comes_before_error_5_6_4_3);
  RUN_TEST(test_the_gate_order_for_an_in_range_move_is_5_then_6_then_4_then_3);
  RUN_TEST(test_the_running_move_counts_toward_the_queue_depth);
  RUN_TEST(test_a_finished_move_frees_a_slot);
  RUN_TEST(test_the_next_move_starts_only_after_the_joints_stopped);
  RUN_TEST(test_moves_run_in_the_order_they_were_sent);
  RUN_TEST(test_a_move_to_the_place_the_joints_already_are_finishes_without_starting_anything);
  RUN_TEST(test_accel_is_ok_and_above_the_ceiling_is_error_2);
  RUN_TEST(test_accel_needs_no_homing_and_no_drivers);
  RUN_TEST(test_accel_is_refused_in_an_estop);
  RUN_TEST(test_a_move_uses_the_accel_that_was_current_when_it_was_queued);
  RUN_TEST(test_before_any_accel_the_ceiling_is_used);
  RUN_TEST(test_wait_on_an_empty_queue_is_ok_then_done_at_once);
  RUN_TEST(test_wait_sends_its_done_only_when_the_queue_has_drained);
  RUN_TEST(test_two_waits_each_get_a_done);
  RUN_TEST(test_a_wait_works_in_every_state_that_accepts_it_but_not_in_an_estop);
  RUN_TEST(test_disable_stops_the_pulses_discards_the_queue_and_cancels_a_pending_wait);
  RUN_TEST(test_estop_stops_a_running_move_and_cancels_the_wait);
  RUN_TEST(test_reset_stops_a_running_move_and_cancels_the_wait);
  RUN_TEST(test_a_heartbeat_timeout_stops_a_running_move_keeps_the_drivers_and_cancels_the_wait);
  RUN_TEST(test_a_move_the_board_cannot_start_is_an_estop_and_never_a_finished_move);
  RUN_TEST(test_a_wait_after_the_clear_finds_an_empty_queue);
  RUN_TEST(test_nothing_starts_after_a_clear_even_if_the_queue_had_more);
  RUN_TEST(test_pos_at_power_up_reports_zero_steps_and_the_z_state_of_the_switches);
  RUN_TEST(test_pos_after_a_move_reports_the_commanded_angles_and_the_steps_issued);
  RUN_TEST(test_pos_while_a_move_runs_follows_the_steps_issued_so_far);
  RUN_TEST(test_pos_after_a_stop_short_of_the_target_reports_where_the_joints_stopped);
  RUN_TEST(test_pos_is_accepted_in_an_estop_and_never_needs_the_drivers_or_homing);
  RUN_TEST(test_a_negative_angle_is_printed_with_its_sign_and_trailing_zeros_are_dropped);
  return UNITY_END();
}
