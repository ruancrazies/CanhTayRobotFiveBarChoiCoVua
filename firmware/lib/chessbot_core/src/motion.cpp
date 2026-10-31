#include "motion.h"

#include <math.h>

namespace chessbot {

namespace {

constexpr double kDegreesPerTurn = 360.0;  // mathematics, not a setting

int64_t absolute(int64_t value) { return value < 0 ? -value : value; }

// Whole steps per second (or per second squared) from a real value; never 0, because FastAccelStepper refuses 0
// and a joint that has to run must be able to.
uint32_t at_least_one(double value) {
  const long rounded = lround(value);
  return rounded < 1 ? 1U : static_cast<uint32_t>(rounded);
}

}  // namespace

MotionConfig MotionConfig::from_project() {
  MotionConfig config;
  config.full_steps_per_rev = config::kFullStepsPerRev;
  config.microstepping = config::kMicrostepping;
  config.gear_ratio = static_cast<double>(config::kGearRatio);
  config.max_speed_deg_s = static_cast<double>(config::kMaxSpeedDegPerS);
  config.max_accel_deg_s2 = static_cast<double>(config::kMaxAccelDegPerS2);
  config.limits_configured = config::kJointLimitsConfigured;
  config.alpha_min_deg = static_cast<double>(config::kJointAlphaMinDeg);
  config.alpha_max_deg = static_cast<double>(config::kJointAlphaMaxDeg);
  config.beta_min_deg = static_cast<double>(config::kJointBetaMinDeg);
  config.beta_max_deg = static_cast<double>(config::kJointBetaMaxDeg);
  config.bench_move_without_homing = config::kBenchMoveWithoutHoming;
  return config;
}

double MotionConfig::steps_per_degree() const {
  return static_cast<double>(full_steps_per_rev) * static_cast<double>(microstepping) * gear_ratio / kDegreesPerTurn;
}

bool MotionConfig::speed_allowed(double deg_per_s) const { return max_speed_deg_s > 0 && deg_per_s <= max_speed_deg_s; }

bool MotionConfig::accel_allowed(double deg_per_s2) const { return max_accel_deg_s2 > 0 && deg_per_s2 <= max_accel_deg_s2; }

bool MotionConfig::within_limits(double alpha_deg, double beta_deg) const {
  if (!limits_configured) {
    return true;
  }
  return alpha_deg >= alpha_min_deg && alpha_deg <= alpha_max_deg && beta_deg >= beta_min_deg && beta_deg <= beta_max_deg;
}

int32_t angle_to_steps(const MotionConfig& config, double angle_deg) {
  return static_cast<int32_t>(lround(angle_deg * config.steps_per_degree()));
}

double steps_to_angle(const MotionConfig& config, int32_t steps) { return static_cast<double>(steps) / config.steps_per_degree(); }

JointMove plan_joint_move(const MotionConfig& config, const int32_t current_steps[kJointCount], double alpha_deg, double beta_deg,
                          double speed_deg_s, double accel_deg_s2) {
  const double targets_deg[kJointCount] = {alpha_deg, beta_deg};
  int64_t distance[kJointCount];
  int64_t longest = 0;
  JointMove move;
  for (uint8_t joint = 0; joint < kJointCount; ++joint) {
    move.target_steps[joint] = angle_to_steps(config, targets_deg[joint]);
    distance[joint] = absolute(static_cast<int64_t>(move.target_steps[joint]) - static_cast<int64_t>(current_steps[joint]));
    move.runs[joint] = distance[joint] != 0;
    move.speed_hz[joint] = 0;
    move.accel_steps_s2[joint] = 0;
    if (distance[joint] > longest) {
      longest = distance[joint];
    }
  }
  // The joint with the longer way runs at F and the full acceleration; the other one at the same fraction of both, so a
  // trapezoid of the same duration: t = d / v + v / a does not change when d, v and a are all scaled by one factor.
  const double top_speed = speed_deg_s * config.steps_per_degree();
  const double top_accel = accel_deg_s2 * config.steps_per_degree();
  for (uint8_t joint = 0; joint < kJointCount; ++joint) {
    if (!move.runs[joint]) {
      continue;
    }
    const double share = static_cast<double>(distance[joint]) / static_cast<double>(longest);
    move.speed_hz[joint] = at_least_one(top_speed * share);
    move.accel_steps_s2[joint] = at_least_one(top_accel * share);
  }
  return move;
}

}  // namespace chessbot
