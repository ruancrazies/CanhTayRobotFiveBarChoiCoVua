// Joint motion rules of docs/SPEC.md sections 5, 6.1 and 10.2, as plain arithmetic with no hardware access:
// angle <-> step conversion, the soft limits, the speed and acceleration ceilings, and the plan that makes both
// joints finish together. The processor decides WHEN to call them; the board glue (src/main.cpp) only turns a
// JointMove into FastAccelStepper calls.
//
// Every number comes from a MotionConfig, which the firmware fills from the generated config.h
// (MotionConfig::from_project()) and the host tests fill by hand.
#ifndef CHESSBOT_MOTION_H
#define CHESSBOT_MOTION_H

#include <stdint.h>

#include "project_headers.h"

namespace chessbot {

// The two joints: alpha (motor 1) and beta (motor 2).
constexpr uint8_t kJointCount = 2;
constexpr uint8_t kJointAlpha = 0;
constexpr uint8_t kJointBeta = 1;

struct MotionConfig {
  uint16_t full_steps_per_rev;  // drive.full_steps_per_rev (the "200" of SPEC 6.1)
  uint16_t microstepping;       // drive.microstepping (f)
  double gear_ratio;            // drive.gear_ratio (i)
  double max_speed_deg_s;       // motion.max_speed_deg_s; 0 = not decided: every MOVE is refused (error:2)
  double max_accel_deg_s2;      // motion.max_accel_deg_s2; 0 = not decided: every ACCEL and MOVE is refused (error:2)
  bool limits_configured;       // all four soft limits are known
  double alpha_min_deg;
  double alpha_max_deg;
  double beta_min_deg;
  double beta_max_deg;
  bool bench_move_without_homing;  // BENCH ONLY: a MOVE needs no homed flag (motion.bench_move_without_homing)

  // The values of the generated config.h.
  static MotionConfig from_project();

  // SPEC 6.1: steps = 200 * f * i * angle / 360, so this is 200 * f * i / 360.
  double steps_per_degree() const;
  // F of a MOVE is allowed: positive (the parser checks that) and not above the ceiling.
  bool speed_allowed(double deg_per_s) const;
  // S of an ACCEL is allowed.
  bool accel_allowed(double deg_per_s2) const;
  // Both target angles are inside the soft limits (inclusive); always true when no limit is configured.
  bool within_limits(double alpha_deg, double beta_deg) const;
};

// Step counts are relative to the power-up position until homing exists (stage 4b-5): step 0 is angle 0.
int32_t angle_to_steps(const MotionConfig& config, double angle_deg);
double steps_to_angle(const MotionConfig& config, int32_t steps);

// What the board has to do for one MOVE: absolute target step counts and, per joint, the speed and acceleration
// that make both joints arrive together (the joint with the longer way runs at F, the other proportionally slower).
struct JointMove {
  int32_t target_steps[kJointCount];
  uint32_t speed_hz[kJointCount];         // steps per second, at least 1 for a joint that runs
  uint32_t accel_steps_s2[kJointCount];   // steps per second squared, at least 1 for a joint that runs
  bool runs[kJointCount];                 // false: the joint is already at its target
  bool any() const { return runs[kJointAlpha] || runs[kJointBeta]; }
};

JointMove plan_joint_move(const MotionConfig& config, const int32_t current_steps[kJointCount], double alpha_deg, double beta_deg,
                          double speed_deg_s, double accel_deg_s2);

}  // namespace chessbot

#endif  // CHESSBOT_MOTION_H
