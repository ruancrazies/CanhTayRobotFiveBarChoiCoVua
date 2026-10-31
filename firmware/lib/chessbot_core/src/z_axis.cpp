#include "z_axis.h"

#include <math.h>

namespace chessbot {

namespace {

constexpr uint32_t kPercentFull = 100;  // mathematics: a duty and a travel share are percentages

}  // namespace

ZConfig ZConfig::from_project() {
  ZConfig config;
  config.full_travel_ms = config::kZFullTravelMs;
  config.timeout_factor = config::kZTimeoutFactor;
  config.debounce_ms = config::kDebounceMs;
  config.duty_start_percent = config::kZDutyStartPercent;
  config.duty_peak_percent = config::kZDutyPeakPercent;
  config.duty_ramp_ms = config::kZDutyRampMs;
  config.duty_eased_percent = config::kZDutyEasedPercent;
  config.ease_after_travel_percent = config::kZEaseAfterTravelPercent;
  return config;
}

uint32_t ZConfig::timeout_ms() const {
  return static_cast<uint32_t>(lround(static_cast<double>(timeout_factor) * static_cast<double>(full_travel_ms)));
}

uint8_t ZConfig::duty_percent(uint32_t elapsed_ms) const {
  const uint64_t ease_at_ms = static_cast<uint64_t>(full_travel_ms) * ease_after_travel_percent / kPercentFull;
  if (elapsed_ms >= ease_at_ms) {
    return duty_eased_percent;
  }
  if (duty_ramp_ms == 0 || elapsed_ms >= duty_ramp_ms) {
    return duty_peak_percent;
  }
  const int64_t rise = static_cast<int64_t>(duty_peak_percent) - static_cast<int64_t>(duty_start_percent);
  return static_cast<uint8_t>(static_cast<int64_t>(duty_start_percent) + rise * static_cast<int64_t>(elapsed_ms) / duty_ramp_ms);
}

uint32_t z_duty_to_ticks(uint8_t duty_percent, uint8_t resolution_bits) {
  const uint64_t full_scale = (static_cast<uint64_t>(1) << resolution_bits) - 1;
  const uint64_t percent = duty_percent > kPercentFull ? kPercentFull : duty_percent;
  return static_cast<uint32_t>(full_scale * percent / kPercentFull);
}

uint32_t z_full_scale_ticks(uint8_t resolution_bits) { return z_duty_to_ticks(static_cast<uint8_t>(kPercentFull), resolution_bits); }

bool LimitDebouncer::update(bool raw_actuated, uint32_t now_ms) {
  if (raw_actuated == stable_) {
    pending_ = false;  // back to the stable value: the change that was building up was a bounce
  } else if (!pending_) {
    pending_ = true;
    since_ms_ = now_ms;
  } else if (static_cast<uint32_t>(now_ms - since_ms_) >= debounce_ms_) {
    stable_ = raw_actuated;
    pending_ = false;
  }
  return stable_;
}

ZAxis::ZAxis(const ZConfig& config)
    : config_(config),
      top_(config.debounce_ms),
      bottom_(config.debounce_ms),
      drive_(ZDrive::kCoast),
      moving_(false),
      toward_top_(false),
      limit_reached_(false),
      limit_z_latched_(false),
      top_was_stable_(false),
      release_expected_(false),
      release_grace_running_(false),
      release_grace_since_ms_(0),
      started_ms_(0),
      duty_(0) {}

ZAxis::StartResult ZAxis::start(bool toward_top, uint32_t now_ms) {
  if (!config_.configured()) {
    return StartResult::kNotConfigured;
  }
  const protocol::ZState at = state();
  if ((toward_top && at == protocol::ZState::kTOP) || (!toward_top && at == protocol::ZState::kBOTTOM)) {
    if (moving_) {
      stop();  // this command replaces the running move, and the carriage is where it is told to be
    }
    return StartResult::kAlreadyThere;
  }
  drive_ = toward_top ? ZDrive::kRaise : ZDrive::kLower;
  toward_top_ = toward_top;
  if (!toward_top) {
    release_expected_ = true;
    release_grace_running_ = false;
  }
  moving_ = true;
  limit_reached_ = false;
  started_ms_ = now_ms;
  duty_ = config_.duty_percent(0);
  return StartResult::kStarted;
}

void ZAxis::stop() {
  drive_ = ZDrive::kBrake;
  moving_ = false;
  limit_reached_ = false;
}

void ZAxis::reset() {
  stop();
  limit_z_latched_ = false;
}

ZEvent ZAxis::poll(uint32_t now_ms, bool top_actuated, bool bottom_actuated) {
  top_.update(top_actuated, now_ms);
  bottom_.update(bottom_actuated, now_ms);
  const bool top_stable = top_.stable();
  const bool bottom_stable = bottom_.stable();
  const bool was_at_top = top_was_stable_;
  top_was_stable_ = top_stable;

  // The raw reading brakes the motor at once, as the limit-switch interrupt does a moment earlier on the board. The move
  // is only reported done when the debounced switch confirms it.
  if (moving_ && !limit_reached_ && z_must_stop_now(drive_, top_actuated, bottom_actuated)) {
    drive_ = ZDrive::kBrake;
    limit_reached_ = true;
  }

  // A lowering move is the command that lets the top switch release. Once no lowering move runs any more, the release stays
  // expected for one more debounce window: it may have started just before the move ended.
  if (moving_ && !toward_top_) {
    release_grace_running_ = false;
  } else if (release_expected_) {
    if (!release_grace_running_) {
      release_grace_running_ = true;
      release_grace_since_ms_ = now_ms;
    } else if (static_cast<uint32_t>(now_ms - release_grace_since_ms_) > config_.debounce_ms) {
      release_expected_ = false;
      release_grace_running_ = false;
    }
  }

  // LIMIT_Z (SPEC 5.2, 10.2): both switches actuated, or Z left TOP although nobody commanded it.
  // Reported once: after it the device is in an emergency stop, and the releases that follow are not new faults.
  if (!limit_z_latched_ && ((top_stable && bottom_stable) || (was_at_top && !top_stable && !release_expected_))) {
    limit_z_latched_ = true;
    brake_now();
    return ZEvent::kLimitZ;
  }

  if (!moving_) {
    return ZEvent::kNone;
  }
  if (limit_reached_ && (toward_top_ ? top_stable : bottom_stable)) {
    moving_ = false;
    return ZEvent::kDone;
  }
  if (!limit_reached_) {
    duty_ = config_.duty_percent(static_cast<uint32_t>(now_ms - started_ms_));
  }
  if (static_cast<uint32_t>(now_ms - started_ms_) >= config_.timeout_ms()) {
    brake_now();
    return ZEvent::kTimeout;
  }
  return ZEvent::kNone;
}

protocol::ZState ZAxis::state() const {
  if (top_.stable() && !bottom_.stable()) {
    return protocol::ZState::kTOP;
  }
  if (bottom_.stable() && !top_.stable()) {
    return protocol::ZState::kBOTTOM;
  }
  return protocol::ZState::kMID;
}

uint8_t ZAxis::duty_percent() const {
  switch (drive_) {
    case ZDrive::kBrake:
      return static_cast<uint8_t>(kPercentFull);
    case ZDrive::kCoast:
      return 0;
    case ZDrive::kRaise:
    case ZDrive::kLower:
      break;
  }
  return duty_;
}

}  // namespace chessbot
