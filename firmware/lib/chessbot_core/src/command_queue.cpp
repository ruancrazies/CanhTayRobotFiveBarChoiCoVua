#include "command_queue.h"

namespace chessbot {

bool CommandQueue::push(const Command& command, double accel_deg_s2) {
  if (full()) {
    return false;
  }
  const uint8_t slot = static_cast<uint8_t>((head_ + count_) % config::kQueueDepth);
  items_[slot] = command;
  accel_deg_s2_[slot] = accel_deg_s2;
  ++count_;
  return true;
}

bool CommandQueue::peek(Command* out, double* accel_deg_s2) const {
  if (empty()) {
    return false;
  }
  *out = items_[head_];
  if (accel_deg_s2 != nullptr) {
    *accel_deg_s2 = accel_deg_s2_[head_];
  }
  return true;
}

bool CommandQueue::pop(Command* out, double* accel_deg_s2) {
  if (!peek(out, accel_deg_s2)) {
    return false;
  }
  head_ = static_cast<uint8_t>((head_ + 1) % config::kQueueDepth);
  --count_;
  return true;
}

}  // namespace chessbot
