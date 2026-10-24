#include "command_queue.h"

namespace chessbot {

bool CommandQueue::push(const Command& command) {
  if (full()) {
    return false;
  }
  items_[(head_ + count_) % config::kQueueDepth] = command;
  ++count_;
  return true;
}

bool CommandQueue::pop(Command* out) {
  if (empty()) {
    return false;
  }
  *out = items_[head_];
  head_ = static_cast<uint8_t>((head_ + 1) % config::kQueueDepth);
  --count_;
  return true;
}

}  // namespace chessbot
