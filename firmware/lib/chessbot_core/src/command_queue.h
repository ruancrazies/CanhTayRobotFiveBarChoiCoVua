// Bounded FIFO of parsed commands (docs/SPEC.md section 5.2: error:3 when the queue is full). A push
// into a full queue changes nothing: the command is NOT queued. Fixed storage, no heap.
#ifndef CHESSBOT_COMMAND_QUEUE_H
#define CHESSBOT_COMMAND_QUEUE_H

#include <stdint.h>

#include "command_parser.h"
#include "project_headers.h"

namespace chessbot {

class CommandQueue {
 public:
  CommandQueue() : head_(0), count_(0) {}

  static constexpr uint8_t capacity() { return config::kQueueDepth; }

  uint8_t size() const { return count_; }
  bool empty() const { return count_ == 0; }
  bool full() const { return count_ >= config::kQueueDepth; }

  // false when full (nothing is stored). ``accel_deg_s2`` travels with the command: a MOVE runs with the ACCEL that was
  // current when it was queued (SPEC 5.1: "acceleration for following segments").
  bool push(const Command& command, double accel_deg_s2 = 0);
  // false when empty. ``accel_deg_s2`` may be null.
  bool pop(Command* out, double* accel_deg_s2 = nullptr);
  // The oldest command, left in the queue (the running MOVE stays queued until it has finished, so it still counts toward
  // the depth). false when empty.
  bool peek(Command* out, double* accel_deg_s2 = nullptr) const;
  void clear() {
    head_ = 0;
    count_ = 0;
  }

 private:
  Command items_[config::kQueueDepth];
  double accel_deg_s2_[config::kQueueDepth];
  uint8_t head_;
  uint8_t count_;
};

}  // namespace chessbot

#endif  // CHESSBOT_COMMAND_QUEUE_H
