// Bounded FIFO of parsed commands (docs/SPEC.md section 5.2: error:3 when the queue is full). A push
// into a full queue changes nothing: the command is NOT queued. Fixed storage, no heap.
#ifndef CHESSBOT_COMMAND_QUEUE_H
#define CHESSBOT_COMMAND_QUEUE_H

#include <stdint.h>

#include "command_parser.h"
#include "config.h"

namespace chessbot {

class CommandQueue {
 public:
  CommandQueue() : head_(0), count_(0) {}

  static constexpr uint8_t capacity() { return config::kQueueDepth; }

  uint8_t size() const { return count_; }
  bool empty() const { return count_ == 0; }
  bool full() const { return count_ >= config::kQueueDepth; }

  // false when full (nothing is stored).
  bool push(const Command& command);
  // false when empty.
  bool pop(Command* out);
  void clear() {
    head_ = 0;
    count_ = 0;
  }

 private:
  Command items_[config::kQueueDepth];
  uint8_t head_;
  uint8_t count_;
};

}  // namespace chessbot

#endif  // CHESSBOT_COMMAND_QUEUE_H
