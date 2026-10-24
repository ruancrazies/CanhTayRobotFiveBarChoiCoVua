#include "line_framer.h"

namespace chessbot {

namespace {
constexpr uint8_t kNewline = '\n';
constexpr uint8_t kCarriageReturn = '\r';
constexpr uint8_t kFirstPrintable = 0x20;
constexpr uint8_t kLastPrintable = 0x7E;
}  // namespace

void LineFramer::reset() {
  length_ = 0;
  discarding_ = false;
  buffer_[0] = '\0';
}

LineFramer::Result LineFramer::feed(uint8_t input) {
  if (input == kNewline) {
    if (discarding_) {
      discarding_ = false;
      length_ = 0;
      return Result::kNone;
    }
    uint16_t end = length_;
    length_ = 0;
    while (end > 0 && static_cast<uint8_t>(buffer_[end - 1]) == kCarriageReturn) {
      --end;
    }
    if (end == 0) {
      return Result::kNone;
    }
    for (uint16_t i = 0; i < end; ++i) {
      const uint8_t c = static_cast<uint8_t>(buffer_[i]);
      if (c < kFirstPrintable || c > kLastPrintable) {
        return Result::kError;
      }
    }
    buffer_[end] = '\0';
    return Result::kLine;
  }
  if (discarding_) {
    return Result::kNone;
  }
  if (length_ >= config::kMaxLineBytes) {
    length_ = 0;
    discarding_ = true;
    return Result::kError;
  }
  buffer_[length_++] = static_cast<char>(input);
  return Result::kNone;
}

}  // namespace chessbot
