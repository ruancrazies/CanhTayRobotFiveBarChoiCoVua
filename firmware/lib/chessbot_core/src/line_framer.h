// Bytes to lines (docs/SPEC.md section 5). Same rules as the PC side LineFramer in
// desktop_app/hardware/serial_link.py, so both ends frame identically:
//  * a line ends at '\n'; any number of trailing '\r' is dropped;
//  * an empty line is dropped without a result;
//  * a line longer than config::kMaxLineBytes is ONE error; the rest of that line is discarded;
//  * a line with a byte outside printable ASCII (0x20 to 0x7E) is an error.
#ifndef CHESSBOT_LINE_FRAMER_H
#define CHESSBOT_LINE_FRAMER_H

#include <stdint.h>

#include "project_headers.h"

namespace chessbot {

class LineFramer {
 public:
  enum class Result : uint8_t {
    kNone,   // nothing complete yet (or a dropped empty line)
    kLine,   // a good line: text()
    kError,  // an unusable line: the caller answers error:1
  };

  LineFramer() { reset(); }

  void reset();

  // Feed one received byte. Each byte yields at most one result.
  Result feed(uint8_t input);

  // The last good line, NUL terminated, without terminator. Valid until the next feed().
  const char* text() const { return buffer_; }

 private:
  char buffer_[config::kMaxLineBytes + 1];
  uint16_t length_;
  bool discarding_;
};

}  // namespace chessbot

#endif  // CHESSBOT_LINE_FRAMER_H
