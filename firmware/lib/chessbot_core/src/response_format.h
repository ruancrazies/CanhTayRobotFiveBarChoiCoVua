// Response formatter (docs/SPEC.md section 5.2). Every function writes ONE line without terminator
// into the caller's buffer, NUL terminated, and returns false (leaving an empty string) when it does
// not fit. The terminator is added by the transport.
#ifndef CHESSBOT_RESPONSE_FORMAT_H
#define CHESSBOT_RESPONSE_FORMAT_H

#include <stddef.h>
#include <stdint.h>

#include "project_headers.h"

namespace chessbot {

// Appends text, characters and integers to a buffer; fails soft (ok() turns false) on overflow.
class LineBuilder {
 public:
  LineBuilder(char* buffer, size_t capacity);

  LineBuilder& add(const char* text);
  LineBuilder& add_char(char c);
  LineBuilder& add_int(long value);
  // A decimal number with at most protocol::kFloatDecimals decimals, a dot, trailing zeros dropped, "-0" written "0":
  // the same text as format_float() in desktop_app/hardware/protocol.py.
  LineBuilder& add_float(double value);

  // "KEY=value" with a leading space, as in "INFO FW=x PROTO=1".
  LineBuilder& add_text_field(const char* key, const char* value);
  LineBuilder& add_int_field(const char* key, long value);
  LineBuilder& add_float_field(const char* key, double value);

  bool ok() const { return ok_; }
  const char* c_str() const { return buffer_; }

 private:
  char* buffer_;
  size_t capacity_;
  size_t length_;
  bool ok_;
};

bool format_ok(char* buffer, size_t capacity);
bool format_done(char* buffer, size_t capacity);
bool format_error(char* buffer, size_t capacity, protocol::ErrorCode code);
bool format_alarm(char* buffer, size_t capacity, protocol::AlarmCode code);
// "INFO FW=<firmware_version> PROTO=<protocol version>"
bool format_info(char* buffer, size_t capacity, const char* firmware_version);

// The values of a POS data line (SPEC 5.1): commanded angles, sensor angles, steps issued, Z state name.
struct PosReport {
  double a_deg;
  double b_deg;
  double sensor_a_deg;
  double sensor_b_deg;
  long steps_a;
  long steps_b;
  const char* z_state;
};
// "POS A=.. B=.. SA=.. SB=.. NA=.. NB=.. Z=..", the fields in the order of CB_DATA_FIELDS.
bool format_pos(char* buffer, size_t capacity, const PosReport& report);

}  // namespace chessbot

#endif  // CHESSBOT_RESPONSE_FORMAT_H
