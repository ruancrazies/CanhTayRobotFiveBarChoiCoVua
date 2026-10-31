#include "response_format.h"

#include <stdio.h>
#include <string.h>

namespace chessbot {

LineBuilder::LineBuilder(char* buffer, size_t capacity) : buffer_(buffer), capacity_(capacity), length_(0), ok_(capacity > 0) {
  if (capacity_ > 0) {
    buffer_[0] = '\0';
  }
}

LineBuilder& LineBuilder::add(const char* text) {
  if (!ok_) {
    return *this;
  }
  const size_t n = strlen(text);
  if (length_ + n + 1 > capacity_) {
    ok_ = false;
    buffer_[0] = '\0';
    return *this;
  }
  memcpy(buffer_ + length_, text, n + 1);
  length_ += n;
  return *this;
}

LineBuilder& LineBuilder::add_char(char c) {
  const char text[2] = {c, '\0'};
  return add(text);
}

LineBuilder& LineBuilder::add_int(long value) {
  // 21 characters hold the sign and the digits of a 64-bit long.
  char digits[24];
  size_t i = sizeof(digits) - 1;
  digits[i] = '\0';
  const bool negative = value < 0;
  unsigned long magnitude = negative ? 0UL - static_cast<unsigned long>(value) : static_cast<unsigned long>(value);
  do {
    digits[--i] = static_cast<char>('0' + magnitude % 10);
    magnitude /= 10;
  } while (magnitude != 0);
  if (negative) {
    digits[--i] = '-';
  }
  return add(digits + i);
}

LineBuilder& LineBuilder::add_float(double value) {
  char text[40];  // room for 15 digits, a sign, the dot and the decimals
  const int written = snprintf(text, sizeof(text), "%.*f", protocol::kFloatDecimals, value);
  if (written < 0 || static_cast<size_t>(written) >= sizeof(text)) {
    ok_ = false;
    buffer_[0] = '\0';
    return *this;
  }
  size_t length = static_cast<size_t>(written);
  if (memchr(text, '.', length) != nullptr) {  // drop the trailing zeros, then a bare dot (the Python rstrip("0").rstrip("."))
    while (length > 0 && text[length - 1] == '0') {
      --length;
    }
    if (length > 0 && text[length - 1] == '.') {
      --length;
    }
  }
  text[length] = '\0';
  if (strcmp(text, "-0") == 0 || length == 0) {
    return add("0");
  }
  return add(text);
}

LineBuilder& LineBuilder::add_text_field(const char* key, const char* value) {
  return add_char(' ').add(key).add_char('=').add(value);
}

LineBuilder& LineBuilder::add_int_field(const char* key, long value) {
  return add_char(' ').add(key).add_char('=').add_int(value);
}

LineBuilder& LineBuilder::add_float_field(const char* key, double value) {
  return add_char(' ').add(key).add_char('=').add_float(value);
}

bool format_ok(char* buffer, size_t capacity) {
  LineBuilder line(buffer, capacity);
  return line.add(protocol::kResponseOk).ok();
}

bool format_done(char* buffer, size_t capacity) {
  LineBuilder line(buffer, capacity);
  return line.add(protocol::kResponseDone).ok();
}

bool format_error(char* buffer, size_t capacity, protocol::ErrorCode code) {
  LineBuilder line(buffer, capacity);
  return line.add(protocol::kErrorPrefix).add_int(protocol::number_of(code)).ok();
}

bool format_alarm(char* buffer, size_t capacity, protocol::AlarmCode code) {
  LineBuilder line(buffer, capacity);
  return line.add(protocol::kAlarmPrefix).add(protocol::name_of(code)).ok();
}

bool format_info(char* buffer, size_t capacity, const char* firmware_version) {
  LineBuilder line(buffer, capacity);
  line.add(protocol::name_of(protocol::Cmd::kINFO));
  // The fields come from the declared INFO data line (protocol.h), in order: the TEXT field carries the
  // firmware version and the INT field the protocol version.
  for (uint8_t i = 0; i < protocol::kDataFieldCount; ++i) {
    const protocol::DataFieldSpec& field = protocol::kDataFields[i];
    if (field.line != protocol::DataLine::kINFO) {
      continue;
    }
    if (field.kind == protocol::FieldKind::kTEXT) {
      line.add_text_field(field.name, firmware_version);
    } else if (field.kind == protocol::FieldKind::kINT) {
      line.add_int_field(field.name, protocol::kProtocolVersion);
    } else {
      return false;  // INFO declares no other kind
    }
  }
  return line.ok();
}

bool format_pos(char* buffer, size_t capacity, const PosReport& report) {
  LineBuilder line(buffer, capacity);
  line.add(protocol::name_of(protocol::Cmd::kPOS));
  // The fields come from the declared POS data line (protocol.h), in order; each is looked up by its name.
  for (uint8_t i = 0; i < protocol::kDataFieldCount; ++i) {
    const protocol::DataFieldSpec& field = protocol::kDataFields[i];
    if (field.line != protocol::DataLine::kPOS) {
      continue;
    }
    if (strcmp(field.name, "A") == 0) {
      line.add_float_field(field.name, report.a_deg);
    } else if (strcmp(field.name, "B") == 0) {
      line.add_float_field(field.name, report.b_deg);
    } else if (strcmp(field.name, "SA") == 0) {
      line.add_float_field(field.name, report.sensor_a_deg);
    } else if (strcmp(field.name, "SB") == 0) {
      line.add_float_field(field.name, report.sensor_b_deg);
    } else if (strcmp(field.name, "NA") == 0) {
      line.add_int_field(field.name, report.steps_a);
    } else if (strcmp(field.name, "NB") == 0) {
      line.add_int_field(field.name, report.steps_b);
    } else if (strcmp(field.name, "Z") == 0) {
      line.add_text_field(field.name, report.z_state);
    } else {
      return false;  // a POS field the formatter does not know: the declaration and this function disagree
    }
  }
  return line.ok();
}

}  // namespace chessbot
