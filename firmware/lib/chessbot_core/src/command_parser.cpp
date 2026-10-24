#include "command_parser.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "project_headers.h"

namespace chessbot {

using protocol::Cmd;
using protocol::ParamKind;
using protocol::ParamSpec;

namespace {

// The same set Python's str.split() treats as separators in printable-plus-control ASCII.
bool is_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r'; }

bool is_digit(char c) { return c >= '0' && c <= '9'; }

// The wire number grammar: [+-] digits [. digits], at least one digit, nothing else. (Python side:
// _parse_number allows only the characters 0-9 + - . and then float(); no exponent, nan or inf.)
bool parse_number(const char* text, uint16_t length, double* out) {
  if (length == 0 || length > config::kMaxLineBytes) {
    return false;
  }
  uint16_t i = 0;
  if (text[i] == '+' || text[i] == '-') {
    ++i;
  }
  uint16_t digits = 0;
  while (i < length && is_digit(text[i])) {
    ++i;
    ++digits;
  }
  if (i < length && text[i] == '.') {
    ++i;
    while (i < length && is_digit(text[i])) {
      ++i;
      ++digits;
    }
  }
  if (digits == 0 || i != length) {
    return false;
  }
  char copy[config::kMaxLineBytes + 1];
  memcpy(copy, text, length);
  copy[length] = '\0';
  const double value = strtod(copy, nullptr);
  if (!isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

bool violates(const ParamSpec& spec, double value) {
  if (spec.positive && !(value > 0.0)) {
    return true;
  }
  if (spec.has_min && value < spec.min_value) {
    return true;
  }
  if (spec.has_max && value > spec.max_value) {
    return true;
  }
  return false;
}

// Index of parameter ``name`` of ``command`` within its slots, or -1. ``spec`` receives the table row.
int find_param(Cmd command, const char* name, uint16_t length, const ParamSpec** spec) {
  int slot = 0;
  for (uint8_t i = 0; i < protocol::kParamCount; ++i) {
    const ParamSpec& row = protocol::kParams[i];
    if (row.command != command) {
      continue;
    }
    if (strlen(row.name) == length && strncmp(row.name, name, length) == 0) {
      if (spec != nullptr) {
        *spec = &row;
      }
      return slot;
    }
    ++slot;
  }
  return -1;
}

}  // namespace

Command::Command() : id(Cmd::kPING) {
  for (uint8_t i = 0; i < protocol::kMaxParamsPerCommand; ++i) {
    present[i] = false;
    values[i] = 0.0;
  }
}

Command::Command(Cmd command) : Command() { id = command; }

bool Command::has(const char* name) const {
  const int slot = find_param(id, name, static_cast<uint16_t>(strlen(name)), nullptr);
  return slot >= 0 && slot < protocol::kMaxParamsPerCommand && present[slot];
}

double Command::value(const char* name) const {
  const int slot = find_param(id, name, static_cast<uint16_t>(strlen(name)), nullptr);
  return (slot >= 0 && slot < protocol::kMaxParamsPerCommand && present[slot]) ? values[slot] : 0.0;
}

bool find_command(const char* name, uint16_t length, Cmd* out) {
  for (uint8_t i = 0; i < protocol::kCommandCount; ++i) {
    const char* candidate = protocol::kCommands[i].name;
    if (strlen(candidate) == length && strncmp(candidate, name, length) == 0) {
      *out = static_cast<Cmd>(i);
      return true;
    }
  }
  return false;
}

ParseStatus parse_command(const char* line, Command* out) {
  const char* p = line;
  while (is_space(*p)) {
    ++p;
  }
  if (*p == '\0') {
    return ParseStatus::kEmpty;
  }
  const char* name = p;
  while (*p != '\0' && !is_space(*p)) {
    ++p;
  }
  Cmd id;
  if (!find_command(name, static_cast<uint16_t>(p - name), &id)) {
    return ParseStatus::kUnrecognized;
  }

  Command command(id);
  for (;;) {
    while (is_space(*p)) {
      ++p;
    }
    if (*p == '\0') {
      break;
    }
    const char* token = p;
    while (*p != '\0' && !is_space(*p)) {
      ++p;
    }
    const uint16_t token_length = static_cast<uint16_t>(p - token);
    const char* equals = static_cast<const char*>(memchr(token, '=', token_length));
    const uint16_t key_length = equals != nullptr ? static_cast<uint16_t>(equals - token) : token_length;

    const ParamSpec* spec = nullptr;
    const int slot = find_param(id, token, key_length, &spec);
    if (slot < 0 || slot >= protocol::kMaxParamsPerCommand) {
      return ParseStatus::kBadParameter;  // unknown parameter
    }
    if (command.present[slot]) {
      return ParseStatus::kBadParameter;  // duplicate
    }
    if (spec->kind == ParamKind::kFLAG) {
      if (equals != nullptr) {
        return ParseStatus::kBadParameter;  // a flag takes no value
      }
      command.present[slot] = true;
      continue;
    }
    if (equals == nullptr) {
      return ParseStatus::kBadParameter;  // needs KEY=value
    }
    double number = 0.0;
    const char* text = equals + 1;
    if (!parse_number(text, static_cast<uint16_t>(token + token_length - text), &number) || violates(*spec, number)) {
      return ParseStatus::kBadParameter;
    }
    command.values[slot] = number;
    command.present[slot] = true;
  }

  int slot = 0;
  for (uint8_t i = 0; i < protocol::kParamCount; ++i) {
    const ParamSpec& row = protocol::kParams[i];
    if (row.command != id) {
      continue;
    }
    if (row.required && !command.present[slot]) {
      return ParseStatus::kBadParameter;  // missing
    }
    ++slot;
  }
  *out = command;
  return ParseStatus::kOk;
}

}  // namespace chessbot
