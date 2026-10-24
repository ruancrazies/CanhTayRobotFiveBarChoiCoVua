// Command line parser, table driven by protocol.h. Behaves like decode_command() in
// desktop_app/hardware/protocol.py:
//  * a blank line is ignored;
//  * an unknown command name is error:1 (names are case sensitive);
//  * an unknown, duplicate, missing, malformed or out-of-range parameter is error:2.
#ifndef CHESSBOT_COMMAND_PARSER_H
#define CHESSBOT_COMMAND_PARSER_H

#include <stdint.h>

#include "project_headers.h"

namespace chessbot {

struct Command {
  protocol::Cmd id;
  // Slot i belongs to the i-th parameter of this command in protocol::kParams.
  bool present[protocol::kMaxParamsPerCommand];
  double values[protocol::kMaxParamsPerCommand];  // FLOAT parameters; unused for flags

  Command();
  explicit Command(protocol::Cmd command);

  // True when the parameter ``name`` was given (always true for a required one in a parsed command).
  bool has(const char* name) const;
  // Value of a FLOAT parameter that was given, else 0.
  double value(const char* name) const;
};

enum class ParseStatus : uint8_t {
  kOk,
  kEmpty,         // blank line: no reply
  kUnrecognized,  // error:1
  kBadParameter,  // error:2
};

ParseStatus parse_command(const char* line, Command* out);

// The command named by ``length`` characters at ``name``; false when there is no such command.
bool find_command(const char* name, uint16_t length, protocol::Cmd* out);

}  // namespace chessbot

#endif  // CHESSBOT_COMMAND_PARSER_H
