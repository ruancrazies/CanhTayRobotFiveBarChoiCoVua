// The serial protocol, declared once for the firmware (docs/SPEC.md section 5, SPEC rule 6).
//
// This header MUST say exactly what config/protocol_commands.py says. tests/test_protocol_declaration.py
// parses this file and compares it with the Python declaration; change both together.
//
// The lists below are X-macro tables, one entry per line, so that the C++ code and the cross-check test
// read the same text. The names are the wire names. Enumerators are written k<NAME> (token pasting keeps
// a macro defined by some Arduino/ESP header from rewriting a protocol word).
//
// Plain C++, no Arduino include: host tests include this file.
#ifndef CHESSBOT_PROTOCOL_H
#define CHESSBOT_PROTOCOL_H

#include <stdint.h>

namespace chessbot {
namespace protocol {

// ---- Wire format ------------------------------------------------------------------------------
constexpr int kProtocolVersion = 1;  // reported by INFO as PROTO=
constexpr char kLineTerminator = '\n';
constexpr int kFloatDecimals = 3;

constexpr char kResponseOk[] = "ok";
constexpr char kResponseDone[] = "done";
constexpr char kErrorPrefix[] = "error:";
constexpr char kAlarmPrefix[] = "!alarm:";

// ---- Error codes: error:<code> (SPEC 5.2) -----------------------------------------------------
// X(NAME, number)
#define CB_ERROR_CODES(X)  \
  X(UNRECOGNIZED, 1)       \
  X(BAD_PARAMETER, 2)      \
  X(QUEUE_FULL, 3)         \
  X(NOT_HOMED, 4)          \
  X(IN_ESTOP, 5)

enum class ErrorCode : uint8_t {
#define CB_ERROR_ENUM(name, number) k##name = number,
  CB_ERROR_CODES(CB_ERROR_ENUM)
#undef CB_ERROR_ENUM
};

// ---- Alarm codes: !alarm:<code>, sent asynchronously (SPEC 5.2) -------------------------------
// X(NAME), in the order of the Python AlarmCode enum.
#define CB_ALARM_CODES(X) \
  X(ESTOP)                \
  X(LIMIT_Z)              \
  X(Z_TIMEOUT)            \
  X(SENSOR_FAULT)         \
  X(STEP_LOSS)            \
  X(HB_TIMEOUT)

enum class AlarmCode : uint8_t {
#define CB_ALARM_ENUM(name) k##name,
  CB_ALARM_CODES(CB_ALARM_ENUM)
#undef CB_ALARM_ENUM
  kCount
};

constexpr uint8_t kAlarmCount = static_cast<uint8_t>(AlarmCode::kCount);

constexpr const char* const kAlarmNames[] = {
#define CB_ALARM_NAME(name) #name,
    CB_ALARM_CODES(CB_ALARM_NAME)
#undef CB_ALARM_NAME
};

// ---- Z axis states reported by POS (SPEC 4.3) -------------------------------------------------
#define CB_Z_STATES(X) \
  X(TOP)               \
  X(BOTTOM)            \
  X(MID)

constexpr const char* const kZStateNames[] = {
#define CB_Z_NAME(name) #name,
    CB_Z_STATES(CB_Z_NAME)
#undef CB_Z_NAME
};

// ---- Commands (SPEC 5.1), in the order of the SPEC table --------------------------------------
// X(NAME, completion, ack, data_line)
//   completion: NONE (ok is the whole answer) | DONE (ok now, an asynchronous done later)
//   ack:        OK | ESTOP_ALARM (ESTOP is acknowledged by !alarm:ESTOP alone, no ok)
//   data_line:  NONE | POS | INFO (one data line is sent before ok)
#define CB_COMMANDS(X)             \
  X(HOME, DONE, OK, NONE)          \
  X(MOVE, NONE, OK, NONE)          \
  X(ZUP, DONE, OK, NONE)           \
  X(ZDOWN, DONE, OK, NONE)         \
  X(ZSTOP, NONE, OK, NONE)         \
  X(ACCEL, NONE, OK, NONE)         \
  X(GRIP, NONE, OK, NONE)          \
  X(ENABLE, NONE, OK, NONE)        \
  X(DISABLE, NONE, OK, NONE)       \
  X(WAIT, DONE, OK, NONE)          \
  X(POS, NONE, OK, POS)            \
  X(INFO, NONE, OK, INFO)          \
  X(PING, NONE, OK, NONE)          \
  X(ESTOP, NONE, ESTOP_ALARM, NONE) \
  X(RESET, NONE, OK, NONE)

enum class Cmd : uint8_t {
#define CB_CMD_ENUM(name, completion, ack, data) k##name,
  CB_COMMANDS(CB_CMD_ENUM)
#undef CB_CMD_ENUM
  kCount
};

enum class Completion : uint8_t { kNONE, kDONE };
enum class Ack : uint8_t { kOK, kESTOP_ALARM };
enum class DataLine : uint8_t { kNONE, kPOS, kINFO };

struct CommandSpec {
  const char* name;
  Completion completion;
  Ack ack;
  DataLine data_line;
};

constexpr CommandSpec kCommands[] = {
#define CB_CMD_ROW(name, completion, ack, data) \
  {#name, Completion::k##completion, Ack::k##ack, DataLine::k##data},
    CB_COMMANDS(CB_CMD_ROW)
#undef CB_CMD_ROW
};

constexpr uint8_t kCommandCount = static_cast<uint8_t>(Cmd::kCount);

// ---- Command parameters -----------------------------------------------------------------------
// X(COMMAND, NAME, kind, required, positive, has_min, min, has_max, max)
//   kind: FLOAT (KEY=value) | FLAG (bare token). min and max are inclusive and count only when has_min /
//   has_max is 1. positive = 1 means the value must be > 0.
#define CB_PARAMS(X)                                    \
  X(HOME, Z, FLAG, 0, 0, 0, 0, 0, 0)                    \
  X(HOME, AB, FLAG, 0, 0, 0, 0, 0, 0)                   \
  X(MOVE, A, FLOAT, 1, 0, 0, 0, 0, 0)                   \
  X(MOVE, B, FLOAT, 1, 0, 0, 0, 0, 0)                   \
  X(MOVE, F, FLOAT, 1, 1, 0, 0, 0, 0)                   \
  X(ACCEL, S, FLOAT, 1, 1, 0, 0, 0, 0)                  \
  X(GRIP, S, FLOAT, 1, 0, 1, 0, 1, 180)

enum class ParamKind : uint8_t { kFLOAT, kFLAG };

struct ParamSpec {
  Cmd command;
  const char* name;
  ParamKind kind;
  bool required;
  bool positive;
  bool has_min;
  double min_value;
  bool has_max;
  double max_value;
};

constexpr ParamSpec kParams[] = {
#define CB_PARAM_ROW(cmd, name, kind, required, positive, has_min, lo, has_max, hi) \
  {Cmd::k##cmd, #name, ParamKind::k##kind, (required) != 0, (positive) != 0, (has_min) != 0, (lo), (has_max) != 0, (hi)},
    CB_PARAMS(CB_PARAM_ROW)
#undef CB_PARAM_ROW
};

constexpr uint8_t kParamCount = sizeof(kParams) / sizeof(kParams[0]);

// The most parameters any one command takes (MOVE: A, B, F). A unit test and the Python cross-check
// verify this against the table above.
constexpr uint8_t kMaxParamsPerCommand = 3;

// ---- Data lines (POS and INFO) ----------------------------------------------------------------
// X(LINE, FIELD, kind): the fields of each data line, in order. kind: FLOAT | INT | TEXT.
// Z is one of the CB_Z_STATES names.
#define CB_DATA_FIELDS(X) \
  X(POS, A, FLOAT)        \
  X(POS, B, FLOAT)        \
  X(POS, SA, FLOAT)       \
  X(POS, SB, FLOAT)       \
  X(POS, NA, INT)         \
  X(POS, NB, INT)         \
  X(POS, Z, TEXT)         \
  X(INFO, FW, TEXT)       \
  X(INFO, PROTO, INT)

enum class FieldKind : uint8_t { kFLOAT, kINT, kTEXT };

struct DataFieldSpec {
  DataLine line;
  const char* name;
  FieldKind kind;
};

constexpr DataFieldSpec kDataFields[] = {
#define CB_FIELD_ROW(line, name, kind) {DataLine::k##line, #name, FieldKind::k##kind},
    CB_DATA_FIELDS(CB_FIELD_ROW)
#undef CB_FIELD_ROW
};

constexpr uint8_t kDataFieldCount = sizeof(kDataFields) / sizeof(kDataFields[0]);

// ---- Small lookups ----------------------------------------------------------------------------
inline const CommandSpec& spec_of(Cmd command) { return kCommands[static_cast<uint8_t>(command)]; }
inline const char* name_of(Cmd command) { return spec_of(command).name; }
inline const char* name_of(AlarmCode alarm) { return kAlarmNames[static_cast<uint8_t>(alarm)]; }
inline uint8_t number_of(ErrorCode code) { return static_cast<uint8_t>(code); }

}  // namespace protocol
}  // namespace chessbot

#endif  // CHESSBOT_PROTOCOL_H
