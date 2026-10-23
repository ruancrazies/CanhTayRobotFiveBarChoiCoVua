"""The single declaration of the serial protocol (docs/SPEC.md section 5, SPEC rule 6).

Everything the PC side knows about the wire format is declared here and nowhere else: the
commands and their parameters, the response types, the error codes 1 to 5 and the alarm codes.
``desktop_app/hardware/protocol.py`` (encoder and parser), the ESP32 simulator and the tests all
build on this module. ``firmware/include/protocol.h`` (phase 4) must declare the same thing; the
cross-check test in ``tests/test_protocol_declaration.py`` compares them.

Pure data and the standard library only: no Qt, no serial port, no hardware values. Values the SPEC
marks [TO CONFIRM] (baud rate, speed, acceleration, Z timeout) are NOT here; they are settings of
the code that uses the protocol.

Wire format (SPEC section 5): one line per message, terminated by a newline. A command is
``NAME KEY=value ...``; a flag is a bare token (``HOME Z AB``). Angles are degrees, the decimal
point is a dot, and floats are written with at most ``FLOAT_DECIMALS`` decimals (the joint
resolution is 0.1125 degrees, so 0.001 degree is far below anything the arm can resolve).

Interpretations the SPEC does not spell out (flagged in the phase 3 report; change the SPEC first
if the owner decides otherwise):

* An unknown, duplicate or malformed parameter is ``error:2``, the same as a missing or
  out-of-range one. An unknown command name is ``error:1``.
* ``ESTOP`` is acknowledged by ``!alarm:ESTOP`` alone (that is its Response in the SPEC table),
  not by ``ok`` followed by the alarm.
* ``ZUP``, ``ZDOWN``, ``HOME`` and ``WAIT`` are acknowledged by ``ok`` and later completed by an
  asynchronous ``done``. ``MOVE`` has no ``done``: the PC uses ``WAIT`` to learn that the queue
  drained. ``done`` carries no tag, so the PC keeps only one completion command outstanding.
* ``POS`` and ``INFO`` answer with one data line and then ``ok``. The SPEC lists what each
  contains but not the syntax; the key names below are this declaration's proposal.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from types import MappingProxyType
from typing import Any, Mapping, Optional

PROTOCOL_VERSION = 1  # our own protocol revision, reported by INFO as PROTO=
LINE_TERMINATOR = "\n"
FLOAT_DECIMALS = 3

RESPONSE_OK = "ok"
RESPONSE_DONE = "done"
ERROR_PREFIX = "error:"
ALARM_PREFIX = "!alarm:"


class CommandName(str, Enum):
    """The command names of SPEC 5.1, in the order of the table. Code elsewhere spells a command
    ``CommandName.PING.value`` instead of copying the word."""

    HOME = "HOME"
    MOVE = "MOVE"
    ZUP = "ZUP"
    ZDOWN = "ZDOWN"
    ZSTOP = "ZSTOP"
    ACCEL = "ACCEL"
    GRIP = "GRIP"
    ENABLE = "ENABLE"
    DISABLE = "DISABLE"
    WAIT = "WAIT"
    POS = "POS"
    INFO = "INFO"
    PING = "PING"
    ESTOP = "ESTOP"
    RESET = "RESET"


class ErrorCode(IntEnum):
    """``error:<code>`` (SPEC 5.2)."""

    UNRECOGNIZED = 1  # unrecognized command
    BAD_PARAMETER = 2  # missing or out-of-range parameter
    QUEUE_FULL = 3  # the motion queue is full; the command was NOT queued
    NOT_HOMED = 4  # not homed (for MOVE also: Z not at TOP)
    IN_ESTOP = 5  # the ESTOP flag is set


class AlarmCode(str, Enum):
    """``!alarm:<code>``, sent asynchronously (SPEC 5.2). STEP_LOSS and HB_TIMEOUT are proposed there."""

    ESTOP = "ESTOP"
    LIMIT_Z = "LIMIT_Z"
    Z_TIMEOUT = "Z_TIMEOUT"
    SENSOR_FAULT = "SENSOR_FAULT"
    STEP_LOSS = "STEP_LOSS"
    HB_TIMEOUT = "HB_TIMEOUT"


class ZState(str, Enum):
    """Z axis state reported by POS (SPEC 4.3)."""

    TOP = "TOP"
    BOTTOM = "BOTTOM"
    MID = "MID"


class ParamKind(str, Enum):
    FLOAT = "float"  # KEY=value
    FLAG = "flag"  # bare token


class Completion(str, Enum):
    NONE = "none"  # ok is the whole answer
    DONE = "done"  # ok now, an asynchronous done later


class Ack(str, Enum):
    OK = "ok"
    ESTOP_ALARM = "!alarm:ESTOP"  # ESTOP has no ok: the alarm is its acknowledgement


class FieldKind(str, Enum):
    FLOAT = "float"
    INT = "int"
    TEXT = "text"  # one token, no whitespace


@dataclass(frozen=True)
class ParamSpec:
    """One parameter of a command. ``minimum`` and ``maximum`` are inclusive; ``positive`` means > 0."""

    name: str
    kind: ParamKind
    required: bool = True
    minimum: Optional[float] = None
    maximum: Optional[float] = None
    positive: bool = False


@dataclass(frozen=True)
class CommandSpec:
    name: str
    params: tuple[ParamSpec, ...]
    summary: str
    ack: Ack = Ack.OK
    completion: Completion = Completion.NONE
    data_line: Optional[str] = None  # name of the data line sent before ok (POS, INFO)


@dataclass(frozen=True)
class FieldSpec:
    """One KEY=value field of a data line."""

    name: str
    kind: FieldKind
    choices: Optional[tuple[str, ...]] = None


def _flag(name: str) -> ParamSpec:
    return ParamSpec(name, ParamKind.FLAG, required=False)


def _float(name: str, *, minimum: Optional[float] = None, maximum: Optional[float] = None, positive: bool = False) -> ParamSpec:
    return ParamSpec(name, ParamKind.FLOAT, minimum=minimum, maximum=maximum, positive=positive)


_COMMAND_LIST: tuple[CommandSpec, ...] = (
    CommandSpec(CommandName.HOME.value, (_flag("Z"), _flag("AB")), "Homing; no flag = Z first, then both joints", completion=Completion.DONE),
    CommandSpec(
        CommandName.MOVE.value,
        (_float("A"), _float("B"), _float("F", positive=True)),
        "Queue a move of both joints to alpha (A) and beta (B) at speed F; no Z parameter",
    ),
    CommandSpec(CommandName.ZUP.value, (), "Raise Z until limit #1 triggers", completion=Completion.DONE),
    CommandSpec(CommandName.ZDOWN.value, (), "Lower Z until limit #2 triggers", completion=Completion.DONE),
    CommandSpec(CommandName.ZSTOP.value, (), "Stop and brake Z immediately"),
    CommandSpec(CommandName.ACCEL.value, (_float("S", positive=True),), "Acceleration for the following segments"),
    CommandSpec(CommandName.GRIP.value, (_float("S", minimum=0.0, maximum=180.0),), "Gripper servo angle in degrees"),
    CommandSpec(CommandName.ENABLE.value, (), "Enable both TB6600 drivers"),
    CommandSpec(CommandName.DISABLE.value, (), "Disable both TB6600 drivers"),
    CommandSpec(CommandName.WAIT.value, (), "Wait for the motion queue to drain", completion=Completion.DONE),
    CommandSpec(CommandName.POS.value, (), "Commanded angles, AS5600 angles, steps issued, Z state", data_line="POS"),
    CommandSpec(CommandName.INFO.value, (), "Firmware and protocol version", data_line="INFO"),
    CommandSpec(CommandName.PING.value, (), "Heartbeat from the PC"),
    CommandSpec(CommandName.ESTOP.value, (), "Stop the joints and Z, set the ESTOP flag", ack=Ack.ESTOP_ALARM),
    CommandSpec(CommandName.RESET.value, (), "Clear ESTOP and the alarms; homing is mandatory afterwards"),
)

COMMANDS: Mapping[str, CommandSpec] = MappingProxyType({spec.name: spec for spec in _COMMAND_LIST})
COMMAND_ORDER: tuple[str, ...] = tuple(spec.name for spec in _COMMAND_LIST)  # the order of the SPEC 5.1 table

DATA_LINES: Mapping[str, tuple[FieldSpec, ...]] = MappingProxyType(
    {
        "POS": (
            FieldSpec("A", FieldKind.FLOAT),  # commanded alpha
            FieldSpec("B", FieldKind.FLOAT),  # commanded beta
            FieldSpec("SA", FieldKind.FLOAT),  # AS5600 alpha
            FieldSpec("SB", FieldKind.FLOAT),  # AS5600 beta
            FieldSpec("NA", FieldKind.INT),  # steps issued, joint 1
            FieldSpec("NB", FieldKind.INT),  # steps issued, joint 2
            FieldSpec("Z", FieldKind.TEXT, choices=tuple(state.value for state in ZState)),
        ),
        "INFO": (
            FieldSpec("FW", FieldKind.TEXT),  # firmware version
            FieldSpec("PROTO", FieldKind.INT),  # protocol version
        ),
    }
)


def declaration_as_dict() -> dict[str, Any]:
    """The whole declaration as plain data: what the phase 4 cross-check compares with protocol.h."""
    return {
        "protocol_version": PROTOCOL_VERSION,
        "responses": {"ok": RESPONSE_OK, "done": RESPONSE_DONE, "error_prefix": ERROR_PREFIX, "alarm_prefix": ALARM_PREFIX},
        "error_codes": {code.name: int(code) for code in ErrorCode},
        "alarm_codes": [code.value for code in AlarmCode],
        "z_states": [state.value for state in ZState],
        "commands": {
            spec.name: {
                "params": [
                    {
                        "name": p.name,
                        "kind": p.kind.value,
                        "required": p.required,
                        "minimum": p.minimum,
                        "maximum": p.maximum,
                        "positive": p.positive,
                    }
                    for p in spec.params
                ],
                "ack": spec.ack.value,
                "completion": spec.completion.value,
                "data_line": spec.data_line,
            }
            for spec in _COMMAND_LIST
        },
        "data_lines": {
            name: [{"name": f.name, "kind": f.kind.value, "choices": list(f.choices) if f.choices else None} for f in fields]
            for name, fields in DATA_LINES.items()
        },
    }
