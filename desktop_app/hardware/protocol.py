"""Encoder for commands, decoder for commands, parser for responses and alarms (docs/SPEC.md section 5).

Everything here is driven by the declaration in ``config/protocol_commands.py``; this module
holds no command name, parameter name, code or range of its own. Pure standard library: no Qt, no
serial port, no hardware constants.

* :func:`encode_command` builds the line the PC sends (no terminator; framing adds it).
* :func:`decode_command` is the other direction. The ESP32 simulator uses it, so the simulator
  and the encoder cannot drift apart, and it raises :class:`CommandError` carrying the ``error:``
  code the firmware would answer with.
* :func:`parse_response` turns one line from the ESP32 into :class:`Ok`, :class:`Done`,
  :class:`ErrorResponse`, :class:`Alarm` or :class:`DataLine`. Anything else is a
  :class:`ParseError`. An alarm whose code is not declared is :class:`UnknownAlarmError`: it is
  never dropped silently, because an alarm is a safety message.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Optional, Union

from config.protocol_commands import (
    ALARM_PREFIX,
    COMMANDS,
    DATA_LINES,
    ERROR_PREFIX,
    FLOAT_DECIMALS,
    LINE_TERMINATOR,
    RESPONSE_DONE,
    RESPONSE_OK,
    AlarmCode,
    CommandSpec,
    ErrorCode,
    FieldKind,
    FieldSpec,
    ParamKind,
    ParamSpec,
)

ParamValue = Union[float, int, bool]
FieldValue = Union[float, int, str]


class ProtocolError(ValueError):
    """Base class of every protocol error."""


class EncodeError(ProtocolError):
    """The PC tried to build a command the declaration does not allow. A bug in the caller."""


class CommandError(ProtocolError):
    """A received command line is invalid; ``code`` is the ``error:<code>`` to answer with."""

    def __init__(self, code: ErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code


class ParseError(ProtocolError):
    """A line from the ESP32 is not a valid response."""


class UnknownAlarmError(ParseError):
    """``!alarm:<code>`` with a code that is not declared. The caller must treat it as a fault."""

    def __init__(self, raw_code: str) -> None:
        super().__init__(f"unknown alarm code {raw_code!r}")
        self.raw_code = raw_code


# --------------------------------------------------------------------------- #
# Values
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Command:
    """One command: its name and its parameters (floats for ``KEY=value``, True for a flag)."""

    name: str
    params: Mapping[str, ParamValue]

    def encode(self) -> str:
        return encode_command(self.name, **self.params)

    def __str__(self) -> str:
        return self.encode()


@dataclass(frozen=True)
class Ok:
    pass


@dataclass(frozen=True)
class Done:
    pass


@dataclass(frozen=True)
class ErrorResponse:
    code: ErrorCode


@dataclass(frozen=True)
class Alarm:
    code: AlarmCode


@dataclass(frozen=True)
class DataLine:
    """The data line of POS or INFO: ``name`` is ``"POS"`` or ``"INFO"``."""

    name: str
    fields: Mapping[str, FieldValue]


Response = Union[Ok, Done, ErrorResponse, Alarm, DataLine]


# --------------------------------------------------------------------------- #
# Numbers
# --------------------------------------------------------------------------- #


def format_float(value: float) -> str:
    """Decimal text with a dot and at most ``FLOAT_DECIMALS`` decimals, no exponent, no ``-0``."""
    number = float(value)
    if not math.isfinite(number):
        raise EncodeError(f"cannot encode the non-finite number {value!r}")
    text = f"{number:.{FLOAT_DECIMALS}f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def _parse_number(text: str) -> float:
    """A finite decimal number. ``float()`` also accepts nan, inf and underscores; the wire does not."""
    if not text or any(ch not in "0123456789+-." for ch in text):
        raise ValueError(f"not a decimal number: {text!r}")
    number = float(text)
    if not math.isfinite(number):
        raise ValueError(f"not a finite number: {text!r}")
    return number


def _violation(spec: ParamSpec, value: float) -> Optional[str]:
    if spec.positive and not value > 0:
        return f"{spec.name} must be > 0, got {value:g}"
    if spec.minimum is not None and value < spec.minimum:
        return f"{spec.name} must be >= {spec.minimum:g}, got {value:g}"
    if spec.maximum is not None and value > spec.maximum:
        return f"{spec.name} must be <= {spec.maximum:g}, got {value:g}"
    return None


def _spec_of(name: str) -> CommandSpec:
    try:
        return COMMANDS[name]
    except KeyError:
        raise EncodeError(f"unknown command {name!r}") from None


# --------------------------------------------------------------------------- #
# Commands: encode and decode
# --------------------------------------------------------------------------- #


def encode_command(name: str, **params: ParamValue) -> str:
    """The line for ``name`` and its parameters, for example ``MOVE A=135.4 B=44.6 F=60``.

    Parameters are written in declaration order. A flag is included when its value is true.

    Raises:
        EncodeError: unknown command or parameter, a missing required parameter, a value outside
            the declared range, or a value of the wrong kind.
    """
    spec = _spec_of(name)
    known = {p.name: p for p in spec.params}
    if unknown := sorted(set(params) - set(known)):
        raise EncodeError(f"{name} has no parameter {unknown}; it takes {list(known) or 'none'}")
    tokens = [name]
    for param in spec.params:
        if param.name not in params:
            if param.required:
                raise EncodeError(f"{name} needs {param.name}")
            continue
        value = params[param.name]
        if param.kind is ParamKind.FLAG:
            if not isinstance(value, bool):
                raise EncodeError(f"{name} {param.name} is a flag: pass True or False, got {value!r}")
            if value:
                tokens.append(param.name)
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise EncodeError(f"{name} {param.name} must be a number, got {value!r}")
        if not math.isfinite(value):
            raise EncodeError(f"{name} {param.name} must be finite, got {value!r}")
        if problem := _violation(param, float(value)):
            raise EncodeError(f"{name}: {problem}")
        tokens.append(f"{param.name}={format_float(value)}")
    return " ".join(tokens)


def decode_command(line: str) -> Optional[Command]:
    """Parse a command line the way the firmware does. A blank line is ignored (returns None).

    Raises:
        CommandError: ``code`` is UNRECOGNIZED for an unknown command name, and BAD_PARAMETER for
            an unknown, duplicate, missing, malformed or out-of-range parameter.
    """
    tokens = line.split()
    if not tokens:
        return None
    name, rest = tokens[0], tokens[1:]
    spec = COMMANDS.get(name)
    if spec is None:
        raise CommandError(ErrorCode.UNRECOGNIZED, f"unrecognized command {name!r}")
    known = {p.name: p for p in spec.params}
    values: dict[str, ParamValue] = {}
    for token in rest:
        key, equals, text = token.partition("=")
        param = known.get(key)
        if param is None:
            raise CommandError(ErrorCode.BAD_PARAMETER, f"{name}: unknown parameter {key!r}")
        if key in values:
            raise CommandError(ErrorCode.BAD_PARAMETER, f"{name}: duplicate parameter {key!r}")
        if param.kind is ParamKind.FLAG:
            if equals:
                raise CommandError(ErrorCode.BAD_PARAMETER, f"{name}: {key} is a flag and takes no value")
            values[key] = True
            continue
        if not equals:
            raise CommandError(ErrorCode.BAD_PARAMETER, f"{name}: {key} needs a value, write {key}=<number>")
        try:
            number = _parse_number(text)
        except ValueError as exc:
            raise CommandError(ErrorCode.BAD_PARAMETER, f"{name}: {key}: {exc}") from None
        if problem := _violation(param, number):
            raise CommandError(ErrorCode.BAD_PARAMETER, f"{name}: {problem}")
        values[key] = number
    for param in spec.params:
        if param.required and param.name not in values:
            raise CommandError(ErrorCode.BAD_PARAMETER, f"{name}: missing parameter {param.name}")
    return Command(name, values)


# --------------------------------------------------------------------------- #
# Responses: encode (used by the simulator) and parse
# --------------------------------------------------------------------------- #


def encode_ok() -> str:
    return RESPONSE_OK


def encode_done() -> str:
    return RESPONSE_DONE


def encode_error(code: ErrorCode) -> str:
    return f"{ERROR_PREFIX}{int(code)}"


def encode_alarm(code: AlarmCode) -> str:
    return f"{ALARM_PREFIX}{code.value}"


def encode_data_line(name: str, **fields: FieldValue) -> str:
    """``POS A=.. B=..`` or ``INFO FW=.. PROTO=..``: every declared field, in declaration order."""
    specs = DATA_LINES.get(name)
    if specs is None:
        raise EncodeError(f"unknown data line {name!r}")
    if set(fields) != {f.name for f in specs}:
        raise EncodeError(f"{name} needs exactly the fields {[f.name for f in specs]}, got {sorted(fields)}")
    tokens = [name]
    for spec in specs:
        value = fields[spec.name]
        if spec.kind is FieldKind.FLOAT:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise EncodeError(f"{name} {spec.name} must be a number, got {value!r}")
            text = format_float(value)
        elif spec.kind is FieldKind.INT:
            if isinstance(value, bool) or not isinstance(value, int):
                raise EncodeError(f"{name} {spec.name} must be an integer, got {value!r}")
            text = str(value)
        else:
            text = str(getattr(value, "value", value))
            if not text or any(ch.isspace() for ch in text) or "=" in text:
                raise EncodeError(f"{name} {spec.name} must be one token without spaces or '=', got {text!r}")
            if spec.choices is not None and text not in spec.choices:
                raise EncodeError(f"{name} {spec.name} must be one of {list(spec.choices)}, got {text!r}")
        tokens.append(f"{spec.name}={text}")
    return " ".join(tokens)


def _parse_field(line_name: str, spec: FieldSpec, text: str) -> FieldValue:
    try:
        if spec.kind is FieldKind.FLOAT:
            return _parse_number(text)
        if spec.kind is FieldKind.INT:
            if not text.lstrip("+-").isdigit():
                raise ValueError(f"not an integer: {text!r}")
            return int(text)
    except ValueError as exc:
        raise ParseError(f"{line_name} {spec.name}: {exc}") from None
    if spec.choices is not None and text not in spec.choices:
        raise ParseError(f"{line_name} {spec.name} must be one of {list(spec.choices)}, got {text!r}")
    return text


def parse_response(line: str) -> Response:
    """Parse one line from the ESP32 (without its terminator).

    Raises:
        UnknownAlarmError: ``!alarm:`` with an undeclared code.
        ParseError: anything that is not exactly one declared response.
    """
    text = line.strip()
    if text == RESPONSE_OK:
        return Ok()
    if text == RESPONSE_DONE:
        return Done()
    if text.startswith(ERROR_PREFIX):
        raw = text[len(ERROR_PREFIX):]
        try:
            return ErrorResponse(ErrorCode(int(raw)))
        except ValueError:
            raise ParseError(f"unknown error code {raw!r}") from None
    if text.startswith(ALARM_PREFIX):
        raw = text[len(ALARM_PREFIX):]
        try:
            return Alarm(AlarmCode(raw))
        except ValueError:
            raise UnknownAlarmError(raw) from None
    tokens = text.split()
    specs = DATA_LINES.get(tokens[0]) if tokens else None
    if specs is None:
        raise ParseError(f"not a response: {line!r}")
    pairs = [token.partition("=") for token in tokens[1:]]
    if any(not equals for _key, equals, _value in pairs):
        raise ParseError(f"{tokens[0]}: every field must be KEY=value, got {line!r}")
    given = {key: value for key, _equals, value in pairs}
    if len(given) != len(pairs):
        raise ParseError(f"{tokens[0]}: duplicate field in {line!r}")
    if set(given) != {f.name for f in specs}:
        raise ParseError(f"{tokens[0]} needs exactly the fields {[f.name for f in specs]}, got {sorted(given)}")
    return DataLine(tokens[0], {f.name: _parse_field(tokens[0], f, given[f.name]) for f in specs})


def frame(line: str) -> str:
    """``line`` with the line terminator, as it goes on the wire."""
    if LINE_TERMINATOR in line or "\r" in line:
        raise EncodeError("a protocol line cannot contain a line break")
    return line + LINE_TERMINATOR
