"""Encoder, decoder and parser: round trips, every error code and every alarm (SPEC section 5)."""

from __future__ import annotations

import math

import pytest

from config.protocol_commands import (
    COMMANDS,
    DATA_LINES,
    AlarmCode,
    Completion,
    ErrorCode,
    FieldKind,
    ParamKind,
    ZState,
)
from desktop_app.hardware.protocol import (
    Alarm,
    Command,
    CommandError,
    DataLine,
    Done,
    EncodeError,
    ErrorResponse,
    Ok,
    ParseError,
    UnknownAlarmError,
    decode_command,
    encode_alarm,
    encode_command,
    encode_data_line,
    encode_done,
    encode_error,
    encode_ok,
    format_float,
    frame,
    parse_response,
)

# One valid example per command, written by hand from the SPEC 5.1 table (not derived from the declaration).
SPEC_EXAMPLES = {
    "HOME": ("HOME", {}),
    "MOVE": ("MOVE A=135.4 B=44.6 F=60", {"A": 135.4, "B": 44.6, "F": 60.0}),
    "ZUP": ("ZUP", {}),
    "ZDOWN": ("ZDOWN", {}),
    "ZSTOP": ("ZSTOP", {}),
    "ACCEL": ("ACCEL S=120", {"S": 120.0}),
    "GRIP": ("GRIP S=45.5", {"S": 45.5}),
    "ENABLE": ("ENABLE", {}),
    "DISABLE": ("DISABLE", {}),
    "WAIT": ("WAIT", {}),
    "POS": ("POS", {}),
    "INFO": ("INFO", {}),
    "PING": ("PING", {}),
    "ESTOP": ("ESTOP", {}),
    "RESET": ("RESET", {}),
}


# --------------------------------------------------------------------------- #
# Numbers
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    ("value", "text"),
    [
        (0, "0"),
        (-0.0, "0"),
        (60, "60"),
        (135.4, "135.4"),
        (44.6, "44.6"),
        (-4.3, "-4.3"),
        (1.0006, "1.001"),  # not 1.0005: that is 1.000499999... in binary and correctly rounds down
        (0.0004, "0"),
        (-0.0004, "0"),
        (100.0, "100"),
        (1e-7, "0"),
        (123456.789, "123456.789"),
    ],
)
def test_format_float(value: float, text: str) -> None:
    assert format_float(value) == text


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_format_float_rejects_non_finite(value: float) -> None:
    with pytest.raises(EncodeError):
        format_float(value)


# --------------------------------------------------------------------------- #
# Commands: encoder
# --------------------------------------------------------------------------- #


def test_every_command_in_the_spec_table_is_declared_and_has_an_example() -> None:
    assert set(SPEC_EXAMPLES) == set(COMMANDS)


@pytest.mark.parametrize("name", sorted(SPEC_EXAMPLES))
def test_encoder_matches_the_spec_example(name: str) -> None:
    line, params = SPEC_EXAMPLES[name]
    assert encode_command(name, **params) == line


def test_the_spec_move_example_is_exact() -> None:
    assert encode_command("MOVE", A=135.4, B=44.6, F=60) == "MOVE A=135.4 B=44.6 F=60"


def test_home_flags() -> None:
    assert encode_command("HOME") == "HOME"
    assert encode_command("HOME", Z=True) == "HOME Z"
    assert encode_command("HOME", AB=True) == "HOME AB"
    assert encode_command("HOME", Z=True, AB=True) == "HOME Z AB"
    assert encode_command("HOME", AB=True, Z=True) == "HOME Z AB"  # declaration order, not call order
    assert encode_command("HOME", Z=False, AB=False) == "HOME"


def test_command_object_encodes_itself() -> None:
    command = Command("MOVE", {"A": 1.0, "B": 2.0, "F": 3.0})
    assert command.encode() == str(command) == "MOVE A=1 B=2 F=3"


@pytest.mark.parametrize(
    ("name", "params", "message"),
    [
        ("NOPE", {}, "unknown command"),
        ("move", {"A": 1, "B": 2, "F": 3}, "unknown command"),  # case matters
        ("MOVE", {"A": 1, "B": 2}, "needs F"),
        ("MOVE", {"A": 1, "B": 2, "F": 3, "Z": 1}, "no parameter"),
        ("MOVE", {"A": 1, "B": 2, "F": 0}, "must be > 0"),
        ("MOVE", {"A": 1, "B": 2, "F": -5}, "must be > 0"),
        ("MOVE", {"A": math.nan, "B": 2, "F": 3}, "finite"),
        ("MOVE", {"A": 1, "B": math.inf, "F": 3}, "finite"),
        ("MOVE", {"A": "1", "B": 2, "F": 3}, "must be a number"),
        ("MOVE", {"A": True, "B": 2, "F": 3}, "must be a number"),
        ("GRIP", {"S": -0.1}, ">= 0"),
        ("GRIP", {"S": 180.1}, "<= 180"),
        ("GRIP", {}, "needs S"),
        ("ACCEL", {"S": 0}, "must be > 0"),
        ("HOME", {"Z": 1}, "flag"),
        ("HOME", {"Z": "yes"}, "flag"),
        ("PING", {"X": 1}, "no parameter"),
    ],
)
def test_encoder_refuses_what_the_declaration_forbids(name: str, params: dict, message: str) -> None:
    with pytest.raises(EncodeError, match=message):
        encode_command(name, **params)


@pytest.mark.parametrize("angle", [0, 0.0, 90, 180, 180.0])
def test_grip_accepts_the_whole_declared_range(angle: float) -> None:
    assert encode_command("GRIP", S=angle) == f"GRIP S={format_float(angle)}"


# --------------------------------------------------------------------------- #
# Commands: round trip encode -> decode
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("name", sorted(SPEC_EXAMPLES))
def test_round_trip_for_every_command(name: str) -> None:
    line, params = SPEC_EXAMPLES[name]
    decoded = decode_command(line)
    assert decoded == Command(name, params)
    assert decoded.encode() == line


@pytest.mark.parametrize(
    "a, b, f",
    [(0, 0, 1), (184.3, 62.9, 60), (-4.3, 96.5, 0.5), (117.1, -4.3, 1234.5), (83.5, 55.4, 1e-3), (-180, 180, 100)],
)
def test_move_round_trip_keeps_values_within_the_wire_resolution(a: float, b: float, f: float) -> None:
    decoded = decode_command(encode_command("MOVE", A=a, B=b, F=f))
    assert decoded is not None
    for key, value in (("A", a), ("B", b), ("F", f)):
        assert decoded.params[key] == pytest.approx(round(value, 3), abs=1e-9)
    # Stable: encoding what was decoded gives the same line.
    assert decoded.encode() == encode_command("MOVE", A=a, B=b, F=f)


def test_decoder_ignores_blank_lines_and_extra_spaces() -> None:
    assert decode_command("") is None
    assert decode_command("   ") is None
    assert decode_command("  MOVE   A=1\tB=2   F=3  ") == Command("MOVE", {"A": 1.0, "B": 2.0, "F": 3.0})


def test_decoder_accepts_parameters_in_any_order() -> None:
    assert decode_command("MOVE F=3 B=2 A=1") == Command("MOVE", {"A": 1.0, "B": 2.0, "F": 3.0})
    assert decode_command("HOME AB Z") == Command("HOME", {"Z": True, "AB": True})


# --------------------------------------------------------------------------- #
# Commands: error:1 and error:2 come from the decoder
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("line", ["FOO", "move A=1 B=2 F=3", "MOVEA=1", "=1", "HOME_Z", "123", "ok", "!alarm:ESTOP"])
def test_unknown_command_is_error_1(line: str) -> None:
    with pytest.raises(CommandError) as caught:
        decode_command(line)
    assert caught.value.code is ErrorCode.UNRECOGNIZED


@pytest.mark.parametrize(
    "line",
    [
        "MOVE",  # missing everything
        "MOVE A=1 B=2",  # missing F
        "MOVE B=2 F=3",  # missing A
        "MOVE A=1 B=2 F=0",  # F must be > 0
        "MOVE A=1 B=2 F=-1",
        "MOVE A=nan B=2 F=3",
        "MOVE A=inf B=2 F=3",
        "MOVE A=1e3 B=2 F=3",  # no exponent on the wire
        "MOVE A=1_0 B=2 F=3",
        "MOVE A= B=2 F=3",
        "MOVE A=1,5 B=2 F=3",  # the decimal point is a dot
        "MOVE A=abc B=2 F=3",
        "MOVE A 1 B=2 F=3",
        "MOVE A=1 A=2 B=3 F=1",  # duplicate
        "MOVE A=1 B=2 F=3 Z=0",  # MOVE has no Z parameter
        "GRIP",
        "GRIP S=-1",
        "GRIP S=181",
        "ACCEL S=0",
        "ACCEL",
        "PING NOW",
        "PING X=1",
        "ZUP Z",
        "HOME Z=1",
        "HOME X",
        "HOME Z Z",
    ],
)
def test_bad_parameters_are_error_2(line: str) -> None:
    with pytest.raises(CommandError) as caught:
        decode_command(line)
    assert caught.value.code is ErrorCode.BAD_PARAMETER


# --------------------------------------------------------------------------- #
# Responses: ok, done, every error code, every alarm
# --------------------------------------------------------------------------- #


def test_ok_and_done() -> None:
    assert encode_ok() == "ok" and parse_response("ok") == Ok()
    assert encode_done() == "done" and parse_response("done") == Done()


def test_the_declared_error_codes_are_exactly_1_to_6() -> None:
    assert {int(code) for code in ErrorCode} == {1, 2, 3, 4, 5, 6}
    assert [code.name for code in ErrorCode] == [
        "UNRECOGNIZED", "BAD_PARAMETER", "QUEUE_FULL", "NOT_HOMED", "IN_ESTOP", "DRIVERS_DISABLED",
    ]


@pytest.mark.parametrize("code", list(ErrorCode), ids=lambda c: f"error:{int(c)}")
def test_every_error_code_round_trips(code: ErrorCode) -> None:
    line = encode_error(code)
    assert line == f"error:{int(code)}"
    assert parse_response(line) == ErrorResponse(code)


@pytest.mark.parametrize("line", ["error:0", "error:7", "error:9", "error:-1", "error:", "error:x", "error:3x", "error: 3x", "ERROR:3"])
def test_undeclared_error_codes_are_rejected(line: str) -> None:
    with pytest.raises(ParseError):
        parse_response(line)


def test_the_declared_alarm_codes_are_the_six_of_the_spec() -> None:
    assert [code.value for code in AlarmCode] == ["ESTOP", "LIMIT_Z", "Z_TIMEOUT", "SENSOR_FAULT", "STEP_LOSS", "HB_TIMEOUT"]


@pytest.mark.parametrize("code", list(AlarmCode), ids=lambda c: c.value)
def test_every_alarm_round_trips(code: AlarmCode) -> None:
    line = encode_alarm(code)
    assert line == f"!alarm:{code.value}"
    assert parse_response(line) == Alarm(code)


@pytest.mark.parametrize("raw", ["", "estop", "OVERHEAT", "ESTOP2", " ", "ESTOP "])
def test_an_undeclared_alarm_is_never_dropped_silently(raw: str) -> None:
    # "ESTOP " (trailing space) is stripped by the line parser, so it is the one that must parse.
    if raw == "ESTOP ":
        assert parse_response(f"!alarm:{raw}") == Alarm(AlarmCode.ESTOP)
        return
    with pytest.raises(UnknownAlarmError) as caught:
        parse_response(f"!alarm:{raw}")
    assert caught.value.raw_code == raw.strip()
    assert isinstance(caught.value, ParseError)  # one except clause still catches it


@pytest.mark.parametrize("line", ["", "   ", "OK", "Done", "okay", "ok ok", "hello", "!alarm", "alarm:ESTOP", "!ALARM:ESTOP", "error"])
def test_garbage_is_a_parse_error(line: str) -> None:
    with pytest.raises(ParseError):
        parse_response(line)


def test_surrounding_whitespace_is_tolerated() -> None:
    assert parse_response("  ok \r") == Ok()
    assert parse_response("done\t") == Done()


# --------------------------------------------------------------------------- #
# Data lines (POS and INFO)
# --------------------------------------------------------------------------- #

POS_FIELDS = {"A": 135.4, "B": 44.6, "SA": 135.0, "SB": 45.1, "NA": 12000, "NB": -350, "Z": "MID"}


def test_pos_round_trip() -> None:
    line = encode_data_line("POS", **POS_FIELDS)
    assert line == "POS A=135.4 B=44.6 SA=135 SB=45.1 NA=12000 NB=-350 Z=MID"
    assert parse_response(line) == DataLine("POS", POS_FIELDS)


@pytest.mark.parametrize("state", list(ZState))
def test_pos_reports_every_z_state(state: ZState) -> None:
    line = encode_data_line("POS", **{**POS_FIELDS, "Z": state})
    parsed = parse_response(line)
    assert isinstance(parsed, DataLine) and parsed.fields["Z"] == state.value


def test_info_round_trip() -> None:
    line = encode_data_line("INFO", FW="0.1.0", PROTO=1)
    assert line == "INFO FW=0.1.0 PROTO=1"
    assert parse_response(line) == DataLine("INFO", {"FW": "0.1.0", "PROTO": 1})


@pytest.mark.parametrize(
    "line",
    [
        "POS",
        "POS A=1",
        "POS A=1 B=2 SA=1 SB=2 NA=1 NB=1",  # no Z
        "POS A=1 B=2 SA=1 SB=2 NA=1 NB=1 Z=UP",  # not a Z state
        "POS A=x B=2 SA=1 SB=2 NA=1 NB=1 Z=TOP",
        "POS A=1 B=2 SA=1 SB=2 NA=1.5 NB=1 Z=TOP",  # steps are integers
        "POS A=1 A=1 B=2 SA=1 SB=2 NA=1 NB=1 Z=TOP",
        "POS A=1 B=2 SA=1 SB=2 NA=1 NB=1 Z=TOP EXTRA=1",
        "POS A=1 B=2 SA=1 SB=2 NA=1 NB=1 TOP",
        "INFO FW=x",
        "INFO FW=x PROTO=one",
    ],
)
def test_malformed_data_lines_are_rejected(line: str) -> None:
    with pytest.raises(ParseError):
        parse_response(line)


@pytest.mark.parametrize(
    ("name", "fields"),
    [
        ("NOPE", {}),
        ("POS", {"A": 1}),
        ("POS", {**POS_FIELDS, "Z": "SIDEWAYS"}),
        ("POS", {**POS_FIELDS, "NA": 1.5}),
        ("POS", {**POS_FIELDS, "A": "x"}),
        ("INFO", {"FW": "a b", "PROTO": 1}),
        ("INFO", {"FW": "", "PROTO": 1}),
        ("INFO", {"FW": "a=b", "PROTO": 1}),
    ],
)
def test_data_line_encoder_refuses_invalid_fields(name: str, fields: dict) -> None:
    with pytest.raises(EncodeError):
        encode_data_line(name, **fields)


# --------------------------------------------------------------------------- #
# Framing
# --------------------------------------------------------------------------- #


def test_frame_adds_one_newline_and_refuses_embedded_breaks() -> None:
    assert frame("PING") == "PING\n"
    for bad in ("PI\nNG", "PING\r", "a\r\nb"):
        with pytest.raises(EncodeError):
            frame(bad)


# --------------------------------------------------------------------------- #
# The declaration drives the codec: every command and field kind is exercised
# --------------------------------------------------------------------------- #


def test_every_declared_parameter_kind_and_completion_is_covered_by_an_example() -> None:
    kinds = {p.kind for spec in COMMANDS.values() for p in spec.params}
    assert kinds == {ParamKind.FLOAT, ParamKind.FLAG}
    assert {spec.completion for spec in COMMANDS.values()} == {Completion.NONE, Completion.DONE}
    assert {f.kind for fields in DATA_LINES.values() for f in fields} == {FieldKind.FLOAT, FieldKind.INT, FieldKind.TEXT}


@pytest.mark.parametrize("name", sorted(COMMANDS))
def test_decoder_and_encoder_agree_for_every_declared_command(name: str) -> None:
    """Build a valid parameter set from the declaration alone and round-trip it."""
    params: dict[str, object] = {}
    for p in COMMANDS[name].params:
        if p.kind is ParamKind.FLAG:
            params[p.name] = True
        else:
            low = p.minimum if p.minimum is not None else 1.0
            params[p.name] = max(low, 1.0) if p.positive or p.minimum is None else low
    line = encode_command(name, **params)  # type: ignore[arg-type]
    assert decode_command(line) == Command(name, params)  # type: ignore[arg-type]
