"""The protocol is declared once (SPEC rule 6): config/protocol_commands.py.

* The declaration is compared with docs/SPEC.md section 5, parsed from the SPEC text itself.
* The codec and the simulator may not carry a copy of the declaration (static scan).
* firmware/include/protocol.h is parsed and compared with the declaration: command names and order,
  error codes, alarm codes, Z states, response words, each command's parameters (kind, required, range),
  completion and acknowledgement, the data lines and their field order. The header parser is itself
  tested with deliberately wrong headers, so the comparison cannot pass by finding nothing.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest
import yaml

from config import protocol_commands as declaration
from config.protocol_commands import (
    ACCEPTED_IN_ESTOP,
    COMMAND_ORDER,
    COMMANDS,
    AlarmCode,
    CommandSpec,
    ErrorCode,
    declaration_as_dict,
)

# The owner's decision (SPEC 5.1 and 10.2), typed out here on purpose so that it is not derived from the very
# declaration it is checked against. In the order of the SPEC 5.1 table.
OWNER_ESTOP_SET = ("ZSTOP", "DISABLE", "POS", "INFO", "PING", "ESTOP", "RESET")

REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC = REPO_ROOT / "docs" / "SPEC.md"
PROTOCOL_H = REPO_ROOT / "firmware" / "include" / "protocol.h"
HARDWARE_DIR = REPO_ROOT / "desktop_app" / "hardware"


def spec_section(number: str) -> str:
    """The text of ``### <number>`` up to the next heading of the same or a higher level."""
    text = SPEC.read_text(encoding="utf-8")
    match = re.search(rf"^### {re.escape(number)} .*?$(.*?)(?=^#{{2,3}} )", text, re.S | re.M)
    assert match, f"SPEC section {number} not found"
    return match.group(1)


# --------------------------------------------------------------------------- #
# Declaration against the SPEC text
# --------------------------------------------------------------------------- #


def spec_command_names() -> list[str]:
    names: list[str] = []
    for row in spec_section("5.1").splitlines():
        cells = [c.strip() for c in row.strip().strip("|").split("|")]
        if len(cells) < 3 or cells[0] in ("Command", "---") or set(cells[0]) <= {"-", " "}:
            continue
        # "HOME [Z] [AB]" -> HOME, "ENABLE / DISABLE" -> ENABLE, DISABLE
        names.extend(re.findall(r"\b[A-Z]{2,}\b", re.split(r"\s[\[<]|\sA=|\sS=", cells[0])[0]))
    return names


def test_commands_match_the_spec_table_in_order() -> None:
    assert spec_command_names() == list(COMMAND_ORDER)
    assert list(COMMANDS) == list(COMMAND_ORDER)


def test_error_codes_match_the_spec_text() -> None:
    line = next(l for l in spec_section("5.2").splitlines() if l.startswith("- `error:<code>`"))
    codes = {int(n) for n in re.findall(r"(\d)\s+[a-z]", line.split(":", 2)[2])}
    assert codes == {int(code) for code in ErrorCode} == {1, 2, 3, 4, 5}


def test_alarm_codes_match_the_spec_text() -> None:
    line = next(l for l in spec_section("5.2").splitlines() if l.startswith("- `!alarm:<code>`"))
    spec_codes = re.findall(r"`([A-Z_]+)`", line)
    assert spec_codes == [code.value for code in AlarmCode]


def test_spec_parameters_match_the_declaration() -> None:
    rows = {
        row.split("|")[1].split()[0]: row
        for row in spec_section("5.1").splitlines()
        if row.startswith("| ") and not row.startswith("| Command") and not row.startswith("| ---")
    }
    assert "A=<α> B=<β> F=<speed>" in rows["MOVE"]
    assert [p.name for p in COMMANDS["MOVE"].params] == ["A", "B", "F"]
    assert "S=<0–180>" in rows["GRIP"] and (COMMANDS["GRIP"].params[0].minimum, COMMANDS["GRIP"].params[0].maximum) == (0.0, 180.0)
    assert "S=<deg/s²>" in rows["ACCEL"] and [p.name for p in COMMANDS["ACCEL"].params] == ["S"]
    assert "[Z] [AB]" in rows["HOME"] and [p.name for p in COMMANDS["HOME"].params] == ["Z", "AB"]
    for name in set(COMMANDS) - {"MOVE", "GRIP", "ACCEL", "HOME"}:
        assert COMMANDS[name].params == (), f"{name} takes no parameters in the SPEC"


def test_completion_matches_the_spec_response_column() -> None:
    rows = {
        row.split("|")[1].split()[0]: row.split("|")[3]
        for row in spec_section("5.1").splitlines()
        if row.startswith("| ") and not row.startswith("| Command") and not row.startswith("| ---")
    }
    for name in ("HOME", "ZUP", "ZDOWN", "WAIT"):
        assert "then done" in rows[name], name
        assert COMMANDS[name].completion.value == "done", name
    for name in set(COMMANDS) - {"HOME", "ZUP", "ZDOWN", "WAIT"}:
        assert COMMANDS[name].completion.value == "none", name
    assert COMMANDS["ESTOP"].ack.value == "!alarm:ESTOP"
    assert "!alarm:ESTOP" in rows["ESTOP"]
    assert "one line, then ok" in rows["POS"] and COMMANDS["POS"].data_line == "POS"
    assert "one line, then ok" in rows["INFO"] and COMMANDS["INFO"].data_line == "INFO"


# --------------------------------------------------------------------------- #
# The ESTOP rules (SPEC 5.1, 5.2, 10.2)
# --------------------------------------------------------------------------- #


def _estop_set_in(section: str, pattern: str) -> list[str]:
    match = re.search(pattern, spec_section(section), re.S)
    assert match, f"SPEC {section} does not state which commands are accepted in an ESTOP"
    return re.findall(r"\b[A-Z]{2,}\b", match.group(1))


def test_the_declared_estop_set_is_the_owner_decision() -> None:
    assert ACCEPTED_IN_ESTOP == OWNER_ESTOP_SET
    assert [name for name in COMMAND_ORDER if COMMANDS[name].accepted_in_estop] == list(ACCEPTED_IN_ESTOP)
    assert "ENABLE" not in ACCEPTED_IN_ESTOP  # refused in an ESTOP until RESET
    assert "DISABLE" in ACCEPTED_IN_ESTOP  # accepted in an ESTOP
    assert set(ACCEPTED_IN_ESTOP) < set(COMMANDS)


def test_a_command_nobody_listed_is_refused_in_an_estop() -> None:
    assert CommandSpec("NEW", (), "a future command").accepted_in_estop is False


def test_the_spec_5_1_names_the_same_estop_set() -> None:
    names = _estop_set_in("5.1", r"\*\*While the ESTOP flag is set\*\* only (.+?) are accepted\.")
    assert sorted(names) == sorted(ACCEPTED_IN_ESTOP)
    assert "Every other command, including ENABLE, is answered `error:5`; ENABLE works again after RESET." in spec_section("5.1")


def test_the_spec_10_2_names_the_same_estop_set() -> None:
    names = _estop_set_in("10.2", r"While the flag is set only (.+?) are accepted;")
    assert sorted(names) == sorted(ACCEPTED_IN_ESTOP)


def test_the_spec_says_disable_clears_the_homed_flag_and_estop_may_repeat() -> None:
    table = spec_section("5.1")
    disable_row = next(row for row in table.splitlines() if row.startswith("| ENABLE / DISABLE"))
    assert "DISABLE also clears the homed flag" in disable_row
    estop_row = next(row for row in table.splitlines() if row.startswith("| ESTOP"))
    assert "while already in ESTOP" in estop_row and "!alarm:ESTOP" in estop_row
    assert "A repeated ESTOP while already in ESTOP is accepted and harmless" in table
    assert "DISABLE clears the homed flag, so homing is mandatory after it" in spec_section("10.2")
    assert "After DISABLE, homing is mandatory" in spec_section("5.3")


# --------------------------------------------------------------------------- #
# The declaration is internally consistent and holds no hardware value
# --------------------------------------------------------------------------- #


def test_declaration_is_plain_serialisable_data() -> None:
    import json

    data = declaration_as_dict()
    assert json.loads(json.dumps(data)) == data
    assert list(data["commands"]) == list(COMMAND_ORDER)
    assert data["error_codes"] == {"UNRECOGNIZED": 1, "BAD_PARAMETER": 2, "QUEUE_FULL": 3, "NOT_HOMED": 4, "IN_ESTOP": 5}
    assert data["alarm_codes"] == ["ESTOP", "LIMIT_Z", "Z_TIMEOUT", "SENSOR_FAULT", "STEP_LOSS", "HB_TIMEOUT"]
    assert data["accepted_in_estop"] == list(OWNER_ESTOP_SET)


def test_every_data_line_named_by_a_command_is_declared() -> None:
    for spec in COMMANDS.values():
        if spec.data_line is not None:
            assert spec.data_line in declaration.DATA_LINES


def test_the_declaration_is_read_only() -> None:
    with pytest.raises(TypeError):
        COMMANDS["NEW"] = COMMANDS["PING"]  # type: ignore[index]
    with pytest.raises(TypeError):
        declaration.DATA_LINES["NEW"] = ()  # type: ignore[index]


def test_the_declaration_module_is_pure_standard_library() -> None:
    tree = ast.parse(Path(declaration.__file__).read_text(encoding="utf-8"))
    roots = {
        (n.module or "").split(".")[0] if isinstance(n, ast.ImportFrom) else alias.name.split(".")[0]
        for n in ast.walk(tree)
        if isinstance(n, (ast.Import, ast.ImportFrom))
        for alias in (n.names if isinstance(n, ast.Import) else [None])
    }
    assert roots <= {"__future__", "dataclasses", "enum", "types", "typing"}, roots


def test_the_declaration_adds_no_hardware_value() -> None:
    """The only numbers are the protocol's own: error codes, the 0-180 servo range, the float decimals."""
    tree = ast.parse(Path(declaration.__file__).read_text(encoding="utf-8"))
    numbers = {
        float(n.value)
        for n in ast.walk(tree)
        if isinstance(n, ast.Constant) and isinstance(n.value, (int, float)) and not isinstance(n.value, bool)
    }
    assert numbers <= {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 180.0}, numbers
    hardware = yaml.safe_load((REPO_ROOT / "config" / "hardware.yaml").read_text(encoding="utf-8"))
    assert "protocol" not in hardware and "baud" not in str(hardware).lower()  # baud is [TO CONFIRM]: not invented here


# --------------------------------------------------------------------------- #
# Nobody else keeps a copy of the declaration
# --------------------------------------------------------------------------- #


def _string_constants(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)}


@pytest.mark.parametrize("module", sorted(p for p in HARDWARE_DIR.glob("*.py") if p.name != "__init__.py"), ids=lambda p: p.name)
def test_no_module_copies_a_command_alarm_or_response_name(module: Path) -> None:
    """Command names, alarm names and response words are spelled only in config/protocol_commands.py.

    Docstrings and log text may mention them; a standalone string that EQUALS one is a copy.
    """
    forbidden = set(COMMANDS) | {code.value for code in AlarmCode} | {"ok", "done", "error:", "!alarm:"}
    copies = sorted(_string_constants(module) & forbidden)
    assert copies == [], f"{module.name} spells declared protocol words itself: {copies}"


# --------------------------------------------------------------------------- #
# firmware/include/protocol.h against the declaration
# --------------------------------------------------------------------------- #


def _strip_comments(text: str) -> str:
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def _macro_rows(text: str, macro: str) -> list[list[str]]:
    """The ``X(a, b, c)`` entries of ``#define <macro>(X) ...`` as lists of stripped arguments."""
    lines = _strip_comments(text).splitlines()
    start = next((i for i, line in enumerate(lines) if re.match(rf"\s*#define\s+{macro}\(X\)", line)), None)
    assert start is not None, f"protocol.h has no #define {macro}(X)"
    body: list[str] = []
    for line in lines[start:]:
        body.append(line.rstrip().rstrip("\\"))
        if not line.rstrip().endswith("\\"):
            break
    rows = re.findall(r"\bX\(([^()]*)\)", " ".join(body))
    return [[argument.strip() for argument in row.split(",")] for row in rows]


def _constant(text: str, name: str) -> str:
    match = re.search(rf"constexpr\s+\w+\s+{name}(?:\[\])?\s*=\s*(.+?);", _strip_comments(text))
    assert match, f"protocol.h does not define {name}"
    return match.group(1).strip()


def _number(text: str) -> float:
    return float(text)


def header_declaration(text: str) -> dict[str, object]:
    """What protocol.h says, in the shape of :func:`comparable_declaration`."""
    commands: dict[str, dict[str, object]] = {}
    for name, completion, ack, data_line in _macro_rows(text, "CB_COMMANDS"):
        commands[name] = {"completion": completion, "ack": ack, "data_line": data_line, "params": []}
    for command, name, kind, required, positive, has_min, low, has_max, high in _macro_rows(text, "CB_PARAMS"):
        assert command in commands, f"CB_PARAMS names the undeclared command {command}"
        commands[command]["params"].append(  # type: ignore[union-attr]
            {
                "name": name,
                "kind": kind,
                "required": required == "1",
                "positive": positive == "1",
                "minimum": _number(low) if has_min == "1" else None,
                "maximum": _number(high) if has_max == "1" else None,
            }
        )
    data_lines: dict[str, list[tuple[str, str]]] = {}
    for line, name, kind in _macro_rows(text, "CB_DATA_FIELDS"):
        data_lines.setdefault(line, []).append((name, kind))
    return {
        "protocol_version": int(_constant(text, "kProtocolVersion")),
        "float_decimals": int(_constant(text, "kFloatDecimals")),
        "line_terminator": _constant(text, "kLineTerminator"),
        "responses": {
            "ok": _constant(text, "kResponseOk").strip('"'),
            "done": _constant(text, "kResponseDone").strip('"'),
            "error_prefix": _constant(text, "kErrorPrefix").strip('"'),
            "alarm_prefix": _constant(text, "kAlarmPrefix").strip('"'),
        },
        "error_codes": {name: int(number) for name, number in _macro_rows(text, "CB_ERROR_CODES")},
        "alarm_codes": [row[0] for row in _macro_rows(text, "CB_ALARM_CODES")],
        "z_states": [row[0] for row in _macro_rows(text, "CB_Z_STATES")],
        "accepted_in_estop": [row[0] for row in _macro_rows(text, "CB_ESTOP_ACCEPTED")],
        "commands": commands,
        "data_lines": data_lines,
        "max_params_per_command": int(_constant(text, "kMaxParamsPerCommand")),
    }


def comparable_declaration() -> dict[str, object]:
    """The Python declaration, written the way the header spells things (upper-case kinds, NONE, ESTOP_ALARM)."""
    data = declaration_as_dict()
    ack_names = {"ok": "OK", "!alarm:ESTOP": "ESTOP_ALARM"}
    commands = {
        name: {
            "completion": spec["completion"].upper(),
            "ack": ack_names[spec["ack"]],
            "data_line": spec["data_line"] or "NONE",
            "params": [
                {
                    "name": p["name"],
                    "kind": p["kind"].upper(),
                    "required": p["required"],
                    "positive": p["positive"],
                    "minimum": p["minimum"],
                    "maximum": p["maximum"],
                }
                for p in spec["params"]
            ],
        }
        for name, spec in data["commands"].items()
    }
    return {
        "protocol_version": data["protocol_version"],
        "float_decimals": declaration.FLOAT_DECIMALS,
        "line_terminator": "'\\n'",
        "responses": data["responses"],
        "error_codes": data["error_codes"],
        "alarm_codes": data["alarm_codes"],
        "z_states": data["z_states"],
        "accepted_in_estop": data["accepted_in_estop"],
        "commands": commands,
        "data_lines": {
            name: [(f["name"], f["kind"].upper()) for f in fields] for name, fields in data["data_lines"].items()
        },
        "max_params_per_command": max(len(spec["params"]) for spec in data["commands"].values()),
    }


@pytest.fixture(scope="module")
def header_text() -> str:
    assert PROTOCOL_H.exists(), "firmware/include/protocol.h is missing"
    return PROTOCOL_H.read_text(encoding="utf-8")


def test_protocol_h_command_names_and_order_match(header_text: str) -> None:
    header = header_declaration(header_text)
    assert list(header["commands"]) == list(COMMAND_ORDER)  # type: ignore[arg-type]


def test_protocol_h_error_codes_match_and_are_1_to_5(header_text: str) -> None:
    header = header_declaration(header_text)
    assert header["error_codes"] == {code.name: int(code) for code in ErrorCode}
    assert sorted(header["error_codes"].values()) == [1, 2, 3, 4, 5]  # type: ignore[union-attr]


def test_protocol_h_alarm_codes_match_in_order(header_text: str) -> None:
    assert header_declaration(header_text)["alarm_codes"] == [code.value for code in AlarmCode]


def test_protocol_h_z_states_match(header_text: str) -> None:
    assert header_declaration(header_text)["z_states"] == comparable_declaration()["z_states"]


def test_protocol_h_estop_set_matches_the_declaration_and_the_owner_decision(header_text: str) -> None:
    header = header_declaration(header_text)
    assert header["accepted_in_estop"] == list(ACCEPTED_IN_ESTOP) == list(OWNER_ESTOP_SET)
    assert all(name in header["commands"] for name in header["accepted_in_estop"])  # type: ignore[operator]


def test_protocol_h_wire_words_and_version_match(header_text: str) -> None:
    header, expected = header_declaration(header_text), comparable_declaration()
    for key in ("protocol_version", "float_decimals", "line_terminator", "responses"):
        assert header[key] == expected[key], key


def test_protocol_h_parameters_match_the_declaration(header_text: str) -> None:
    header, expected = header_declaration(header_text), comparable_declaration()
    for name in COMMAND_ORDER:
        assert header["commands"][name]["params"] == expected["commands"][name]["params"], name  # type: ignore[index]
    assert header["max_params_per_command"] == expected["max_params_per_command"]


def test_protocol_h_completion_ack_and_data_line_match(header_text: str) -> None:
    header, expected = header_declaration(header_text), comparable_declaration()
    for name in COMMAND_ORDER:
        for key in ("completion", "ack", "data_line"):
            assert header["commands"][name][key] == expected["commands"][name][key], f"{name} {key}"  # type: ignore[index]


def test_protocol_h_data_line_fields_match_in_order(header_text: str) -> None:
    header, expected = header_declaration(header_text), comparable_declaration()
    assert header["data_lines"] == expected["data_lines"]
    assert list(header["data_lines"]) == ["POS", "INFO"]  # type: ignore[arg-type]


def test_protocol_h_matches_the_declaration_as_a_whole(header_text: str) -> None:
    assert header_declaration(header_text) == comparable_declaration()


# The comparison must be able to fail: each deliberately wrong header below has to be seen as different.
@pytest.mark.parametrize(
    "old, new",
    [
        ("X(UNRECOGNIZED, 1)", "X(UNRECOGNIZED, 6)"),  # an error code number
        ("X(LIMIT_Z)", "X(LIMIT_ZZ)"),  # an alarm name
        ("X(HB_TIMEOUT)", "X(HB_TIMEOUTS)"),  # the last alarm name
        ("X(ZUP, DONE, OK, NONE)", "X(ZUP, NONE, OK, NONE)"),  # a completion kind
        ("X(ESTOP, NONE, ESTOP_ALARM, NONE)", "X(ESTOP, NONE, OK, NONE)"),  # an acknowledgement
        ("X(GRIP, S, FLOAT, 1, 0, 1, 0, 1, 180)", "X(GRIP, S, FLOAT, 1, 0, 1, 0, 1, 181)"),  # a range
        ("X(MOVE, F, FLOAT, 1, 1, 0, 0, 0, 0)", "X(MOVE, F, FLOAT, 1, 0, 0, 0, 0, 0)"),  # positive
        ("X(HOME, Z, FLAG, 0, 0, 0, 0, 0, 0)", "X(HOME, Z, FLAG, 1, 0, 0, 0, 0, 0)"),  # required
        ("X(POS, SA, FLOAT)", "X(POS, SB, FLOAT)"),  # a field name
        ("X(INFO, PROTO, INT)", "X(INFO, FW, TEXT)"),  # field order / kind
        ("constexpr int kProtocolVersion = 1;", "constexpr int kProtocolVersion = 2;"),
        ('constexpr char kErrorPrefix[] = "error:";', 'constexpr char kErrorPrefix[] = "err:";'),
        ("X(PING, NONE, OK, NONE)", "X(PINGS, NONE, OK, NONE)"),  # a command name
        ("X(DISABLE)", "X(ENABLE)"),  # the ESTOP set with the old/wrong member: ENABLE accepted, DISABLE refused
        ("X(ZSTOP)", "X(ZSTOPS)"),  # a name in the ESTOP set
        ("  X(RESET)\n", "  X(RESET) \\\n  X(ENABLE)\n"),  # one command too many at the end of the ESTOP set
    ],
)
def test_the_cross_check_detects_a_wrong_header(header_text: str, old: str, new: str) -> None:
    assert old in header_text, f"the mutation target {old!r} is not in protocol.h any more: update this test"
    assert header_declaration(header_text.replace(old, new, 1)) != comparable_declaration()


def test_the_cross_check_detects_a_missing_command_and_a_swapped_order(header_text: str) -> None:
    line = "  X(PING, NONE, OK, NONE)          \\\n"
    assert line in header_text
    assert header_declaration(header_text.replace(line, "", 1))["commands"] != comparable_declaration()["commands"]
    swapped = header_text.replace("  X(HOME, DONE, OK, NONE)          \\\n  X(MOVE, NONE, OK, NONE)          \\\n",
                                  "  X(MOVE, NONE, OK, NONE)          \\\n  X(HOME, DONE, OK, NONE)          \\\n", 1)
    assert swapped != header_text
    assert list(header_declaration(swapped)["commands"]) != list(COMMAND_ORDER)  # type: ignore[arg-type]
