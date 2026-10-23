"""The protocol is declared once (SPEC rule 6): config/protocol_commands.py.

* The declaration is compared with docs/SPEC.md section 5, parsed from the SPEC text itself.
* The codec and the simulator may not carry a copy of the declaration (static scan).
* firmware/include/protocol.h does not exist until phase 4, so the cross-check against it is the
  clearly named placeholder ``test_phase4_protocol_h_matches_the_declaration`` below.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest
import yaml

from config import protocol_commands as declaration
from config.protocol_commands import COMMAND_ORDER, COMMANDS, AlarmCode, ErrorCode, declaration_as_dict

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
# The declaration is internally consistent and holds no hardware value
# --------------------------------------------------------------------------- #


def test_declaration_is_plain_serialisable_data() -> None:
    import json

    data = declaration_as_dict()
    assert json.loads(json.dumps(data)) == data
    assert list(data["commands"]) == list(COMMAND_ORDER)
    assert data["error_codes"] == {"UNRECOGNIZED": 1, "BAD_PARAMETER": 2, "QUEUE_FULL": 3, "NOT_HOMED": 4, "IN_ESTOP": 5}
    assert data["alarm_codes"] == ["ESTOP", "LIMIT_Z", "Z_TIMEOUT", "SENSOR_FAULT", "STEP_LOSS", "HB_TIMEOUT"]


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
# PHASE 4 HOOK: the cross-check against firmware/include/protocol.h
# --------------------------------------------------------------------------- #


def test_phase4_protocol_h_matches_the_declaration() -> None:
    """EXTEND IN PHASE 4. Compare firmware/include/protocol.h with config/protocol_commands.py.

    The header does not exist yet (phase 4), so there is nothing to compare and the test says so
    instead of passing silently. When the file appears, this test starts running a first, format
    independent check (every command name and alarm code is spelled in the header). Phase 4 must
    replace that with the exact comparison: the error code numbers, the parameter ranges, the
    completion kind of each command and the field order of POS and INFO, in the header's own syntax.
    """
    if not PROTOCOL_H.exists():
        pytest.skip("firmware/include/protocol.h does not exist yet: phase 4 creates it and extends this test")
    header = PROTOCOL_H.read_text(encoding="utf-8")
    missing = [name for name in [*COMMANDS, *(code.value for code in AlarmCode)] if not re.search(rf"\b{name}\b", header)]
    assert missing == [], f"protocol.h does not declare {missing}"
    error_numbers = sorted(int(code) for code in ErrorCode)
    pytest.fail(
        "protocol.h exists but the exact cross-check is not written yet; extend this test "
        f"(error codes {error_numbers}, parameter ranges, completion kinds, POS/INFO field order)"
    )
