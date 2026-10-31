"""SPEC rule 7: no tunable literal outside config/settings.yaml and the headers generated from it.

The scan reads every Python source of desktop_app/, the config readers (config/*.py except protocol_commands.py,
which holds the protocol words) and tools/, and every firmware source (src/ and lib/chessbot_core/src/). A numeric
literal other than 0, 1, 2, -1 or a string that looks like a file path fails the test, unless ALLOWED_LITERALS lists
it for that file with the reason it is not a tunable value. ALLOWED_LITERALS is the ONE place where exceptions are
documented; an entry that no longer matches anything fails too, so the table cannot go stale.

Not scanned on purpose: config/protocol_commands.py and firmware/include/protocol.h (protocol words), the generated
headers firmware/include/pins.h and config.h, strings_vi.py (UI text), tests, and docstrings and comments.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path
from typing import Iterator, Union

REPO_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = REPO_ROOT / "firmware"

Literal = Union[int, float, str]

# The only values a source may contain besides 0, 1, 2, -1 (indexes, "not found", counting).
TRIVIAL_NUMBERS = frozenset({0, 1, 2, -1})

# THE allowed exceptions: file (relative to the repository root) -> {literal: why it is not a tunable value}.
ALLOWED_LITERALS: dict[str, dict[Literal, str]] = {
    # ---- Python ---------------------------------------------------------------------------------------------------
    "desktop_app/utils/settings.py": {
        "settings.yaml": "bootstrap: where the single settings file lives cannot be written in that file",
        "local.yaml": "bootstrap: where the per-machine overlay lives",
        "/": "path separator test of a unit-less string, not a path",
        "\\": "path separator test, not a path",
        39: "chip fact: the highest ESP32 GPIO number",
        34: "chip fact: GPIO 34..39 are input only",
        40: "chip fact: end of range(34, 40), the input-only GPIOs",
        16: "chip fact: ESP32 LEDC channel count",
        20: "chip fact: ESP32 LEDC maximum resolution in bits",
        4: "chip fact: ESP32 LEDC timer count",
        3: "chip fact: ADS1115 input channels A0..A3",
        72: "chip fact: ADS1115 address 0x48 (ADDR tied to GND)",
        73: "chip fact: ADS1115 address 0x49",
        74: "chip fact: ADS1115 address 0x4A",
        75: "chip fact: ADS1115 address 0x4B",
        255: "C type limit: the generated firmware constants are uint8_t",
        65535: "C type limit: uint16_t",
        4294967295: "C type limit: uint32_t",
        90.0: "mathematics: the singularity margin lives in [0, 90) degrees",
        100: "mathematics: a percentage cannot exceed 100",
        1000.0: "unit conversion: milliseconds per second",
        360.0: "mathematics: degrees in a full turn",
    },
    "desktop_app/kinematics/five_bar.py": {
        1e-09: "numerical tolerance of the floating point geometry, not a physical value",
        1e-12: "numerical tolerance of the floating point geometry, not a physical value",
        360.0: "mathematics: degrees in a full turn",
        180.0: "mathematics: degrees in a half turn",
        4.0: "mathematics: the closed form of the five-bar inverse kinematics",
        0.5: "mathematics: half of a distance in the closed form",
    },
    "desktop_app/kinematics/dimension.py": {
        90.0: "mathematics: the singularity margin lives in [0, 90) degrees",
    },
    "desktop_app/hardware/esp32_simulator.py": {
        1e-09: "float hygiene of the simulated clock, not a physical tolerance",
        360.0: "mathematics: degrees in a full turn",
    },
    "desktop_app/core/game_state.py": {
        3: "chess rule: threefold repetition",
        100: "chess rule: the 50-move rule counts 100 half-moves",
        "1/2-1/2": "chess rule: the PGN result string of a draw (not a path)",
    },
    "tools/gen_firmware_config.py": {
        "pins.h": "output name of the generator (the generated file itself)",
        "config.h": "output name of the generator (the generated file itself)",
        "settings.yaml": "input of the generator: the single settings file",
        255: "C type limit: uint8_t",
        65535: "C type limit: uint16_t",
        4294967295: "C type limit: uint32_t",
        "//": "C++ comment marker, not a path",
        "\\\\": "C string escaping",
        '\\"': "C string escaping",
        "\\": "C string escaping",
        96: "layout of a generated comment rule, not a value of the robot",
        8: "layout of a generated comment rule, not a value of the robot",
        3: "layout of a generated comment rule, not a value of the robot",
    },
    "tools/gen_config_reference.py": {
        "CONFIG_REFERENCE.md": "output name of the generator (the generated file itself)",
        "\\|": "Markdown table escaping, not a path",
    },
    "tools/settings_doc.py": {
        "deg/s^2": "unit name printed in the reference, not a path",
        "deg/s": "unit name printed in the reference, not a path",
    },
    # ---- Firmware (C++) ---------------------------------------------------------------------------------------------
    "firmware/lib/chessbot_core/src/command_parser.cpp": {
        "0.0": "float zero: a missing value is not a setting",
    },
    "firmware/lib/chessbot_core/src/device_state.h": {
        "8": "bits of the uint8_t alarm mask (a C type fact)",
    },
    "firmware/lib/chessbot_core/src/line_framer.cpp": {
        "0x20": "ASCII: first printable character of the wire protocol",
        "0x7E": "ASCII: last printable character of the wire protocol",
    },
    "firmware/lib/chessbot_core/src/response_format.cpp": {
        "24": "size of the digit buffer for a 64-bit long, not a protocol limit (the line limit is config.h)",
        "10": "decimal base of the number formatter",
        "40": "size of the text buffer of a formatted float (15 digits, sign, dot and decimals), not a protocol limit",
    },
    "firmware/lib/chessbot_core/src/motion.cpp": {
        "360.0": "mathematics: degrees in a full turn (steps per degree = steps per revolution / 360)",
    },
}

_PATH_LIKE = re.compile(r"[/\\]|\.(json|yaml|yml|fen|pgn|h|pt|onnx|txt|md|csv|ini)\b")


def python_sources() -> list[Path]:
    found = list((REPO_ROOT / "desktop_app").rglob("*.py"))
    found += [p for p in (REPO_ROOT / "config").glob("*.py") if p.name != "protocol_commands.py"]
    found += list((REPO_ROOT / "tools").glob("*.py"))
    return sorted(p for p in found if "__pycache__" not in p.parts and p.name != "strings_vi.py")


def firmware_sources() -> list[Path]:
    roots = [FIRMWARE / "src", FIRMWARE / "lib" / "chessbot_core" / "src"]
    return sorted(p for root in roots for p in root.rglob("*") if p.suffix in {".cpp", ".h", ".c", ".ino"})


def python_literals(source: str) -> list[tuple[int, Literal]]:
    """Every numeric literal (not trivial) and path-like string of a Python source, docstrings excluded."""
    tree = ast.parse(source)
    docstrings = {
        id(node.body[0].value)
        for node in ast.walk(tree)
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
        and node.body
        and isinstance(node.body[0], ast.Expr)
        and isinstance(node.body[0].value, ast.Constant)
        and isinstance(node.body[0].value.value, str)
    }
    found: list[tuple[int, Literal]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Constant) or id(node) in docstrings:
            continue
        value = node.value
        if isinstance(value, bool) or value is None:
            continue
        if isinstance(value, (int, float)):
            if value not in TRIVIAL_NUMBERS:
                found.append((node.lineno, value))
        elif isinstance(value, str) and "\n" not in value and " " not in value and len(value) < 100 and _PATH_LIKE.search(value):
            found.append((node.lineno, value))
    return found


_CPP_NUMBER = re.compile(
    r"(?<![\w.])(0[xX][0-9A-Fa-f]+|\d+\.\d*(?:[eE][-+]?\d+)?|\d+(?:[eE][-+]?\d+)?|\.\d+)[fFuUlL]*(?![\w.])"
)


def cpp_literals(source: str) -> list[tuple[int, str]]:
    """Every numeric literal (not 0, 1, 2) of a C++ source; comments and string/char literals are blanked first."""
    source = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), source, flags=re.S)
    found: list[tuple[int, str]] = []
    for number, line in enumerate(source.splitlines(), start=1):
        line = re.sub(r'"(\\.|[^"\\])*"', '""', line)
        line = re.sub(r"'(\\.|[^'\\])+'", "''", line)
        line = re.sub(r"//.*", "", line)
        for match in _CPP_NUMBER.finditer(line):
            if match.group(1) not in {"0", "1", "2"}:
                found.append((number, match.group(1)))
    return found


def _relative(path: Path) -> str:
    return path.relative_to(REPO_ROOT).as_posix()


def _scan() -> Iterator[tuple[str, int, Literal]]:
    for path in python_sources():
        for line, value in python_literals(path.read_text(encoding="utf-8")):
            yield _relative(path), line, value
    for path in firmware_sources():
        for line, text in cpp_literals(path.read_text(encoding="utf-8")):
            yield _relative(path), line, text


def _is_allowed(file: str, value: Literal) -> bool:
    allowed = ALLOWED_LITERALS.get(file, {})
    return value in allowed  # 3 == 3.0, so an int entry also covers its float form


def test_no_tunable_literal_is_hard_coded_outside_settings_yaml() -> None:
    offending = [f"{file}:{line}: {value!r}" for file, line, value in _scan() if not _is_allowed(file, value)]
    assert offending == [], (
        "a tunable value belongs in config/settings.yaml (and its generated headers); a fact of mathematics, "
        "the chess rules or the chip belongs in ALLOWED_LITERALS with the reason:\n  " + "\n  ".join(offending)
    )


def test_every_allowed_literal_is_still_used() -> None:
    used: dict[str, set[Literal]] = {}
    for file, _line, value in _scan():
        used.setdefault(file, set()).add(value)
    stale = [f"{file}: {literal!r}" for file, entries in ALLOWED_LITERALS.items() for literal in entries if literal not in used.get(file, set())]
    assert stale == [], "remove these from ALLOWED_LITERALS, they no longer occur:\n  " + "\n  ".join(stale)


def test_every_allowed_literal_has_a_reason_and_names_a_scanned_file() -> None:
    scanned = {_relative(p) for p in [*python_sources(), *firmware_sources()]}
    for file, entries in ALLOWED_LITERALS.items():
        assert file in scanned, f"{file} is not a scanned source"
        for literal, reason in entries.items():
            assert reason.strip(), f"{file}: {literal!r} needs a reason"


def test_the_scan_catches_a_hard_coded_python_value() -> None:
    source = 'timeout = 2.5\nport = "/dev/ttyUSB0"\nname = "stockfish"\nfine = [0, 1, 2, -1]\n"""docs 99"""\n'
    assert python_literals(source) == [(1, 2.5), (2, "/dev/ttyUSB0")]


def test_the_scan_catches_a_hard_coded_firmware_value() -> None:
    source = (
        "constexpr uint32_t kTimeout = 2000;  // 3000 in a comment is ignored\n"
        '/* 4000 */ const char* s = "5000"; int x = 0; float y = 0.5f;\n'
        "uint8_t pin = 0x1F;\n"
    )
    assert cpp_literals(source) == [(1, "2000"), (2, "0.5"), (3, "0x1F")]
