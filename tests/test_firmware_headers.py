"""Every firmware header must compile on its own, and the firmware must survive the ESP32 build's include
environment (phase 4a fixes).

The ESP32 build failed in ways the host unit tests could not see: a library header that could not find
include/protocol.h, a header that depended on another one's include path, and a function called ``bit``
that Arduino.h turns into a macro. This test compiles each header and source on its own (``-fsyntax-only``)
in include setups that imitate that build:

* no project include path at all (PlatformIO gives libraries in lib/ none);
* an SDK-style decoy ``config.h`` / ``pins.h`` / ``protocol.h`` found BEFORE (or after) the project's
  include/ (PlatformIO appends project -I flags after the framework's directories); reading the decoy is an
  ``#error``;
* the Arduino stub force-included, which defines the real Arduino macros and global names.

It needs a GCC or Clang compatible C++ compiler (``$CXX``, ``g++``, ``clang++`` or ``c++``). Without one the
whole module is skipped, and the skip reason says so: run firmware/tools/run_host_tests.sh on a machine that
has one. It never compiles for the ESP32.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = REPO_ROOT / "firmware"
INCLUDE = FIRMWARE / "include"
CORE_SRC = FIRMWARE / "lib" / "chessbot_core" / "src"
MAIN_CPP = FIRMWARE / "src" / "main.cpp"
STUB_DIR = FIRMWARE / "tools" / "host_main"
STUB = STUB_DIR / "Arduino.h"
DECOY = STUB_DIR / "sdk_decoy"
UNITY_DIR = FIRMWARE / "tools" / "host_unity"


def _find_compiler() -> str | None:
    candidates = [os.environ.get("CXX", ""), "g++", "clang++", "c++"]
    for name in candidates:
        found = shutil.which(name) if name else None
        if found:
            return found
    return None


COMPILER = _find_compiler()
pytestmark = pytest.mark.skipif(
    COMPILER is None,
    reason="no GCC/Clang C++ compiler found ($CXX, g++, clang++, c++): the firmware include checks did NOT run; "
    "run firmware/tools/run_host_tests.sh on a machine that has one",
)

WARNINGS = ["-std=gnu++17", "-Wall", "-Wextra", "-Wshadow", "-Werror", "-fsyntax-only"]

VARIANTS: dict[str, list[str]] = {
    "no project include path": [f"-I{CORE_SRC}"],
    "SDK decoy first": [f"-I{DECOY}", f"-I{INCLUDE}", f"-I{CORE_SRC}"],
    "SDK decoy last": [f"-I{INCLUDE}", f"-I{DECOY}", f"-I{CORE_SRC}"],
    "Arduino macros, no include path": ["-include", str(STUB), f"-I{CORE_SRC}"],
    "Arduino macros, SDK decoy first": ["-include", str(STUB), f"-I{DECOY}", f"-I{INCLUDE}", f"-I{CORE_SRC}"],
}

HEADERS = sorted([*INCLUDE.glob("*.h"), *CORE_SRC.glob("*.h")])
CORE_SOURCES = sorted(CORE_SRC.glob("*.cpp"))


def compile_text(source: str, flags: list[str], tmp_path: Path) -> subprocess.CompletedProcess[str]:
    unit = tmp_path / "unit.cpp"
    unit.write_text(source, encoding="utf-8")
    assert COMPILER is not None
    return subprocess.run([COMPILER, *WARNINGS, *flags, str(unit)], capture_output=True, text=True, check=False)


def compile_file(path: Path, flags: list[str]) -> subprocess.CompletedProcess[str]:
    assert COMPILER is not None
    return subprocess.run([COMPILER, *WARNINGS, *flags, str(path)], capture_output=True, text=True, check=False)


def _first_errors(result: subprocess.CompletedProcess[str]) -> str:
    return "\n".join(line for line in result.stderr.splitlines() if "error" in line or "warning" in line)[:1500]


# --------------------------------------------------------------------------- #
# The real firmware
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("variant", list(VARIANTS))
@pytest.mark.parametrize("header", HEADERS, ids=lambda p: p.name)
def test_every_header_compiles_on_its_own(header: Path, variant: str, tmp_path: Path) -> None:
    result = compile_text(f'#include "{header}"\n', VARIANTS[variant], tmp_path)
    assert result.returncode == 0, f"{header.relative_to(REPO_ROOT)} is not self-contained [{variant}]:\n{_first_errors(result)}"


@pytest.mark.parametrize("variant", ["SDK decoy first", "Arduino macros, SDK decoy first"])
@pytest.mark.parametrize("source", CORE_SOURCES, ids=lambda p: p.name)
def test_every_core_source_compiles_in_the_esp32_include_environment(source: Path, variant: str) -> None:
    result = compile_file(source, VARIANTS[variant])
    assert result.returncode == 0, f"{source.name} [{variant}]:\n{_first_errors(result)}"


@pytest.mark.parametrize("decoys_first", [True, False])
def test_main_cpp_compiles_against_the_arduino_stub(decoys_first: bool) -> None:
    flags = ["-include", str(STUB), f"-I{STUB_DIR}"]
    flags += [f"-I{DECOY}"] if decoys_first else []
    flags += [f"-I{INCLUDE}", f"-I{CORE_SRC}"]
    result = compile_file(MAIN_CPP, flags)
    assert result.returncode == 0, f"main.cpp (decoys first: {decoys_first}):\n{_first_errors(result)}"


def test_the_decoys_exist_for_every_project_header_name() -> None:
    assert sorted(p.name for p in DECOY.glob("*.h")) == sorted(p.name for p in INCLUDE.glob("*.h"))


# --------------------------------------------------------------------------- #
# The checks themselves must be able to fail: each of the three ESP32 build errors, reproduced
# --------------------------------------------------------------------------- #


def test_a_library_header_with_a_bare_include_cannot_find_the_project_headers(tmp_path: Path) -> None:
    """Error 1 and 2 of the ESP32 build: ``#include "protocol.h"`` / ``"config.h"`` with no project path."""
    (tmp_path / "bad.h").write_text('#include "config.h"\nconstexpr int kBad = chessbot::config::kBaudRate;\n', encoding="utf-8")
    result = compile_text(f'#include "{tmp_path / "bad.h"}"\n', VARIANTS["no project include path"], tmp_path)
    assert result.returncode != 0
    assert "config.h" in result.stderr


def test_a_bare_include_is_caught_when_an_sdk_header_of_the_same_name_comes_first(tmp_path: Path) -> None:
    (tmp_path / "bad.h").write_text('#include "protocol.h"\n', encoding="utf-8")
    result = compile_text(f'#include "{tmp_path / "bad.h"}"\n', VARIANTS["SDK decoy first"], tmp_path)
    assert result.returncode != 0
    assert "wrong protocol.h" in result.stderr


def test_a_header_that_uses_another_headers_names_without_including_it_is_caught(tmp_path: Path) -> None:
    (tmp_path / "needy.h").write_text("constexpr int kNeedy = chessbot::config::kBaudRate;\n", encoding="utf-8")
    result = compile_text(f'#include "{tmp_path / "needy.h"}"\n', VARIANTS["SDK decoy last"], tmp_path)
    assert result.returncode != 0


def test_a_function_called_bit_breaks_the_build_when_the_arduino_macros_are_defined(tmp_path: Path) -> None:
    """Error 3 of the ESP32 build: Arduino.h defines bit(b)."""
    source = "#include <stdint.h>\nstruct S { static uint8_t bit(int a) { return static_cast<uint8_t>(1U << a); } };\n"
    plain = compile_text(source, [], tmp_path)
    assert plain.returncode == 0, "the snippet is valid C++ without Arduino.h"
    with_macros = compile_text(source, ["-include", str(STUB)], tmp_path)
    assert with_macros.returncode != 0


# word is left out on purpose: the real word(...) macro renames a same-named function to makeWord(...), which
# still compiles inside one file; tests/test_firmware_layout.py catches that name by scanning the source.
@pytest.mark.parametrize("name", ["min", "max", "abs", "round", "sq", "constrain", "radians", "degrees", "bitRead"])
def test_other_function_like_arduino_macros_break_a_same_named_function(name: str, tmp_path: Path) -> None:
    source = f"int {name}(int a, int b = 0, int c = 0) {{ return a + b + c; }}\n"
    assert compile_text(source, [], tmp_path).returncode == 0
    assert compile_text(source, ["-include", str(STUB)], tmp_path).returncode != 0


@pytest.mark.parametrize("name", ["HIGH", "LOW", "INPUT", "OUTPUT", "DEFAULT", "B0", "B7", "PI"])
def test_object_like_arduino_macros_break_a_same_named_constant(name: str, tmp_path: Path) -> None:
    source = f"enum class E {{ {name} = 5 }};\n"
    assert compile_text(source, [], tmp_path).returncode == 0
    assert compile_text(source, ["-include", str(STUB)], tmp_path).returncode != 0


def test_a_parameter_named_byte_is_flagged_by_wshadow_once_arduino_defines_the_type(tmp_path: Path) -> None:
    source = "#include <stdint.h>\nstruct F { int feed(uint8_t byte) { return byte; } };\n"
    assert compile_text(source, [], tmp_path).returncode == 0
    assert compile_text(source, ["-include", str(STUB)], tmp_path).returncode != 0
