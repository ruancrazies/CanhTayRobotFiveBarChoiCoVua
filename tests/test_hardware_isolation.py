"""Source rules for desktop_app/hardware/ and config/protocol_commands.py (SPEC section 2 rules 2, 6, 7, 9, 10).

* No Qt and no gui imports in hardware/. Only robot_commander.py may touch the core (Event Bus, FSM triggers), and
  the rest of hardware/ is pure Python even transitively (fresh-interpreter ``sys.modules`` check).
* No hard-coded hardware constant: tests/test_no_hardcoded_constants.py scans every source for tunable literals.
* ASCII English only, no print(), full type hints.
"""

from __future__ import annotations

import ast
import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
HARDWARE = REPO_ROOT / "desktop_app" / "hardware"
MODULES = sorted(p for p in HARDWARE.glob("*.py") if p.name != "__init__.py")
SOURCES = [*MODULES, REPO_ROOT / "config" / "protocol_commands.py"]

QT_PACKAGES = {"PySide6", "PyQt5", "PyQt6", "shiboken6"}
# Modules that must stay free of Qt even transitively (the Commander reaches it only through core.event_bus).
PURE_MODULES = ["protocol", "serial_link", "esp32_simulator"]


def _imports(path: Path) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            names.add(node.module or "")
    return names


def test_the_package_has_the_expected_modules() -> None:
    assert {p.stem for p in MODULES} >= {"protocol", "serial_link", "esp32_simulator", "robot_commander"}


@pytest.mark.parametrize("path", MODULES, ids=lambda p: p.name)
def test_no_qt_and_no_gui_import(path: Path) -> None:
    for name in _imports(path):
        top = name.split(".")[0]
        assert top not in QT_PACKAGES, f"{path.name} imports {name}"
        assert not name.startswith("desktop_app.gui"), f"{path.name} imports {name}"
        assert not name.startswith("desktop_app.vision"), f"{path.name} imports {name}"


@pytest.mark.parametrize("path", MODULES, ids=lambda p: p.name)
def test_only_the_commander_imports_the_core(path: Path) -> None:
    core = sorted(n for n in _imports(path) if n.startswith("desktop_app.core"))
    if path.name == "robot_commander.py":
        assert set(core) <= {"desktop_app.core.event_bus", "desktop_app.core.fsm", "desktop_app.core.move_planner"}
    else:
        assert core == [], f"{path.name} imports {core}"


@pytest.mark.parametrize("module", PURE_MODULES)
def test_pure_modules_import_no_qt_even_transitively(module: str) -> None:
    code = (
        "import json, sys\n"
        f"import desktop_app.hardware.{module}\n"
        "print(json.dumps(sorted(m for m in sys.modules if m.split('.')[0] in "
        f"{sorted(QT_PACKAGES)!r} or m.startswith('desktop_app.gui') or m.startswith('desktop_app.core'))))\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", f"import sys; sys.path.insert(0, {str(REPO_ROOT)!r})\n" + code],
        cwd=REPO_ROOT, capture_output=True, text=True, check=True,
    )
    assert json.loads(result.stdout) == []


@pytest.mark.parametrize("path", SOURCES, ids=lambda p: p.name)
def test_files_are_plain_ascii_english(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    bad = sorted({ch for ch in text if ord(ch) > 127})
    assert not bad, f"{path.name} contains non-ASCII characters {bad}"


@pytest.mark.parametrize("path", MODULES, ids=lambda p: p.name)
def test_modules_use_logging_never_print(path: Path) -> None:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    prints = [n.lineno for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "print"]
    assert prints == [], f"print() at lines {prints}"


@pytest.mark.parametrize("path", SOURCES, ids=lambda p: p.name)
def test_every_function_has_type_hints(path: Path) -> None:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    missing = [
        f"{n.name}:{n.lineno}"
        for n in ast.walk(tree)
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
        and (n.returns is None or any(a.annotation is None for a in n.args.args + n.args.kwonlyargs if a.arg not in ("self", "cls")))
    ]
    assert missing == [], f"functions without full type hints: {missing}"
