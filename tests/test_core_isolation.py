"""SPEC rule 2: core/ never imports gui/.

Three layers of proof:
1. the checker itself is tested against snippets that must and must not be flagged,
   so the real check below cannot pass vacuously;
2. every source file under desktop_app/core/ is scanned statically (AST);
3. at runtime, importing every core module must not load desktop_app.gui.
"""

from __future__ import annotations

import ast
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
CORE_DIR = REPO_ROOT / "desktop_app" / "core"
GUI_PACKAGE = "desktop_app.gui"


def _is_gui_module(name: str) -> bool:
    """True for desktop_app.gui[.x] and for a bare gui[.x] reached through a path hack."""
    return name in (GUI_PACKAGE, "gui") or name.startswith((GUI_PACKAGE + ".", "gui."))


def _package_of(py_file: Path) -> list[str]:
    """Package path (as module name parts) that contains ``py_file``."""
    parts = list(py_file.relative_to(REPO_ROOT).with_suffix("").parts)
    return parts if parts[-1] == "__init__" and parts.pop() else parts[:-1]


def find_gui_imports(source: str, package: list[str]) -> list[str]:
    """Return every module name in ``source`` that resolves into the GUI package."""
    candidates: list[str] = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            candidates.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                keep = max(0, len(package) - (node.level - 1))
                module_parts = package[:keep] + (node.module.split(".") if node.module else [])
            else:
                module_parts = (node.module or "").split(".")
            base = ".".join(module_parts)
            candidates.append(base)
            candidates.extend(f"{base}.{alias.name}" for alias in node.names)
        elif isinstance(node, ast.Call):
            func = node.func
            name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", "")
            if name in ("import_module", "__import__") and node.args:
                first = node.args[0]
                if isinstance(first, ast.Constant) and isinstance(first.value, str):
                    candidates.append(first.value)
    return [name for name in candidates if _is_gui_module(name)]


CORE_PACKAGE = ["desktop_app", "core"]

MUST_BE_FLAGGED = [
    "import desktop_app.gui",
    "import desktop_app.gui.main_window as window",
    "from desktop_app.gui import strings_vi",
    "from desktop_app.gui.play_tab import PlayTab",
    "from desktop_app import gui",
    "from .. import gui",
    "from ..gui import strings_vi",
    "from ..gui.play_tab import PlayTab",
    "import gui",
    "from gui.play_tab import PlayTab",
    "import importlib\nimportlib.import_module('desktop_app.gui.play_tab')",
    "__import__('desktop_app.gui')",
    "def late():\n    from desktop_app.gui import strings_vi",
    "try:\n    import desktop_app.gui\nexcept ImportError:\n    pass",
]

MUST_NOT_BE_FLAGGED = [
    "import logging",
    "from desktop_app.utils.settings import load_settings",
    "from desktop_app.kinematics import five_bar",
    "from desktop_app import core",
    "from . import event_bus",
    "from .event_bus import EventBus",
    "from PySide6.QtCore import QObject",
    "import desktop_app.guide",
    "from desktop_app.guidance import tips",
    "import importlib\nimportlib.import_module('desktop_app.core.event_bus')",
]


@pytest.mark.parametrize("snippet", MUST_BE_FLAGGED)
def test_checker_flags_gui_imports(snippet: str) -> None:
    assert find_gui_imports(snippet, CORE_PACKAGE), f"not flagged: {snippet!r}"


@pytest.mark.parametrize("snippet", MUST_NOT_BE_FLAGGED)
def test_checker_ignores_unrelated_imports(snippet: str) -> None:
    assert find_gui_imports(snippet, CORE_PACKAGE) == []


def test_core_package_has_sources_to_check() -> None:
    names = {path.name for path in CORE_DIR.rglob("*.py")}
    assert {"__init__.py", "event_bus.py"} <= names


def test_core_sources_never_import_gui() -> None:
    violations: dict[str, list[str]] = {}
    for py_file in sorted(CORE_DIR.rglob("*.py")):
        found = find_gui_imports(py_file.read_text(encoding="utf-8"), _package_of(py_file))
        if found:
            violations[str(py_file.relative_to(REPO_ROOT))] = found
    assert violations == {}, f"core/ must never import gui/: {violations}"


def test_importing_all_core_modules_does_not_load_gui() -> None:
    script = textwrap.dedent(
        """
        import importlib, pkgutil, sys
        import desktop_app.core as core

        for info in pkgutil.walk_packages(core.__path__, core.__name__ + "."):
            importlib.import_module(info.name)

        loaded = sorted(m for m in sys.modules if m == "desktop_app.gui" or m.startswith("desktop_app.gui."))
        if loaded:
            sys.exit("core pulled in the GUI: " + ", ".join(loaded))
        """
    )
    env = {**os.environ, "PYTHONPATH": str(REPO_ROOT), "QT_QPA_PLATFORM": "offscreen"}
    result = subprocess.run(
        [sys.executable, "-c", script], cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=60
    )
    assert result.returncode == 0, result.stderr or result.stdout
