"""The kinematics package is pure NumPy and math, built only from the config.

* No Qt, no serial, and no imports from gui/, hardware/, core/ or vision/ (static AST scan
  plus a fresh-interpreter import that inspects sys.modules).
* No hard-coded hardware constant: numeric literals in the modules are scanned against the
  values in config/hardware.yaml.
"""

from __future__ import annotations

import ast
import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
KINEMATICS_DIR = REPO_ROOT / "desktop_app" / "kinematics"
MODULES = sorted(p for p in KINEMATICS_DIR.glob("*.py"))

FORBIDDEN_TOP_LEVEL = {"PySide6", "PyQt5", "PyQt6", "shiboken6", "serial", "cv2", "ultralytics", "chess"}
FORBIDDEN_PACKAGES = {"gui", "hardware", "core", "vision"}

# Hardware numbers (SPEC 4.1 and derived values) that must come from the config.
HARDWARE_NUMBERS = {
    100, 200, 250, 150, 31, 26, 70, 16, 3200, 0.1125, 191.5, 108.5, 408.5, 300, 50, 450, 8,
}


def _imported_modules(source: str, package: list[str]) -> list[str]:
    names: list[str] = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            names.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                keep = max(0, len(package) - (node.level - 1))
                base = ".".join(package[:keep] + (node.module.split(".") if node.module else []))
            else:
                base = node.module or ""
            names.append(base)
            names.extend(f"{base}.{alias.name}" for alias in node.names)
    return names


def _is_forbidden(name: str) -> bool:
    parts = name.split(".")
    if parts[0] in FORBIDDEN_TOP_LEVEL:
        return True
    if parts[0] == "desktop_app" and len(parts) > 1 and parts[1] in FORBIDDEN_PACKAGES:
        return True
    return parts[0] in FORBIDDEN_PACKAGES  # bare "gui", "core", ... reached through a path hack


def test_the_package_has_the_expected_modules() -> None:
    assert {p.name for p in MODULES} >= {"__init__.py", "five_bar.py", "dimension.py", "trajectory.py"}


def test_the_checker_flags_what_it_should() -> None:
    package = ["desktop_app", "kinematics"]
    flagged = ["import PySide6.QtCore", "from serial import Serial", "from desktop_app.gui import x",
               "from desktop_app import core", "from ..hardware import robot", "import desktop_app.vision.yolo",
               "from ..core.fsm import Machine"]
    for snippet in flagged:
        assert any(_is_forbidden(n) for n in _imported_modules(snippet, package)), snippet
    for snippet in ["import math", "import numpy as np", "from desktop_app.utils.hardware_config import load_hardware_config",
                    "from desktop_app.kinematics.five_bar import FiveBarKinematics", "from . import five_bar"]:
        assert not any(_is_forbidden(n) for n in _imported_modules(snippet, package)), snippet


@pytest.mark.parametrize("module", MODULES, ids=lambda p: p.name)
def test_no_forbidden_imports(module: Path) -> None:
    offending = [n for n in _imported_modules(module.read_text(encoding="utf-8"), ["desktop_app", "kinematics"]) if _is_forbidden(n)]
    assert offending == []


def test_importing_the_package_loads_no_qt_serial_or_other_layers() -> None:
    code = (
        "import sys, json, importlib\n"
        "sys.path.insert(0, sys.argv[1])\n"
        "importlib.import_module('desktop_app.kinematics')\n"
        "for name in ('five_bar', 'dimension', 'trajectory'):\n"
        "    importlib.import_module('desktop_app.kinematics.' + name)\n"
        "print(json.dumps(sorted(sys.modules)))\n"
    )
    result = subprocess.run([sys.executable, "-I", "-c", code, str(REPO_ROOT)], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    loaded = set(json.loads(result.stdout.strip().splitlines()[-1]))
    bad = sorted(name for name in loaded if _is_forbidden(name))
    assert bad == []
    assert "numpy" in loaded


@pytest.mark.parametrize("module", MODULES, ids=lambda p: p.name)
def test_no_hardware_constant_is_hard_coded(module: Path) -> None:
    tree = ast.parse(module.read_text(encoding="utf-8"))
    literals = [
        (node.lineno, node.value)
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)) and not isinstance(node.value, bool)
    ]
    # The owner's PROPOSED zone layout is [TO CONFIRM] data, not a hardware.yaml constant:
    # exactly the assignments named PROPOSED_* are exempt, nothing else.
    exempt = {
        child.lineno
        for node in ast.walk(tree)
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id.startswith("PROPOSED_")
        for child in ast.walk(node)
        if hasattr(child, "lineno")
    }
    literals = [(line, value) for line, value in literals if line not in exempt]
    offending = [(line, value) for line, value in literals if value in HARDWARE_NUMBERS]
    assert offending == [], f"{module.name} hard-codes hardware numbers: {offending}"


def test_the_literal_scan_would_catch_a_hard_coded_constant() -> None:
    tree = ast.parse("L = 100.0\nM = 200\nsteps = 3200 * x")
    found = [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and n.value in HARDWARE_NUMBERS]
    assert found == [100.0, 200, 3200]


def test_only_the_proposed_zone_layout_is_exempt_from_the_literal_scan() -> None:
    dimension = (KINEMATICS_DIR / "dimension.py").read_text(encoding="utf-8")
    tree = ast.parse(dimension)
    exempt_names = [
        node.target.id
        for node in ast.walk(tree)
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id.startswith("PROPOSED_")
    ]
    assert exempt_names == ["PROPOSED_CAPTURE_SLOTS", "PROPOSED_RESERVE_SLOTS", "PROPOSED_PARK_POSITION"]
