"""Source rules that apply to the phase 2 modules (SPEC section 2 rules 2, 7, 10 and 11)."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
CORE = REPO_ROOT / "desktop_app" / "core"
PHASE2_MODULES = [
    CORE / "game_state.py",
    CORE / "engine.py",
    CORE / "fsm.py",
    CORE / "move_planner.py",
    REPO_ROOT / "desktop_app" / "console_runner.py",
]
PHASE2_FILES = [*PHASE2_MODULES, REPO_ROOT / "config" / "difficulty_levels.json"]


@pytest.mark.parametrize("path", PHASE2_FILES, ids=lambda p: p.name)
def test_files_are_plain_ascii_english(path: Path) -> None:
    """Rule 11: no Vietnamese (or any non-ASCII text) outside gui/strings_vi.py and the docs."""
    text = path.read_text(encoding="utf-8")
    bad = sorted({ch for ch in text if ord(ch) > 127})
    assert not bad, f"{path.name} contains non-ASCII characters {bad}"


@pytest.mark.parametrize("path", PHASE2_MODULES, ids=lambda p: p.name)
def test_modules_use_logging_never_print(path: Path) -> None:
    """Rule 10: leveled logging, no print."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    prints = [n.lineno for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "print"]
    assert prints == [], f"print() at lines {prints}"


@pytest.mark.parametrize("path", PHASE2_MODULES, ids=lambda p: p.name)
def test_modules_have_type_hints_on_every_function(path: Path) -> None:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    missing = [
        f"{n.name}:{n.lineno}"
        for n in ast.walk(tree)
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
        and (n.returns is None or any(a.annotation is None for a in n.args.args + n.args.kwonlyargs if a.arg not in ("self", "cls")))
    ]
    assert missing == [], f"functions without full type hints: {missing}"


def test_core_never_depends_on_the_console_runner_or_other_front_ends() -> None:
    """Rule 2 and layering: core/ imports neither gui/ nor the runner."""
    for path in CORE.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported = {
            (n.module or "") if isinstance(n, ast.ImportFrom) else alias.name
            for n in ast.walk(tree)
            for alias in (n.names if isinstance(n, (ast.Import, ast.ImportFrom)) else [])
        } | {n.module or "" for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        assert not any(m.startswith(("desktop_app.console_runner", "desktop_app.gui", "desktop_app.hardware", "desktop_app.vision"))
                       for m in imported), f"{path.name} imports {imported}"


def _hardware_numbers() -> set[float]:
    data = yaml.safe_load((REPO_ROOT / "config" / "hardware.yaml").read_text(encoding="utf-8"))
    found: set[float] = set()

    def walk(node: object) -> None:
        if isinstance(node, dict):
            for value in node.values():
                walk(value)
        elif isinstance(node, (list, tuple)):
            for value in node:
                walk(value)
        elif isinstance(node, (int, float)) and not isinstance(node, bool) and node not in (0, 1, 2):
            found.add(float(node))

    walk(data)
    return found | {408.5, 108.5, 191.5, 3200.0}


# Numbers that appear in hardware.yaml by coincidence but mean something else in these modules.
NOT_HARDWARE = {"game_state.py": {100.0}}  # 100 half-moves = the 50-move rule


@pytest.mark.parametrize("path", PHASE2_MODULES, ids=lambda p: p.name)
def test_no_hardware_constant_is_hard_coded(path: Path) -> None:
    """Rule 7: dimensions, pins and drive values come from config/hardware.yaml only."""
    hardware = _hardware_numbers() - NOT_HARDWARE.get(path.name, set())
    tree = ast.parse(path.read_text(encoding="utf-8"))
    hits = sorted(
        {
            (n.lineno, n.value)
            for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and isinstance(n.value, (int, float)) and not isinstance(n.value, bool)
            and float(n.value) in hardware
        }
    )
    # The runner's PLACEHOLDER gripper angles are not hardware.yaml values; they are flagged placeholders.
    assert hits == [], f"{path.name} hard-codes hardware numbers: {hits}"


def test_hardware_yaml_was_not_changed_by_phase_2() -> None:
    """Phase 2 adds no keys and changes no constant in hardware.yaml (it lists nothing to confirm)."""
    data = yaml.safe_load((REPO_ROOT / "config" / "hardware.yaml").read_text(encoding="utf-8"))
    assert data["to_confirm"] == []
    assert data["geometry"]["square_size_mm"] == 31.0 and data["drive"]["gear_ratio"] == 1.0
