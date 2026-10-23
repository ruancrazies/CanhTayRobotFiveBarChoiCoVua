"""Shared pytest setup."""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any

import pytest
import yaml

# Must be set before any Qt platform plugin is loaded so tests run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def _isolate_engine_path_overrides(monkeypatch: pytest.MonkeyPatch, tmp_path_factory: pytest.TempPathFactory) -> None:
    """A developer's own STOCKFISH_PATH or config/local.json must never change a test outcome.

    Tests that exercise the overrides pass their own ``environ`` / ``local_config_path`` (or set
    these two things themselves, after this fixture has run).
    """
    from desktop_app.core import engine

    monkeypatch.delenv(engine.STOCKFISH_PATH_ENV, raising=False)
    monkeypatch.setattr(engine, "DEFAULT_LOCAL_CONFIG_PATH", tmp_path_factory.getbasetemp() / "no-local-config.json")


@pytest.fixture(autouse=True)
def _drop_handlers_a_test_left_on_the_root_logger() -> Any:
    """A command-line test may install a console handler on the captured (later closed) stderr.

    Left behind, it makes every later log call from a background thread print "Logging error" noise and,
    on a slow machine, delays those threads. Handlers added during a test are removed when it ends.
    """
    root = logging.getLogger()
    before = list(root.handlers)
    level = root.level
    yield
    for handler in list(root.handlers):
        if handler not in before:
            root.removeHandler(handler)
    root.setLevel(level)


@pytest.fixture
def hardware_yaml_data() -> dict[str, Any]:
    """A fresh, mutable copy of the parsed default config/hardware.yaml."""
    with (REPO_ROOT / "config" / "hardware.yaml").open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)
