"""Shared pytest setup."""

from __future__ import annotations

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


@pytest.fixture
def hardware_yaml_data() -> dict[str, Any]:
    """A fresh, mutable copy of the parsed default config/hardware.yaml."""
    with (REPO_ROOT / "config" / "hardware.yaml").open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)
