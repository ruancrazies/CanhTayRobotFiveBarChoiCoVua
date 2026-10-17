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


@pytest.fixture
def hardware_yaml_data() -> dict[str, Any]:
    """A fresh, mutable copy of the parsed default config/hardware.yaml."""
    with (REPO_ROOT / "config" / "hardware.yaml").open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)
