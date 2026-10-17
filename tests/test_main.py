"""Tests for the application entry point."""

from __future__ import annotations

import logging
from collections.abc import Iterator
from pathlib import Path

import pytest

from desktop_app.main import main
from desktop_app.utils.logging_setup import reset_logging


@pytest.fixture(autouse=True)
def _clean_logging() -> Iterator[None]:
    root_level = logging.getLogger().level
    yield
    reset_logging()
    logging.getLogger().setLevel(root_level)


def test_main_starts_with_the_default_config() -> None:
    assert main(["--log-level", "WARNING"]) == 0


def test_main_refuses_to_start_on_a_bad_config(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.ERROR, logger="desktop_app.main"):
        exit_code = main(["--hardware-config", str(tmp_path / "missing.yaml")])
    assert exit_code == 2
    assert any("Cannot start" in record.getMessage() for record in caplog.records)
