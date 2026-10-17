"""Tests for the logging setup."""

from __future__ import annotations

import logging
from collections.abc import Iterator
from pathlib import Path

import pytest

from desktop_app.utils.logging_setup import configure_logging, reset_logging


@pytest.fixture(autouse=True)
def _clean_logging() -> Iterator[None]:
    root_level = logging.getLogger().level
    yield
    reset_logging()
    logging.getLogger().setLevel(root_level)


def _installed_handlers() -> list[logging.Handler]:
    return [h for h in logging.getLogger().handlers if getattr(h, "_chess_robot_handler", False)]


def test_configure_logging_installs_a_console_handler_at_the_requested_level() -> None:
    configure_logging(logging.WARNING)
    assert logging.getLogger().level == logging.WARNING
    assert len(_installed_handlers()) == 1


def test_configure_logging_is_idempotent() -> None:
    configure_logging(logging.INFO)
    configure_logging(logging.INFO)
    assert len(_installed_handlers()) == 1


def test_log_file_receives_messages_as_utf8(tmp_path: Path) -> None:
    log_file = tmp_path / "logs" / "app.log"
    configure_logging(logging.DEBUG, log_file)

    logging.getLogger("desktop_app.test").info("hello from the test")
    for handler in _installed_handlers():
        handler.flush()

    text = log_file.read_text(encoding="utf-8")
    assert "hello from the test" in text
    assert "INFO" in text
    assert "desktop_app.test" in text


def test_reset_logging_removes_only_our_handlers() -> None:
    foreign = logging.NullHandler()
    logging.getLogger().addHandler(foreign)
    try:
        configure_logging(logging.INFO)
        reset_logging()
        assert _installed_handlers() == []
        assert foreign in logging.getLogger().handlers
    finally:
        logging.getLogger().removeHandler(foreign)
