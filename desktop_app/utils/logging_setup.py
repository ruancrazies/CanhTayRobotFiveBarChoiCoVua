"""Leveled logging setup (SPEC rule 10: use logging, never print).

All log messages are English. ``configure_logging`` is idempotent: calling it
again replaces the handlers it installed earlier and leaves foreign handlers
(for example pytest's caplog) untouched.
"""

from __future__ import annotations

import logging
import logging.handlers
from pathlib import Path

LOG_FORMAT = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"

_HANDLER_MARK = "_chess_robot_handler"
_MAX_LOG_BYTES = 1_000_000
_BACKUP_COUNT = 3


def reset_logging() -> None:
    """Remove and close every handler installed by ``configure_logging``."""
    root = logging.getLogger()
    for handler in list(root.handlers):
        if getattr(handler, _HANDLER_MARK, False):
            root.removeHandler(handler)
            handler.close()


def configure_logging(level: int = logging.INFO, log_file: Path | None = None) -> None:
    """Log to stderr and, optionally, to a rotating UTF-8 file."""
    reset_logging()
    root = logging.getLogger()
    root.setLevel(level)

    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file is not None:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(
            logging.handlers.RotatingFileHandler(
                log_file, maxBytes=_MAX_LOG_BYTES, backupCount=_BACKUP_COUNT, encoding="utf-8"
            )
        )

    formatter = logging.Formatter(LOG_FORMAT)
    for handler in handlers:
        handler.setFormatter(formatter)
        setattr(handler, _HANDLER_MARK, True)
        root.addHandler(handler)
