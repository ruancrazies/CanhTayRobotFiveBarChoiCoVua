"""Application entry point.

Phase 0 only wires the foundations together (logging, hardware config, Event
Bus). The FSM, hardware link, vision and GUI are added in later phases.

Run from the repository root with: ``python -m desktop_app.main``
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Sequence

from desktop_app.core.event_bus import EventBus
from desktop_app.utils.hardware_config import ConfigError, load_hardware_config
from desktop_app.utils.logging_setup import configure_logging

logger = logging.getLogger(__name__)


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Five-bar chess robot desktop application")
    parser.add_argument(
        "--hardware-config",
        type=Path,
        default=None,
        help="path to hardware.yaml (default: config/hardware.yaml in the repository)",
    )
    parser.add_argument(
        "--log-level",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    configure_logging(getattr(logging, args.log_level))

    try:
        hardware = load_hardware_config(args.hardware_config)
    except ConfigError as exc:
        logger.error("Cannot start: %s", exc)
        return 2

    bus = EventBus()
    logger.debug("Event Bus created: %r", bus)
    logger.info(
        "Foundations ready: %d pulses/rev, %.4f deg joint resolution. FSM, hardware link, vision and GUI are not built yet.",
        hardware.drive.pulses_per_rev,
        hardware.drive.joint_resolution_deg,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
