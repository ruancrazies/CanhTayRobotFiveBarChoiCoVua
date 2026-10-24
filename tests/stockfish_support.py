"""Where the real Stockfish binary is, decided ONCE, before any test fixture runs.

The autouse fixture in conftest.py hides STOCKFISH_PATH and config/local.json from every test, so that a
developer's machine never changes a test outcome. The one test that needs the real binary must still honour
those overrides. A ``skipif`` is evaluated at collection (before the fixture) and the test body runs after it,
so asking the question twice gave two answers. Here the question is asked once, at import, and both the
``skipif`` and the test body use the answer.
"""

from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Mapping, Optional

from desktop_app.core import engine as engine_module
from desktop_app.core.engine import EngineConfigError, load_engine_config


def locate_stockfish(environ: Mapping[str, str], local_config_path: Path) -> tuple[Optional[str], str]:
    """The usable binary (or None) and a one-line reason for the skip message.

    Precedence is the application's own (STOCKFISH_PATH, config/local.json, difficulty_levels.json), because
    it is the application's loader that decides.
    """
    try:
        configured = load_engine_config(environ=environ, local_config_path=local_config_path)
    except EngineConfigError as exc:
        return None, f"the Stockfish configuration is invalid: {exc}"
    found = shutil.which(configured.path) or (configured.path if Path(configured.path).is_file() else None)
    if found is None:
        return None, f"the Stockfish binary {configured.path!r} (from {configured.path_source.value}) is not installed"
    return found, ""


REAL_STOCKFISH_PATH, REAL_STOCKFISH_SKIP_REASON = locate_stockfish(dict(os.environ), engine_module.DEFAULT_LOCAL_CONFIG_PATH)
