"""Stockfish over UCI with python-chess (docs/SPEC.md section 9).

* Every query is made with a FEN string taken from GameState. A fresh ``chess.Board`` is built
  from it for each attempt, so nothing can drift when the engine process restarts.
* If the engine does not answer within think time + margin (or dies), the process is restarted,
  the mode options are applied again and the FEN is reloaded. After ``max_restarts`` failed
  restarts :class:`EngineUnavailableError` is raised.
* The five modes and the engine path come from ``config/difficulty_levels.json``. A new mode
  takes effect from the next engine move (options are applied lazily, right before a query).
* The engine path can be overridden without editing a tracked file. Precedence, highest first:
  the ``STOCKFISH_PATH`` environment variable (blank counts as unset), then the optional,
  git-ignored ``config/local.json`` (``{"stockfish_path": "..."}``), then ``engine.path`` in
  ``difficulty_levels.json``. ``EngineConfig.path_source`` says which one won.
* The UCI process sits behind the small :class:`UciBackend` protocol, so tests use a fake engine
  and never need the Stockfish binary.

The engine never touches GameState; it only returns a legal move.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Callable, Mapping, Optional, Protocol, Union

import chess
import chess.engine

from desktop_app.core.event_bus import EventBus, Notification

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "difficulty_levels.json"
DEFAULT_LOCAL_CONFIG_PATH = REPO_ROOT / "config" / "local.json"  # optional, never tracked by git
STOCKFISH_PATH_ENV = "STOCKFISH_PATH"
LOCAL_CONFIG_KEYS = frozenset({"stockfish_path"})

MODE_KEYS = ("beginner", "amateur", "player", "master", "grandmaster")

OptionValue = Union[bool, int, str]


class EngineError(Exception):
    """Base class of every engine error."""


class EngineConfigError(EngineError):
    """config/difficulty_levels.json is missing or invalid."""


class EngineTimeoutError(EngineError):
    """The engine did not answer in time (raised by a backend)."""


class EngineCrashedError(EngineError):
    """The engine process died or broke the protocol (raised by a backend)."""


class EngineUnavailableError(EngineError):
    """The engine could not be started, or kept failing after every restart."""


@dataclass(frozen=True)
class EngineRestarted(Notification):
    """Published when the engine is restarted (the GUI shows ``msg.engine_restart``)."""

    reason: str
    attempt: int


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class DifficultyMode:
    key: str
    options: Mapping[str, OptionValue]
    think_time_s: float
    max_depth: Optional[int]


class PathSource(str, Enum):
    """Where ``EngineConfig.path`` came from (highest precedence first)."""

    ENVIRONMENT = "environment"  # the STOCKFISH_PATH variable
    LOCAL_FILE = "local_file"  # config/local.json
    DIFFICULTY_FILE = "difficulty_file"  # engine.path in difficulty_levels.json


@dataclass(frozen=True)
class EngineConfig:
    path: str
    answer_margin_s: float
    max_restarts: int
    default_options: Mapping[str, OptionValue]
    default_mode: str
    modes: Mapping[str, DifficultyMode]
    path_source: PathSource = PathSource.DIFFICULTY_FILE


def _is_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _options(name: str, raw: object) -> Mapping[str, OptionValue]:
    if not isinstance(raw, dict):
        raise EngineConfigError(f"{name} must be an object of UCI options")
    for key, value in raw.items():
        if not isinstance(key, str) or not isinstance(value, (bool, int, str)):
            raise EngineConfigError(f"{name}: option {key!r} must be a bool, int or string, got {value!r}")
    return MappingProxyType(dict(raw))


def _read_local_stockfish_path(path: Path) -> Optional[str]:
    """The ``stockfish_path`` in the local override file, or None when it overrides nothing.

    A missing file or an empty object overrides nothing. A file that exists but is broken is an
    error, never silently ignored: the operator wrote it to change the engine and would otherwise
    be running a different binary than they think.
    """
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    except OSError as exc:
        raise EngineConfigError(f"cannot read {path}: {exc}") from exc
    try:
        data = json.loads(text)
    except ValueError as exc:
        raise EngineConfigError(f"{path} is not valid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise EngineConfigError(f"{path}: top level must be an object")
    if unknown := sorted(set(data) - LOCAL_CONFIG_KEYS):
        raise EngineConfigError(f"{path}: unknown key(s) {unknown}; allowed: {sorted(LOCAL_CONFIG_KEYS)}")
    if "stockfish_path" not in data:
        return None
    value = data["stockfish_path"]
    if not isinstance(value, str) or not value.strip():
        raise EngineConfigError(f"{path}: stockfish_path must be a non-empty string, got {value!r}")
    return value.strip()


def resolve_engine_path(
    default_path: str,
    *,
    environ: Optional[Mapping[str, str]] = None,
    local_config_path: Optional[Path] = None,
) -> tuple[str, PathSource]:
    """Pick the Stockfish path: STOCKFISH_PATH, then config/local.json, then ``default_path``.

    The local file is read only when the environment variable does not decide. A blank
    ``STOCKFISH_PATH`` counts as unset (a shell that exports an empty variable is common).
    """
    env = os.environ if environ is None else environ
    from_env = env.get(STOCKFISH_PATH_ENV, "").strip()
    if from_env:
        return from_env, PathSource.ENVIRONMENT
    local_path = Path(local_config_path) if local_config_path is not None else DEFAULT_LOCAL_CONFIG_PATH
    from_local = _read_local_stockfish_path(local_path)
    if from_local is not None:
        return from_local, PathSource.LOCAL_FILE
    return default_path, PathSource.DIFFICULTY_FILE


def load_engine_config(
    path: Optional[Path] = None,
    *,
    environ: Optional[Mapping[str, str]] = None,
    local_config_path: Optional[Path] = None,
) -> EngineConfig:
    """Load and validate config/difficulty_levels.json (or ``path``).

    The engine path may be overridden by ``STOCKFISH_PATH`` or ``config/local.json`` (see
    :func:`resolve_engine_path`). ``environ`` and ``local_config_path`` exist so tests and
    callers do not depend on the real environment or on a file in the repository.
    """
    source = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    try:
        data = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise EngineConfigError(f"cannot read {source}: {exc}") from exc
    if not isinstance(data, dict):
        raise EngineConfigError(f"{source}: top level must be an object")
    try:
        engine = data["engine"]
        path_value = engine["path"]
        margin = engine["answer_margin_s"]
        restarts = engine["max_restarts"]
        raw_modes = data["modes"]
        default_mode = data["default_mode"]
    except (KeyError, TypeError) as exc:
        raise EngineConfigError(f"{source}: missing key {exc}") from exc

    if not isinstance(path_value, str) or not path_value.strip():
        raise EngineConfigError("engine.path must be a non-empty string")
    if not _is_number(margin) or margin <= 0:
        raise EngineConfigError("engine.answer_margin_s must be a number > 0")
    if isinstance(restarts, bool) or not isinstance(restarts, int) or restarts < 0:
        raise EngineConfigError("engine.max_restarts must be an integer >= 0")
    if not isinstance(raw_modes, dict) or set(raw_modes) != set(MODE_KEYS):
        raise EngineConfigError(f"modes must be exactly {list(MODE_KEYS)}, got {list(raw_modes) if isinstance(raw_modes, dict) else raw_modes!r}")
    if default_mode not in MODE_KEYS:
        raise EngineConfigError(f"default_mode must be one of {list(MODE_KEYS)}, got {default_mode!r}")

    modes: dict[str, DifficultyMode] = {}
    for key in MODE_KEYS:
        raw = raw_modes[key]
        if not isinstance(raw, dict):
            raise EngineConfigError(f"modes.{key} must be an object")
        think = raw.get("think_time_s")
        depth = raw.get("max_depth")
        if not _is_number(think) or think <= 0:
            raise EngineConfigError(f"modes.{key}.think_time_s must be a number > 0")
        if depth is not None and (isinstance(depth, bool) or not isinstance(depth, int) or depth < 1):
            raise EngineConfigError(f"modes.{key}.max_depth must be null or an integer >= 1")
        modes[key] = DifficultyMode(
            key=key,
            options=_options(f"modes.{key}.options", raw.get("options", {})),
            think_time_s=float(think),
            max_depth=depth,
        )
    resolved_path, path_source = resolve_engine_path(path_value, environ=environ, local_config_path=local_config_path)
    if path_source is not PathSource.DIFFICULTY_FILE:
        logger.info(
            "Engine path %r comes from %s (it overrides %r from %s)", resolved_path, path_source.value, path_value, source.name
        )
    return EngineConfig(
        path=resolved_path,
        answer_margin_s=float(margin),
        max_restarts=restarts,
        default_options=_options("default_options", data.get("default_options", {})),
        default_mode=default_mode,
        modes=MappingProxyType(modes),
        path_source=path_source,
    )


# --------------------------------------------------------------------------- #
# Backends
# --------------------------------------------------------------------------- #


class UciBackend(Protocol):
    """What the Engine needs from a UCI process (the real one, or a fake in tests)."""

    def configure(self, options: Mapping[str, OptionValue]) -> None: ...

    def play(
        self, board: chess.Board, *, time_s: float, depth: Optional[int], answer_margin_s: float
    ) -> chess.Move:
        """Best move for ``board``. Raise EngineTimeoutError / EngineCrashedError on failure."""
        ...

    def quit(self) -> None: ...


class StockfishBackend:
    """Real Stockfish through ``chess.engine.SimpleEngine``."""

    def __init__(self, path: str) -> None:
        try:
            self._engine = chess.engine.SimpleEngine.popen_uci(path)
        except (OSError, chess.engine.EngineError, chess.engine.EngineTerminatedError) as exc:
            raise EngineUnavailableError(f"cannot start the engine {path!r}: {exc}") from exc

    def configure(self, options: Mapping[str, OptionValue]) -> None:
        try:
            self._engine.configure(dict(options))
        except (chess.engine.EngineTerminatedError, TimeoutError, asyncio.TimeoutError) as exc:
            raise EngineCrashedError(f"engine died while it was being configured: {exc}") from exc
        except chess.engine.EngineError as exc:
            raise EngineConfigError(f"the engine rejected the options {dict(options)}: {exc}") from exc

    def play(
        self, board: chess.Board, *, time_s: float, depth: Optional[int], answer_margin_s: float
    ) -> chess.Move:
        # python-chess waits ``timeout + limit.time`` before it gives up.
        self._engine.timeout = answer_margin_s
        try:
            result = self._engine.play(board, chess.engine.Limit(time=time_s, depth=depth))
        except (TimeoutError, asyncio.TimeoutError) as exc:
            raise EngineTimeoutError(f"no answer within {time_s + answer_margin_s:g} s") from exc
        except chess.engine.EngineTerminatedError as exc:
            raise EngineCrashedError(f"the engine process died: {exc}") from exc
        except chess.engine.EngineError as exc:
            raise EngineCrashedError(f"engine protocol error: {exc}") from exc
        if result.move is None:
            raise EngineCrashedError("the engine returned no move")
        return result.move

    def quit(self) -> None:
        try:
            self._engine.quit()
        except Exception:  # the process may already be dead or hung
            logger.debug("quit failed, closing the engine transport", exc_info=True)
            try:
                self._engine.close()
            except Exception:
                logger.debug("close failed", exc_info=True)


# --------------------------------------------------------------------------- #
# Engine
# --------------------------------------------------------------------------- #


class Engine:
    """Stockfish with the five difficulty modes.

    Args:
        config: loaded with :func:`load_engine_config`.
        backend_factory: builds a started backend; default launches ``config.path``.
        bus: optional Event Bus that receives :class:`EngineRestarted`.
        mode: initial mode key (default: ``config.default_mode``).
    """

    def __init__(
        self,
        config: EngineConfig,
        *,
        backend_factory: Optional[Callable[[], UciBackend]] = None,
        bus: Optional[EventBus] = None,
        mode: Optional[str] = None,
    ) -> None:
        self._config = config
        self._factory = backend_factory or (lambda: StockfishBackend(config.path))
        self._bus = bus
        self._lock = threading.Lock()
        self._backend: Optional[UciBackend] = None
        self._applied_mode: Optional[str] = None
        self._mode = config.default_mode
        self.set_mode(mode or config.default_mode)

    @property
    def mode(self) -> str:
        return self._mode

    def set_mode(self, key: str) -> None:
        """Select a mode; it is applied from the next ``best_move`` call."""
        if key not in self._config.modes:
            raise EngineConfigError(f"unknown mode {key!r}, expected one of {list(self._config.modes)}")
        self._mode = key

    def best_move(self, fen: str) -> chess.Move:
        """Best move for the position ``fen`` (always the FEN from GameState)."""
        try:
            probe = chess.Board(fen)
        except ValueError as exc:
            raise EngineError(f"invalid FEN {fen!r}: {exc}") from exc
        if not any(probe.legal_moves):
            raise EngineError(f"no legal move in {fen!r}")

        with self._lock:
            mode = self._config.modes[self._mode]
            last_error: Optional[EngineError] = None
            for attempt in range(self._config.max_restarts + 1):
                board = chess.Board(fen)  # reload the FEN for every attempt
                try:
                    backend = self._ensure_backend()
                    if self._applied_mode != mode.key:
                        backend.configure({**self._config.default_options, **mode.options})
                        self._applied_mode = mode.key
                    move = backend.play(
                        board,
                        time_s=mode.think_time_s,
                        depth=mode.max_depth,
                        answer_margin_s=self._config.answer_margin_s,
                    )
                except (EngineTimeoutError, EngineCrashedError) as exc:
                    last_error = exc
                    logger.warning("engine failure on attempt %d: %s", attempt + 1, exc)
                    self._discard_backend()
                    if attempt < self._config.max_restarts:
                        self._announce_restart(str(exc), attempt + 1)
                    continue
                if move not in probe.legal_moves:
                    self._discard_backend()
                    raise EngineError(f"the engine returned an illegal move {move.uci()} for {fen!r}")
                return move
            raise EngineUnavailableError(
                f"the engine did not answer after {self._config.max_restarts + 1} attempt(s): {last_error}"
            ) from last_error

    def close(self) -> None:
        """Stop the engine process (safe to call more than once)."""
        with self._lock:
            self._discard_backend()

    def __enter__(self) -> "Engine":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    # ------------------------------------------------------------------ #

    def _ensure_backend(self) -> UciBackend:
        if self._backend is None:
            self._backend = self._factory()
            self._applied_mode = None  # a new process has none of our options
        return self._backend

    def _discard_backend(self) -> None:
        backend, self._backend = self._backend, None
        self._applied_mode = None
        if backend is not None:
            try:
                backend.quit()
            except Exception:
                logger.debug("ignoring an error while stopping the engine", exc_info=True)

    def _announce_restart(self, reason: str, attempt: int) -> None:
        if self._bus is not None:
            self._bus.publish(EngineRestarted(reason=reason, attempt=attempt))
