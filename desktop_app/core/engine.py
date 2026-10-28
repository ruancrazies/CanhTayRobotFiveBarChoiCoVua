"""Stockfish over UCI with python-chess (docs/SPEC.md section 9).

* Every query is made with a FEN string taken from GameState. A fresh ``chess.Board`` is built
  from it for each attempt, so nothing can drift when the engine process restarts.
* If the engine does not answer within think time + margin (or dies), the process is restarted,
  the mode options are applied again and the FEN is reloaded. After ``max_restarts`` failed
  restarts :class:`EngineUnavailableError` is raised.
* The five modes and the engine path come from the ``engine`` section of ``config/settings.yaml``. A new mode
  takes effect from the next engine move (options are applied lazily, right before a query).
* The engine path can be overridden without editing a tracked file. Precedence, highest first:
  the ``STOCKFISH_PATH`` environment variable (blank counts as unset), then ``engine.path`` of the optional,
  git-ignored ``config/local.yaml``, then ``engine.path`` in ``config/settings.yaml`` (the typed loader
  ``desktop_app.utils.settings`` applies it). ``EngineConfig.path_source`` says which one won.
* The UCI process sits behind the small :class:`UciBackend` protocol, so tests use a fake engine
  and never need the Stockfish binary.

The engine never touches GameState; it only returns a legal move.
"""

from __future__ import annotations

import asyncio
import dataclasses
import logging
import threading
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Callable, Mapping, Optional, Protocol, Union

import chess
import chess.engine

from desktop_app.core.event_bus import EventBus, Notification
from desktop_app.utils.settings import (
    ConfigError,
    ModesConfig,
    PathSource,
    Settings,
    load_settings,
    to_seconds,
)

logger = logging.getLogger(__name__)

MODE_KEYS = tuple(f.name for f in dataclasses.fields(ModesConfig))  # the five modes, in the SPEC 9 order

OptionValue = Union[bool, int, str]


class EngineError(Exception):
    """Base class of every engine error."""


class EngineConfigError(EngineError):
    """The engine section of config/settings.yaml (or the overlay) is missing or invalid."""


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


@dataclass(frozen=True)
class EngineConfig:
    path: str
    answer_margin_s: float
    max_restarts: int
    default_options: Mapping[str, OptionValue]
    default_mode: str
    modes: Mapping[str, DifficultyMode]
    path_source: PathSource = PathSource.SETTINGS_FILE


def engine_config_from_settings(settings: Settings) -> EngineConfig:
    """The engine configuration of already loaded settings (think times and the margin converted to seconds)."""
    section = settings.engine
    modes = {
        key: DifficultyMode(
            key=key,
            options=mode.options,
            think_time_s=to_seconds(mode.think_time_ms),
            max_depth=mode.max_depth,
        )
        for key, mode in section.modes.as_dict().items()
    }
    return EngineConfig(
        path=section.path,
        answer_margin_s=to_seconds(section.answer_margin_ms),
        max_restarts=section.max_restarts,
        default_options=MappingProxyType(dict(section.default_options)),
        default_mode=section.default_mode,
        modes=MappingProxyType(modes),
        path_source=settings.path_source,
    )


def load_engine_config(
    path: Optional[Path] = None,
    *,
    environ: Optional[Mapping[str, str]] = None,
    local_config_path: Optional[Path] = None,
) -> EngineConfig:
    """Load and validate the engine section of config/settings.yaml (or ``path``).

    The engine path may be overridden by ``STOCKFISH_PATH`` or ``config/local.yaml`` (see
    :func:`desktop_app.utils.settings.load_settings`). ``environ`` and ``local_config_path`` exist so tests and
    callers do not depend on the real environment or on a file in the repository.
    """
    try:
        settings = load_settings(path, local_path=local_config_path, environ=environ)
    except ConfigError as exc:
        raise EngineConfigError(str(exc)) from exc
    return engine_config_from_settings(settings)


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
