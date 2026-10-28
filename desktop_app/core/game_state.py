"""GameState: the single source of truth for the chess position (docs/SPEC.md sections 2 and 7.1).

Wraps ``chess.Board``. The position changes in exactly one place, :meth:`GameState.commit_move`.
Everything else is read-only or works on copies:

* ``preview_move`` returns a copy of the board after a trial move.
* ``board`` returns a copy, ``legal_moves`` returns a new list.
* ``set_pending_move`` / ``clear_pending_move`` only manage the *proposed* engine move; the
  position is untouched. Loading a game is done by the ``load`` / ``new_game`` factories, which
  build a new object instead of mutating an existing one.

Persistence (``data_dir``, default ``data/games``):

* ``current.fen``  position after the last commit,
* ``current.pgn``  full game with a ``source=human|engine`` comment on every move,
* ``pending.json`` the engine move together with the position it was computed for, kept from
  ENGINE_THINK until VERIFY succeeds.

Every file is written atomically (temp file, fsync, rename). ``commit_move`` persists before it
changes memory, so a failed write leaves the object exactly as it was.

This module knows nothing about the FSM, the camera or the robot.
"""

from __future__ import annotations

import io
import json
import logging
import os
import threading
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Optional, Sequence

import chess
import chess.pgn

from desktop_app.core.event_bus import EventBus, Notification
from desktop_app.utils.settings import get_settings

logger = logging.getLogger(__name__)



def default_games_dir() -> Path:
    """The saved-game folder (``paths.games_dir`` of config/settings.yaml)."""
    return get_settings().games_dir


# --------------------------------------------------------------------------- #
# Errors, enums, value objects, events
# --------------------------------------------------------------------------- #


class GameStateError(ValueError):
    """Base class of every GameState error."""


class IllegalMoveError(GameStateError):
    """The move is not legal in the current position."""


class GameOverError(GameStateError):
    """The game has ended; no further move can be committed."""


class PendingMoveError(GameStateError):
    """A pending-move rule was violated (see :meth:`GameState.commit_move`)."""


class PersistenceError(GameStateError):
    """A game file could not be written or is unreadable."""


class MoveSource(str, Enum):
    """Who made a committed move."""

    HUMAN = "human"
    ENGINE = "engine"


@dataclass(frozen=True)
class MoveRecord:
    """One committed move. Square numbers and piece types follow python-chess."""

    uci: str
    san: str
    source: MoveSource
    color: chess.Color
    piece: chess.PieceType
    from_square: chess.Square
    to_square: chess.Square
    captured: Optional[chess.PieceType]
    en_passant: bool
    castling: bool
    promotion: Optional[chess.PieceType]
    fen_after: str


@dataclass(frozen=True)
class GameOutcome:
    """Why and how the game ended."""

    result: str  # "1-0", "0-1" or "1/2-1/2"
    termination: str  # python-chess Termination name, lower case: "checkmate", "stalemate", ...
    winner: Optional[chess.Color]


@dataclass(frozen=True)
class GameUpdated(Notification):
    """Published on the Event Bus after every commit (SPEC 7.1 ``game_updated``)."""

    fen: str
    move_uci: Optional[str]
    san: Optional[str]
    source: Optional[str]
    game_over: bool
    result: Optional[str]
    termination: Optional[str]


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #


def _atomic_write(path: Path, text: str) -> None:
    """Write ``text`` to ``path`` so a crash never leaves a half-written file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    try:
        with tmp.open("w", encoding="utf-8", newline="\n") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
    except OSError as exc:
        raise PersistenceError(f"cannot write {path}: {exc}") from exc


def _record_for(board: chess.Board, move: chess.Move, source: MoveSource) -> MoveRecord:
    """Describe ``move`` played from ``board`` (call BEFORE pushing the move)."""
    piece = board.piece_at(move.from_square)
    if piece is None:  # pragma: no cover - legal moves always start on a piece
        raise IllegalMoveError(f"no piece on {chess.square_name(move.from_square)}")
    en_passant = board.is_en_passant(move)
    captured_piece = board.piece_at(move.to_square)
    captured = chess.PAWN if en_passant else (captured_piece.piece_type if captured_piece else None)
    san = board.san(move)
    after = board.copy(stack=False)
    after.push(move)
    return MoveRecord(
        uci=move.uci(),
        san=san,
        source=source,
        color=piece.color,
        piece=piece.piece_type,
        from_square=move.from_square,
        to_square=move.to_square,
        captured=captured,
        en_passant=en_passant,
        castling=board.is_castling(move),
        promotion=move.promotion,
        fen_after=after.fen(),
    )


def _outcome_of(board: chess.Board) -> Optional[GameOutcome]:
    """Checkmate, stalemate, insufficient material, and draws that have actually happened.

    Threefold repetition and the 50-move rule end the game only once the position has really
    repeated / the clock has reached 100 half-moves. python-chess' ``claim_draw=True`` would also
    end the game one move EARLY (when a move merely could repeat the position), so it is not used.
    """
    outcome = board.outcome(claim_draw=False)  # includes fivefold repetition and the 75-move rule
    if outcome is not None:
        return GameOutcome(result=outcome.result(), termination=outcome.termination.name.lower(), winner=outcome.winner)
    if board.is_repetition(3):
        return GameOutcome(result="1/2-1/2", termination="threefold_repetition", winner=None)
    if board.halfmove_clock >= 100:
        return GameOutcome(result="1/2-1/2", termination="fifty_moves", winner=None)
    return None


def _build_pgn(initial_fen: str, history: Sequence[MoveRecord], outcome: Optional[GameOutcome]) -> str:
    game = chess.pgn.Game()
    game.headers["Event"] = "Five-bar chess robot"
    game.headers["Result"] = outcome.result if outcome else "*"
    if initial_fen != chess.STARTING_FEN:
        game.setup(chess.Board(initial_fen))
    node: chess.pgn.GameNode = game
    for record in history:
        node = node.add_variation(chess.Move.from_uci(record.uci))
        node.comment = f"source={record.source.value}"
    exporter = chess.pgn.StringExporter(headers=True, variations=False, comments=True)
    return game.accept(exporter) + "\n"


# --------------------------------------------------------------------------- #
# GameState
# --------------------------------------------------------------------------- #


class GameState:
    """Wraps ``chess.Board``; the only place allowed to change the position.

    Args:
        fen: starting position (default: the standard start position).
        data_dir: folder for ``current.fen``, ``current.pgn`` and ``pending.json``.
            ``None`` keeps everything in memory (no persistence).
        bus: Event Bus that receives ``GameUpdated`` after every commit.

    Use ``new_game`` to start and persist a game, and ``load`` to resume a saved one.
    """

    def __init__(
        self,
        *,
        fen: Optional[str] = None,
        data_dir: Optional[Path] = None,
        bus: Optional[EventBus] = None,
    ) -> None:
        try:
            board = chess.Board(fen) if fen is not None else chess.Board()
        except ValueError as exc:
            raise GameStateError(f"invalid FEN {fen!r}: {exc}") from exc
        self._init(board, board.fen(), [], data_dir, bus, None)

    def _init(
        self,
        board: chess.Board,
        initial_fen: str,
        history: list[MoveRecord],
        data_dir: Optional[Path],
        bus: Optional[EventBus],
        pending: Optional[chess.Move],
    ) -> None:
        self._lock = threading.RLock()
        self._board = board
        self._initial_fen = initial_fen
        self._history = history
        self._data_dir = Path(data_dir) if data_dir is not None else None
        self._bus = bus
        self._pending = pending

    # ------------------------------------------------------------------ #
    # Factories (build a new object; never mutate an existing position)
    # ------------------------------------------------------------------ #

    @classmethod
    def new_game(
        cls, data_dir: Optional[Path] = None, bus: Optional[EventBus] = None, *, fen: Optional[str] = None
    ) -> "GameState":
        """Start a fresh game and persist it. Any pending.json of an older game is removed."""
        state = cls(fen=fen, data_dir=data_dir, bus=bus)
        if state._data_dir is not None:
            state._save_position(state._board, state._history, state._initial_fen)
            state._remove_pending_file()
        return state

    @classmethod
    def load(cls, data_dir: Path, bus: Optional[EventBus] = None) -> Optional["GameState"]:
        """Reload the game saved in ``data_dir``; ``None`` when there is no ``current.fen``.

        The position comes from ``current.fen``. The PGN supplies the move history when it
        agrees with the FEN. A pending.json is restored only when it was computed for exactly
        this position; one that is already applied (crash between the FEN write and the
        pending.json removal) or that does not fit is discarded with a warning.
        """
        data_dir = Path(data_dir)
        fen_path = data_dir / get_settings().paths.fen_file
        if not fen_path.exists():
            return None
        try:
            fen = fen_path.read_text(encoding="utf-8").strip()
            board = chess.Board(fen)
        except (OSError, ValueError) as exc:
            raise PersistenceError(f"cannot load {fen_path}: {exc}") from exc

        initial_fen = board.fen()
        history: list[MoveRecord] = []
        replayed = cls._replay_pgn(data_dir / get_settings().paths.pgn_file, board.fen())
        if replayed is not None:
            replay_board, initial_fen, history = replayed
            board = replay_board

        state = cls.__new__(cls)
        state._init(board, initial_fen, history, data_dir, bus, None)
        state._pending = state._load_pending()
        return state

    @staticmethod
    def _replay_pgn(path: Path, expected_fen: str) -> Optional[tuple[chess.Board, str, list[MoveRecord]]]:
        if not path.exists():
            logger.warning("%s is missing; the move history is lost, the position is kept", path.name)
            return None
        try:
            game = chess.pgn.read_game(io.StringIO(path.read_text(encoding="utf-8")))
        except (OSError, ValueError) as exc:
            logger.warning("cannot read %s (%s); the move history is lost", path, exc)
            return None
        if game is None:
            return None
        board = game.board()
        initial_fen = board.fen()
        history: list[MoveRecord] = []
        try:
            for node in game.mainline():
                move = node.move
                if move is None or move not in board.legal_moves:
                    raise ValueError(f"illegal move {move} in PGN")
                comment = node.comment.strip()
                source = MoveSource.ENGINE if comment == f"source={MoveSource.ENGINE.value}" else MoveSource.HUMAN
                history.append(_record_for(board, move, source))
                board.push(move)
        except ValueError as exc:
            logger.warning("PGN does not replay (%s); the move history is lost", exc)
            return None
        if board.fen() != expected_fen:
            logger.warning("PGN ends in a different position than current.fen; the move history is lost")
            return None
        return board, initial_fen, history

    def _load_pending(self) -> Optional[chess.Move]:
        path = self._pending_path
        if path is None or not path.exists():
            return None
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            move = chess.Move.from_uci(str(data["pending_move"]))
            fen_before = str(data["current_fen"])
        except (OSError, ValueError, KeyError, TypeError) as exc:
            logger.warning("ignoring unreadable %s: %s", path.name, exc)
            self._remove_pending_file()
            return None
        if fen_before == self._board.fen() and move in self._board.legal_moves:
            return move
        # Crash between writing current.fen and removing pending.json: the move was committed.
        try:
            before = chess.Board(fen_before)
            if move in before.legal_moves:
                before.push(move)
                if before.fen() == self._board.fen():
                    logger.info("pending move %s was already committed; discarding pending.json", move.uci())
                    self._remove_pending_file()
                    return None
        except ValueError:
            pass
        logger.warning("pending.json does not fit the saved position; discarding it")
        self._remove_pending_file()
        return None

    # ------------------------------------------------------------------ #
    # Read-only view
    # ------------------------------------------------------------------ #

    def fen(self) -> str:
        """FEN of the current position; the only thing sent to Stockfish."""
        with self._lock:
            return self._board.fen()

    @property
    def board(self) -> chess.Board:
        """A copy of the board; changing it never affects the game."""
        with self._lock:
            return self._board.copy(stack=True)

    @property
    def turn(self) -> chess.Color:
        with self._lock:
            return self._board.turn

    @property
    def ply(self) -> int:
        with self._lock:
            return self._board.ply()

    @property
    def legal_moves(self) -> list[chess.Move]:
        """Legal moves of the current position (for the Move Detector)."""
        with self._lock:
            return list(self._board.legal_moves)

    @property
    def history(self) -> tuple[MoveRecord, ...]:
        with self._lock:
            return tuple(self._history)

    @property
    def pending_move(self) -> Optional[chess.Move]:
        with self._lock:
            return self._pending

    @property
    def outcome(self) -> Optional[GameOutcome]:
        with self._lock:
            return _outcome_of(self._board)

    @property
    def is_game_over(self) -> bool:
        return self.outcome is not None

    @property
    def data_dir(self) -> Optional[Path]:
        return self._data_dir

    def san(self, move: chess.Move) -> str:
        with self._lock:
            return self._board.san(move)

    def pgn(self) -> str:
        with self._lock:
            return _build_pgn(self._initial_fen, self._history, _outcome_of(self._board))

    def preview_move(self, move: chess.Move) -> chess.Board:
        """Copy of the board after ``move``; the real position is not changed (used by VERIFY)."""
        with self._lock:
            self._require_legal(move)
            preview = self._board.copy(stack=True)
            preview.push(move)
            return preview

    # ------------------------------------------------------------------ #
    # Pending engine move (not part of the position)
    # ------------------------------------------------------------------ #

    def set_pending_move(self, move: chess.Move) -> None:
        """Store the engine's proposed move with the position it was computed for."""
        with self._lock:
            if self._outcome_unlocked() is not None:
                raise GameOverError("the game is over")
            self._require_legal(move)
            if self._pending is not None:
                raise PendingMoveError(f"pending move {self._pending.uci()} must be resolved first")
            if self._data_dir is not None:
                payload = {"pending_move": move.uci(), "current_fen": self._board.fen()}
                _atomic_write(self._data_dir / get_settings().paths.pending_file, json.dumps(payload, indent=2) + "\n")
            self._pending = move

    def clear_pending_move(self) -> None:
        """Cancel the pending move (the position is untouched)."""
        with self._lock:
            self._remove_pending_file()
            self._pending = None

    # ------------------------------------------------------------------ #
    # The only mutation of the position
    # ------------------------------------------------------------------ #

    def commit_move(self, move: chess.Move, source: MoveSource | str) -> MoveRecord:
        """Validate and play ``move``, record it, save the files and emit ``GameUpdated``.

        Rules:
          * the move must be legal and the game not over;
          * ``source=engine`` requires ``move`` to be the stored pending move (it is cleared);
          * ``source=human`` is refused while a pending move exists.

        On any error nothing changes, including when a file cannot be written.
        """
        try:
            src = MoveSource(source)
        except ValueError:
            raise GameStateError(f"source must be 'human' or 'engine', got {source!r}") from None
        with self._lock:
            if self._outcome_unlocked() is not None:
                raise GameOverError("the game is over")
            self._require_legal(move)
            if src is MoveSource.ENGINE:
                if self._pending != move:
                    raise PendingMoveError(
                        f"engine move {move.uci()} does not match the pending move "
                        f"{self._pending.uci() if self._pending else None}"
                    )
            elif self._pending is not None:
                raise PendingMoveError("a human move cannot be committed while an engine move is pending")

            record = _record_for(self._board, move, src)
            new_board = self._board.copy(stack=True)
            new_board.push(move)
            new_history = [*self._history, record]

            # Persist first: if this raises, memory is still the old, consistent state.
            self._save_position(new_board, new_history, self._initial_fen)

            self._board = new_board
            self._history = new_history
            if src is MoveSource.ENGINE:
                self._pending = None
                self._remove_pending_file()
            outcome = _outcome_of(self._board)
            event = GameUpdated(
                fen=self._board.fen(),
                move_uci=record.uci,
                san=record.san,
                source=src.value,
                game_over=outcome is not None,
                result=outcome.result if outcome else None,
                termination=outcome.termination if outcome else None,
            )
        logger.info("committed %s (%s) by %s", record.san, record.uci, src.value)
        if self._bus is not None:
            self._bus.publish(event)
        return record

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _outcome_unlocked(self) -> Optional[GameOutcome]:
        return _outcome_of(self._board)

    def _require_legal(self, move: Any) -> None:
        if not isinstance(move, chess.Move):
            raise IllegalMoveError(f"expected a chess.Move, got {move!r}")
        if move not in self._board.legal_moves:
            raise IllegalMoveError(f"{move.uci()} is not legal in {self._board.fen()}")

    @property
    def _pending_path(self) -> Optional[Path]:
        return self._data_dir / get_settings().paths.pending_file if self._data_dir is not None else None

    def _save_position(self, board: chess.Board, history: Sequence[MoveRecord], initial_fen: str) -> None:
        if self._data_dir is None:
            return
        pgn = _build_pgn(initial_fen, history, _outcome_of(board))
        # PGN first, FEN last: current.fen is the commit point that ``load`` trusts.
        _atomic_write(self._data_dir / get_settings().paths.pgn_file, pgn)
        _atomic_write(self._data_dir / get_settings().paths.fen_file, board.fen() + "\n")

    def _remove_pending_file(self) -> None:
        path = self._pending_path
        if path is None:
            return
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            raise PersistenceError(f"cannot remove {path}: {exc}") from exc
