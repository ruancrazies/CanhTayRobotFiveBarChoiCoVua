"""GameState: commit rules, preview, pending move, persistence, game end, events (SPEC 7.1)."""

from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Any

import chess
import chess.pgn
import pytest

from desktop_app.core.event_bus import EventBus
from desktop_app.core.game_state import (
    GameOverError,
    GameState,
    GameStateError,
    GameUpdated,
    IllegalMoveError,
    MoveSource,
    PendingMoveError,
    PersistenceError,
)
from desktop_app.utils.settings import get_settings
from phase2_support import E2E4, E7E5, MATE_IN_ONE_FEN

FEN_FILE = get_settings().paths.fen_file
PENDING_FILE = get_settings().paths.pending_file
PGN_FILE = get_settings().paths.pgn_file

STALEMATE_FEN = "7k/5Q2/6K1/8/8/8/8/8 b - - 0 1"
UCI = chess.Move.from_uci


@pytest.fixture
def game(tmp_path: Path) -> GameState:
    return GameState.new_game(tmp_path)


def play(game: GameState, *ucis: str) -> None:
    """Commit alternating human/engine moves (engine moves go through pending_move)."""
    for index, text in enumerate(ucis):
        move = UCI(text)
        if index % 2 == 0:
            game.commit_move(move, MoveSource.HUMAN)
        else:
            game.set_pending_move(move)
            game.commit_move(move, MoveSource.ENGINE)


# --------------------------------------------------------------------------- #
# Basics
# --------------------------------------------------------------------------- #


def test_new_game_starts_from_the_initial_position_and_saves_it(game: GameState, tmp_path: Path) -> None:
    assert game.fen() == chess.STARTING_FEN
    assert (tmp_path / FEN_FILE).read_text().strip() == chess.STARTING_FEN
    assert (tmp_path / PGN_FILE).exists()
    assert game.history == ()
    assert game.pending_move is None and not game.is_game_over
    assert len(game.legal_moves) == 20


def test_invalid_fen_is_refused() -> None:
    with pytest.raises(GameStateError):
        GameState(fen="not a fen")


def test_commit_records_the_move_and_changes_the_position(game: GameState) -> None:
    record = game.commit_move(E2E4, "human")
    assert record.uci == "e2e4" and record.san == "e4"
    assert record.source is MoveSource.HUMAN and record.color == chess.WHITE
    assert record.piece == chess.PAWN and record.captured is None and not record.castling
    assert game.fen() == record.fen_after
    assert game.turn == chess.BLACK
    assert game.history == (record,)


def test_commit_record_describes_special_moves(tmp_path: Path) -> None:
    capture = GameState(fen="4k3/8/8/3p4/4P3/8/8/4K3 w - - 0 1")
    assert capture.commit_move(UCI("e4d5"), "human").captured == chess.PAWN
    ep = GameState(fen="4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1")
    record = ep.commit_move(UCI("e5d6"), "human")
    assert record.en_passant and record.captured == chess.PAWN
    castle = GameState(fen="4k3/8/8/8/8/8/8/4K2R w K - 0 1")
    assert castle.commit_move(UCI("e1g1"), "human").castling
    promo = GameState(fen="4k3/P7/8/8/8/8/8/4K3 w - - 0 1")
    assert promo.commit_move(UCI("a7a8q"), "human").promotion == chess.QUEEN


def test_source_must_be_human_or_engine(game: GameState) -> None:
    with pytest.raises(GameStateError):
        game.commit_move(E2E4, "camera")
    assert game.fen() == chess.STARTING_FEN


@pytest.mark.parametrize("bad", ["e2e5", "e7e5", "a1a2", "e2e4q"])
def test_illegal_move_changes_nothing(game: GameState, bad: str, tmp_path: Path) -> None:
    saved = (tmp_path / FEN_FILE).read_text()
    with pytest.raises(IllegalMoveError):
        game.commit_move(UCI(bad), "human")
    assert game.fen() == chess.STARTING_FEN and game.history == ()
    assert (tmp_path / FEN_FILE).read_text() == saved


def test_a_non_move_object_is_refused(game: GameState) -> None:
    with pytest.raises(IllegalMoveError):
        game.commit_move("e2e4", "human")  # type: ignore[arg-type]


# --------------------------------------------------------------------------- #
# preview_move
# --------------------------------------------------------------------------- #


def test_preview_returns_the_board_after_the_move_without_changing_the_position(game: GameState) -> None:
    before = game.fen()
    preview = game.preview_move(E2E4)
    assert preview.piece_at(chess.E4) == chess.Piece(chess.PAWN, chess.WHITE)
    assert preview.piece_at(chess.E2) is None
    assert game.fen() == before and game.history == ()


def test_preview_of_an_illegal_move_is_refused(game: GameState) -> None:
    with pytest.raises(IllegalMoveError):
        game.preview_move(UCI("e2e5"))


def test_preview_is_a_copy(game: GameState) -> None:
    preview = game.preview_move(E2E4)
    preview.push(E7E5)
    assert game.fen() == chess.STARTING_FEN


# --------------------------------------------------------------------------- #
# Read-only views are copies
# --------------------------------------------------------------------------- #


def test_board_property_is_a_copy(game: GameState) -> None:
    board = game.board
    board.push(E2E4)
    board.clear()
    assert game.fen() == chess.STARTING_FEN


def test_legal_moves_is_a_fresh_list(game: GameState) -> None:
    moves = game.legal_moves
    moves.clear()
    assert len(game.legal_moves) == 20


# --------------------------------------------------------------------------- #
# Pending move
# --------------------------------------------------------------------------- #


def test_set_pending_move_is_saved_with_the_position_it_was_computed_for(game: GameState, tmp_path: Path) -> None:
    game.commit_move(E2E4, "human")
    fen_before = game.fen()
    game.set_pending_move(E7E5)
    assert game.pending_move == E7E5
    assert game.fen() == fen_before  # the position is not touched
    data = json.loads((tmp_path / PENDING_FILE).read_text())
    assert data == {"pending_move": "e7e5", "current_fen": fen_before}


def test_pending_must_be_legal_and_unique(game: GameState) -> None:
    game.commit_move(E2E4, "human")
    with pytest.raises(IllegalMoveError):
        game.set_pending_move(E2E4)
    game.set_pending_move(E7E5)
    with pytest.raises(PendingMoveError):
        game.set_pending_move(UCI("d7d5"))


def test_engine_commit_requires_the_matching_pending_move(game: GameState) -> None:
    game.commit_move(E2E4, "human")
    with pytest.raises(PendingMoveError):
        game.commit_move(E7E5, "engine")  # nothing pending
    game.set_pending_move(E7E5)
    with pytest.raises(PendingMoveError):
        game.commit_move(UCI("d7d5"), "engine")  # a different move
    assert game.fen() != "" and game.pending_move == E7E5


def test_engine_commit_clears_pending_and_removes_the_file(game: GameState, tmp_path: Path) -> None:
    play(game, "e2e4", "e7e5")
    assert game.pending_move is None
    assert not (tmp_path / PENDING_FILE).exists()
    assert [r.source for r in game.history] == [MoveSource.HUMAN, MoveSource.ENGINE]


def test_human_commit_is_refused_while_an_engine_move_is_pending(game: GameState) -> None:
    game.commit_move(E2E4, "human")
    game.set_pending_move(E7E5)
    with pytest.raises(PendingMoveError):
        game.commit_move(E7E5, "human")


def test_clear_pending_move_cancels_it_without_touching_the_position(game: GameState, tmp_path: Path) -> None:
    game.commit_move(E2E4, "human")
    fen = game.fen()
    game.set_pending_move(E7E5)
    game.clear_pending_move()
    assert game.pending_move is None and game.fen() == fen
    assert not (tmp_path / PENDING_FILE).exists()


# --------------------------------------------------------------------------- #
# Persistence
# --------------------------------------------------------------------------- #


def test_fen_and_pgn_are_saved_after_every_commit(game: GameState, tmp_path: Path) -> None:
    for index, text in enumerate(("e2e4", "e7e5", "g1f3")):
        play_one = [text]
        if index % 2 == 0:
            game.commit_move(UCI(text), "human")
        else:
            game.set_pending_move(UCI(text))
            game.commit_move(UCI(text), "engine")
        assert (tmp_path / FEN_FILE).read_text().strip() == game.fen(), play_one
        saved = chess.pgn.read_game(open(tmp_path / PGN_FILE, encoding="utf-8"))
        assert saved is not None and len(list(saved.mainline_moves())) == index + 1
    assert "source=engine" in (tmp_path / PGN_FILE).read_text()


def test_no_stray_temp_files_remain(game: GameState, tmp_path: Path) -> None:
    play(game, "e2e4", "e7e5")
    assert not [p for p in tmp_path.iterdir() if p.name.endswith(".tmp")]


def test_a_failed_write_leaves_memory_and_files_unchanged(
    game: GameState, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    game.commit_move(E2E4, "human")
    fen = game.fen()
    saved = (tmp_path / FEN_FILE).read_text()

    def boom(*_args: Any) -> None:
        raise OSError("disk full")

    monkeypatch.setattr(os, "replace", boom)
    with pytest.raises(PersistenceError):
        game.commit_move(E7E5, "human")
    assert game.fen() == fen and len(game.history) == 1
    assert (tmp_path / FEN_FILE).read_text() == saved


def test_load_returns_none_without_a_saved_game(tmp_path: Path) -> None:
    assert GameState.load(tmp_path) is None


def test_load_restores_position_history_and_sources(tmp_path: Path) -> None:
    first = GameState.new_game(tmp_path)
    play(first, "e2e4", "e7e5", "g1f3", "b8c6")
    loaded = GameState.load(tmp_path)
    assert loaded is not None
    assert loaded.fen() == first.fen()
    assert [(r.uci, r.source) for r in loaded.history] == [(r.uci, r.source) for r in first.history]
    assert loaded.history[1].source is MoveSource.ENGINE
    assert loaded.pending_move is None
    loaded.commit_move(UCI("f1c4"), "human")  # and it keeps working
    assert len(loaded.history) == 5


def test_load_keeps_the_game_playable_after_a_finished_game(tmp_path: Path) -> None:
    game = GameState.new_game(tmp_path, fen=MATE_IN_ONE_FEN)
    game.commit_move(UCI("d8h4"), "human")
    loaded = GameState.load(tmp_path)
    assert loaded is not None and loaded.is_game_over
    assert loaded.outcome is not None and loaded.outcome.result == "0-1"


def test_load_from_a_custom_start_position(tmp_path: Path) -> None:
    first = GameState.new_game(tmp_path, fen="4k3/P7/8/8/8/8/8/4K3 w - - 0 1")
    first.commit_move(UCI("a7a8q"), "human")
    loaded = GameState.load(tmp_path)
    assert loaded is not None and loaded.fen() == first.fen()
    assert [r.uci for r in loaded.history] == ["a7a8q"]


def test_load_without_pgn_keeps_the_position_and_warns(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    first = GameState.new_game(tmp_path)
    play(first, "e2e4", "e7e5")
    (tmp_path / PGN_FILE).unlink()
    with caplog.at_level(logging.WARNING):
        loaded = GameState.load(tmp_path)
    assert loaded is not None and loaded.fen() == first.fen() and loaded.history == ()
    assert any("history is lost" in r.getMessage() for r in caplog.records)


def test_pgn_that_disagrees_with_the_fen_loses_the_history_but_not_the_position(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    first = GameState.new_game(tmp_path)
    play(first, "e2e4", "e7e5")
    stale = (tmp_path / PGN_FILE).read_text()
    play(first, "g1f3")
    (tmp_path / PGN_FILE).write_text(stale)  # PGN one move behind current.fen
    with caplog.at_level(logging.WARNING):
        loaded = GameState.load(tmp_path)
    assert loaded is not None and loaded.fen() == first.fen() and loaded.history == ()


def test_a_corrupt_fen_file_is_an_error(tmp_path: Path) -> None:
    (tmp_path / FEN_FILE).write_text("garbage")
    with pytest.raises(PersistenceError):
        GameState.load(tmp_path)


# --------------------------------------------------------------------------- #
# pending.json recovery inside load
# --------------------------------------------------------------------------- #


def _crash_with_pending(tmp_path: Path) -> tuple[str, str]:
    """A game where e4 was played and e7e5 is pending when the app dies. Returns (fen before, fen after)."""
    game = GameState.new_game(tmp_path)
    game.commit_move(E2E4, "human")
    before = game.fen()
    after = game.preview_move(E7E5).fen()
    game.set_pending_move(E7E5)
    return before, after


def test_pending_move_survives_a_shutdown(tmp_path: Path) -> None:
    before, after = _crash_with_pending(tmp_path)
    loaded = GameState.load(tmp_path)
    assert loaded is not None
    assert loaded.fen() == before  # NOT the position after the pending move
    assert loaded.pending_move == E7E5
    assert loaded.preview_move(E7E5).fen() == after
    loaded.commit_move(E7E5, "engine")  # the recovered move can be committed
    assert loaded.pending_move is None and not (tmp_path / PENDING_FILE).exists()


def test_pending_move_can_be_cancelled_after_a_shutdown(tmp_path: Path) -> None:
    before, _ = _crash_with_pending(tmp_path)
    loaded = GameState.load(tmp_path)
    assert loaded is not None
    loaded.clear_pending_move()
    assert loaded.pending_move is None and loaded.fen() == before
    again = GameState.load(tmp_path)
    assert again is not None and again.pending_move is None


def test_pending_that_was_already_committed_is_discarded(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    """Crash between writing current.fen and removing pending.json."""
    _crash_with_pending(tmp_path)
    pending_text = (tmp_path / PENDING_FILE).read_text()
    game = GameState.load(tmp_path)
    assert game is not None
    game.commit_move(E7E5, "engine")
    (tmp_path / PENDING_FILE).write_text(pending_text)  # the stale file is back
    with caplog.at_level(logging.INFO):
        loaded = GameState.load(tmp_path)
    assert loaded is not None and loaded.pending_move is None and loaded.fen() == game.fen()
    assert not (tmp_path / PENDING_FILE).exists()


@pytest.mark.parametrize(
    "content",
    ["not json", "{}", json.dumps({"pending_move": "e7e5", "current_fen": chess.STARTING_FEN}),
     json.dumps({"pending_move": "zzzz", "current_fen": "x"})],
)
def test_unusable_pending_files_are_discarded(tmp_path: Path, content: str) -> None:
    game = GameState.new_game(tmp_path)
    game.commit_move(E2E4, "human")
    (tmp_path / PENDING_FILE).write_text(content)
    loaded = GameState.load(tmp_path)
    assert loaded is not None and loaded.pending_move is None
    assert not (tmp_path / PENDING_FILE).exists()


def test_new_game_removes_an_old_pending_file(tmp_path: Path) -> None:
    _crash_with_pending(tmp_path)
    GameState.new_game(tmp_path)
    assert not (tmp_path / PENDING_FILE).exists()


# --------------------------------------------------------------------------- #
# Game end
# --------------------------------------------------------------------------- #


def test_checkmate_is_detected_and_blocks_further_commits(tmp_path: Path) -> None:
    game = GameState.new_game(tmp_path, fen=MATE_IN_ONE_FEN)
    record = game.commit_move(UCI("d8h4"), "human")
    assert record.san == "Qh4#"
    assert game.is_game_over
    assert game.outcome is not None
    assert game.outcome.result == "0-1" and game.outcome.termination == "checkmate" and game.outcome.winner == chess.BLACK
    assert game.legal_moves == []
    with pytest.raises(GameOverError):
        game.commit_move(UCI("a2a3"), "human")
    assert 'Result "0-1"' in (tmp_path / PGN_FILE).read_text()


def test_stalemate_is_a_draw() -> None:
    game = GameState(fen=STALEMATE_FEN)
    assert game.is_game_over and game.outcome is not None
    assert game.outcome.result == "1/2-1/2" and game.outcome.termination == "stalemate"
    assert game.outcome.winner is None


def test_insufficient_material_is_a_draw() -> None:
    game = GameState(fen="4k3/8/8/8/8/8/8/4K3 w - - 0 1")
    assert game.outcome is not None and game.outcome.termination == "insufficient_material"


def test_no_pending_move_can_be_set_after_the_game_ended() -> None:
    game = GameState(fen=MATE_IN_ONE_FEN)
    game.commit_move(UCI("d8h4"), "human")
    with pytest.raises(GameOverError):
        game.set_pending_move(UCI("a2a3"))


# --------------------------------------------------------------------------- #
# Event Bus
# --------------------------------------------------------------------------- #


def test_game_updated_is_published_after_every_commit(tmp_path: Path) -> None:
    bus = EventBus()
    seen: list[GameUpdated] = []
    bus.subscribe(GameUpdated, seen.append)
    game = GameState.new_game(tmp_path, bus)
    game.commit_move(E2E4, "human")
    game.set_pending_move(E7E5)
    assert len(seen) == 1  # setting a pending move is not a game update
    game.commit_move(E7E5, "engine")
    assert [(e.move_uci, e.san, e.source) for e in seen] == [("e2e4", "e4", "human"), ("e7e5", "e5", "engine")]
    assert seen[-1].fen == game.fen() and not seen[-1].game_over


def test_game_updated_reports_the_game_end(tmp_path: Path) -> None:
    bus = EventBus()
    seen: list[GameUpdated] = []
    bus.subscribe(GameUpdated, seen.append)
    game = GameState.new_game(tmp_path, bus, fen=MATE_IN_ONE_FEN)
    game.commit_move(UCI("d8h4"), "human")
    assert seen[-1].game_over and seen[-1].result == "0-1" and seen[-1].termination == "checkmate"


def test_a_refused_commit_publishes_nothing(tmp_path: Path) -> None:
    bus = EventBus()
    seen: list[GameUpdated] = []
    bus.subscribe(GameUpdated, seen.append)
    game = GameState.new_game(tmp_path, bus)
    with pytest.raises(IllegalMoveError):
        game.commit_move(UCI("e2e5"), "human")
    assert seen == []


def test_in_memory_game_needs_no_folder() -> None:
    game = GameState()
    game.commit_move(E2E4, "human")
    assert game.data_dir is None and game.pgn().startswith("[Event")


def test_threefold_repetition_ends_the_game_only_when_it_has_happened(tmp_path: Path) -> None:
    game = GameState.new_game(tmp_path)
    shuffle = ["g1f3", "g8f6", "f3g1", "f6g8"] * 2
    for index, text in enumerate(shuffle[:-1]):
        game.commit_move(UCI(text), "human")
        assert not game.is_game_over, f"ended early after ply {index + 1}"
    game.commit_move(UCI(shuffle[-1]), "human")
    assert game.outcome is not None and game.outcome.termination == "threefold_repetition"


def test_fifty_move_rule_ends_the_game_when_the_clock_reaches_100() -> None:
    almost = GameState(fen="4k3/8/8/8/8/8/R7/4K3 w - - 98 80")
    almost.commit_move(UCI("a2a3"), "human")
    assert not almost.is_game_over  # 99 half-moves
    almost.commit_move(UCI("e8d8"), "human")
    assert almost.outcome is not None and almost.outcome.termination == "fifty_moves"
