"""SPEC rule 1: GameState is changed by commit_move and by nothing else.

Two independent proofs:
  * static: no module except game_state.py calls a position-changing python-chess method, and
    only the console runner (the stand-in for the future controller) calls commit_move;
  * dynamic: a whole game is played while every collaborator (engine, planner, FSM) is checked
    to leave the position exactly as the last commit left it, and every ``chess.Board.push`` that
    originates in desktop_app/ is traced to game_state.py.
"""

from __future__ import annotations

import ast
import io
import sys
from pathlib import Path
from typing import Any, Callable

import chess
import pytest

from desktop_app.console_runner import ConsoleGame
from desktop_app.core.fsm import Fsm
from desktop_app.core.game_state import GameState
from phase2_support import ScriptedEngine, make_planner

REPO_ROOT = Path(__file__).resolve().parents[1]
APP = REPO_ROOT / "desktop_app"

POSITION_CHANGERS = {
    "push", "push_uci", "push_san", "push_xboard", "set_fen", "set_board_fen", "set_piece_at",
    "remove_piece_at", "clear_board", "set_castling_fen", "set_piece_map",
}
POSITION_STATE_FIELDS = {"_board", "_history"}


def _sources() -> list[Path]:
    return sorted(p for p in APP.rglob("*.py") if p.name != "game_state.py")


def _calls(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}


def test_there_are_modules_to_check() -> None:
    names = {p.name for p in _sources()}
    assert {"engine.py", "fsm.py", "move_planner.py", "console_runner.py", "five_bar.py"} <= names


def test_no_module_except_game_state_changes_a_position() -> None:
    offenders = {p.name: sorted(_calls(p) & POSITION_CHANGERS) for p in _sources() if _calls(p) & POSITION_CHANGERS}
    assert offenders == {}, f"only GameState may change a position: {offenders}"


def test_only_the_runner_commits_moves_and_only_the_runner_stores_pending_moves() -> None:
    committers = {p.name for p in _sources() if "commit_move" in _calls(p)}
    storers = {p.name for p in _sources() if {"set_pending_move", "clear_pending_move"} & _calls(p)}
    assert committers == {"console_runner.py"}
    assert storers == {"console_runner.py"}


def test_nobody_reaches_into_the_private_position_of_game_state() -> None:
    for path in _sources():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        # ``self._board`` is a module's own attribute (Phase 1 BoardGeometry has one); what is
        # forbidden is reading or writing ``_board`` / ``_history`` of ANOTHER object.
        touched = {
            n.attr
            for n in ast.walk(tree)
            if isinstance(n, ast.Attribute) and n.attr in POSITION_STATE_FIELDS
            and not (isinstance(n.value, ast.Name) and n.value.id == "self")
        }
        assert not touched, f"{path.name} reaches into GameState internals: {touched}"


# --------------------------------------------------------------------------- #
# Dynamic proof
# --------------------------------------------------------------------------- #


def _play_with_checks(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> tuple[GameState, list[str]]:
    game = GameState.new_game(tmp_path)
    expected = {"fen": game.fen(), "plies": 0}
    seen_callers: list[str] = []

    # 1. every chess.Board.push called from inside desktop_app/ must come from game_state.py
    real_push = chess.Board.push

    def traced_push(self: chess.Board, move: chess.Move) -> Any:
        caller = sys._getframe(1).f_code.co_filename
        if str(APP) in caller:
            seen_callers.append(Path(caller).name)
        return real_push(self, move)

    monkeypatch.setattr(chess.Board, "push", traced_push)

    # 2. the position may change only when commit_move is called
    real_commit = game.commit_move

    def commit(move: chess.Move, source: Any) -> Any:
        record = real_commit(move, source)
        expected["fen"], expected["plies"] = game.fen(), expected["plies"] + 1
        return record

    monkeypatch.setattr(game, "commit_move", commit)

    def unchanged(label: str) -> None:
        assert game.fen() == expected["fen"], f"{label} changed the position"
        assert len(game.history) == expected["plies"], f"{label} changed the history"

    engine = ScriptedEngine(["e7e5", "b8c6", "g8f6", "f8c5"])
    planner = make_planner()
    fsm = Fsm()

    def wrap(obj: Any, name: str) -> None:
        original: Callable[..., Any] = getattr(obj, name)

        def checked(*args: Any, **kwargs: Any) -> Any:
            unchanged(f"before {name}")
            result = original(*args, **kwargs)
            unchanged(f"{name}")
            return result

        monkeypatch.setattr(obj, name, checked)

    wrap(engine, "best_move")
    wrap(planner, "plan")
    wrap(fsm, "handle")
    wrap(game, "preview_move")

    inputs = iter(["e4", "Nf3", "Nc3", "Bc4", "quit"])
    runner = ConsoleGame(game=game, engine=engine, planner=planner, fsm=fsm, input_fn=lambda _p: next(inputs), out=io.StringIO())
    result = runner.run(chess.WHITE)
    assert result.exit_code == 0 and expected["plies"] == 8
    return game, seen_callers


def test_a_whole_game_changes_the_position_only_through_commit_move(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    game, callers = _play_with_checks(tmp_path, monkeypatch)
    assert game.history[-1].san == "Bc5"
    assert callers, "the trace saw no push at all: the test would pass vacuously"
    assert set(callers) == {"game_state.py"}, f"a board was pushed from {set(callers)}"


def test_engine_planner_and_fsm_calls_leave_the_position_alone(tmp_path: Path) -> None:
    game = GameState.new_game(tmp_path)
    game.commit_move(chess.Move.from_uci("e2e4"), "human")
    before = (game.fen(), game.history, game.pending_move)
    planner = make_planner()
    planner.plan(game.board, chess.Move.from_uci("e7e5"))
    ScriptedEngine(["e7e5"]).best_move(game.fen())
    preview = game.preview_move(chess.Move.from_uci("e7e5"))
    preview.push(chess.Move.from_uci("g1f3"))  # mutate the copy
    board = game.board
    board.push(chess.Move.from_uci("g1f3"))
    game.legal_moves.clear()
    fsm = Fsm(game)
    for trigger in list(fsm_triggers()):
        fsm.handle(trigger)
    assert (game.fen(), game.history, game.pending_move) == before


def fsm_triggers() -> list[Any]:
    from desktop_app.core.fsm import Trigger

    return list(Trigger)


def test_a_failed_commit_changes_nothing(tmp_path: Path) -> None:
    game = GameState.new_game(tmp_path)
    before = (game.fen(), game.history)
    for bad in (chess.Move.from_uci("e2e5"), None, "e2e4"):
        with pytest.raises(Exception):
            game.commit_move(bad, "human")  # type: ignore[arg-type]
    with pytest.raises(Exception):
        game.commit_move(chess.Move.from_uci("e2e4"), "robot")
    assert (game.fen(), game.history) == before
