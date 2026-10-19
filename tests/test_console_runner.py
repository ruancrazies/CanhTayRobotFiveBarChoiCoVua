"""The console game: typed moves, engine replies, printed command chains, stubs for camera and robot.

Engines here are scripted, so no Stockfish binary is needed. Every engine move goes through the
real FSM, GameState, Move Planner and a stub robot that executes the chain on a simulated board.
"""

from __future__ import annotations

import builtins
import io
import logging
import sys
from pathlib import Path
from typing import Callable, Optional

import chess
import pytest

from desktop_app import console_runner
from desktop_app.console_runner import ConsoleGame, PhysicalWorld, RunResult
from desktop_app.core.engine import EngineUnavailableError
from desktop_app.core.fsm import Fsm, State
from desktop_app.core.game_state import GameState, MoveSource
from desktop_app.core.move_planner import MovePlanner
from phase2_support import MATE_IN_ONE_FEN, ScriptedEngine, make_planner

UCI = chess.Move.from_uci


def play(
    tmp_path: Path,
    human_inputs: list[str],
    engine_moves: list[str],
    *,
    fen: Optional[str] = None,
    human: chess.Color = chess.WHITE,
    planner: Optional[MovePlanner] = None,
    engine: object = None,
    initial_world: Optional[PhysicalWorld] = None,
) -> tuple[ConsoleGame, RunResult, str, list[str]]:
    game = GameState.new_game(tmp_path, fen=fen)
    script = iter(human_inputs)
    prompts: list[str] = []

    def ask(prompt: str) -> str:
        prompts.append(prompt)
        return next(script)

    out = io.StringIO()
    runner = ConsoleGame(
        game=game,
        engine=engine or ScriptedEngine(engine_moves),  # type: ignore[arg-type]
        planner=planner or make_planner(),
        fsm=Fsm(),
        input_fn=ask,
        out=out,
        initial_world=initial_world,
    )
    result = runner.run(human)
    return runner, result, out.getvalue(), prompts


def roles(runner: ConsoleGame) -> list[str]:
    return [t.role.value for chain in runner.chains for t in chain.transfers]


# --------------------------------------------------------------------------- #
# A whole game
# --------------------------------------------------------------------------- #


def test_a_full_game_plays_to_checkmate_with_the_command_chains_printed(tmp_path: Path) -> None:
    runner, result, output, _ = play(tmp_path, ["f3", "g4"], ["e7e5", "d8h4"])
    assert result == RunResult(0, State.GAME_OVER, "0-1 checkmate")
    assert runner.game.is_game_over and runner.game.history[-1].san == "Qh4#"
    assert "chain for e5 (e7e5)" in output and "chain for Qh4# (d8h4)" in output
    assert output.count("MOVE ") >= 6 and "ZDOWN" in output and "GRIP  close" in output
    assert "Game over: 0-1 (checkmate)." in output and "current.pgn" in output
    saved = (tmp_path / "current.pgn").read_text()
    assert "source=human" in saved and "source=engine" in saved and "Qh4#" in saved


def test_the_engine_is_always_asked_with_the_fen_from_game_state(tmp_path: Path) -> None:
    engine = ScriptedEngine(["e7e5", "d8h4"])
    runner, _, _, _ = play(tmp_path, ["f3", "g4"], [], engine=engine)
    assert engine.fens[0] == "rnbqkbnr/pppppppp/8/8/8/5P2/PPPPP1PP/RNBQKBNR b KQkq - 0 1"
    assert engine.fens[1] == "rnbqkbnr/pppp1ppp/8/4p3/6P1/5P2/PPPPP2P/RNBQKBNR b KQkq - 0 2"
    assert all(chess.Board(fen).turn == chess.BLACK for fen in engine.fens)  # always the engine's turn


def test_the_human_may_type_san_or_uci_and_get_help(tmp_path: Path) -> None:
    _, result, output, _ = play(tmp_path, ["e2e4", "moves", "Nf3", "Qh5", "quit"], ["e7e5", "b8c6"])
    assert result.exit_code == 0 and result.final_state is State.IDLE
    assert "'Qh5' is not a legal move here" in output  # the queen is blocked by its own pawn on e4
    assert "Nf3" in output  # listed by 'moves'
    assert result.exit_code == 0


def test_illegal_or_garbage_input_is_refused_and_the_game_goes_on(tmp_path: Path) -> None:
    runner, _, output, _ = play(tmp_path, ["e2e5", "hello", "", "e4", "quit"], ["e7e5"])
    assert output.count("is not a legal move here") == 3
    assert [r.san for r in runner.game.history] == ["e4", "e5"]


def test_pause_and_resume_between_moves(tmp_path: Path) -> None:
    runner, result, output, prompts = play(tmp_path, ["pause", "resume", "e4", "quit"], ["e7e5"])
    assert any("Paused" in p for p in prompts) and result.exit_code == 0
    assert [r.san for r in runner.game.history] == ["e4", "e5"]


def test_the_human_can_play_black_and_the_engine_moves_first(tmp_path: Path) -> None:
    runner, _, output, _ = play(tmp_path, ["e5", "quit"], ["e2e4", "g1f3"], human=chess.BLACK)
    assert [r.san for r in runner.game.history] == ["e4", "e5", "Nf3"]
    assert [r.source for r in runner.game.history] == [MoveSource.ENGINE, MoveSource.HUMAN, MoveSource.ENGINE]


def test_a_longer_game_with_an_engine_capture_and_castling(tmp_path: Path) -> None:
    human = ["e4", "Nf3", "Bc4", "d3", "Nc3", "Bg5", "Nd5", "exd5", "quit"]
    engine = ["e7e5", "b8c6", "f8c5", "g8f6", "e8g8", "d7d6", "f6d5", "c6e7"]
    runner, result, output, _ = play(tmp_path, human, engine)
    assert result.exit_code == 0
    assert "castling_king" in roles(runner) and "castling_rook" in roles(runner)
    assert "captured_piece" in roles(runner)
    castle = next(r for r in runner.game.history if r.castling)
    assert castle.san == "O-O" and castle.source is MoveSource.ENGINE and castle.color == chess.BLACK
    assert runner.world is not None
    squares = {chess.square_name(sq): p for sq, p in runner.game.board.piece_map().items()}
    assert runner.world.squares() == squares  # the simulated board matches GameState


# --------------------------------------------------------------------------- #
# Special moves played by the engine side, end to end
# --------------------------------------------------------------------------- #


def test_engine_en_passant(tmp_path: Path) -> None:
    fen = "rnbqkbnr/ppp1pppp/8/8/3p4/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
    runner, _, output, _ = play(tmp_path, ["e4", "quit"], ["d4e3"], fen=fen)
    assert runner.game.history[-1].en_passant
    assert roles(runner) == ["en_passant_pawn", "moving_piece"]
    assert "pawn e4 -> capture_slot[0]" in output  # the captured pawn stands on e4, not on the target e3


def test_engine_promotion_takes_a_piece_from_the_reserve(tmp_path: Path) -> None:
    runner, _, output, _ = play(tmp_path, ["Kg3", "quit"], ["a2a1q", "a1a2"], fen="4k3/8/8/8/8/8/p6K/8 b - - 0 1")
    assert roles(runner)[:2] == ["promoted_pawn", "promotion_piece"]
    assert runner.chains[0].reserve_slot_used == 0 and "reserve_slot[0]" in output
    board = runner.game.board
    assert board.piece_at(chess.A2) == chess.Piece(chess.QUEEN, chess.BLACK)  # queen moved on from a1


def test_engine_promotion_with_capture(tmp_path: Path) -> None:
    runner, _, _, _ = play(tmp_path, ["quit"], ["a2b1q"], fen="4k3/8/8/8/8/8/p6K/1N6 b - - 0 1")
    assert roles(runner) == ["captured_piece", "promoted_pawn", "promotion_piece"]
    assert runner.chains[0].capture_slots_used == (0, 1)


# --------------------------------------------------------------------------- #
# Problems the operator has to deal with
# --------------------------------------------------------------------------- #


def test_a_full_capture_zone_stops_the_robot_until_the_operator_empties_it(tmp_path: Path) -> None:
    fen = "4k3/8/8/2p5/3P4/8/4P3/4K3 b - - 0 1"
    runner, result, output, prompts = play(
        tmp_path, ["e3", "emptied", "quit"], ["c5d4", "d4e3"], fen=fen, planner=make_planner(capture_slots=1)
    )
    assert "the capture zone is full" in output
    assert any("emptied" in p for p in prompts)
    assert runner.game.history[-1].san == "dxe3" and result.exit_code == 0
    assert runner.chains[-1].capture_slots_used == (0,)  # the zone was emptied, so slot 0 is free again


def test_a_planning_error_leaves_game_state_and_fsm_untouched(tmp_path: Path) -> None:
    fen = "4k3/8/8/2p5/3P4/8/4P3/4K3 b - - 0 1"
    runner, result, _, _ = play(
        tmp_path, ["e3", "quit"], ["c5d4", "d4e3"], fen=fen, planner=make_planner(capture_slots=1)
    )
    # the second capture could not be planned: the move stays pending, nothing was committed
    assert [r.san for r in runner.game.history] == ["cxd4", "e3"]
    assert runner.game.pending_move == UCI("d4e3")
    assert result.final_state is State.IDLE and (tmp_path / "pending.json").exists()


def test_an_engine_error_is_reported_and_can_be_retried(tmp_path: Path) -> None:
    class Flaky(ScriptedEngine):
        calls = 0

        def best_move(self, fen: str) -> chess.Move:
            Flaky.calls += 1
            if Flaky.calls == 1:
                raise EngineUnavailableError("no answer")
            return super().best_move(fen)

    runner, result, output, _ = play(tmp_path, ["e4", "", "quit"], [], engine=Flaky(["e7e5"]))
    assert "Engine error: no answer" in output
    assert [r.san for r in runner.game.history] == ["e4", "e5"]


def test_the_operator_can_quit_when_the_engine_is_down(tmp_path: Path) -> None:
    class Dead:
        def best_move(self, fen: str) -> chess.Move:
            raise EngineUnavailableError("gone")

    runner, result, _, _ = play(tmp_path, ["e4", "quit"], [], engine=Dead())
    assert result.final_state is State.IDLE and [r.san for r in runner.game.history] == ["e4"]


def test_a_chain_that_cannot_be_executed_ends_in_error_recover_and_keeps_the_pending_move(tmp_path: Path) -> None:
    """The pawn on e7 is gone when the robot reaches for it, so the stub robot cannot grip it."""
    game = GameState.new_game(tmp_path)
    inputs = iter(["e4"])
    out = io.StringIO()
    runner = ConsoleGame(game=game, engine=ScriptedEngine(["e7e5"]), planner=make_planner(), fsm=Fsm(),
                         input_fn=lambda _p: next(inputs), out=out)
    original = runner._robot_exec

    def pawn_disappears() -> Optional[RunResult]:
        assert runner.world is not None
        runner.world.pieces.pop("e7")
        return original()

    runner._robot_exec = pawn_disappears  # type: ignore[method-assign]
    result = runner.run(chess.WHITE)
    assert result.exit_code == 1 and result.final_state is State.ERROR_RECOVER
    assert "Robot stub error" in out.getvalue() and "nothing to grip at e7" in out.getvalue()
    assert game.pending_move == UCI("e7e5") and len(game.history) == 1  # kept; nothing committed


def test_a_verify_mismatch_is_reported_with_suspect_squares(tmp_path: Path) -> None:
    game = GameState.new_game(tmp_path)
    inputs = iter(["e4"])
    out = io.StringIO()
    runner = ConsoleGame(game=game, engine=ScriptedEngine(["e7e5"]), planner=make_planner(), fsm=Fsm(),
                         input_fn=lambda _p: next(inputs), out=out)
    original = runner._robot_exec

    def drop_the_piece_on_the_way() -> Optional[RunResult]:
        result = original()
        assert runner.world is not None
        runner.world.pieces.pop("e5")  # the pawn fell off the board
        return result

    runner._robot_exec = drop_the_piece_on_the_way  # type: ignore[method-assign]
    result = runner.run(chess.WHITE)
    assert "VERIFY failed, suspect squares: e5" in out.getvalue()
    assert result.final_state is State.ERROR_RECOVER and game.pending_move == UCI("e7e5")


def test_end_of_input_stops_cleanly(tmp_path: Path) -> None:
    class Closed:
        def __call__(self, _prompt: str) -> str:
            raise EOFError

    game = GameState.new_game(tmp_path)
    runner = ConsoleGame(game=game, engine=ScriptedEngine([]), planner=make_planner(), fsm=Fsm(),
                         input_fn=Closed(), out=io.StringIO())
    assert runner.run(chess.WHITE).final_state is State.IDLE


# --------------------------------------------------------------------------- #
# Command line
# --------------------------------------------------------------------------- #


def test_the_command_line_refuses_to_run_on_unconfirmed_hardware_values_without_the_flag(
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert console_runner.main(["--data-dir", "unused"]) == 2
    assert "TO CONFIRM" in capsys.readouterr().out


def test_the_command_line_plays_with_placeholders_and_says_so(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str], caplog: pytest.LogCaptureFixture
) -> None:
    monkeypatch.setattr(builtins, "input", lambda _prompt="": "quit")
    with caplog.at_level(logging.WARNING):
        code = console_runner.main(["--demo-placeholders", "--data-dir", str(tmp_path), "--new"])
    assert code == 0
    assert "placeholder" in capsys.readouterr().out
    assert (tmp_path / "current.fen").exists()


def test_the_command_line_resumes_a_saved_game(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    saved = GameState.new_game(tmp_path)
    saved.commit_move(UCI("e2e4"), "human")
    asked: list[str] = []
    monkeypatch.setattr(builtins, "input", lambda prompt="": asked.append(prompt) or "quit")
    # black is to move: with --color white the engine would think first, so play black here
    monkeypatch.setattr(console_runner, "Engine", lambda *a, **k: ScriptedEngine([]))
    assert console_runner.main(["--demo-placeholders", "--data-dir", str(tmp_path), "--color", "black"]) == 0
    assert asked and "Black (you)" in asked[0]  # it continued from the saved position
