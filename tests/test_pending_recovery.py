"""Recovery of pending_move after a shutdown in the middle of a move (SPEC 7.1, 7.2, 7.3).

A "shutdown" here means the objects are dropped without a commit and a new process loads the
files from the data folder, exactly as an abrupt power loss would leave them.
"""

from __future__ import annotations

import io
import json
from pathlib import Path

import chess
import pytest

from desktop_app.console_runner import ConsoleGame, PhysicalWorld, classify_pending
from desktop_app.core.fsm import Fsm, State, Trigger
from desktop_app.core.game_state import GameState, MoveSource
from desktop_app.utils.settings import get_settings
from desktop_app.core.move_planner import ZoneInventory
from phase2_support import E2E4, E7E5, Rig, ScriptedEngine, TEST_RESERVE, make_planner

T, S = Trigger, State
PENDING_FILE = get_settings().paths.pending_file
UCI = chess.Move.from_uci


def shutdown_during_robot_exec(data_dir: Path) -> tuple[str, str]:
    """e4 played, e7e5 chosen by the engine and the robot is mid-chain when power is lost."""
    rig = Rig(data_dir).reach(S.ROBOT_EXEC)
    assert rig.game.pending_move == E7E5
    before = rig.game.fen()
    after = rig.game.preview_move(E7E5).fen()
    return before, after  # nothing is committed; the objects are simply dropped


def restart(data_dir: Path) -> tuple[GameState, Fsm]:
    """A new process: load the game, then INIT -> CALIBRATE -> HOMING -> SETUP_CHECK."""
    game = GameState.load(data_dir)
    assert game is not None
    fsm = Fsm(game)
    for trigger in (T.YOLO_ON, T.INIT_DONE, T.CALIBRATE_DONE, T.HOMING_DONE):
        assert fsm.handle(trigger).accepted
    assert fsm.state is S.SETUP_CHECK
    return game, fsm


def test_pending_json_holds_the_move_and_the_position_it_was_computed_for(tmp_path: Path) -> None:
    before, _ = shutdown_during_robot_exec(tmp_path)
    assert json.loads((tmp_path / PENDING_FILE).read_text()) == {"pending_move": "e7e5", "current_fen": before}


def test_after_a_restart_the_position_is_the_one_before_the_move_and_the_move_is_pending(tmp_path: Path) -> None:
    before, after = shutdown_during_robot_exec(tmp_path)
    game, fsm = restart(tmp_path)
    assert game.fen() == before
    assert game.pending_move == E7E5
    assert game.preview_move(E7E5).fen() == after  # both positions are available to compare with the camera
    assert [r.san for r in game.history] == ["e4"]


def test_the_fsm_will_not_start_a_game_with_an_unresolved_pending_move(tmp_path: Path) -> None:
    shutdown_during_robot_exec(tmp_path)
    _, fsm = restart(tmp_path)
    assert not fsm.handle(T.START, human_color=chess.WHITE).accepted
    assert fsm.state is S.SETUP_CHECK


def test_camera_shows_the_position_before_so_the_move_is_redone(tmp_path: Path) -> None:
    shutdown_during_robot_exec(tmp_path)
    game, fsm = restart(tmp_path)
    world = PhysicalWorld(game.board, {})  # the camera sees the position before the move
    assert classify_pending(world.squares(), game.board, game.preview_move(E7E5)) == "before"
    assert fsm.handle(T.START, human_color=chess.WHITE, pending="redo").accepted
    assert fsm.state is S.PLAN
    # ... and the redone move completes normally
    fsm.handle(T.PLAN_READY), fsm.handle(T.EXEC_DONE)
    game.commit_move(E7E5, MoveSource.ENGINE)
    assert fsm.handle(T.VERIFY_OK).accepted and fsm.state is S.WAIT_HUMAN
    assert not (tmp_path / PENDING_FILE).exists()
    assert GameState.load(tmp_path).fen() == game.fen()  # type: ignore[union-attr]


def test_camera_shows_the_position_after_so_the_move_is_committed_not_redone(tmp_path: Path) -> None:
    """The robot had finished and power was lost before VERIFY committed the move."""
    shutdown_during_robot_exec(tmp_path)
    game, fsm = restart(tmp_path)
    after = game.preview_move(E7E5)
    assert classify_pending(PhysicalWorld(after, {}).squares(), game.board, after) == "after"
    game.commit_move(E7E5, MoveSource.ENGINE)
    assert fsm.handle(T.START, human_color=chess.WHITE).accepted
    assert fsm.state is S.WAIT_HUMAN  # the human is to move; the robot does not repeat its move
    assert game.fen() == after.fen()


def test_a_piece_left_half_way_matches_neither_position(tmp_path: Path) -> None:
    shutdown_during_robot_exec(tmp_path)
    game, _ = restart(tmp_path)
    after = game.preview_move(E7E5)
    half_way = game.board
    half_way.remove_piece_at(chess.E7)  # the pawn was picked up and the power failed
    assert classify_pending(PhysicalWorld(half_way, {}).squares(), game.board, after) == "neither"


def test_cancelling_the_pending_move_lets_the_engine_think_again(tmp_path: Path) -> None:
    before, _ = shutdown_during_robot_exec(tmp_path)
    game, fsm = restart(tmp_path)
    game.clear_pending_move()
    assert fsm.handle(T.START, human_color=chess.WHITE).accepted
    assert fsm.state is S.ENGINE_THINK and game.fen() == before
    assert GameState.load(tmp_path).pending_move is None  # type: ignore[union-attr]


def test_a_second_shutdown_during_the_redo_still_recovers(tmp_path: Path) -> None:
    before, _ = shutdown_during_robot_exec(tmp_path)
    game, fsm = restart(tmp_path)
    fsm.handle(T.START, human_color=chess.WHITE, pending="redo")
    fsm.handle(T.PLAN_READY)  # ... power is lost again in ROBOT_EXEC
    again, _ = restart(tmp_path)
    assert again.fen() == before and again.pending_move == E7E5


def test_yolo_off_in_the_middle_of_robot_exec_keeps_pending_move_for_the_next_start(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.ROBOT_EXEC)
    rig.do(T.YOLO_OFF)
    rig.do(T.EXEC_DONE)
    assert rig.state is S.IDLE
    game, fsm = restart(tmp_path)  # YOLO on again: unfinished game found
    assert game.pending_move == E7E5


def test_a_crash_after_the_commit_but_before_pending_json_was_removed_does_not_replay_the_move(tmp_path: Path) -> None:
    shutdown_during_robot_exec(tmp_path)
    pending_text = (tmp_path / PENDING_FILE).read_text()
    game, _ = restart(tmp_path)
    game.commit_move(E7E5, MoveSource.ENGINE)
    (tmp_path / PENDING_FILE).write_text(pending_text)  # the removal never happened
    reloaded, fsm = restart(tmp_path)
    assert reloaded.pending_move is None and reloaded.fen() == game.fen()
    assert fsm.handle(T.START, human_color=chess.WHITE).accepted and fsm.state is S.WAIT_HUMAN


def test_slot_usage_survives_a_restart_so_a_captured_piece_never_lands_on_an_occupied_slot(tmp_path: Path) -> None:
    planner = make_planner()
    game = GameState.new_game(tmp_path, fen="4k3/8/8/3p4/4P3/8/8/4K3 b - - 0 1")
    chain = planner.plan(game.board, UCI("d5e4"), ZoneInventory())
    game.set_pending_move(UCI("d5e4"))
    game.commit_move(UCI("d5e4"), MoveSource.ENGINE)
    reloaded = GameState.load(tmp_path)
    assert reloaded is not None
    inventory = ZoneInventory.from_history(reloaded.history, TEST_RESERVE)
    assert inventory == ZoneInventory().consume(chain)
    next_chain = planner.plan(chess.Board("4k3/8/8/8/8/8/3p4/4K3 w - - 0 2").copy(), UCI("e1d2"), inventory)
    assert next_chain.capture_slots_used == (1,)  # the next free slot, not the one already holding a piece


# --------------------------------------------------------------------------- #
# The console runner resolves a recovered pending move on start
# --------------------------------------------------------------------------- #


def run_runner(tmp_path: Path, inputs: list[str], engine_moves: list[str]) -> tuple[ConsoleGame, str, int]:
    game = GameState.load(tmp_path)
    assert game is not None
    script = iter(inputs)
    out = io.StringIO()
    runner = ConsoleGame(
        game=game,
        engine=ScriptedEngine(engine_moves),
        planner=make_planner(),
        fsm=Fsm(),
        input_fn=lambda _prompt: next(script),
        out=out,
    )
    result = runner.run(chess.WHITE)
    return runner, out.getvalue(), result.exit_code


def test_runner_redoes_a_recovered_move_when_the_board_still_shows_the_old_position(tmp_path: Path) -> None:
    shutdown_during_robot_exec(tmp_path)
    runner, output, code = run_runner(tmp_path, ["quit"], engine_moves=[])
    assert code == 0 and "the board shows before" in output
    assert "chain for e5" in output  # the chain was planned again from the stored pending move
    assert runner.game.pending_move is None and runner.game.history[-1].san == "e5"
    assert runner.game.history[-1].source is MoveSource.ENGINE
    assert not (tmp_path / PENDING_FILE).exists()


def test_runner_commits_a_recovered_move_when_the_board_already_shows_the_new_position(tmp_path: Path) -> None:
    """Simulated by loading, moving the physical pieces by hand, and starting the runner on that world."""
    shutdown_during_robot_exec(tmp_path)
    game = GameState.load(tmp_path)
    assert game is not None
    after = game.preview_move(E7E5)
    out = io.StringIO()
    runner = ConsoleGame(game=game, engine=ScriptedEngine([]), planner=make_planner(), fsm=Fsm(),
                         input_fn=lambda _p: "quit", out=out, initial_world=PhysicalWorld(after, {}))
    runner.run(chess.WHITE)
    assert "the board shows after" in out.getvalue()
    assert game.history[-1].san == "e5" and game.pending_move is None
    assert "chain for" not in out.getvalue()  # the move was not made a second time


def test_runner_refuses_to_guess_when_a_piece_is_half_way(tmp_path: Path) -> None:
    shutdown_during_robot_exec(tmp_path)
    game = GameState.load(tmp_path)
    assert game is not None
    half = game.board
    half.remove_piece_at(chess.E7)
    out = io.StringIO()
    runner = ConsoleGame(game=game, engine=ScriptedEngine([]), planner=make_planner(), fsm=Fsm(),
                         input_fn=lambda _p: "quit", out=out, initial_world=PhysicalWorld(half, {}))
    result = runner.run(chess.WHITE)
    assert result.exit_code == 1 and "resync by hand" in result.message
    assert game.pending_move == E7E5  # untouched: nothing is guessed
