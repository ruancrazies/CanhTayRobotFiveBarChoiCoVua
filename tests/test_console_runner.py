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


def test_the_game_starts_by_homing_the_stub_robot_before_setup_check(tmp_path: Path) -> None:
    runner, _, output, _ = play(tmp_path, ["quit"], [])
    assert runner.states_visited[:3] == [State.HOMING, State.SETUP_CHECK, State.WAIT_HUMAN]
    assert output.count("Homing (stub)") == 1  # INIT -> CALIBRATE -> HOMING -> SETUP_CHECK


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
        tmp_path, ["e3", "emptied", "resync", "quit"], ["c5d4", "d4e3"], fen=fen, planner=make_planner(capture_slots=1)
    )
    assert "the capture zone is full" in output
    assert any("emptied" in p for p in prompts)
    assert State.ERROR_RECOVER in runner.states_visited  # a planning failure is an error, not a retry loop
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
    assert State.ERROR_RECOVER in runner.states_visited
    assert result.final_state is State.IDLE and (tmp_path / "pending.json").exists()


def test_an_engine_error_is_reported_and_can_be_retried(tmp_path: Path) -> None:
    class Flaky(ScriptedEngine):
        calls = 0

        def best_move(self, fen: str) -> chess.Move:
            Flaky.calls += 1
            if Flaky.calls == 1:
                raise EngineUnavailableError("no answer")
            return super().best_move(fen)

    # engine fails -> ERROR_RECOVER -> resync -> ENGINE_THINK asks the engine again (no homing: the arm never moved)
    runner, result, output, _ = play(tmp_path, ["e4", "resync", "quit"], [], engine=Flaky(["e7e5"]))
    assert "Engine error: no answer" in output
    assert runner.states_visited == [
        State.HOMING, State.SETUP_CHECK, State.WAIT_HUMAN, State.ENGINE_THINK, State.ERROR_RECOVER,
        State.ENGINE_THINK, State.PLAN, State.ROBOT_EXEC, State.VERIFY, State.WAIT_HUMAN, State.IDLE,
    ]
    assert [r.san for r in runner.game.history] == ["e4", "e5"]


def test_the_operator_can_quit_when_the_engine_is_down(tmp_path: Path) -> None:
    class Dead:
        def best_move(self, fen: str) -> chess.Move:
            raise EngineUnavailableError("gone")

    runner, result, _, _ = play(tmp_path, ["e4", "quit"], [], engine=Dead())
    assert result.final_state is State.IDLE and [r.san for r in runner.game.history] == ["e4"]
    assert State.ERROR_RECOVER in runner.states_visited


def test_a_chain_that_cannot_be_executed_ends_in_error_recover_and_keeps_the_pending_move(tmp_path: Path) -> None:
    """The pawn on e7 is gone when the robot reaches for it, so the stub robot cannot grip it."""
    game = GameState.new_game(tmp_path)
    inputs = iter(["e4", "quit"])
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
    assert State.ERROR_RECOVER in runner.states_visited and result.final_state is State.IDLE  # the operator quit
    assert "Robot stub error" in out.getvalue() and "nothing to grip at e7" in out.getvalue()
    assert game.pending_move == UCI("e7e5") and len(game.history) == 1  # kept; nothing committed


def test_a_verify_mismatch_is_reported_with_suspect_squares(tmp_path: Path) -> None:
    game = GameState.new_game(tmp_path)
    inputs = iter(["e4", "quit"])
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
    assert State.ERROR_RECOVER in runner.states_visited and result.final_state is State.IDLE
    assert game.pending_move == UCI("e7e5")


def test_end_of_input_stops_cleanly(tmp_path: Path) -> None:
    class Closed:
        def __call__(self, _prompt: str) -> str:
            raise EOFError

    game = GameState.new_game(tmp_path)
    runner = ConsoleGame(game=game, engine=ScriptedEngine([]), planner=make_planner(), fsm=Fsm(),
                         input_fn=Closed(), out=io.StringIO())
    assert runner.run(chess.WHITE).final_state is State.IDLE


# --------------------------------------------------------------------------- #
# Alarms and failures: ERROR_RECOVER end to end, the arm lock and the detector lock
# --------------------------------------------------------------------------- #


class Drive:
    """A game scripted by state: the human plays ``moves``, every error is answered with ``resync``.

    ``alarm`` is raised ONCE, the first time the loop enters ``alarm_in``.
    """

    def __init__(self, tmp_path: Path, moves: list[str], engine_moves: list[str], *, alarm: str | None = None,
                 alarm_in: State | None = None, fen: str | None = None) -> None:
        self.moves = list(moves)
        self.alarm, self.alarm_in = alarm, alarm_in
        self.fired = False
        self.asked: list[tuple[State, str]] = []  # (state when the prompt appeared, prompt)
        self.out = io.StringIO()
        self.game = GameState.new_game(tmp_path, fen=fen)
        self.runner = ConsoleGame(
            game=self.game, engine=ScriptedEngine(engine_moves), planner=make_planner(), fsm=Fsm(),
            input_fn=self._input, out=self.out, alarm_source=self._alarm,
        )

    def _alarm(self, state: State) -> str | None:
        if self.alarm is not None and state is self.alarm_in and not self.fired:
            self.fired = True
            return self.alarm
        return None

    def _input(self, prompt: str) -> str:
        state = self.runner.fsm.state
        self.asked.append((state, prompt))
        assert len(self.asked) < 60, f"runaway loop, last states: {self.runner.states_visited[-6:]}"
        if state is State.ERROR_RECOVER:
            return "resync"
        return self.moves.pop(0) if self.moves else "quit"

    def run(self, color: chess.Color = chess.WHITE) -> RunResult:
        return self.runner.run(color)


RUNNING_LOOP_STATES = [
    State.HOMING, State.SETUP_CHECK, State.WAIT_HUMAN, State.ENGINE_THINK, State.PLAN, State.ROBOT_EXEC, State.VERIFY,
]


@pytest.mark.parametrize("alarm", ["ESTOP", "CAMERA_LOST"])
@pytest.mark.parametrize("state", RUNNING_LOOP_STATES, ids=lambda s: s.value)
def test_an_alarm_in_any_state_is_recovered_and_the_game_plays_on(tmp_path: Path, state: State, alarm: str) -> None:
    drive = Drive(tmp_path, ["e4"], ["e7e5"], alarm=alarm, alarm_in=state)
    result = drive.run()
    runner = drive.runner
    assert drive.fired and result.exit_code == 0 and result.final_state is State.IDLE
    visited = runner.states_visited
    after_alarm = visited[visited.index(state) + 1]
    assert after_alarm is State.ERROR_RECOVER  # the FSM, not the runner, sent it there
    assert [r.san for r in drive.game.history] == ["e4", "e5"]  # the game went on and nothing was lost or doubled
    assert drive.game.pending_move is None
    assert runner.world is not None
    assert runner.world.squares() == {chess.square_name(q): p for q, p in drive.game.board.piece_map().items()}


def test_estop_during_the_robot_chain_homes_and_redoes_the_move(tmp_path: Path) -> None:
    drive = Drive(tmp_path, ["e4"], ["e7e5"], alarm="ESTOP", alarm_in=State.ROBOT_EXEC)
    drive.run()
    assert drive.runner.states_visited == [
        State.HOMING, State.SETUP_CHECK, State.WAIT_HUMAN, State.ENGINE_THINK, State.PLAN, State.ROBOT_EXEC,
        State.ERROR_RECOVER, State.HOMING, State.SETUP_CHECK, State.PLAN, State.ROBOT_EXEC, State.VERIFY,
        State.WAIT_HUMAN, State.IDLE,
    ]
    assert "ALARM ESTOP" in drive.out.getvalue() and "Homing is mandatory" in drive.out.getvalue()


def test_camera_loss_after_the_move_was_made_commits_it_instead_of_redoing_it(tmp_path: Path) -> None:
    drive = Drive(tmp_path, ["e4"], ["e7e5"], alarm="CAMERA_LOST", alarm_in=State.VERIFY)
    drive.run()
    visited = drive.runner.states_visited
    assert visited[visited.index(State.VERIFY):][:3] == [State.VERIFY, State.ERROR_RECOVER, State.WAIT_HUMAN]
    assert len(drive.runner.chains) == 1  # the chain was NOT sent a second time
    assert "board shows after" in drive.out.getvalue()
    assert drive.game.history[-1].source is MoveSource.ENGINE


def test_estop_in_verify_homes_and_then_commits_the_finished_move(tmp_path: Path) -> None:
    drive = Drive(tmp_path, ["e4"], ["e7e5"], alarm="ESTOP", alarm_in=State.VERIFY)
    drive.run()
    visited = drive.runner.states_visited
    assert visited[visited.index(State.VERIFY):][:5] == [
        State.VERIFY, State.ERROR_RECOVER, State.HOMING, State.SETUP_CHECK, State.WAIT_HUMAN]
    assert len(drive.runner.chains) == 1 and [r.san for r in drive.game.history] == ["e4", "e5"]


def test_the_capture_slots_are_counted_once_when_a_finished_move_is_committed_on_recovery(tmp_path: Path) -> None:
    fen = "4k3/8/8/2p5/3P4/8/4P3/4K3 b - - 0 1"
    drive = Drive(tmp_path, ["e3"], ["c5d4", "d4e3"], alarm="CAMERA_LOST", alarm_in=State.VERIFY, fen=fen)
    drive.run()
    assert [r.san for r in drive.game.history] == ["cxd4", "e3", "dxe3"]
    first, second = drive.runner.chains
    assert first.capture_slots_used == (0,) and second.capture_slots_used == (1,)  # slot 0 was counted exactly once
    assert drive.runner.inventory.capture_used == 2


def test_an_alarm_typed_by_the_operator_is_sent_to_the_fsm(tmp_path: Path) -> None:
    drive = Drive(tmp_path, ["alarm HB_TIMEOUT", "e4"], ["e7e5"])
    result = drive.run()
    visited = drive.runner.states_visited
    assert visited[:6] == [
        State.HOMING, State.SETUP_CHECK, State.WAIT_HUMAN, State.ERROR_RECOVER, State.HOMING, State.SETUP_CHECK]
    assert result.exit_code == 0 and [r.san for r in drive.game.history] == ["e4", "e5"]


def test_an_unknown_alarm_code_is_refused_and_the_game_goes_on(tmp_path: Path) -> None:
    drive = Drive(tmp_path, ["alarm FIRE", "quit"], [])
    drive.run()
    assert "Unknown alarm 'FIRE'" in drive.out.getvalue()
    assert State.ERROR_RECOVER not in drive.runner.states_visited


def test_a_half_way_piece_blocks_resync_until_the_operator_fixes_the_board(tmp_path: Path) -> None:
    drive = Drive(tmp_path, ["e4"], ["e7e5"])
    runner = drive.runner
    original = runner._robot_exec
    dropped = []

    def drop_the_piece_once() -> Optional[RunResult]:
        result = original()
        if not dropped:  # only the first attempt loses the pawn
            dropped.append(True)
            assert runner.world is not None
            runner.world.pieces.pop("e5")
        return result

    runner._robot_exec = drop_the_piece_once  # type: ignore[method-assign]
    commands = iter(["resync", "fixed", "resync"])
    runner._input = lambda prompt: (  # type: ignore[assignment]
        next(commands) if runner.fsm.state is State.ERROR_RECOVER else (drive.moves.pop(0) if drive.moves else "quit")
    )
    result = runner.run(chess.WHITE)
    assert result.exit_code == 0
    assert "A piece is half-way" in drive.out.getvalue()  # the first resync was refused: still ERROR_RECOVER
    visited = runner.states_visited
    first_error = visited.index(State.ERROR_RECOVER)
    assert visited[first_error:first_error + 3] == [State.ERROR_RECOVER] * 3  # resync refused, fixed, then resync
    assert visited[first_error + 3] is State.PLAN  # the kept move is planned again
    assert [r.san for r in drive.game.history] == ["e4", "e5"] and drive.game.pending_move is None


def test_the_stub_robot_refuses_to_move_when_the_fsm_says_no(tmp_path: Path) -> None:
    from desktop_app.console_runner import ArmLockedError, StubRobot

    game = GameState.new_game(tmp_path)
    planner = make_planner()
    chain = planner.plan(chess.Board(), UCI("e2e4"))
    world = PhysicalWorld(game.board, {})
    locked = StubRobot(world, may_move=lambda: False)
    for action in (locked.home, lambda: locked.execute(chain), lambda: locked.park(chain)):
        with pytest.raises(ArmLockedError):
            action()
    allowed = StubRobot(world, may_move=lambda: True)
    allowed.home()
    allowed.execute(chain)


def test_the_arm_moves_only_in_homing_robot_exec_and_verify_through_a_whole_messy_game(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from desktop_app.console_runner import StubRobot

    drive = Drive(tmp_path, ["e4", "alarm CAMERA_LOST", "Nf3"], ["e7e5", "b8c6"], alarm="ESTOP", alarm_in=State.ROBOT_EXEC)
    moved_in: list[State] = []
    originals = {name: getattr(StubRobot, name) for name in ("home", "execute", "park")}

    def spy(name: str):  # type: ignore[no-untyped-def]
        def wrapper(self, *args, **kwargs):  # type: ignore[no-untyped-def]
            moved_in.append(drive.runner.fsm.state)
            return originals[name](self, *args, **kwargs)

        return wrapper

    for name in originals:
        monkeypatch.setattr(StubRobot, name, spy(name))
    result = drive.run()
    assert result.exit_code == 0
    assert set(moved_in) == {State.HOMING, State.ROBOT_EXEC, State.VERIFY}  # all three, and nothing else
    assert State.ERROR_RECOVER in drive.runner.states_visited


def test_engine_and_planner_work_happens_only_in_compute_states(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    drive = Drive(tmp_path, ["e4", "Nf3"], ["e7e5", "b8c6"])
    runner = drive.runner
    seen: list[tuple[str, State, bool]] = []
    real_best, real_plan = runner.engine.best_move, runner.planner.plan

    def best(fen: str) -> chess.Move:
        seen.append(("engine", runner.fsm.state, runner.fsm.arm_may_move))
        return real_best(fen)

    def plan(*args, **kwargs):  # type: ignore[no-untyped-def]
        seen.append(("plan", runner.fsm.state, runner.fsm.arm_may_move))
        return real_plan(*args, **kwargs)

    monkeypatch.setattr(runner.engine, "best_move", best)
    monkeypatch.setattr(runner.planner, "plan", plan)
    drive.run()
    assert {(name, state) for name, state, _ in seen} == {("engine", State.ENGINE_THINK), ("plan", State.PLAN)}
    assert not any(arm for _, _, arm in seen)  # nothing physical can happen while computing


def test_the_human_is_asked_for_a_move_only_in_wait_human(tmp_path: Path) -> None:
    drive = Drive(tmp_path, ["e4", "alarm CAMERA_LOST", "Nf3"], ["e7e5", "b8c6"], alarm="ESTOP", alarm_in=State.PLAN)
    drive.run()
    move_prompts = [state for state, prompt in drive.asked if "(you) move" in prompt]
    assert move_prompts and set(move_prompts) == {State.WAIT_HUMAN}  # detection stays locked everywhere else
    assert {state for state, _ in drive.asked} <= {State.WAIT_HUMAN, State.ERROR_RECOVER, State.PAUSED}


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
