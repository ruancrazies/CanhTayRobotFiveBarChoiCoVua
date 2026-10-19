"""FSM: every transition of the SPEC 7.2 table, the pause rules and the YOLO on/off rules (7.3).

The expected table below is typed out from the SPEC, independently of ``fsm.TRANSITIONS``.
"""

from __future__ import annotations

import ast
from pathlib import Path

import chess
import pytest

from desktop_app.core.event_bus import EventBus
from desktop_app.core.fsm import (
    HOMING_MANDATORY_ALARMS,
    TRANSITIONS,
    Fsm,
    FsmRequest,
    RequestRejected,
    State,
    StateChanged,
    Trigger,
)
from desktop_app.core.game_state import GameState, MoveSource
from phase2_support import E2E4, E7E5, MATE_IN_ONE_FEN, Rig, game_over_rig

S, T = State, Trigger
REPO_ROOT = Path(__file__).resolve().parents[1]
UCI = chess.Move.from_uci

# --------------------------------------------------------------------------- #
# The SPEC 7.2 table, typed out: (state, trigger) -> state reached.
# ``None`` = accepted but only remembered (deferred); the state does not change yet.
# --------------------------------------------------------------------------- #

SPEC_TABLE: dict[tuple[State, Trigger], State | None] = {
    (S.IDLE, T.YOLO_ON): S.INIT,                       # IDLE -> INIT when YOLO is switched on
    (S.INIT, T.INIT_DONE): S.CALIBRATE,                # INIT -> CALIBRATE
    (S.CALIBRATE, T.CALIBRATE_DONE): S.SETUP_CHECK,    # CALIBRATE / HOMING -> SETUP_CHECK
    (S.HOMING, T.HOMING_DONE): S.SETUP_CHECK,
    (S.SETUP_CHECK, T.START): S.WAIT_HUMAN,            # -> WAIT_HUMAN, ENGINE_THINK or PLAN (branches below)
    (S.WAIT_HUMAN, T.HUMAN_MOVE): S.COMMIT_HUMAN,      # WAIT_HUMAN -> COMMIT_HUMAN
    (S.COMMIT_HUMAN, T.COMMIT_DONE): S.ENGINE_THINK,   # -> ENGINE_THINK or GAME_OVER
    (S.ENGINE_THINK, T.ENGINE_MOVE_READY): S.PLAN,     # ENGINE_THINK -> PLAN
    (S.PLAN, T.PLAN_READY): S.ROBOT_EXEC,              # PLAN -> ROBOT_EXEC
    (S.ROBOT_EXEC, T.EXEC_DONE): S.VERIFY,             # -> VERIFY or ERROR_RECOVER
    (S.ROBOT_EXEC, T.EXEC_FAILED): S.ERROR_RECOVER,
    (S.VERIFY, T.VERIFY_OK): S.WAIT_HUMAN,             # -> WAIT_HUMAN, GAME_OVER or ERROR_RECOVER
    (S.VERIFY, T.VERIFY_FAILED): S.ERROR_RECOVER,
    (S.PAUSED, T.RESUME): S.WAIT_HUMAN,                # PAUSED -> previous state
    (S.ERROR_RECOVER, T.RESYNC): S.PLAN,               # -> WAIT_HUMAN, PLAN or HOMING (branches below)
    (S.GAME_OVER, T.NEW_GAME): S.SETUP_CHECK,          # GAME_OVER -> INIT or SETUP_CHECK
    (S.GAME_OVER, T.RESTART): S.INIT,
    # 7.3 pause
    (S.WAIT_HUMAN, T.PAUSE): S.PAUSED,                 # immediate
    (S.ENGINE_THINK, T.PAUSE): None,
    (S.PLAN, T.PAUSE): None,
    (S.ROBOT_EXEC, T.PAUSE): None,
    (S.VERIFY, T.PAUSE): None,
    # 7.3 YOLO off: immediate where nothing is in flight, deferred where a chain/step is running
    **{(state, T.YOLO_OFF): S.IDLE for state in (
        S.INIT, S.SETUP_CHECK, S.WAIT_HUMAN, S.ENGINE_THINK, S.PLAN, S.PAUSED, S.ERROR_RECOVER, S.GAME_OVER)},
    **{(state, T.YOLO_OFF): None for state in (S.CALIBRATE, S.HOMING, S.COMMIT_HUMAN, S.ROBOT_EXEC, S.VERIFY)},
}

ALL_STATES = list(State)
PAYLOADS: dict[Trigger, dict] = {
    T.START: {"human_color": chess.WHITE},
    T.HUMAN_MOVE: {"move": E2E4},
}


def rig_in(tmp_path: Path, state: State) -> Rig:
    if state is S.GAME_OVER:
        return game_over_rig(tmp_path)
    return Rig(tmp_path).reach(state)


def test_the_fixture_reaches_every_state(tmp_path: Path) -> None:
    for index, state in enumerate(ALL_STATES):
        assert rig_in(tmp_path / str(index), state).state is state


def test_the_state_enum_is_the_spec_list() -> None:
    assert {s.value for s in State} == {
        "IDLE", "INIT", "CALIBRATE", "HOMING", "SETUP_CHECK", "WAIT_HUMAN", "COMMIT_HUMAN", "ENGINE_THINK",
        "PLAN", "ROBOT_EXEC", "VERIFY", "PAUSED", "ERROR_RECOVER", "GAME_OVER",
    }


@pytest.mark.parametrize("state", ALL_STATES, ids=lambda s: s.value)
def test_a_state_accepts_exactly_the_triggers_of_the_spec_table(tmp_path: Path, state: State) -> None:
    rig = rig_in(tmp_path, state)
    accepted = {t for t in Trigger if rig.fsm.can_accept(t, **PAYLOADS.get(t, {}))}
    expected = {t for (s, t) in SPEC_TABLE if s is state}
    assert accepted == expected, f"{state.value}: unexpected {accepted - expected}, missing {expected - accepted}"


@pytest.mark.parametrize("pair", sorted(SPEC_TABLE, key=lambda p: (p[0].value, p[1].value)), ids=lambda p: f"{p[0].value}-{p[1].value}")
def test_each_table_transition_goes_where_the_spec_says(tmp_path: Path, pair: tuple[State, Trigger]) -> None:
    state, trigger = pair
    rig = rig_in(tmp_path, state)
    before_fen, before_history = rig.game.fen(), rig.game.history
    result = rig.fsm.handle(trigger, **PAYLOADS.get(trigger, {}))
    target = SPEC_TABLE[pair]
    assert result.accepted, result.reason
    assert result.previous is state
    if target is None:
        assert rig.state is state and result.deferred
    else:
        assert rig.state is target
    assert (rig.game.fen(), rig.game.history) == (before_fen, before_history)  # the FSM never changes the game


def test_every_state_change_the_fsm_can_make_is_in_the_spec_table() -> None:
    """fsm.TRANSITIONS (what the code allows) is within the typed-out SPEC table plus its branches."""
    branch_targets = {
        (S.SETUP_CHECK, T.START): {S.WAIT_HUMAN, S.ENGINE_THINK, S.PLAN},
        (S.COMMIT_HUMAN, T.COMMIT_DONE): {S.ENGINE_THINK, S.GAME_OVER},
        (S.ROBOT_EXEC, T.EXEC_DONE): {S.VERIFY},
        (S.VERIFY, T.VERIFY_OK): {S.WAIT_HUMAN, S.GAME_OVER, S.PAUSED},  # PAUSED: pause requested earlier (7.3)
        (S.ERROR_RECOVER, T.RESYNC): {S.WAIT_HUMAN, S.PLAN, S.HOMING},
        (S.PAUSED, T.RESUME): {S.WAIT_HUMAN, S.PLAN, S.ROBOT_EXEC},
        (S.ENGINE_THINK, T.ENGINE_MOVE_READY): {S.PLAN, S.PAUSED},
        (S.PLAN, T.PLAN_READY): {S.ROBOT_EXEC, S.PAUSED},
    }
    # IDLE is reachable only through YOLO off (7.3), immediately or after the running work finished.
    shutdown_states = {S.CALIBRATE, S.HOMING, S.COMMIT_HUMAN, S.ROBOT_EXEC, S.VERIFY}
    for (state, trigger), targets in TRANSITIONS.items():
        assert (state, trigger) in SPEC_TABLE, f"{state.value} + {trigger.value} is not in the SPEC table"
        allowed = {SPEC_TABLE[(state, trigger)]} | branch_targets.get((state, trigger), set())
        if trigger is T.YOLO_OFF or state in shutdown_states:
            allowed.add(S.IDLE)
        extra = set(targets) - allowed
        assert not extra, f"{state.value} + {trigger.value} may reach {extra}"


# --------------------------------------------------------------------------- #
# Branches of the table
# --------------------------------------------------------------------------- #


def test_start_goes_to_wait_human_or_engine_think_by_whose_turn_it_is(tmp_path: Path) -> None:
    white = Rig(tmp_path / "w").reach(S.SETUP_CHECK)
    white.do(T.START, human_color=chess.WHITE)
    assert white.state is S.WAIT_HUMAN and white.fsm.human_color == chess.WHITE
    black = Rig(tmp_path / "b").reach(S.SETUP_CHECK)
    black.do(T.START, human_color=chess.BLACK)  # the engine plays white and moves first
    assert black.state is S.ENGINE_THINK


def test_start_with_a_pending_move_redoes_it_through_plan(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.SETUP_CHECK)
    rig.game.commit_move(E2E4, MoveSource.HUMAN)
    rig.game.set_pending_move(E7E5)
    assert not rig.fsm.can_accept(T.START, human_color=chess.WHITE)  # unresolved: neither redone nor cancelled
    rig.do(T.START, human_color=chess.WHITE, pending="redo")
    assert rig.state is S.PLAN and rig.game.pending_move == E7E5  # pending_move is kept


def test_start_after_cancelling_the_pending_move_lets_the_engine_think_again(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.SETUP_CHECK)
    rig.game.commit_move(E2E4, MoveSource.HUMAN)
    rig.game.set_pending_move(E7E5)
    rig.game.clear_pending_move()
    rig.do(T.START, human_color=chess.WHITE)
    assert rig.state is S.ENGINE_THINK


def test_commit_done_goes_to_game_over_when_the_human_move_ends_the_game(tmp_path: Path) -> None:
    assert game_over_rig(tmp_path).state is S.GAME_OVER


def test_verify_ok_goes_to_game_over_when_the_engine_move_ends_the_game(tmp_path: Path) -> None:
    mate = UCI("d8h4")
    rig = Rig(tmp_path, fen=MATE_IN_ONE_FEN, human=chess.WHITE).reach(S.SETUP_CHECK)
    rig.do(T.START, human_color=chess.WHITE)
    assert rig.state is S.ENGINE_THINK  # black (the engine) is to move
    rig.game.set_pending_move(mate)
    rig.do(T.ENGINE_MOVE_READY)
    rig.do(T.PLAN_READY)
    rig.do(T.EXEC_DONE)
    rig.game.commit_move(mate, MoveSource.ENGINE)
    rig.do(T.VERIFY_OK)
    assert rig.state is S.GAME_OVER


@pytest.mark.parametrize("alarm", sorted(HOMING_MANDATORY_ALARMS))
def test_homing_is_mandatory_after_estop_step_loss_heartbeat_and_sensor_faults(tmp_path: Path, alarm: str) -> None:
    rig = Rig(tmp_path).reach(S.ROBOT_EXEC)
    rig.do(T.EXEC_FAILED, alarm=alarm)
    assert rig.state is S.ERROR_RECOVER and rig.fsm.homing_required
    rig.do(T.RESYNC)
    assert rig.state is S.HOMING
    assert rig.game.pending_move == E7E5  # pending_move is kept through the error
    rig.do(T.HOMING_DONE)
    assert rig.state is S.SETUP_CHECK and not rig.fsm.homing_required
    rig.do(T.START, human_color=chess.WHITE, pending="redo")
    assert rig.state is S.PLAN


@pytest.mark.parametrize("alarm", [None, "LIMIT_Z", "Z_TIMEOUT"])
def test_other_failures_resync_back_to_plan_without_homing(tmp_path: Path, alarm: str | None) -> None:
    rig = Rig(tmp_path).reach(S.ROBOT_EXEC)
    rig.do(T.EXEC_FAILED, alarm=alarm) if alarm else rig.do(T.EXEC_FAILED)
    assert not rig.fsm.homing_required
    rig.do(T.RESYNC)
    assert rig.state is S.PLAN and rig.game.pending_move == E7E5


def test_a_failed_verify_keeps_the_pending_move_and_resyncs_to_plan(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.VERIFY, ready=False)
    rig.do(T.VERIFY_FAILED)
    assert rig.state is S.ERROR_RECOVER and rig.game.pending_move == E7E5
    rig.do(T.RESYNC)
    assert rig.state is S.PLAN


def test_resync_back_to_wait_human_after_the_move_turned_out_to_be_made(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.VERIFY, ready=False)
    rig.do(T.VERIFY_FAILED)
    rig.game.commit_move(E7E5, MoveSource.ENGINE)  # the operator confirms the move was fully made
    rig.do(T.RESYNC)
    assert rig.state is S.WAIT_HUMAN


def test_game_over_leads_to_a_new_game_or_a_full_restart(tmp_path: Path) -> None:
    rig = game_over_rig(tmp_path / "a")
    rig.do(T.NEW_GAME)
    assert rig.state is S.SETUP_CHECK
    rig = game_over_rig(tmp_path / "b")
    rig.do(T.RESTART)
    assert rig.state is S.INIT


# --------------------------------------------------------------------------- #
# The FSM decides: guards that depend on the game
# --------------------------------------------------------------------------- #


def test_an_illegal_human_move_is_refused(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.WAIT_HUMAN)
    for bad in (UCI("e2e5"), UCI("e7e5"), "e2e4", None):
        result = rig.fsm.handle(T.HUMAN_MOVE, move=bad)
        assert not result.accepted and rig.state is S.WAIT_HUMAN


def test_commit_done_is_refused_until_the_human_move_is_in_game_state(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.COMMIT_HUMAN, ready=False)
    assert not rig.fsm.handle(T.COMMIT_DONE).accepted
    rig.game.commit_move(E2E4, MoveSource.HUMAN)
    assert rig.fsm.handle(T.COMMIT_DONE).accepted and rig.state is S.ENGINE_THINK


def test_engine_move_ready_is_refused_until_pending_move_is_stored(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.ENGINE_THINK, ready=False)
    assert not rig.fsm.handle(T.ENGINE_MOVE_READY).accepted
    rig.game.set_pending_move(E7E5)
    assert rig.fsm.handle(T.ENGINE_MOVE_READY).accepted and rig.state is S.PLAN


def test_verify_ok_is_refused_until_the_engine_move_is_committed(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.VERIFY, ready=False)
    assert not rig.fsm.handle(T.VERIFY_OK).accepted  # pending_move still unresolved
    rig.game.commit_move(E7E5, MoveSource.ENGINE)
    assert rig.fsm.handle(T.VERIFY_OK).accepted and rig.state is S.WAIT_HUMAN


def test_start_needs_a_side_and_a_game(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.SETUP_CHECK)
    assert not rig.fsm.handle(T.START).accepted
    assert not rig.fsm.handle(T.START, human_color="white").accepted
    assert not rig.fsm.handle(T.START, human_color=chess.WHITE, pending="skip").accepted
    assert not rig.fsm.handle(T.START, human_color=chess.WHITE, pending="redo").accepted  # nothing to redo
    bare = Fsm()
    bare.handle(T.YOLO_ON), bare.handle(T.INIT_DONE), bare.handle(T.CALIBRATE_DONE)
    assert not bare.handle(T.START, human_color=chess.WHITE).accepted  # no GameState attached


def test_start_is_refused_for_a_finished_game(tmp_path: Path) -> None:
    rig = Rig(tmp_path, fen="7k/5Q2/6K1/8/8/8/8/8 b - - 0 1").reach(S.SETUP_CHECK)
    assert not rig.fsm.handle(T.START, human_color=chess.WHITE).accepted


def test_a_refused_request_changes_nothing(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.WAIT_HUMAN)
    rig.do(T.PAUSE)
    snapshot = (rig.state, rig.fsm.pause_pending, rig.fsm.shutdown_pending, rig.fsm.resume_state, rig.game.fen())
    for trigger in (T.PAUSE, T.HUMAN_MOVE, T.START, T.EXEC_DONE, T.RESYNC, T.YOLO_ON):
        assert not rig.fsm.handle(trigger, **PAYLOADS.get(trigger, {})).accepted
    assert (rig.state, rig.fsm.pause_pending, rig.fsm.shutdown_pending, rig.fsm.resume_state, rig.game.fen()) == snapshot


def test_the_game_cannot_be_swapped_while_it_is_being_played(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.WAIT_HUMAN)
    with pytest.raises(RuntimeError):
        rig.fsm.attach_game_state(GameState())
    done = game_over_rig(tmp_path / "over")
    done.fsm.attach_game_state(GameState())  # allowed once the game is over


@pytest.mark.parametrize("state", ALL_STATES, ids=lambda s: s.value)
def test_the_move_detector_runs_only_in_wait_human(tmp_path: Path, state: State) -> None:
    assert rig_in(tmp_path, state).fsm.move_detector_enabled is (state is S.WAIT_HUMAN)


# --------------------------------------------------------------------------- #
# Pause rules (SPEC 7.3)
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("state", [S.IDLE, S.INIT, S.CALIBRATE, S.HOMING, S.SETUP_CHECK, S.COMMIT_HUMAN, S.PAUSED, S.ERROR_RECOVER, S.GAME_OVER],
                         ids=lambda s: s.value)
def test_pause_is_refused_where_the_spec_does_not_allow_it(tmp_path: Path, state: State) -> None:
    rig = rig_in(tmp_path, state)
    assert not rig.fsm.handle(T.PAUSE).accepted and not rig.fsm.pause_pending


def test_pause_in_wait_human_takes_effect_immediately(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.WAIT_HUMAN)
    result = rig.fsm.handle(T.PAUSE)
    assert result.accepted and not result.deferred and rig.state is S.PAUSED
    assert not rig.fsm.move_detector_enabled  # the Move Detector is locked
    assert rig.fsm.resume_state is S.WAIT_HUMAN
    rig.do(T.RESUME)
    assert rig.state is S.WAIT_HUMAN and rig.fsm.move_detector_enabled


def test_pause_in_engine_think_waits_for_the_proposed_move_then_stops_before_the_robot(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.ENGINE_THINK, ready=False)
    result = rig.fsm.handle(T.PAUSE)
    assert result.accepted and result.deferred and rig.state is S.ENGINE_THINK and rig.fsm.pause_pending
    rig.game.set_pending_move(E7E5)  # the engine proposes its move
    rig.do(T.ENGINE_MOVE_READY)
    assert rig.state is S.PAUSED  # stopped before PLAN, so the robot has not moved
    assert rig.game.pending_move == E7E5 and rig.game.fen().startswith("rnbqkbnr/pppppppp/8/8/4P3")
    rig.do(T.RESUME)
    assert rig.state is S.PLAN  # carries on with the stored move, the engine is not asked again


def test_pause_in_plan_waits_for_the_chain_then_stops_before_the_robot_moves(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.PLAN)
    assert rig.fsm.handle(T.PAUSE).deferred and rig.state is S.PLAN
    rig.do(T.PLAN_READY)
    assert rig.state is S.PAUSED and not rig.fsm.move_detector_enabled
    rig.do(T.RESUME)
    assert rig.state is S.ROBOT_EXEC


def test_pause_in_robot_exec_finishes_the_pick_and_place_and_verify_first(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.ROBOT_EXEC)
    assert rig.fsm.handle(T.PAUSE).deferred
    rig.do(T.EXEC_DONE)
    assert rig.state is S.VERIFY  # the chain finished; VERIFY still runs before pausing
    assert rig.fsm.pause_pending
    rig.game.commit_move(E7E5, MoveSource.ENGINE)
    rig.do(T.VERIFY_OK)
    assert rig.state is S.PAUSED and rig.game.pending_move is None  # no piece left hanging, move committed
    assert not rig.fsm.move_detector_enabled
    rig.do(T.RESUME)
    assert rig.state is S.WAIT_HUMAN


def test_pause_requested_during_verify_pauses_after_it(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.VERIFY, ready=False)
    rig.do(T.PAUSE)
    rig.game.commit_move(E7E5, MoveSource.ENGINE)
    rig.do(T.VERIFY_OK)
    assert rig.state is S.PAUSED


def test_a_fault_cancels_a_pending_pause(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.ROBOT_EXEC)
    rig.do(T.PAUSE)
    rig.do(T.EXEC_FAILED)
    assert rig.state is S.ERROR_RECOVER and not rig.fsm.pause_pending  # the error is handled first


def test_game_over_takes_precedence_over_a_pending_pause(tmp_path: Path) -> None:
    rig = Rig(tmp_path, fen=MATE_IN_ONE_FEN, human=chess.WHITE).reach(S.SETUP_CHECK)
    rig.do(T.START, human_color=chess.WHITE)
    mate = UCI("d8h4")
    rig.game.set_pending_move(mate)
    rig.do(T.ENGINE_MOVE_READY), rig.do(T.PLAN_READY)
    rig.do(T.PAUSE)
    rig.do(T.EXEC_DONE)
    rig.game.commit_move(mate, MoveSource.ENGINE)
    rig.do(T.VERIFY_OK)
    assert rig.state is S.GAME_OVER and not rig.fsm.pause_pending


def test_resume_cancels_a_pause_that_has_not_taken_effect_yet(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.PLAN)
    rig.do(T.PAUSE)
    result = rig.fsm.handle(T.RESUME)
    assert result.accepted and not rig.fsm.pause_pending
    rig.do(T.PLAN_READY)
    assert rig.state is S.ROBOT_EXEC  # no pause happened


def test_resume_is_refused_when_there_is_nothing_to_resume(tmp_path: Path) -> None:
    assert not Rig(tmp_path).reach(S.WAIT_HUMAN).fsm.handle(T.RESUME).accepted


def test_a_pause_that_is_already_pending_can_be_requested_again(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.PLAN)
    rig.do(T.PAUSE)
    rig.do(T.PAUSE)
    rig.do(T.PLAN_READY)
    assert rig.state is S.PAUSED


# --------------------------------------------------------------------------- #
# YOLO on / off (SPEC 7.3)
# --------------------------------------------------------------------------- #


def test_yolo_on_enters_init_and_is_refused_when_already_on(tmp_path: Path) -> None:
    rig = Rig(tmp_path)
    assert rig.state is S.IDLE and not rig.fsm.handle(T.YOLO_OFF).accepted
    rig.do(T.YOLO_ON)
    assert rig.state is S.INIT and not rig.fsm.handle(T.YOLO_ON).accepted


@pytest.mark.parametrize("state", [S.INIT, S.SETUP_CHECK, S.WAIT_HUMAN, S.ENGINE_THINK, S.PLAN, S.PAUSED, S.ERROR_RECOVER, S.GAME_OVER],
                         ids=lambda s: s.value)
def test_yolo_off_enters_idle_at_once_where_no_chain_is_running(tmp_path: Path, state: State) -> None:
    rig = rig_in(tmp_path, state)
    result = rig.fsm.handle(T.YOLO_OFF)
    assert result.accepted and not result.deferred and rig.state is S.IDLE
    assert not rig.fsm.move_detector_enabled


def test_yolo_off_in_robot_exec_finishes_the_chain_and_keeps_pending_move(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.ROBOT_EXEC)
    result = rig.fsm.handle(T.YOLO_OFF)
    assert result.accepted and result.deferred and rig.state is S.ROBOT_EXEC and rig.fsm.shutdown_pending
    rig.do(T.EXEC_DONE)
    assert rig.state is S.IDLE  # chain finished, arm parked by the caller, then IDLE: no VERIFY
    assert rig.game.pending_move == E7E5  # kept: the next start resolves it
    assert (rig.data_dir / "pending.json").exists()
    assert not rig.fsm.shutdown_pending


def test_yolo_off_in_verify_lets_verify_finish_first(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.VERIFY, ready=False)
    rig.do(T.YOLO_OFF)
    assert rig.state is S.VERIFY
    rig.game.commit_move(E7E5, MoveSource.ENGINE)
    rig.do(T.VERIFY_OK)
    assert rig.state is S.IDLE and rig.game.history[-1].san == "e5"


@pytest.mark.parametrize("state, done", [(S.CALIBRATE, T.CALIBRATE_DONE), (S.HOMING, T.HOMING_DONE)])
def test_yolo_off_during_calibration_or_homing_waits_for_it_to_finish(tmp_path: Path, state: State, done: Trigger) -> None:
    rig = Rig(tmp_path).reach(state)
    assert rig.fsm.handle(T.YOLO_OFF).deferred and rig.state is state
    rig.do(done)
    assert rig.state is S.IDLE


def test_yolo_off_during_commit_human_lets_the_commit_finish(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.COMMIT_HUMAN)
    rig.do(T.YOLO_OFF)
    rig.do(T.COMMIT_DONE)
    assert rig.state is S.IDLE and len(rig.game.history) == 1  # the human move is saved


def test_a_fault_cancels_a_pending_shutdown(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.ROBOT_EXEC)
    rig.do(T.YOLO_OFF)
    rig.do(T.EXEC_FAILED, alarm="ESTOP")
    assert rig.state is S.ERROR_RECOVER and not rig.fsm.shutdown_pending
    rig.do(T.YOLO_OFF)  # the user can still switch off from there
    assert rig.state is S.IDLE


def test_shutdown_wins_over_a_pending_pause(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.ROBOT_EXEC)
    rig.do(T.PAUSE)
    rig.do(T.YOLO_OFF)
    rig.do(T.EXEC_DONE)
    assert rig.state is S.IDLE and not rig.fsm.pause_pending


def test_yolo_on_after_off_starts_again_from_init(tmp_path: Path) -> None:
    rig = Rig(tmp_path).reach(S.WAIT_HUMAN)
    rig.do(T.YOLO_OFF)
    rig.do(T.YOLO_ON)
    assert rig.state is S.INIT and rig.game.fen() == chess.STARTING_FEN


# --------------------------------------------------------------------------- #
# Event Bus
# --------------------------------------------------------------------------- #


def test_state_changes_and_refusals_are_published(tmp_path: Path) -> None:
    bus = EventBus()
    changed: list[StateChanged] = []
    rejected: list[RequestRejected] = []
    bus.subscribe(StateChanged, changed.append)
    bus.subscribe(RequestRejected, rejected.append)
    rig = Rig(tmp_path, bus=bus)
    rig.do(T.YOLO_ON)
    rig.fsm.handle(T.PLAN_READY)
    assert [(c.previous, c.current, c.trigger) for c in changed] == [(S.IDLE, S.INIT, T.YOLO_ON)]
    assert [(r.trigger, r.state) for r in rejected] == [(T.PLAN_READY, S.INIT)]
    assert rejected[0].reason


def test_a_deferred_request_publishes_no_state_change(tmp_path: Path) -> None:
    bus = EventBus()
    changed: list[StateChanged] = []
    bus.subscribe(StateChanged, changed.append)
    rig = Rig(tmp_path, bus=bus).reach(S.PLAN)
    count = len(changed)
    rig.do(T.PAUSE)
    assert len(changed) == count


def test_requests_from_the_gui_arrive_through_the_bus_and_the_fsm_decides(tmp_path: Path) -> None:
    bus = EventBus()
    rejected: list[RequestRejected] = []
    bus.subscribe(RequestRejected, rejected.append)
    rig = Rig(tmp_path, bus=bus)
    bus.publish(FsmRequest(T.YOLO_ON))
    assert rig.state is S.INIT
    bus.publish(FsmRequest(T.PAUSE))  # not allowed in INIT
    assert rig.state is S.INIT and len(rejected) == 1


# --------------------------------------------------------------------------- #
# The FSM never writes to the game
# --------------------------------------------------------------------------- #


def test_the_fsm_source_never_calls_a_game_state_mutator() -> None:
    tree = ast.parse((REPO_ROOT / "desktop_app" / "core" / "fsm.py").read_text(encoding="utf-8"))
    called = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert not called & {"commit_move", "set_pending_move", "clear_pending_move", "push", "pop", "set_fen", "reset"}, called
