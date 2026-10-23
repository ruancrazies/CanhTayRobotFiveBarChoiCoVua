"""The whole stack, nothing mocked: FSM to Move Planner to Robot Commander to serial link to ESP32 simulator.

The simulator's trace (every Z-down and every GRIP with the joint angles at that moment) is replayed through the
forward kinematics onto a physical board, so a pick-and-place is checked by where the arm really went.
"""

from __future__ import annotations

from pathlib import Path

import chess
import pytest

from config.protocol_commands import CommandName, ErrorCode
from desktop_app.core.fsm import State
from desktop_app.hardware.robot_commander import CommandFailedError
from test_robot_commander import plan_for
from phase3_support import REPLAY_TOLERANCE_MM, SCENARIOS, build_stack

# (picks, places) the arm must have made, in order, for each scenario.
EXPECTED_ROUTES = {
    "normal": (["e7"], ["e5"]),
    "capture": (["e4", "d5"], ["capture_slot[0]", "e4"]),
    "castling": (["e8", "h8"], ["g8", "f8"]),
    "promotion": (["a2", "reserve_slot[0]"], ["capture_slot[0]", "a1"]),
    "en_passant": (["e4", "d4"], ["capture_slot[0]", "e3"]),
    "capture_promotion": (["b1", "a2", "reserve_slot[0]"], ["capture_slot[0]", "capture_slot[1]", "b1"]),
}


@pytest.mark.parametrize("scenario", sorted(SCENARIOS))
def test_full_move_end_to_end(tmp_path: Path, scenario: str) -> None:
    stack = build_stack(tmp_path, scenario)
    try:
        stack.run_full_move(scenario)

        # FSM: HOMING, then through PLAN, ROBOT_EXEC and VERIFY back to the human's turn
        assert stack.fsm.state is State.WAIT_HUMAN
        for state in (State.HOMING, State.PLAN, State.ROBOT_EXEC, State.VERIFY):
            assert state in stack.states
        assert stack.states.index(State.PLAN) < stack.states.index(State.ROBOT_EXEC) < stack.states.index(State.VERIFY)

        # the engine move is in GameState, which is the only thing that changed the position
        _fen, _human, engine = SCENARIOS[scenario]
        assert stack.game.board.peek() == chess.Move.from_uci(engine)
        assert stack.game.pending_move is None

        # what the arm physically did
        picks, places = EXPECTED_ROUTES[scenario]
        assert stack.replay.errors == []
        assert (stack.replay.picks, stack.replay.places) == (picks, places)
        assert stack.replay.worst_error_mm < REPLAY_TOLERANCE_MM
        assert stack.world.squares() == {
            chess.square_name(sq): piece for sq, piece in stack.game.board.piece_map().items()
        }

        # the arm is parked again and homed; nothing unsafe happened on the wire
        assert stack.sim.violations == []
        assert stack.sim.z_state.value == "TOP"
        assert stack.sim.alarms == ()
        assert stack.alarm_requests == []
        assert stack.commander.homed
    finally:
        stack.close()


def test_wire_order_is_wait_before_every_z_move_and_grip(tmp_path: Path) -> None:
    stack = build_stack(tmp_path, "capture")
    try:
        stack.run_full_move("capture")
        names = [line.split()[0] for line in stack.sim.received if line.split()[0] != CommandName.PING.value]
        for index, name in enumerate(names):
            if name in (CommandName.ZDOWN.value, CommandName.GRIP.value) and names[index - 1] == CommandName.MOVE.value:
                pytest.fail("a MOVE is followed directly by " + name)
            if name == CommandName.ZDOWN.value:
                assert names[index - 1] == CommandName.WAIT.value
    finally:
        stack.close()


def test_two_moves_in_a_row_use_the_inventory(tmp_path: Path) -> None:
    """The second chain starts where the first ended (parked) and takes the next free capture slot."""
    stack = build_stack(tmp_path, "capture")
    try:
        stack.run_full_move("capture")
        assert stack.inventory.capture_used == 1
        # a second chain from the same stack: the arm is at park and homed, so it is accepted
        chain = stack.planner.plan(chess.Board(), chess.Move.from_uci("e2e4"), stack.inventory)
        stack.commander.execute_chain(chain)
        stack.commander.park(chain)
        assert stack.sim.violations == []
    finally:
        stack.close()


# --------------------------------------------------------------------------- #
# Queue full
# --------------------------------------------------------------------------- #


def test_queue_full_is_retried_and_the_order_is_kept(tmp_path: Path) -> None:
    """A tiny firmware queue forces error:3 again and again; the chain still lands every piece correctly."""
    stack = build_stack(tmp_path, "capture", sim_overrides=dict(queue_capacity=2))
    try:
        stack.run_full_move("capture")
        assert any(line == f"error:{int(ErrorCode.QUEUE_FULL)}" for line in stack.sim.sent), "no queue-full happened"
        assert stack.replay.picks == EXPECTED_ROUTES["capture"][0]
        assert stack.replay.places == EXPECTED_ROUTES["capture"][1]
        assert stack.sim.violations == []
    finally:
        stack.close()


def test_a_queue_that_stays_full_stops_the_chain_with_error_3(tmp_path: Path) -> None:
    """Slow motion (low speed) and a one-entry queue: the retry gives up and the FSM goes to ERROR_RECOVER."""
    stack = build_stack(
        tmp_path,
        sim_overrides=dict(queue_capacity=1),
        settings_overrides=dict(move_speed_deg_s=1.0, queue_full_timeout_s=0.1, queue_retry_s=0.01),
    )
    try:
        stack.homing()
        stack.reach_plan(*SCENARIOS["normal"][1:])
        stack.plan()
        failure = stack.execute()
        assert isinstance(failure, CommandFailedError)
        assert failure.code is ErrorCode.QUEUE_FULL
        assert stack.fsm.state is State.ERROR_RECOVER
        assert stack.game.pending_move is not None  # kept: the engine move is redone after recovery
        assert not stack.fsm.homing_required  # nothing moved wrongly; no alarm, no homing demanded by the FSM
        assert stack.alarm_requests == []
    finally:
        stack.close()


def test_an_error_other_than_queue_full_is_not_retried(tmp_path: Path) -> None:
    """The PC believes it is homed, the ESP32 is not (a reboot): error:4 stops the chain at once, no retry."""
    stack = build_stack(tmp_path)
    try:
        stack.commander._homed = True  # white-box on purpose: the simulator was never homed
        _planner, chain = plan_for("normal")
        with pytest.raises(CommandFailedError) as raised:
            stack.commander.execute_chain(chain)
        assert raised.value.code is ErrorCode.NOT_HOMED
        moves = [l for l in stack.sim.received if l.split()[0] == CommandName.MOVE.value]
        assert len(moves) == 1, "error:4 must stop the chain at the first MOVE, with no retry"
        assert stack.sim.sent.count(f"error:{int(ErrorCode.NOT_HOMED)}") == 1
    finally:
        stack.close()
