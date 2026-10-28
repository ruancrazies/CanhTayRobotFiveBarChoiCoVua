"""Shared helpers for the phase 2 tests: fake engine, planner builders and an FSM rig.

Nothing here needs the Stockfish binary. The grasp angles below are TEST VALUES ONLY: the real
ones are hardware values that are assumed and live in the ``grasp`` section of config/settings.yaml.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Optional

import chess

from desktop_app.core.engine import EngineConfig, UciBackend, load_engine_config
from desktop_app.core.fsm import Fsm, State, Trigger
from desktop_app.core.game_state import GameState, MoveSource
from desktop_app.core.move_planner import GraspProfiles, MovePlanner
from desktop_app.kinematics.dimension import BoardGeometry, Workspace, Zones
from desktop_app.kinematics.five_bar import FiveBarKinematics
from desktop_app.utils.settings import get_settings

# The capture, reserve and park layout and the singularity margin of config/settings.yaml, as test fixtures.
CAPTURE_SLOTS = get_settings().layout.capture_slots.slots()
RESERVE_SLOTS = get_settings().layout.reserve_slots.slots()
PARK_POSITION = get_settings().layout.park.xy_mm
MIN_SINGULARITY_MARGIN_DEG = get_settings().motion.singularity_margin_deg

TEST_GRASP = {"open_deg": 100, "close_deg": {"P": 41, "N": 42, "B": 43, "R": 44, "Q": 45, "K": 46}}  # TEST ONLY
TEST_RESERVE = (chess.QUEEN, chess.ROOK, chess.BISHOP, chess.KNIGHT)

E2E4 = chess.Move.from_uci("e2e4")
E7E5 = chess.Move.from_uci("e7e5")

# After 1.f3 e5 2.g4 black mates with Qh4.
MATE_IN_ONE_FEN = "rnbqkbnr/pppp1ppp/8/4p3/6P1/5P2/PPPPP2P/RNBQKBNR b KQkq - 0 2"
MATE_IN_ONE = chess.Move.from_uci("d8h4")


# --------------------------------------------------------------------------- #
# Fake engine
# --------------------------------------------------------------------------- #


class FakeBackends:
    """A backend factory whose backends follow a shared script.

    ``script`` holds one item per ``play`` call: ``"ok"`` (first legal move), a ``chess.Move``,
    or an exception instance to raise. Past the end of the script every call answers ``"ok"``.
    """

    def __init__(self, script: Optional[list[object]] = None) -> None:
        self.script = list(script or [])
        self.created = 0
        self.backends: list[FakeBackend] = []
        self.plays: list[dict[str, Any]] = []

    def __call__(self) -> "FakeBackend":
        self.created += 1
        backend = FakeBackend(self)
        self.backends.append(backend)
        return backend


class FakeBackend(UciBackend):  # type: ignore[misc]
    def __init__(self, owner: FakeBackends) -> None:
        self._owner = owner
        self.configured: list[dict[str, object]] = []
        self.quit_called = False

    def configure(self, options: Mapping[str, object]) -> None:
        self.configured.append(dict(options))

    def play(self, board: chess.Board, *, time_s: float, depth: Optional[int], answer_margin_s: float) -> chess.Move:
        self._owner.plays.append(
            {
                "fen": board.fen(),
                "stack": len(board.move_stack),
                "time_s": time_s,
                "depth": depth,
                "margin": answer_margin_s,
                "backend": self,
            }
        )
        item = self._owner.script.pop(0) if self._owner.script else "ok"
        if isinstance(item, Exception):
            raise item
        if isinstance(item, chess.Move):
            return item
        return next(iter(board.legal_moves))

    def quit(self) -> None:
        self.quit_called = True


def engine_config() -> EngineConfig:
    return load_engine_config()


class ScriptedEngine:
    """Console-runner engine: plays the listed UCI moves in order and remembers the FENs it got."""

    def __init__(self, moves: list[str]) -> None:
        self.moves = list(moves)
        self.fens: list[str] = []

    def best_move(self, fen: str) -> chess.Move:
        self.fens.append(fen)
        return chess.Move.from_uci(self.moves.pop(0))

    def close(self) -> None:
        pass


# --------------------------------------------------------------------------- #
# Planner builders
# --------------------------------------------------------------------------- #


def make_workspace(min_margin_deg: float = MIN_SINGULARITY_MARGIN_DEG) -> Workspace:
    return Workspace(FiveBarKinematics.from_config(), min_margin_deg)


def make_geometry() -> BoardGeometry:
    return BoardGeometry.from_config()


def make_zones(
    capture_slots: Optional[int] = len(CAPTURE_SLOTS),
    reserve_slots: Optional[int] = len(RESERVE_SLOTS),
    park: bool = True,
    is_placeholder: bool = True,
) -> Zones:
    """Zones validated with the standard workspace; ``None`` leaves a zone undefined ([TO CONFIRM]).

    The layout is a placeholder unless a test says otherwise (``is_placeholder=False``)."""
    return Zones(
        make_geometry(),
        make_workspace(),
        capture_zone=CAPTURE_SLOTS[:capture_slots] if capture_slots is not None else None,
        reserve_zone=RESERVE_SLOTS[:reserve_slots] if reserve_slots is not None else None,
        park_position=PARK_POSITION if park else None,
        is_placeholder=is_placeholder,
    )


def make_planner(
    *,
    capture_slots: Optional[int] = len(CAPTURE_SLOTS),
    reserve_pieces: tuple[chess.PieceType, ...] = TEST_RESERVE,
    workspace: Optional[Workspace] = None,
    park: bool = True,
    zones_placeholder: bool = True,
) -> MovePlanner:
    reserve_slots = len(reserve_pieces) if reserve_pieces else None
    return MovePlanner(
        workspace or make_workspace(),
        make_geometry(),
        make_zones(capture_slots, reserve_slots, park, is_placeholder=zones_placeholder),
        GraspProfiles.from_dict(TEST_GRASP),
        reserve_pieces=reserve_pieces,
    )


# --------------------------------------------------------------------------- #
# FSM rig
# --------------------------------------------------------------------------- #


class Rig:
    """An FSM with a real GameState in a temp folder, plus helpers to drive it to any state."""

    def __init__(self, data_dir: Path, *, fen: Optional[str] = None, human: chess.Color = chess.WHITE, bus: Any = None) -> None:
        self.data_dir = data_dir
        self.human = human
        self.game = GameState.new_game(data_dir, fen=fen)
        self.fsm = Fsm(self.game, bus)

    def do(self, trigger: Trigger, **payload: Any) -> None:
        result = self.fsm.handle(trigger, **payload)
        assert result.accepted, f"{trigger.value} refused in {result.previous.value}: {result.reason}"

    def reach(self, target: State, *, ready: bool = True) -> "Rig":
        """Drive a fresh rig to ``target``. ``ready`` also finishes the work that state is waiting for
        (for example the human move is committed in COMMIT_HUMAN, the engine move is stored in
        ENGINE_THINK, and the engine move is committed in VERIFY), so its natural trigger is valid."""
        game, fsm = self.game, self.fsm
        if target is State.IDLE:
            return self
        self.do(Trigger.YOLO_ON)
        if target is State.INIT:
            return self
        self.do(Trigger.INIT_DONE)
        if target is State.CALIBRATE:
            return self
        self.do(Trigger.CALIBRATE_DONE)
        if target is State.HOMING:  # the startup HOMING: INIT -> CALIBRATE -> HOMING -> SETUP_CHECK
            return self
        self.do(Trigger.HOMING_DONE)
        if target is State.SETUP_CHECK:
            return self
        self.do(Trigger.START, human_color=self.human)
        if target is State.WAIT_HUMAN:
            return self
        if target is State.PAUSED:
            self.do(Trigger.PAUSE)
            return self

        human_move = MATE_IN_ONE if game.fen() == MATE_IN_ONE_FEN else E2E4
        self.do(Trigger.HUMAN_MOVE, move=human_move)
        if target is State.COMMIT_HUMAN:
            if ready:
                game.commit_move(human_move, MoveSource.HUMAN)
            return self
        game.commit_move(human_move, MoveSource.HUMAN)
        self.do(Trigger.COMMIT_DONE)
        if target is State.GAME_OVER:
            return self
        if target is State.ENGINE_THINK:
            if ready:
                game.set_pending_move(E7E5)
            return self
        game.set_pending_move(E7E5)
        self.do(Trigger.ENGINE_MOVE_READY)
        if target is State.PLAN:
            return self
        self.do(Trigger.PLAN_READY)
        if target is State.ROBOT_EXEC:
            return self
        if target is State.VERIFY:
            self.do(Trigger.EXEC_DONE)
            if ready:
                game.commit_move(E7E5, MoveSource.ENGINE)
            return self
        if target is State.ERROR_RECOVER:
            self.do(Trigger.EXEC_FAILED)
            return self
        raise AssertionError(f"cannot reach {target}")

    @property
    def state(self) -> State:
        return self.fsm.state


def game_over_rig(data_dir: Path) -> Rig:
    """A rig in GAME_OVER (the human, black, mates in one)."""
    return Rig(data_dir, fen=MATE_IN_ONE_FEN, human=chess.BLACK).reach(State.GAME_OVER)
