"""Console runner: play one full game with stubs instead of camera and robot (phase 2).

You type moves (SAN like ``e4`` / ``Nf3`` / ``O-O`` / ``e8=Q`` or UCI like ``e2e4``), the engine
replies, and the Move Planner prints the pick-and-place chain. A stub robot executes each chain
step by step on a simulated physical board and a stub camera reports what is on the squares, so
VERIFY compares the real outcome of the chain with the expected position.

Run from the repository root::

    python -m desktop_app.console_runner --color white --mode beginner --demo-placeholders

The capture/reserve/park layout comes from ``Zones.proposed`` (the owner's proposal, still
[TO CONFIRM]). The gripper angles and the content of each reserve slot are also [TO CONFIRM]
and are NOT in config yet, so ``--demo-placeholders`` substitutes clearly marked placeholders.

Commands while it is your turn: a move, ``moves``, ``pause`` (then ``resume``), ``quit``.
Everything is driven through the FSM: the runner does the work of a state and reports the result.
"""

from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Protocol, Sequence, TextIO

import chess

from desktop_app.core.engine import Engine, EngineError, load_engine_config
from desktop_app.core.fsm import Fsm, State, Trigger
from desktop_app.core.game_state import DEFAULT_GAMES_DIR, GameState, MoveSource
from desktop_app.core.move_planner import (
    DEFAULT_GRASP_PROFILES_PATH,
    GraspProfiles,
    GripAction,
    GripStep,
    MoveChain,
    MovePlanError,
    MovePlanner,
    MoveStep,
    ZDownStep,
    ZoneInventory,
    ZUpStep,
)
from desktop_app.kinematics.dimension import BoardGeometry, Workspace, Zones
from desktop_app.kinematics.five_bar import FiveBarKinematics
from desktop_app.utils.hardware_config import load_hardware_config
from desktop_app.utils.logging_setup import configure_logging

logger = logging.getLogger(__name__)

# PLACEHOLDERS, [TO CONFIRM] with the hardware owner. Used only with --demo-placeholders and
# never written to config/. They exist so the console game can run before the real values do.
PLACEHOLDER_GRASP = {
    "open_deg": 90,
    "close_deg": {"P": 60, "N": 60, "B": 60, "R": 60, "Q": 60, "K": 60},
}
PLACEHOLDER_RESERVE_PIECES = (chess.QUEEN, chess.ROOK, chess.BISHOP, chess.KNIGHT)


class EngineLike(Protocol):
    def best_move(self, fen: str) -> chess.Move: ...


# --------------------------------------------------------------------------- #
# Stubs: a simulated physical world, a robot that executes chains, a camera
# --------------------------------------------------------------------------- #


class StubRobotError(RuntimeError):
    """The chain asked the robot to do something physically impossible."""


class PhysicalWorld:
    """What is physically where: board squares ("e4"), capture slots and reserve slots by name."""

    def __init__(self, board: chess.Board, reserve: dict[str, chess.Piece]) -> None:
        self.pieces: dict[str, chess.Piece] = {}
        self.pieces.update(reserve)
        self.sync_squares(board)

    def sync_squares(self, board: chess.Board) -> None:
        """The human moved a piece by hand: make the squares match the board."""
        for name in [n for n in self.pieces if not n.startswith(("capture_slot", "reserve_slot"))]:
            del self.pieces[name]
        for square, piece in board.piece_map().items():
            self.pieces[chess.square_name(square)] = piece

    def squares(self) -> dict[str, chess.Piece]:
        return {n: p for n, p in self.pieces.items() if not n.startswith(("capture_slot", "reserve_slot"))}


class StubRobot:
    """Executes a MoveChain on the PhysicalWorld, refusing anything physically impossible."""

    def __init__(self, world: PhysicalWorld) -> None:
        self._world = world
        self._z_top = True
        self._holding: Optional[chess.Piece] = None
        self._at = "park"

    def execute(self, chain: MoveChain) -> None:
        for step in chain.steps:
            self._do(step)

    def park(self, chain: MoveChain) -> None:
        self._do(chain.park_leg)

    def _do(self, step: object) -> None:
        world = self._world
        if isinstance(step, MoveStep):
            if not self._z_top:
                raise StubRobotError(f"MOVE to {step.target} with Z not at TOP")
            self._at = step.target.name
        elif isinstance(step, ZDownStep):
            self._z_top = False
        elif isinstance(step, ZUpStep):
            self._z_top = True
        elif isinstance(step, GripStep):
            if self._z_top:
                raise StubRobotError("GRIP with Z at TOP")
            if step.action is GripAction.CLOSE:
                if self._holding is not None:
                    raise StubRobotError("GRIP close while already holding a piece")
                piece = world.pieces.get(self._at)
                if piece is None or piece.piece_type != step.piece_type:
                    raise StubRobotError(f"nothing to grip at {self._at}: expected {chess.piece_name(step.piece_type)}")
                self._holding = world.pieces.pop(self._at)
            else:
                if self._holding is None:
                    raise StubRobotError("GRIP open with nothing in the gripper")
                if self._at in world.pieces:
                    raise StubRobotError(f"{self._at} is occupied")
                world.pieces[self._at] = self._holding
                self._holding = None
        else:  # pragma: no cover
            raise StubRobotError(f"unknown step {step!r}")


def suspect_squares(expected: chess.Board, observed: dict[str, chess.Piece]) -> list[str]:
    """Squares where the observed pieces differ from the expected position (empty list = match)."""
    want = {chess.square_name(sq): piece for sq, piece in expected.piece_map().items()}
    return sorted(name for name in set(want) | set(observed) if want.get(name) != observed.get(name))


def classify_pending(
    observed: dict[str, chess.Piece], before: chess.Board, after: chess.Board
) -> str:
    """Compare the camera with the position before and after a pending move (SPEC 7.1).

    Returns ``"before"`` (the move never started: redo it), ``"after"`` (the move was fully
    made: commit it) or ``"neither"`` (a piece is half-way: resync by hand first).
    """
    if not suspect_squares(before, observed):
        return "before"
    if not suspect_squares(after, observed):
        return "after"
    return "neither"


# --------------------------------------------------------------------------- #
# The game loop
# --------------------------------------------------------------------------- #


@dataclass
class RunResult:
    exit_code: int
    final_state: State
    message: str


class ConsoleGame:
    """Drives one game through the FSM with injected input and output (so tests can script it)."""

    def __init__(
        self,
        *,
        game: GameState,
        engine: EngineLike,
        planner: MovePlanner,
        fsm: Fsm,
        input_fn: Callable[[str], str],
        out: TextIO,
        inventory: Optional[ZoneInventory] = None,
        initial_world: Optional[PhysicalWorld] = None,
    ) -> None:
        """``initial_world`` is what the stub camera sees at start (default: the GameState position)."""
        self._initial_world = initial_world
        self.game = game
        self.engine = engine
        self.planner = planner
        self.fsm = fsm
        self._input = input_fn
        self._out = out
        self.inventory = inventory if inventory is not None else ZoneInventory()
        self.world: Optional[PhysicalWorld] = None
        self.robot: Optional[StubRobot] = None
        self.chains: list[MoveChain] = []

    # ------------------------------------------------------------------ #

    def _say(self, text: str = "") -> None:
        self._out.write(text + "\n")

    def _ask(self, prompt: str) -> Optional[str]:
        try:
            return self._input(prompt).strip()
        except EOFError:
            return None

    def _build_world(self, human_color: chess.Color) -> None:
        if self._initial_world is not None:
            self.world = self._initial_world
        else:
            robot_color = not human_color
            reserve = {
                f"reserve_slot[{i}]": chess.Piece(piece_type, robot_color)
                for i, piece_type in enumerate(self.planner.reserve_pieces)
            }
            self.world = PhysicalWorld(self.game.board, reserve)
        self.robot = StubRobot(self.world)

    def run(self, human_color: chess.Color) -> RunResult:
        fsm, game = self.fsm, self.game
        fsm.attach_game_state(game)
        for trigger in (Trigger.YOLO_ON, Trigger.INIT_DONE, Trigger.CALIBRATE_DONE):  # stubbed: nothing to connect
            fsm.handle(trigger)
        self._build_world(human_color)

        pending_choice = self._resolve_pending_on_start()
        if pending_choice is None:
            return RunResult(1, fsm.state, "the saved pending move cannot be resolved automatically; resync by hand")
        started = fsm.handle(Trigger.START, human_color=human_color, **pending_choice)
        if not started.accepted:
            return RunResult(1, fsm.state, started.reason)

        while True:
            state = fsm.state
            if state is State.WAIT_HUMAN:
                result = self._wait_human()
            elif state is State.ENGINE_THINK:
                result = self._engine_think()
            elif state is State.PLAN:
                result = self._plan()
            elif state is State.ROBOT_EXEC:
                result = self._robot_exec()
            elif state is State.VERIFY:
                result = self._verify()
            elif state is State.PAUSED:
                result = self._paused()
            elif state is State.GAME_OVER:
                return self._game_over()
            elif state is State.IDLE:
                return RunResult(0, state, "stopped")
            else:  # ERROR_RECOVER, HOMING, ...: the stubs never get here unless something is wrong
                return RunResult(1, state, f"stopped in {state.value}")
            if result is not None:
                return result

    # ------------------------------------------------------------------ #
    # Work of each state; each returns a RunResult only when the run must end
    # ------------------------------------------------------------------ #

    def _resolve_pending_on_start(self) -> Optional[dict[str, str]]:
        """SETUP_CHECK: compare the camera with the position before/after a recovered pending move."""
        game = self.game
        move = game.pending_move
        if move is None:
            return {}
        assert self.world is not None
        outcome = classify_pending(self.world.squares(), game.board, game.preview_move(move))
        self._say(f"Recovered an unfinished engine move {game.san(move)} ({move.uci()}): the board shows {outcome}.")
        if outcome == "before":
            return {"pending": "redo"}
        if outcome == "after":
            game.commit_move(move, MoveSource.ENGINE)
            self._say("The move had been fully made; it is committed now.")
            return {}
        return None

    def _wait_human(self) -> Optional[RunResult]:
        game, fsm = self.game, self.fsm
        assert self.world is not None
        self._say()
        self._say(str(game.board))
        raw = self._ask(f"{'White' if game.turn else 'Black'} (you) move [{game.board.fullmove_number}]: ")
        if raw is None or raw.lower() in ("quit", "exit"):
            fsm.handle(Trigger.YOLO_OFF)
            return None
        if raw.lower() == "moves":
            board = game.board
            self._say(" ".join(sorted(board.san(m) for m in game.legal_moves)))
            return None
        if raw.lower() == "pause":
            fsm.handle(Trigger.PAUSE)
            return None
        move = self._parse(raw)
        if move is None:
            self._say(f"'{raw}' is not a legal move here. Type 'moves' to list them.")
            return None
        if not fsm.handle(Trigger.HUMAN_MOVE, move=move).accepted:
            self._say("The FSM refused that move.")
            return None
        game.commit_move(move, MoveSource.HUMAN)
        self.world.sync_squares(game.board)  # the human moved the piece by hand
        fsm.handle(Trigger.COMMIT_DONE)
        return None

    def _parse(self, text: str) -> Optional[chess.Move]:
        board = self.game.board
        for parse in (board.parse_san, board.parse_uci):
            try:
                move = parse(text)
            except ValueError:
                continue
            if move in board.legal_moves:
                return move
        return None

    def _paused(self) -> Optional[RunResult]:
        raw = self._ask("Paused. Type 'resume' or 'quit': ")
        if raw is None or raw.lower() in ("quit", "exit"):
            self.fsm.handle(Trigger.YOLO_OFF)
        elif raw.lower() == "resume":
            self.fsm.handle(Trigger.RESUME)
        return None

    def _engine_think(self) -> Optional[RunResult]:
        game, fsm = self.game, self.fsm
        self._say("Engine is thinking...")
        try:
            move = self.engine.best_move(game.fen())  # always the FEN from GameState
        except EngineError as exc:
            self._say(f"Engine error: {exc}")
            raw = self._ask("Press Enter to retry or type 'quit': ")
            if raw is None or raw.lower() in ("quit", "exit"):
                fsm.handle(Trigger.YOLO_OFF)
            return None
        game.set_pending_move(move)
        fsm.handle(Trigger.ENGINE_MOVE_READY)
        return None

    def _plan(self) -> Optional[RunResult]:
        game, fsm = self.game, self.fsm
        move = game.pending_move
        assert move is not None
        try:
            chain = self.planner.plan(game.board, move, self.inventory)
        except MovePlanError as exc:
            self._say(f"Cannot plan {move.uci()}: {exc}")
            raw = self._ask("Fix it, then press Enter to retry ('emptied' = capture zone emptied, 'quit'): ")
            if raw is None or raw.lower() in ("quit", "exit"):
                fsm.handle(Trigger.YOLO_OFF)
            elif raw.lower() == "emptied":
                self.inventory = ZoneInventory(capture_used=0, reserve_used=self.inventory.reserve_used)
                if self.world is not None:  # the operator physically took the pieces out of the zone
                    for name in [n for n in self.world.pieces if n.startswith("capture_slot")]:
                        del self.world.pieces[name]
            return None
        self.chains.append(chain)
        for line in chain.describe():
            self._say(line)
        self._pending_chain = chain
        fsm.handle(Trigger.PLAN_READY)
        return None

    def _robot_exec(self) -> Optional[RunResult]:
        assert self.robot is not None
        try:
            self.robot.execute(self._pending_chain)
        except StubRobotError as exc:
            self._say(f"Robot stub error: {exc}")
            self.fsm.handle(Trigger.EXEC_FAILED)
            return None
        self.fsm.handle(Trigger.EXEC_DONE)
        return None

    def _verify(self) -> Optional[RunResult]:
        game, fsm = self.game, self.fsm
        assert self.robot is not None and self.world is not None
        move = game.pending_move
        assert move is not None
        self.robot.park(self._pending_chain)
        expected = game.preview_move(move)
        wrong = suspect_squares(expected, self.world.squares())
        if wrong:
            self._say(f"VERIFY failed, suspect squares: {', '.join(wrong)}")
            fsm.handle(Trigger.VERIFY_FAILED)
            return None
        record = game.commit_move(move, MoveSource.ENGINE)
        self.inventory = self.inventory.consume(self._pending_chain)
        self._say(f"Engine plays {record.san}.")
        fsm.handle(Trigger.VERIFY_OK)
        return None

    def _game_over(self) -> RunResult:
        outcome = self.game.outcome
        assert outcome is not None
        self._say()
        self._say(str(self.game.board))
        self._say(f"Game over: {outcome.result} ({outcome.termination}).")
        if self.game.data_dir is not None:
            self._say(f"PGN saved in {self.game.data_dir / 'current.pgn'}")
        return RunResult(0, State.GAME_OVER, f"{outcome.result} {outcome.termination}")

    _pending_chain: MoveChain


# --------------------------------------------------------------------------- #
# Command line
# --------------------------------------------------------------------------- #


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Play one console game against Stockfish with the Move Planner.")
    parser.add_argument("--color", choices=("white", "black"), default="white", help="your side")
    parser.add_argument("--mode", choices=("beginner", "amateur", "player", "master", "grandmaster"), default=None)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_GAMES_DIR, help="where current.fen/pgn/pending.json go")
    parser.add_argument("--new", action="store_true", help="ignore a saved game and start a new one")
    parser.add_argument("--demo-placeholders", action="store_true", help="use placeholder gripper angles and reserve contents")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="WARNING")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    configure_logging(getattr(logging, args.log_level))
    out = sys.stdout

    if DEFAULT_GRASP_PROFILES_PATH.exists():
        grasp = GraspProfiles.from_file()
    elif args.demo_placeholders:
        grasp = GraspProfiles.from_dict(PLACEHOLDER_GRASP)
    else:
        out.write(
            "config/grasp_profiles.json does not exist and the gripper angles are [TO CONFIRM].\n"
            "Ask the hardware owner, or run with --demo-placeholders to use clearly marked placeholders.\n"
        )
        return 2
    if not args.demo_placeholders:
        out.write(
            "The content of each reserve slot is [TO CONFIRM] and not in config yet; "
            "run with --demo-placeholders to use placeholder reserve pieces.\n"
        )
        return 2
    out.write(
        "NOTE: placeholder values in use: reserve slot pieces "
        f"{[chess.piece_name(p) for p in PLACEHOLDER_RESERVE_PIECES]}"
        + (", gripper angles" if not DEFAULT_GRASP_PROFILES_PATH.exists() else "")
        + ". Capture/reserve/park coordinates are the owner's PROPOSED layout. All are [TO CONFIRM].\n"
    )

    hardware = load_hardware_config()
    workspace = Workspace(FiveBarKinematics.from_config(hardware))
    geometry = BoardGeometry.from_config(hardware)
    zones = Zones.proposed(geometry, workspace)
    planner = MovePlanner(workspace, geometry, zones, grasp, reserve_pieces=PLACEHOLDER_RESERVE_PIECES)

    game = None if args.new else GameState.load(args.data_dir)
    if game is None:
        game = GameState.new_game(args.data_dir)
    inventory = ZoneInventory.from_history(game.history, planner.reserve_pieces)

    engine = Engine(load_engine_config(), mode=args.mode)
    try:
        runner = ConsoleGame(
            game=game, engine=engine, planner=planner, fsm=Fsm(), input_fn=input, out=out, inventory=inventory
        )
        result = runner.run(chess.WHITE if args.color == "white" else chess.BLACK)
    finally:
        engine.close()
    out.write(f"{result.message}\n")
    return result.exit_code


if __name__ == "__main__":
    sys.exit(main())
