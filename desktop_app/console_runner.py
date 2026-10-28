"""Console runner: play one full game with stubs instead of camera and robot (phase 2).

You type moves (SAN like ``e4`` / ``Nf3`` / ``O-O`` / ``e8=Q`` or UCI like ``e2e4``), the engine
replies, and the Move Planner prints the pick-and-place chain. A stub robot executes each chain
step by step on a simulated physical board and a stub camera reports what is on the squares, so
VERIFY compares the real outcome of the chain with the expected position.

Run from the repository root::

    python -m desktop_app.console_runner --color white --mode beginner --demo-placeholders

The capture/reserve/park layout, the gripper angles and the content of each reserve slot come from
config/settings.yaml. While any of them is listed in its ``assumed`` list (not confirmed on the real board) the
runner refuses to start, unless ``--demo-placeholders`` says the assumed values may be used.

Commands while it is your turn: a move, ``moves``, ``pause`` (then ``resume``), ``alarm <CODE>``
(simulates ESTOP, HB_TIMEOUT, SENSOR_FAULT, CAMERA_LOST, ...), ``quit``.
Everything is driven through the FSM: the runner does the work of a state and reports the result.
An engine failure, a planning failure, a robot or VERIFY failure or an alarm leads to ERROR_RECOVER,
where you fix the cause and type ``resync`` (``fixed``, ``emptied`` and ``quit`` also work there).
The stub robot may only move in HOMING, ROBOT_EXEC and VERIFY (SPEC 7.2).
"""

from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Protocol, Sequence, TextIO

import chess

from desktop_app.core.engine import MODE_KEYS, Engine, EngineError, engine_config_from_settings
from desktop_app.core.fsm import ALARM_CODES, Fsm, State, Trigger
from desktop_app.core.game_state import GameState, MoveSource, default_games_dir
from desktop_app.core.move_planner import (
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
    reserve_slots_from_settings,
)
from desktop_app.kinematics.dimension import BoardGeometry, Workspace, Zones
from desktop_app.kinematics.five_bar import FiveBarKinematics
from desktop_app.utils.logging_setup import configure_logging
from desktop_app.utils.settings import ConfigError, get_settings, load_settings

logger = logging.getLogger(__name__)


class EngineLike(Protocol):
    def best_move(self, fen: str) -> chess.Move: ...


# --------------------------------------------------------------------------- #
# Stubs: a simulated physical world, a robot that executes chains, a camera
# --------------------------------------------------------------------------- #


class StubRobotError(RuntimeError):
    """The chain asked the robot to do something physically impossible."""


class ArmLockedError(RuntimeError):
    """The arm was asked to move in a state where it must not (SPEC 7.2). A bug, never a normal failure."""


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

    def __init__(self, world: PhysicalWorld, may_move: Optional[Callable[[], bool]] = None) -> None:
        """``may_move`` says whether the arm is allowed to move now (the FSM's ``arm_may_move``)."""
        self._world = world
        self._may_move = may_move
        self._z_top = True
        self._holding: Optional[chess.Piece] = None
        self._at = "park"

    def _check_allowed(self, what: str) -> None:
        if self._may_move is not None and not self._may_move():
            raise ArmLockedError(f"the arm may not move now ({what}): only HOMING, ROBOT_EXEC and VERIFY move it")

    def home(self) -> None:
        """Homing: Z to TOP, arm to the park position. A piece left in the gripper is dropped by hand."""
        self._check_allowed("home")
        self._z_top = True
        self._holding = None
        self._at = "park"

    def execute(self, chain: MoveChain) -> None:
        self._check_allowed("execute")
        for step in chain.steps:
            self._do(step)

    def park(self, chain: MoveChain) -> None:
        self._check_allowed("park")
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
        alarm_source: Optional[Callable[[State], Optional[str]]] = None,
    ) -> None:
        """``initial_world`` is what the stub camera sees at start (default: the GameState position).

        ``alarm_source`` stands in for the ESP32 and camera threads: it is asked once per state the
        loop enters and may return an alarm code (for example ``"ESTOP"``) to raise in that state.
        """
        self._initial_world = initial_world
        self._alarm_source = alarm_source
        self._human_color: chess.Color = chess.WHITE
        self._error = ""
        self._pending_chain: Optional[MoveChain] = None
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
        self.states_visited: list[State] = []

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
        self.robot = StubRobot(self.world, may_move=lambda: self.fsm.arm_may_move)

    def run(self, human_color: chess.Color) -> RunResult:
        fsm, game = self.fsm, self.game
        self._human_color = human_color
        fsm.attach_game_state(game)
        # INIT and CALIBRATE are stubbed (nothing to connect). The FSM then enters HOMING, which the loop
        # below performs, and only after that SETUP_CHECK: INIT -> CALIBRATE -> HOMING -> SETUP_CHECK.
        for trigger in (Trigger.YOLO_ON, Trigger.INIT_DONE, Trigger.CALIBRATE_DONE):
            fsm.handle(trigger)
        self._build_world(human_color)

        while True:
            state = fsm.state
            self.states_visited.append(state)
            if self._inject_alarm(state):
                continue
            result: Optional[RunResult]
            if state is State.SETUP_CHECK:
                result = self._setup_check()
            elif state is State.HOMING:
                result = self._homing()
            elif state is State.WAIT_HUMAN:
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
            elif state is State.ERROR_RECOVER:
                result = self._error_recover()
            elif state is State.GAME_OVER:
                return self._game_over()
            elif state is State.IDLE:
                return RunResult(0, state, "stopped")
            else:  # INIT / CALIBRATE are stubbed away above, so this is a bug
                return RunResult(1, state, f"stopped in {state.value}")
            if result is not None:
                return result

    # ------------------------------------------------------------------ #
    # Work of each state; each returns a RunResult only when the run must end
    # ------------------------------------------------------------------ #

    def _raise_alarm(self, code: str, text: str) -> bool:
        """Report an alarm to the FSM; True when it was accepted (the FSM decides)."""
        result = self.fsm.handle(Trigger.ALARM, alarm=code)
        if result.accepted:
            self._error = text
            self._say(f"ALARM {code}: {text}")
        else:
            self._say(f"The FSM refused the alarm: {result.reason}")
        return result.accepted

    def _inject_alarm(self, state: State) -> bool:
        if self._alarm_source is None:
            return False
        code = self._alarm_source(state)
        return code is not None and self._raise_alarm(code, f"simulated alarm in {state.value}")

    def _fail(self, trigger: Trigger, text: str) -> None:
        """Report a failure of the work of a state: the FSM moves to ERROR_RECOVER."""
        self._error = text
        self.fsm.handle(trigger, **({"reason": text} if trigger in (Trigger.ENGINE_FAILED, Trigger.PLAN_FAILED) else {}))

    def _setup_check(self) -> Optional[RunResult]:
        """SETUP_CHECK: compare the camera with a recovered pending move, then press Start."""
        pending_choice = self._resolve_pending()
        if pending_choice is None:
            return RunResult(1, self.fsm.state, "the saved pending move cannot be resolved automatically; resync by hand")
        started = self.fsm.handle(Trigger.START, human_color=self._human_color, **pending_choice)
        if not started.accepted:
            return RunResult(1, self.fsm.state, started.reason)
        return None

    def _homing(self) -> Optional[RunResult]:
        assert self.robot is not None
        self._say("Homing (stub): Z up, then both joints.")
        self.robot.home()
        self.fsm.handle(Trigger.HOMING_DONE)
        return None

    def _error_recover(self) -> Optional[RunResult]:
        game = self.game
        self._say(f"ERROR_RECOVER: {self._error or 'unknown error'}")
        if game.pending_move is not None:
            self._say(f"The pending move {game.pending_move.uci()} is kept.")
        if self.fsm.homing_required:
            self._say("Homing is mandatory before the game goes on.")
        raw = self._ask(
            "Fix the cause, then type 'resync' ('fixed' = pieces put back to the position before the move, "
            "'emptied' = capture zone emptied, 'quit'): "
        )
        if raw is None or raw.lower() in ("quit", "exit"):
            self.fsm.handle(Trigger.YOLO_OFF)
        elif raw.lower() == "emptied":
            self._empty_capture_zone()
        elif raw.lower() == "fixed":
            assert self.world is not None
            self.world.sync_squares(game.board)  # the operator put every piece back by hand
            self.robot = StubRobot(self.world, may_move=lambda: self.fsm.arm_may_move)
        elif raw.lower() in ("resync", ""):
            if game.pending_move is not None and not self.fsm.homing_required:
                # Like SETUP_CHECK: the board may already show the finished move (for example the camera
                # was lost during VERIFY). Commit it instead of redoing it; refuse while a piece is half-way.
                if self._resolve_pending() is None:
                    self._say("A piece is half-way: put the pieces back ('fixed') or finish the move, then resync.")
                    return None
            result = self.fsm.handle(Trigger.RESYNC)
            if not result.accepted:
                self._say(f"Cannot resync: {result.reason}")
        else:
            self._say(f"Unknown command '{raw}'.")
        return None

    def _empty_capture_zone(self) -> None:
        self.inventory = ZoneInventory(capture_used=0, reserve_used=self.inventory.reserve_used)
        if self.world is not None:  # the operator physically took the pieces out of the zone
            for name in [n for n in self.world.pieces if n.startswith("capture_slot")]:
                del self.world.pieces[name]

    def _resolve_pending(self) -> Optional[dict[str, str]]:
        """Compare the camera with the position before/after a pending move (SETUP_CHECK and resync)."""
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
            chain = self._pending_chain
            if chain is not None and chain.move_uci == move.uci():
                self.inventory = self.inventory.consume(chain)
            self._pending_chain = None
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
        if raw.lower().startswith("alarm "):
            code = raw.split(None, 1)[1].upper()
            if code not in ALARM_CODES:
                self._say(f"Unknown alarm '{code}'. Known: {', '.join(sorted(ALARM_CODES))}.")
            else:
                self._raise_alarm(code, "simulated alarm typed by the operator")
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
            self._fail(Trigger.ENGINE_FAILED, f"engine error: {exc}")
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
            self._fail(Trigger.PLAN_FAILED, f"cannot plan {move.uci()}: {exc}")
            return None
        self.chains.append(chain)
        for line in chain.describe():
            self._say(line)
        self._pending_chain = chain
        fsm.handle(Trigger.PLAN_READY)
        return None

    def _robot_exec(self) -> Optional[RunResult]:
        assert self.robot is not None and self._pending_chain is not None
        try:
            self.robot.execute(self._pending_chain)
        except StubRobotError as exc:
            self._say(f"Robot stub error: {exc}")
            self._fail(Trigger.EXEC_FAILED, f"robot stub error: {exc}")
            return None
        self.fsm.handle(Trigger.EXEC_DONE)
        return None

    def _verify(self) -> Optional[RunResult]:
        game, fsm = self.game, self.fsm
        assert self.robot is not None and self.world is not None and self._pending_chain is not None
        move = game.pending_move
        assert move is not None
        self.robot.park(self._pending_chain)
        expected = game.preview_move(move)
        wrong = suspect_squares(expected, self.world.squares())
        if wrong:
            self._say(f"VERIFY failed, suspect squares: {', '.join(wrong)}")
            self._fail(Trigger.VERIFY_FAILED, f"VERIFY failed, suspect squares: {', '.join(wrong)}")
            return None
        record = game.commit_move(move, MoveSource.ENGINE)
        self.inventory = self.inventory.consume(self._pending_chain)
        self._pending_chain = None
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
            self._say(f"PGN saved in {self.game.data_dir / get_settings().paths.pgn_file}")
        return RunResult(0, State.GAME_OVER, f"{outcome.result} {outcome.termination}")


# --------------------------------------------------------------------------- #
# Command line
# --------------------------------------------------------------------------- #


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Play one console game against Stockfish with the Move Planner.")
    parser.add_argument("--color", choices=("white", "black"), default="white", help="your side")
    parser.add_argument("--mode", choices=MODE_KEYS, default=None)
    parser.add_argument("--data-dir", type=Path, default=None, help="where the saved game goes (default: paths.games_dir of settings.yaml)")
    parser.add_argument("--new", action="store_true", help="ignore a saved game and start a new one")
    parser.add_argument(
        "--demo-placeholders",
        action="store_true",
        help="allow the assumed (unconfirmed) gripper angles, reserve contents and layout of settings.yaml",
    )
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="WARNING")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    configure_logging(getattr(logging, args.log_level))
    out = sys.stdout

    try:
        settings = load_settings()
    except ConfigError as exc:
        out.write(f"Cannot start: {exc}\n")
        return 2
    workspace = Workspace(FiveBarKinematics.from_config(settings))
    geometry = BoardGeometry.from_config(settings)
    zones = Zones.from_settings(geometry, workspace, settings)
    planner = MovePlanner(
        workspace,
        geometry,
        zones,
        GraspProfiles.from_settings(settings),
        reserve_pieces=reserve_slots_from_settings(settings),
    )
    assumed = tuple(key for prefix in ("grasp", "layout") for key in settings.assumed_under(prefix))
    if planner.placeholders() and not args.demo_placeholders:
        out.write(
            "These values in config/settings.yaml are still assumed (not confirmed on the real board): "
            f"{', '.join(assumed)}.\n"
            "Ask the hardware owner, or run with --demo-placeholders to use the assumed values.\n"
        )
        return 2
    if planner.placeholders():
        out.write(
            "NOTE: placeholder values in use (assumed in settings.yaml, not confirmed): reserve slot pieces "
            f"{[chess.piece_name(p) for p in planner.reserve_pieces]}, gripper angles, capture/reserve/park layout.\n"
        )
        out.write(f"Still placeholders (a real serial port would be refused): {', '.join(planner.placeholders())}.\n")

    data_dir = args.data_dir if args.data_dir is not None else default_games_dir()
    game = None if args.new else GameState.load(data_dir)
    if game is None:
        game = GameState.new_game(data_dir)
    inventory = ZoneInventory.from_history(game.history, planner.reserve_pieces)

    engine = Engine(engine_config_from_settings(settings), mode=args.mode)
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
