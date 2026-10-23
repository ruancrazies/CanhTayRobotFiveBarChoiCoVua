"""Shared rig for the phase 3 tests: the whole stack, FSM to Move Planner to Commander to simulator.

Every timing and speed below is a TEST VALUE: the real ones are [TO CONFIRM] and measured on the
hardware in phases 4 and 5. Nothing here needs a serial port.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

import chess

from desktop_app.console_runner import PhysicalWorld, suspect_squares
from desktop_app.core.event_bus import EventBus
from desktop_app.core.fsm import FsmRequest, State, StateChanged, Trigger
from desktop_app.core.game_state import MoveSource
from desktop_app.core.move_planner import MoveChain, MovePlanner, ZoneInventory
from desktop_app.hardware.esp32_simulator import Esp32Simulator, TraceEvent, make_loopback_simulator
from desktop_app.hardware.robot_commander import CommanderError, CommanderSettings, RobotCommander
from desktop_app.hardware.serial_link import LinkSettings, LoopbackPair, LoopbackTransport, SerialLink, Transport
from desktop_app.kinematics.dimension import PROPOSED_PARK_POSITION
from desktop_app.kinematics.five_bar import FiveBarKinematics
from desktop_app.utils.hardware_config import load_hardware_config
from phase2_support import Rig, make_geometry, make_planner, make_workspace, make_zones
from sim_support import fast_sim_config

# (FEN before the human move, human move, engine move). The human is white, the engine (the robot) black.
SCENARIOS: dict[str, tuple[str, str, str]] = {
    "normal": (chess.STARTING_FEN, "e2e4", "e7e5"),
    "capture": ("rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2", "g1f3", "d5e4"),
    "castling": ("rnbqk2r/pppp1ppp/5n2/2b1p3/2B1P3/5N2/PPPP1PPP/RNBQK2R w KQkq - 4 4", "d2d3", "e8g8"),
    "promotion": ("4k3/8/8/8/8/8/p5K1/8 w - - 0 1", "g2g3", "a2a1q"),
    "en_passant": ("4k3/8/8/8/3p4/8/4P2K/8 w - - 0 1", "e2e4", "d4e3"),
    "capture_promotion": ("1n2k3/8/8/8/8/8/p5K1/1R6 w - - 0 1", "g2g3", "a2b1q"),
}

# How closely the arm must land on a square/slot centre in the replay (mm). The planner passes the exact IK
# solution and the wire keeps 3 decimals of a degree, which is a few thousandths of a millimetre at this reach.
REPLAY_TOLERANCE_MM = 0.05


def park_joints() -> tuple[float, float]:
    alpha, beta = FiveBarKinematics.from_config().ik(*PROPOSED_PARK_POSITION)
    return float(alpha), float(beta)


def commander_settings(**overrides: Any) -> CommanderSettings:
    hardware = load_hardware_config()
    values: dict[str, Any] = dict(
        move_speed_deg_s=2000.0,
        ping_interval_s=0.02,
        reply_timeout_s=2.0,
        done_timeout_s=2.0,
        grip_settle_s=0.0,
        start_tolerance_deg=hardware.electrical.as5600.system_error_deg,
        send_timeout_s=2.0,
        queue_retry_s=0.002,
        queue_full_timeout_s=3.0,
    )
    values.update(overrides)
    return CommanderSettings(**values)


class RealLookingTransport(Transport):
    """Wraps a loopback end but does NOT say it is simulated, so the Commander treats it as a real port."""

    def __init__(self, inner: LoopbackTransport) -> None:
        self._inner = inner
        self.written: list[bytes] = []

    def open(self) -> None:
        self._inner.open()

    def close(self) -> None:
        self._inner.close()

    @property
    def is_open(self) -> bool:
        return self._inner.is_open

    def write(self, data: bytes) -> None:
        self.written.append(data)
        self._inner.write(data)

    def read(self, timeout_s: float) -> bytes:
        return self._inner.read(timeout_s)


# --------------------------------------------------------------------------- #
# Replay of what the arm physically did
# --------------------------------------------------------------------------- #


class ArmReplay:
    """Turns the simulator's trace into pick and place events on a PhysicalWorld.

    A ``z_bottom`` event says where the gripper is (forward kinematics of the joint angles, matched to the
    nearest named location); the ``grip`` event that follows closes on a piece (angle differs from the open
    angle) or releases it (equals the open angle).
    """

    def __init__(self, planner: MovePlanner, world: PhysicalWorld, open_deg: float) -> None:
        self._model = FiveBarKinematics.from_config()
        self._world = world
        self._open_deg = open_deg
        geometry, zones = make_geometry(), make_zones()
        self.locations: dict[str, tuple[float, float]] = dict(geometry.all_square_centers())
        for index, point in enumerate(zones.capture_zone):
            self.locations[f"capture_slot[{index}]"] = point
        for index, point in enumerate(zones.reserve_zone):
            self.locations[f"reserve_slot[{index}]"] = point
        self.locations["park"] = zones.park_position
        self.holding: Optional[chess.Piece] = None
        self.errors: list[str] = []
        self.picks: list[str] = []
        self.places: list[str] = []
        self.worst_error_mm = 0.0

    def locate(self, joints: tuple[float, float]) -> tuple[str, float]:
        x, y = self._model.fk(*joints)
        name, distance = min(
            ((n, math.hypot(x - cx, y - cy)) for n, (cx, cy) in self.locations.items()), key=lambda item: item[1]
        )
        return name, distance

    def replay(self, trace: list[TraceEvent]) -> None:
        at: Optional[str] = None
        for event in trace:
            if event.kind == "z_bottom":
                at, distance = self.locate(event.joints_deg)
                self.worst_error_mm = max(self.worst_error_mm, distance)
                if distance > REPLAY_TOLERANCE_MM:
                    self.errors.append(f"Z went down {distance:.3f} mm away from {at}")
            elif event.kind == "grip":
                if at is None:
                    self.errors.append("GRIP with Z never lowered")
                elif event.grip_deg != self._open_deg:
                    piece = self._world.pieces.pop(at, None)
                    if piece is None or self.holding is not None:
                        self.errors.append(f"close at {at}: piece={piece} holding={self.holding}")
                        continue
                    self.holding = piece
                    self.picks.append(at)
                else:
                    if self.holding is None or at in self._world.pieces:
                        self.errors.append(f"open at {at}: holding={self.holding}, occupied={at in self._world.pieces}")
                        continue
                    self._world.pieces[at] = self.holding
                    self.holding = None
                    self.places.append(at)


# --------------------------------------------------------------------------- #
# The stack
# --------------------------------------------------------------------------- #


@dataclass
class Stack:
    """FSM + GameState + Move Planner + Commander + link + simulator, all on one Event Bus."""

    rig: Rig
    bus: EventBus
    planner: MovePlanner
    sim: Esp32Simulator
    pair: LoopbackPair
    link: SerialLink
    commander: RobotCommander
    world: PhysicalWorld
    inventory: ZoneInventory = field(default_factory=ZoneInventory)
    chain: Optional[MoveChain] = None
    states: list[State] = field(default_factory=list)
    alarm_requests: list[str] = field(default_factory=list)
    errors: list[BaseException] = field(default_factory=list)

    @property
    def fsm(self) -> Any:
        return self.rig.fsm

    @property
    def game(self) -> Any:
        return self.rig.game

    # -- startup and the engine move, state by state -------------------------------------------

    def homing(self) -> None:
        """INIT, CALIBRATE (stubbed), HOMING through the Commander, SETUP_CHECK."""
        for trigger in (Trigger.YOLO_ON, Trigger.INIT_DONE, Trigger.CALIBRATE_DONE):
            assert self.fsm.handle(trigger).accepted
        assert self.fsm.state is State.HOMING
        self.commander.home()
        assert self.fsm.handle(Trigger.HOMING_DONE).accepted

    def reach_plan(self, human_move: str, engine_move: str) -> None:
        """START, the human move, the engine's pending move, up to PLAN."""
        fsm, game = self.fsm, self.game
        assert fsm.handle(Trigger.START, human_color=self.rig.human).accepted
        human = chess.Move.from_uci(human_move)
        assert fsm.handle(Trigger.HUMAN_MOVE, move=human).accepted
        game.commit_move(human, MoveSource.HUMAN)
        self.world.sync_squares(game.board)
        assert fsm.handle(Trigger.COMMIT_DONE).accepted
        game.set_pending_move(chess.Move.from_uci(engine_move))
        assert fsm.handle(Trigger.ENGINE_MOVE_READY).accepted
        assert fsm.state is State.PLAN

    def redo_pending(self) -> None:
        """After recovery through HOMING: SETUP_CHECK, then START with pending='redo' goes back to PLAN."""
        assert self.fsm.state is State.SETUP_CHECK
        assert self.fsm.handle(Trigger.START, human_color=self.rig.human, pending="redo").accepted
        assert self.fsm.state is State.PLAN

    def plan(self) -> MoveChain:
        move = self.game.pending_move
        self.chain = self.planner.plan(self.game.board, move, self.inventory)
        assert self.fsm.handle(Trigger.PLAN_READY).accepted
        assert self.fsm.state is State.ROBOT_EXEC
        return self.chain

    def execute(self) -> Optional[CommanderError]:
        """ROBOT_EXEC: run the chain. A failure that is not an alarm is reported as EXEC_FAILED, as the runner does."""
        assert self.chain is not None
        try:
            self.commander.execute_chain(self.chain)
        except CommanderError as exc:
            if self.fsm.state is State.ROBOT_EXEC:  # an alarm has already moved the FSM to ERROR_RECOVER
                self.fsm.handle(Trigger.EXEC_FAILED)
            return exc
        assert self.fsm.handle(Trigger.EXEC_DONE).accepted
        return None

    def verify(self) -> list[str]:
        """VERIFY: park, replay what the arm did, compare with the expected position, commit. Returns suspect squares."""
        assert self.chain is not None
        self.commander.park(self.chain)
        self.replay = ArmReplay(self.planner, self.world, self.planner.grasp.open_deg)
        self.replay.replay(self.sim.trace)
        move = self.game.pending_move
        wrong = suspect_squares(self.game.preview_move(move), self.world.squares())
        if not wrong and not self.replay.errors:
            self.game.commit_move(move, MoveSource.ENGINE)
            self.inventory = self.inventory.consume(self.chain)
            assert self.fsm.handle(Trigger.VERIFY_OK).accepted
        return wrong

    replay: ArmReplay = field(init=False)

    def sent_commands(self) -> list[str]:
        """Every command line the simulator received, except the background PING heartbeat."""
        return [line for line in self.sim.received if line.split()[0] != "PING"]

    def run_full_move(self, scenario: str) -> None:
        _fen, human, engine = SCENARIOS[scenario]
        self.homing()
        self.reach_plan(human, engine)
        self.plan()
        failure = self.execute()
        assert failure is None, failure
        assert self.verify() == []

    def close(self) -> None:
        self.commander.stop()
        self.link.stop()
        self.sim.stop()


def build_stack(
    tmp_path: Path,
    scenario: str = "normal",
    *,
    planner: Optional[MovePlanner] = None,
    sim_overrides: Optional[dict[str, Any]] = None,
    settings_overrides: Optional[dict[str, Any]] = None,
    link_settings: Optional[LinkSettings] = None,
    start_commander: bool = True,
    transport_wrapper: Optional[Callable[[LoopbackTransport], Transport]] = None,
) -> Stack:
    fen, _human, _engine = SCENARIOS[scenario]
    bus = EventBus()
    rig = Rig(tmp_path, fen=fen, human=chess.WHITE, bus=bus)
    planner = planner or make_planner()

    sim_values: dict[str, Any] = dict(initial_joint_deg=park_joints())
    sim_values.update(sim_overrides or {})
    sim, pair = make_loopback_simulator(fast_sim_config(**sim_values))
    sim.start()

    transport: Transport = transport_wrapper(pair.pc) if transport_wrapper else pair.pc
    link = SerialLink(transport, link_settings or LinkSettings(max_in_flight=3, read_poll_s=0.005))
    link.start()
    commander = RobotCommander(link, planner, commander_settings(**(settings_overrides or {})), bus)

    reserve = {
        f"reserve_slot[{i}]": chess.Piece(piece_type, chess.BLACK) for i, piece_type in enumerate(planner.reserve_pieces)
    }
    stack = Stack(rig, bus, planner, sim, pair, link, commander, PhysicalWorld(rig.game.board, reserve))
    bus.subscribe(FsmRequest, lambda request: stack.alarm_requests.append(str(request.payload.get("alarm"))))
    bus.subscribe(StateChanged, lambda change: stack.states.append(change.current))
    if start_commander:
        commander.start()
    return stack
