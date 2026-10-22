"""Move Planner: pick-and-place command chains as abstract data (docs/SPEC.md sections 6.3 and 6.4).

The planner turns one legal engine move into a :class:`MoveChain`: an ordered list of
:class:`Transfer` objects (one pick-and-place each) made of abstract steps (``MoveStep``,
``ZDownStep``, ``GripStep``, ``ZUpStep``). It produces no serial text; the Robot Commander
(phase 3) will translate the steps into protocol commands.

Every pick/drop point and every point of every straight-line leg is checked through
``Workspace.check_point`` (reach and mu >= margin) BEFORE the chain is returned. The planner
never calls raw ``ik``. A bad point, an exhausted zone or a missing [TO CONFIRM] value raises
:class:`MovePlanError`, and no chain exists. The planner never changes GameState or the
inventory: it is a pure function of (board, move, inventory).

Sequences (SPEC 6.4, Z is at TOP during every MOVE)::

    normal      main piece: source -> target
    capture     captured piece: target -> next free capture slot, then the main piece
    en passant  captured pawn: its own square -> capture slot, then the main pawn
    castling    king first, then the rook
    promotion   [captured piece -> capture slot], pawn: source -> capture slot,
                spare piece of the promotion type: reserve slot -> promotion square

Assumptions the SPEC does not spell out (flagged in the phase 2 report):
  * the pawn that is promoted goes to the next free capture slot ("off the board");
  * the reserve zone holds spare pieces of the robot's own colour, one per slot, and the piece
    type of each slot is given to the planner (``reserve_pieces``);
  * grasp angles, reserve slot contents and the capture/reserve/park layout stay PLACEHOLDERS until
    phase 5. They are fail-safe: a value is real only when it says so (``GraspProfiles.from_file``,
    ``ReserveSlot(is_placeholder=False)`` or ``Zones(is_placeholder=False)``).
    ``MovePlanner.placeholders()`` lists what is still a placeholder, so the phase 3 Robot
    Commander can see it before it drives a real serial port;
  * the arm starts every chain at the park position and ``MoveChain.park_leg`` is the validated
    return trip from the last drop point to the park position (VERIFY sends it).
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Mapping, Optional, Sequence, Union

import chess

from desktop_app.core.game_state import MoveRecord, MoveSource
from desktop_app.kinematics.dimension import BoardGeometry, Workspace, Zones
from desktop_app.kinematics.five_bar import KinematicsError, Point
from desktop_app.kinematics.trajectory import DEFAULT_MAX_SEGMENT_MM, Trajectory, straight_line

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GRASP_PROFILES_PATH = REPO_ROOT / "config" / "grasp_profiles.json"


# --------------------------------------------------------------------------- #
# Errors
# --------------------------------------------------------------------------- #


class MovePlanError(ValueError):
    """The move cannot be planned. No chain is returned when this is raised."""


class CaptureZoneFullError(MovePlanError):
    """There are not enough free capture slots for the pieces this move removes."""


class ReservePieceUnavailableError(MovePlanError):
    """No unused spare piece of the promotion type is left in the reserve zone."""


class GraspProfilesError(MovePlanError):
    """config/grasp_profiles.json is missing or invalid ([TO CONFIRM] values)."""


# --------------------------------------------------------------------------- #
# Grasp profiles (config/grasp_profiles.json)
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class GraspProfiles:
    """Gripper angles: one closing angle per piece type and one opening angle.

    File format (all angles are servo degrees)::

        {"open_deg": <number>, "close_deg": {"P": <n>, "N": <n>, "B": <n>, "R": <n>, "Q": <n>, "K": <n>}}

    The values are hardware-dependent and [TO CONFIRM]: this module never invents them. The
    servo range itself is enforced by the firmware (``GRIP`` is rejected out of range).
    """

    open_deg: float
    close_deg: Mapping[chess.PieceType, float]
    # Fail-safe: a profile is real only when it was loaded from config/grasp_profiles.json
    # (``from_file``). Anything else (demo values, test values, a hand-built object) is a
    # placeholder until phase 5, and the phase 3 Robot Commander must not drive a real port with it.
    is_placeholder: bool = True

    @classmethod
    def from_dict(cls, data: object, *, placeholder: bool = True) -> "GraspProfiles":
        if not isinstance(data, dict) or set(data) != {"open_deg", "close_deg"}:
            raise GraspProfilesError('grasp profiles must be an object with exactly "open_deg" and "close_deg"')

        def angle(name: str, value: object) -> float:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise GraspProfilesError(f"{name} must be a finite number >= 0, got {value!r}")
            return float(value)

        raw = data["close_deg"]
        symbols = {chess.piece_symbol(t).upper(): t for t in chess.PIECE_TYPES}
        if not isinstance(raw, dict) or set(raw) != set(symbols):
            raise GraspProfilesError(f"close_deg must have exactly the keys {sorted(symbols)}")
        return cls(
            open_deg=angle("open_deg", data["open_deg"]),
            close_deg={symbols[key]: angle(f"close_deg.{key}", value) for key, value in raw.items()},
            is_placeholder=placeholder,
        )

    @classmethod
    def from_file(cls, path: Optional[Path] = None) -> "GraspProfiles":
        source = Path(path) if path is not None else DEFAULT_GRASP_PROFILES_PATH
        if not source.exists():
            raise GraspProfilesError(
                f"{source} does not exist. The gripper angles are hardware values and [TO CONFIRM]; "
                "ask the hardware owner and create the file"
            )
        try:
            return cls.from_dict(json.loads(source.read_text(encoding="utf-8")), placeholder=False)
        except (OSError, ValueError) as exc:
            if isinstance(exc, GraspProfilesError):
                raise
            raise GraspProfilesError(f"cannot read {source}: {exc}") from exc

    def close_angle(self, piece_type: chess.PieceType) -> float:
        return self.close_deg[piece_type]


# --------------------------------------------------------------------------- #
# Reserve slot contents
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class ReserveSlot:
    """The spare piece in one reserve slot, and whether that content is still a placeholder.

    Fail-safe default: the content is a placeholder unless the caller says it was confirmed
    (``is_placeholder=False``, once the real content comes from config in phase 5).
    """

    piece_type: chess.PieceType
    is_placeholder: bool = True


# --------------------------------------------------------------------------- #
# Abstract chain data
# --------------------------------------------------------------------------- #


class LocationKind(str, Enum):
    SQUARE = "square"
    CAPTURE_SLOT = "capture_slot"
    RESERVE_SLOT = "reserve_slot"
    PARK = "park"


@dataclass(frozen=True)
class Location:
    kind: LocationKind
    name: str  # "e4", "capture_slot[3]", "reserve_slot[0]", "park"
    xy: Point  # mm, robot frame

    def __str__(self) -> str:
        return self.name


class GripAction(str, Enum):
    CLOSE = "close"
    OPEN = "open"


@dataclass(frozen=True, eq=False)
class MoveStep:
    """Move the gripper (Z at TOP) in a straight line to ``target``; every point is validated."""

    target: Location
    trajectory: Trajectory


@dataclass(frozen=True)
class ZDownStep:
    pass


@dataclass(frozen=True)
class ZUpStep:
    pass


@dataclass(frozen=True)
class GripStep:
    action: GripAction
    angle_deg: float
    piece_type: chess.PieceType


Step = Union[MoveStep, ZDownStep, ZUpStep, GripStep]


class TransferRole(str, Enum):
    MOVING_PIECE = "moving_piece"
    CAPTURED_PIECE = "captured_piece"
    EN_PASSANT_PAWN = "en_passant_pawn"
    CASTLING_KING = "castling_king"
    CASTLING_ROOK = "castling_rook"
    PROMOTED_PAWN = "promoted_pawn"
    PROMOTION_PIECE = "promotion_piece"


@dataclass(frozen=True)
class Transfer:
    """One pick-and-place: MOVE source, ZDOWN, GRIP close, ZUP, MOVE target, ZDOWN, GRIP open, ZUP."""

    role: TransferRole
    piece_type: chess.PieceType
    source: Location
    target: Location
    steps: tuple[Step, ...]


@dataclass(frozen=True)
class MoveChain:
    """The validated command chain of one engine move."""

    move_uci: str
    san: str
    transfers: tuple[Transfer, ...]
    park_leg: MoveStep
    capture_slots_used: tuple[int, ...]
    reserve_slot_used: Optional[int]
    steps: tuple[Step, ...] = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "steps", tuple(step for transfer in self.transfers for step in transfer.steps))

    def describe(self) -> list[str]:
        """Human-readable lines for the console runner and the log."""
        lines = [f"chain for {self.san} ({self.move_uci}): {len(self.transfers)} pick-and-place(s)"]
        for number, transfer in enumerate(self.transfers, start=1):
            piece = chess.piece_name(transfer.piece_type)
            lines.append(f"  {number}. {transfer.role.value}: {piece} {transfer.source} -> {transfer.target}")
            for step in transfer.steps:
                lines.append("       " + _describe_step(step))
        lines.append("  after the move (sent by VERIFY): " + _describe_step(self.park_leg))
        return lines


def _describe_step(step: Step) -> str:
    if isinstance(step, MoveStep):
        x, y = step.target.xy
        return (
            f"MOVE  {step.target} ({x:.1f}, {y:.1f}) mm, {len(step.trajectory)} points, "
            f"min mu {step.trajectory.min_margin_deg:.1f} deg"
        )
    if isinstance(step, ZDownStep):
        return "ZDOWN"
    if isinstance(step, ZUpStep):
        return "ZUP"
    return f"GRIP  {step.action.value} {chess.piece_name(step.piece_type)} at {step.angle_deg:g} deg"


# --------------------------------------------------------------------------- #
# Zone inventory (which capture / reserve slots are used)
# --------------------------------------------------------------------------- #


def choose_reserve_slot(
    reserve_pieces: Sequence[chess.PieceType], used: frozenset[int], piece_type: chess.PieceType
) -> int:
    """First unused reserve slot that holds a spare piece of ``piece_type``."""
    for index, held in enumerate(reserve_pieces):
        if held == piece_type and index not in used:
            return index
    raise ReservePieceUnavailableError(
        f"no unused spare {chess.piece_name(piece_type)} left in the reserve zone "
        f"({len(used)} of {len(reserve_pieces)} slots used)"
    )


@dataclass(frozen=True)
class ZoneInventory:
    """How many capture slots are filled and which reserve slots are empty.

    Immutable: planning never changes it. The caller applies a chain with :meth:`consume` once
    the move is committed (so a cancelled pending move never leaks a slot), or rebuilds it after
    a restart with :meth:`from_history`.
    """

    capture_used: int = 0
    reserve_used: frozenset[int] = frozenset()

    def consume(self, chain: MoveChain) -> "ZoneInventory":
        return ZoneInventory(
            capture_used=self.capture_used + len(chain.capture_slots_used),
            reserve_used=self.reserve_used | ({chain.reserve_slot_used} if chain.reserve_slot_used is not None else set()),
        )

    @classmethod
    def from_history(cls, history: Sequence[MoveRecord], reserve_pieces: Sequence[chess.PieceType]) -> "ZoneInventory":
        """Replay the ENGINE moves of a game: they are the only ones the robot physically made.

        (Pieces captured by the human are handled by the human and never reach the capture zone.)
        """
        capture_used = 0
        reserve_used: frozenset[int] = frozenset()
        for record in history:
            if record.source is not MoveSource.ENGINE:
                continue
            capture_used += (1 if record.captured is not None else 0) + (1 if record.promotion is not None else 0)
            if record.promotion is not None:
                reserve_used = reserve_used | {choose_reserve_slot(reserve_pieces, reserve_used, record.promotion)}
        return cls(capture_used=capture_used, reserve_used=reserve_used)


# --------------------------------------------------------------------------- #
# Planner
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class _Spec:
    role: TransferRole
    piece_type: chess.PieceType
    source: Location
    target: Location


class MovePlanner:
    """Builds validated pick-and-place chains.

    Args:
        workspace: reach and singularity check (``Workspace.check_point``).
        geometry: board geometry (square centres).
        zones: capture slots, reserve slots and park position ([TO CONFIRM] until provided).
        grasp: gripper angles ([TO CONFIRM] until provided).
        reserve_pieces: the spare piece in each reserve slot, in the order of ``zones.reserve_zone``.
            Required when the reserve zone is defined. Each item is a :class:`ReserveSlot`, or a
            bare piece type, which counts as a PLACEHOLDER slot.
        ds: maximum straight-line segment length in mm.
    """

    def __init__(
        self,
        workspace: Workspace,
        geometry: BoardGeometry,
        zones: Zones,
        grasp: GraspProfiles,
        *,
        reserve_pieces: Optional[Sequence[Union[chess.PieceType, ReserveSlot]]] = None,
        ds: float = DEFAULT_MAX_SEGMENT_MM,
    ) -> None:
        self._workspace = workspace
        self._geometry = geometry
        self._zones = zones
        self._grasp = grasp
        self._ds = ds
        self._reserve_slots: tuple[ReserveSlot, ...] = tuple(
            item if isinstance(item, ReserveSlot) else ReserveSlot(item) for item in (reserve_pieces or ())
        )
        self._reserve_pieces: tuple[chess.PieceType, ...] = tuple(slot.piece_type for slot in self._reserve_slots)
        if zones.is_defined("reserve_zone"):
            slots = len(zones.reserve_zone)
            if len(self._reserve_pieces) != slots:
                raise MovePlanError(
                    f"reserve_pieces must name the piece of each of the {slots} reserve slots, "
                    f"got {len(self._reserve_pieces)}"
                )
        for piece_type in self._reserve_pieces:
            if piece_type not in (chess.KNIGHT, chess.BISHOP, chess.ROOK, chess.QUEEN):
                raise MovePlanError(f"a reserve slot cannot hold {chess.piece_name(piece_type)}: only promotion pieces")

    @property
    def reserve_pieces(self) -> tuple[chess.PieceType, ...]:
        return self._reserve_pieces

    @property
    def reserve_slots(self) -> tuple[ReserveSlot, ...]:
        return self._reserve_slots

    @property
    def grasp(self) -> GraspProfiles:
        return self._grasp

    def placeholders(self) -> tuple[str, ...]:
        """Everything that is still a placeholder, in this order: ``"grasp_profiles"``, ``"zones"``
        (the capture/reserve/park layout) and ``"reserve_slot[i]"`` (one per unconfirmed slot content).

        Empty only when every grasp angle, the zone layout and every reserve slot content is
        confirmed. The phase 3 Robot Commander uses this to guard a real serial port (SPEC 7.2).
        """
        found: list[str] = ["grasp_profiles"] if self._grasp.is_placeholder else []
        if self._zones.is_placeholder:
            found.append("zones")
        found += [f"reserve_slot[{i}]" for i, slot in enumerate(self._reserve_slots) if slot.is_placeholder]
        return tuple(found)

    @property
    def has_placeholders(self) -> bool:
        return bool(self.placeholders())

    def plan(self, board: chess.Board, move: chess.Move, inventory: Optional[ZoneInventory] = None) -> MoveChain:
        """Plan ``move`` (played from ``board``). ``board`` and ``inventory`` are not modified.

        Raises:
            MovePlanError: the move is not legal, a zone is undefined or full, or a point is
                outside the workspace / below the singularity margin. Nothing is returned.
        """
        inventory = inventory if inventory is not None else ZoneInventory()
        if not isinstance(move, chess.Move) or move not in board.legal_moves:
            raise MovePlanError(f"{move!r} is not a legal move in {board.fen()}")
        try:
            return self._plan(board, move, inventory)
        except MovePlanError:
            raise
        except KinematicsError as exc:  # TrajectoryError, ZoneNotDefinedError, ...
            raise MovePlanError(f"cannot plan {move.uci()}: {exc}") from exc

    # ------------------------------------------------------------------ #

    def _plan(self, board: chess.Board, move: chess.Move, inventory: ZoneInventory) -> MoveChain:
        specs, capture_slots, reserve_slot = self._specs(board, move, inventory)
        park = Location(LocationKind.PARK, "park", self._zones.park_position)

        # 1. Every pick/drop point must be inside the workspace with mu >= margin.
        for spec in specs:
            for location in (spec.source, spec.target):
                self._check(location)
        self._check(park)

        # 2. Every point of every straight-line leg (this raises before anything is returned).
        position = park
        transfers: list[Transfer] = []
        for spec in specs:
            steps: list[Step] = [
                self._leg(position, spec.source),
                ZDownStep(),
                GripStep(GripAction.CLOSE, self._grasp.close_angle(spec.piece_type), spec.piece_type),
                ZUpStep(),
                self._leg(spec.source, spec.target),
                ZDownStep(),
                GripStep(GripAction.OPEN, self._grasp.open_deg, spec.piece_type),
                ZUpStep(),
            ]
            transfers.append(Transfer(spec.role, spec.piece_type, spec.source, spec.target, tuple(steps)))
            position = spec.target
        park_leg = self._leg(position, park)

        return MoveChain(
            move_uci=move.uci(),
            san=board.san(move),
            transfers=tuple(transfers),
            park_leg=park_leg,
            capture_slots_used=tuple(capture_slots),
            reserve_slot_used=reserve_slot,
        )

    def _check(self, location: Location) -> None:
        try:
            self._workspace.check_point(*location.xy)
        except KinematicsError as exc:
            raise MovePlanError(f"{location} at ({location.xy[0]:g}, {location.xy[1]:g}) is not usable: {exc}") from exc

    def _leg(self, start: Location, end: Location) -> MoveStep:
        try:
            trajectory = straight_line(self._workspace, start.xy, end.xy, self._ds)
        except KinematicsError as exc:
            raise MovePlanError(f"the path {start} -> {end} is not usable: {exc}") from exc
        return MoveStep(target=end, trajectory=trajectory)

    def _square(self, square: chess.Square) -> Location:
        xy = self._geometry.square_center(chess.square_file(square), chess.square_rank(square))
        return Location(LocationKind.SQUARE, chess.square_name(square), xy)

    def _capture_slot(self, index: int) -> Location:
        return Location(LocationKind.CAPTURE_SLOT, f"capture_slot[{index}]", self._zones.capture_zone[index])

    def _specs(
        self, board: chess.Board, move: chess.Move, inventory: ZoneInventory
    ) -> tuple[list[_Spec], list[int], Optional[int]]:
        mover = board.piece_at(move.from_square)
        assert mover is not None  # legal move
        source, target = self._square(move.from_square), self._square(move.to_square)

        # What leaves the board besides the mover?
        removals: list[tuple[TransferRole, chess.PieceType, Location]] = []
        if board.is_en_passant(move):
            victim_square = chess.square(chess.square_file(move.to_square), chess.square_rank(move.from_square))
            removals.append((TransferRole.EN_PASSANT_PAWN, chess.PAWN, self._square(victim_square)))
        else:
            victim = board.piece_at(move.to_square)
            if victim is not None:
                removals.append((TransferRole.CAPTURED_PIECE, victim.piece_type, target))

        specs: list[_Spec] = []
        capture_slots: list[int] = []
        reserve_slot: Optional[int] = None

        def next_slot() -> int:
            index = inventory.capture_used + len(capture_slots)
            available = len(self._zones.capture_zone)
            if index >= available:
                raise CaptureZoneFullError(
                    f"the capture zone is full ({inventory.capture_used} of {available} slots used); "
                    "empty it before this move"
                )
            capture_slots.append(index)
            return index

        for role, piece_type, location in removals:
            specs.append(_Spec(role, piece_type, location, self._capture_slot(next_slot())))

        if board.is_castling(move):
            rank = chess.square_rank(move.from_square)
            kingside = board.is_kingside_castling(move)
            rook_from = chess.square(chess.square_file(chess.H1 if kingside else chess.A1), rank)
            rook_to = chess.square(chess.square_file(chess.F1 if kingside else chess.D1), rank)
            specs.append(_Spec(TransferRole.CASTLING_KING, chess.KING, source, target))  # king first
            specs.append(_Spec(TransferRole.CASTLING_ROOK, chess.ROOK, self._square(rook_from), self._square(rook_to)))
        elif move.promotion is not None:
            specs.append(_Spec(TransferRole.PROMOTED_PAWN, chess.PAWN, source, self._capture_slot(next_slot())))
            reserve = self._zones.reserve_zone  # [TO CONFIRM] until provided
            reserve_slot = choose_reserve_slot(self._reserve_pieces, inventory.reserve_used, move.promotion)
            specs.append(
                _Spec(
                    TransferRole.PROMOTION_PIECE,
                    move.promotion,
                    Location(LocationKind.RESERVE_SLOT, f"reserve_slot[{reserve_slot}]", reserve[reserve_slot]),
                    target,
                )
            )
        else:
            specs.append(_Spec(TransferRole.MOVING_PIECE, mover.piece_type, source, target))
        return specs, capture_slots, reserve_slot
