"""Board geometry, workspace check and special zones (docs/SPEC.md sections 4.1, 6.2, 6.4).

Pure NumPy and math, no Qt or serial. Every dimension comes from
config/hardware.yaml through the existing loader.

The capture zone, the reserve zone (each a list of slots) and the park position are
[TO CONFIRM]. :class:`Zones` keeps each of them as ``None`` until coordinates are
provided, raises :class:`ZoneNotDefinedError` when one is requested too early, and
validates every provided slot (off the board, inside the workspace, mu >= the margin).
``Zones.proposed`` builds the owner's PROPOSED layout, which is still [TO CONFIRM].

The layout also carries an ``is_placeholder`` flag (SPEC 7.2, "Rules clarified in phase 2"). It is
fail-safe: a layout is a placeholder unless the caller says it was confirmed on the real board
(``is_placeholder=False``). ``MovePlanner.placeholders()`` reports it so the phase 3 Robot Commander
can see that the coordinates are not real yet.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any, Sequence

from desktop_app.kinematics.five_bar import (
    MIN_SINGULARITY_MARGIN_DEG,
    FiveBarKinematics,
    KinematicsError,
    OutOfReachError,
    Point,
    SingularityError,
)
from desktop_app.utils.hardware_config import HardwareConfig, load_hardware_config

logger = logging.getLogger(__name__)

_FILES = "abcdefghijklmnopqrstuvwxyz"


class ZoneNotDefinedError(KinematicsError):
    """A [TO CONFIRM] zone was requested before its coordinates were provided."""


class ZoneValidationError(KinematicsError):
    """A provided zone coordinate is on the board, outside the workspace, or too close to a singularity."""


@dataclass(frozen=True)
class BoardExtent:
    """Axis-aligned rectangle of the whole board, border included (mm)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x: float, y: float) -> bool:
        """True when the point is on the board or its border (edges included)."""
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass(frozen=True)
class BoardGeometry:
    """Square centres and extent of the board in the robot frame (SPEC 4.1).

    Files and ranks are 0-based indices here: file 0 is A (x < 0) and rank
    index 0 is rank 1 (nearest the robot).
    """

    squares_per_side: int
    square_size_mm: float
    board_border_mm: float
    first_square_center: Point
    extent: BoardExtent

    @classmethod
    def from_config(cls, config: HardwareConfig | None = None) -> "BoardGeometry":
        cfg = config if config is not None else load_hardware_config()
        geometry = cfg.geometry
        half = geometry.board_size_mm / 2.0
        near_edge = geometry.motor_to_board_edge_mm
        return cls(
            squares_per_side=geometry.squares_per_side,
            square_size_mm=geometry.square_size_mm,
            board_border_mm=geometry.board_border_mm,
            first_square_center=geometry.first_square_center_xy_mm,
            extent=BoardExtent(
                x_min=-half,
                x_max=half,
                y_min=near_edge,
                y_max=near_edge + geometry.board_size_mm,
            ),
        )

    def square_center(self, file: int, rank: int) -> Point:
        """Centre of a square by 0-based file (A = 0) and rank index (rank 1 = 0).

        x = first_x + s * file,  y = first_y + s * rank  (SPEC 4.1).
        """
        for name, value in (("file", file), ("rank", rank)):
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < self.squares_per_side:
                raise KinematicsError(f"{name} index must be an int in 0..{self.squares_per_side - 1}, got {value!r}")
        x0, y0 = self.first_square_center
        return (x0 + self.square_size_mm * file, y0 + self.square_size_mm * rank)

    def square_name(self, file: int, rank: int) -> str:
        """Algebraic name such as ``"e4"`` for 0-based indices."""
        self.square_center(file, rank)  # validates the indices
        return f"{_FILES[file]}{rank + 1}"

    def square_center_by_name(self, name: str) -> Point:
        """Centre of a square given its algebraic name (``"a1"`` .. ``"h8"``, any case)."""
        text = name.strip().lower() if isinstance(name, str) else ""
        if len(text) < 2 or text[0] not in _FILES or not text[1:].isdigit():
            raise KinematicsError(f"invalid square name {name!r}")
        return self.square_center(_FILES.index(text[0]), int(text[1:]) - 1)

    def all_square_centers(self) -> dict[str, Point]:
        """Every square centre keyed by name, in the order a1, a2, ..., a8, b1, ..., h8."""
        return {
            self.square_name(file, rank): self.square_center(file, rank)
            for file in range(self.squares_per_side)
            for rank in range(self.squares_per_side)
        }

    def is_on_board(self, x: float, y: float) -> bool:
        """True when the point lies on the board including its border."""
        return self.extent.contains(x, y)


class Workspace:
    """Reachable, safely non-singular region of the arm.

    A point is inside when IK succeeds and mu >= ``min_margin_deg``. ``check_point``
    is the single entry point used by trajectories and zone validation, so a
    subclass can wrap or count it.
    """

    def __init__(self, model: FiveBarKinematics, min_margin_deg: float = MIN_SINGULARITY_MARGIN_DEG) -> None:
        if not (math.isfinite(min_margin_deg) and 0.0 <= min_margin_deg < 90.0):
            raise KinematicsError(f"min_margin_deg must be in [0, 90), got {min_margin_deg!r}")
        self.model = model
        self.min_margin_deg = float(min_margin_deg)

    def check_point(self, x: float, y: float) -> float:
        """Return mu at the point, or raise.

        Raises:
            OutOfReachError: the point is out of reach.
            SingularityError: mu is below ``min_margin_deg``.
        """
        mu = self.model.margin_at_point(x, y)
        if mu < self.min_margin_deg:
            raise SingularityError(
                f"point ({x:g}, {y:g}) has singularity margin mu = {mu:.3f} deg, "
                f"below the required {self.min_margin_deg:g} deg"
            )
        return mu

    def contains(self, x: float, y: float) -> bool:
        try:
            self.check_point(x, y)
        except (OutOfReachError, SingularityError):
            return False
        return True


def _grid(xs: tuple[float, ...], ys: tuple[float, ...]) -> tuple[Point, ...]:
    """Slots of a rectangular grid, x-major: every y of the first x, then the next x, ..."""
    return tuple((x, y) for x in xs for y in ys)


# PROPOSED slot layout, [TO CONFIRM] on the real board. These are the owner's proposed
# values (not hardware.yaml constants); ``Zones.proposed`` is the only place that uses
# them and it logs a warning each time. The tuple order is the suggested fill order.
PROPOSED_CAPTURE_SLOTS: tuple[Point, ...] = _grid((-190.0, -225.0, -260.0), (160.0, 195.0, 230.0, 265.0, 300.0))
PROPOSED_RESERVE_SLOTS: tuple[Point, ...] = _grid((190.0, 225.0), (200.0, 235.0))
PROPOSED_PARK_POSITION: Point = (0.0, 120.0)


class Zones:
    """Capture slots, reserve slots and park position (all [TO CONFIRM], ``None`` until provided).

    * ``capture_zone``: a tuple of slots where captured pieces are dropped.
    * ``reserve_zone``: a tuple of slots where spare promotion pieces are taken from.
    * ``park_position``: one (x, y) point outside the camera view.

    Every slot and the park point are validated immediately: off the board (the board
    border counts as board), inside the workspace, and mu >= the workspace margin.
    Slots must be distinct, also across zones. Coordinates are mm in the robot frame.

    ``is_placeholder`` says whether the whole layout (capture, reserve and park together) is still
    unconfirmed on the real board. Fail-safe default: ``True``. Pass ``is_placeholder=False`` only
    for coordinates that were measured and confirmed on the real board (phase 5). The flag is
    read-only after construction and does not change validation or behaviour; it is information
    for the guard.
    """

    def __init__(
        self,
        board: BoardGeometry,
        workspace: Workspace,
        *,
        capture_zone: Sequence[Point] | None = None,
        reserve_zone: Sequence[Point] | None = None,
        park_position: Point | None = None,
        is_placeholder: bool = True,
    ) -> None:
        # Strict on purpose: a falsy non-bool (0, None, "") must never read as "confirmed".
        if not isinstance(is_placeholder, bool):
            raise ZoneValidationError(f"is_placeholder must be True or False, got {is_placeholder!r}")
        self._is_placeholder = is_placeholder
        self._board = board
        self._workspace = workspace
        capture = self._validate_slots("capture_zone", capture_zone)
        reserve = self._validate_slots("reserve_zone", reserve_zone)
        park = self._validate_point("park_position", park_position) if park_position is not None else None
        self._slots: dict[str, tuple[Point, ...] | None] = {"capture_zone": capture, "reserve_zone": reserve}
        self._park = park
        self._check_distinct()

    @classmethod
    def proposed(cls, board: BoardGeometry, workspace: Workspace) -> "Zones":
        """The proposed layout (15 capture slots, 4 reserve slots, park at (0, 120)), validated.

        These values are [TO CONFIRM] on the real board; a warning is logged on every call. The
        returned layout is always a placeholder (``is_placeholder`` is ``True``): confirming it means
        building ``Zones(..., is_placeholder=False)`` from the coordinates measured on the real board.
        """
        logger.warning(
            "using the PROPOSED capture/reserve/park layout, which is [TO CONFIRM] on the real board "
            "(%d capture slots, %d reserve slots, park %s)",
            len(PROPOSED_CAPTURE_SLOTS),
            len(PROPOSED_RESERVE_SLOTS),
            PROPOSED_PARK_POSITION,
        )
        return cls(
            board,
            workspace,
            capture_zone=PROPOSED_CAPTURE_SLOTS,
            reserve_zone=PROPOSED_RESERVE_SLOTS,
            park_position=PROPOSED_PARK_POSITION,
            is_placeholder=True,
        )

    @property
    def is_placeholder(self) -> bool:
        """True until the capture, reserve and park layout is confirmed on the real board."""
        return self._is_placeholder

    def _validate_point(self, name: str, point: Any) -> Point:
        try:
            if isinstance(point, (str, bytes)) or len(point) != 2:
                raise ValueError
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError, IndexError):
            raise ZoneValidationError(f"{name} must be an (x, y) pair of numbers, got {point!r}") from None
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ZoneValidationError(f"{name} must be finite, got ({x!r}, {y!r})")
        if self._board.is_on_board(x, y):
            raise ZoneValidationError(f"{name} ({x:g}, {y:g}) is on the board; it must be off the board")
        try:
            self._workspace.check_point(x, y)
        except (OutOfReachError, SingularityError) as exc:
            raise ZoneValidationError(f"{name} ({x:g}, {y:g}) is not usable: {exc}") from exc
        return (x, y)

    def _validate_slots(self, name: str, slots: Sequence[Point] | None) -> tuple[Point, ...] | None:
        if slots is None:
            return None
        if isinstance(slots, (str, bytes)) or not hasattr(slots, "__iter__"):
            raise ZoneValidationError(f"{name} must be a sequence of (x, y) slots, got {slots!r}")
        validated: list[Point] = []
        for index, slot in enumerate(slots):
            point = self._validate_point(f"{name} slot {index}", slot)
            if point in validated:
                raise ZoneValidationError(f"{name} slot {index} {point} repeats slot {validated.index(point)}")
            validated.append(point)
        if not validated:
            raise ZoneValidationError(f"{name} must contain at least one slot")
        return tuple(validated)

    def _check_distinct(self) -> None:
        owner: dict[Point, str] = {}
        everything: list[tuple[str, Point]] = []
        for name in ("capture_zone", "reserve_zone"):
            everything.extend((name, p) for p in (self._slots[name] or ()))
        if self._park is not None:
            everything.append(("park_position", self._park))
        for name, point in everything:
            if point in owner and owner[point] != name:
                raise ZoneValidationError(f"{name} {point} coincides with a point of {owner[point]}")
            owner.setdefault(point, name)

    def _undefined(self, name: str) -> ZoneNotDefinedError:
        return ZoneNotDefinedError(
            f"{name} is [TO CONFIRM] and has not been provided; ask the hardware owner for its coordinates in mm"
        )

    @property
    def capture_zone(self) -> tuple[Point, ...]:
        """Slots where captured pieces are dropped. Raises ZoneNotDefinedError until provided."""
        slots = self._slots["capture_zone"]
        if slots is None:
            raise self._undefined("capture_zone")
        return slots

    @property
    def reserve_zone(self) -> tuple[Point, ...]:
        """Slots where spare promotion pieces are taken from. Raises ZoneNotDefinedError until provided."""
        slots = self._slots["reserve_zone"]
        if slots is None:
            raise self._undefined("reserve_zone")
        return slots

    @property
    def park_position(self) -> Point:
        """Park position outside the camera view. Raises ZoneNotDefinedError until provided."""
        if self._park is None:
            raise self._undefined("park_position")
        return self._park

    def is_defined(self, name: str) -> bool:
        """True when ``name`` ("capture_zone", "reserve_zone" or "park_position") was provided."""
        if name == "park_position":
            return self._park is not None
        if name in self._slots:
            return self._slots[name] is not None
        raise KeyError(name)

    @property
    def undefined(self) -> tuple[str, ...]:
        """Names of the zones that are still [TO CONFIRM]."""
        return tuple(
            name for name in ("capture_zone", "reserve_zone", "park_position") if not self.is_defined(name)
        )
