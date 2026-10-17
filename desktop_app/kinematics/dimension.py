"""Board geometry, workspace check and special zones (docs/SPEC.md sections 4.1, 6.2, 6.4).

Pure NumPy and math, no Qt or serial. Every dimension comes from
config/hardware.yaml through the existing loader.

The capture zone, the reserve zone and the park position are [TO CONFIRM] in
the SPEC. They are NOT defined here: :class:`Zones` keeps each of them as
``None`` until the owner provides coordinates, raises
:class:`ZoneNotDefinedError` when one is requested too early, and validates a
provided coordinate (off the board, inside the workspace, mu >= the margin).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from desktop_app.kinematics.five_bar import (
    MIN_SINGULARITY_MARGIN_DEG,
    FiveBarKinematics,
    KinematicsError,
    OutOfReachError,
    Point,
    SingularityError,
)
from desktop_app.utils.hardware_config import HardwareConfig, load_hardware_config

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


class Zones:
    """Capture zone, reserve zone and park position (all [TO CONFIRM], ``None`` until provided).

    Each is a single (x, y) point in mm in the robot frame. A provided point is
    validated immediately: it must be off the board (the board border counts as
    board), inside the workspace and have mu >= the workspace margin.
    """

    def __init__(
        self,
        board: BoardGeometry,
        workspace: Workspace,
        *,
        capture_zone: Point | None = None,
        reserve_zone: Point | None = None,
        park_position: Point | None = None,
    ) -> None:
        self._board = board
        self._workspace = workspace
        self._points: dict[str, Point | None] = {
            "capture_zone": self._validate("capture_zone", capture_zone),
            "reserve_zone": self._validate("reserve_zone", reserve_zone),
            "park_position": self._validate("park_position", park_position),
        }

    def _validate(self, name: str, point: Point | None) -> Point | None:
        if point is None:
            return None
        try:
            x, y = (float(point[0]), float(point[1]))
            if len(point) != 2:
                raise ValueError
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

    def _get(self, name: str) -> Point:
        point = self._points[name]
        if point is None:
            raise ZoneNotDefinedError(
                f"{name} is [TO CONFIRM] and has not been provided; ask the hardware owner for its (x, y) in mm"
            )
        return point

    @property
    def capture_zone(self) -> Point:
        """Where captured pieces are dropped. Raises ZoneNotDefinedError until provided."""
        return self._get("capture_zone")

    @property
    def reserve_zone(self) -> Point:
        """Where spare promotion pieces are taken from. Raises ZoneNotDefinedError until provided."""
        return self._get("reserve_zone")

    @property
    def park_position(self) -> Point:
        """Park position outside the camera view. Raises ZoneNotDefinedError until provided."""
        return self._get("park_position")

    def is_defined(self, name: str) -> bool:
        """True when ``name`` ("capture_zone", "reserve_zone" or "park_position") was provided."""
        if name not in self._points:
            raise KeyError(name)
        return self._points[name] is not None

    @property
    def undefined(self) -> tuple[str, ...]:
        """Names of the zones that are still [TO CONFIRM]."""
        return tuple(name for name, point in self._points.items() if point is None)
