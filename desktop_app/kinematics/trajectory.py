"""Straight-line gripper trajectory (docs/SPEC.md section 6.3).

The path from source to target is split into segments no longer than ``ds``
(default: motion.trajectory_step_mm of config/settings.yaml). Every point, the two end points included, is checked against
the workspace (reach and mu >= margin) BEFORE anything is returned: a bad point
raises :class:`TrajectoryError` carrying its index, and no partial result ever
leaves this module. Pure NumPy and math.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from desktop_app.kinematics.dimension import Workspace
from desktop_app.kinematics.five_bar import KinematicsError, Point
from desktop_app.utils.settings import get_settings


class TrajectoryError(KinematicsError):
    """A trajectory point is outside the workspace or below the singularity margin.

    Attributes:
        index: index of the first bad point along the path (0 = start).
        point: (x, y) of that point in mm.
    """

    def __init__(self, index: int, point: Point, reason: KinematicsError) -> None:
        super().__init__(f"trajectory point {index} at ({point[0]:.3f}, {point[1]:.3f}) is not usable: {reason}")
        self.index = index
        self.point = point
        self.reason = reason


@dataclass(frozen=True)
class Trajectory:
    """A validated straight-line trajectory.

    Attributes:
        points_xy: (N, 2) gripper points in mm, first = start, last = end.
        joints_deg: (N, 2) joint angles (alpha, beta) in degrees, elbows-out.
        margins_deg: (N,) singularity margin mu at each point, in degrees.
    """

    points_xy: np.ndarray
    joints_deg: np.ndarray
    margins_deg: np.ndarray

    def __len__(self) -> int:
        return int(self.points_xy.shape[0])

    @property
    def min_margin_deg(self) -> float:
        return float(self.margins_deg.min())


def _as_point(name: str, value: Point) -> tuple[float, float]:
    try:
        if len(value) != 2:
            raise ValueError
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError):
        raise KinematicsError(f"{name} must be an (x, y) pair of numbers, got {value!r}") from None
    if not (math.isfinite(x) and math.isfinite(y)):
        raise KinematicsError(f"{name} must be finite, got ({x!r}, {y!r})")
    return (x, y)


def straight_line_points(start: Point, end: Point, ds: float | None = None) -> np.ndarray:
    """Points of the straight line start -> end, spaced at most ``ds`` mm apart.

    The segment count is ``ceil(length / ds)`` and the spacing is uniform, so
    every segment is <= ds. A zero-length path is a single point, not two
    identical ones. Returns an (N, 2) array including both end points.
    """
    if ds is None:
        ds = get_settings().motion.trajectory_step_mm
    if not (isinstance(ds, (int, float)) and not isinstance(ds, bool) and math.isfinite(ds) and ds > 0):
        raise KinematicsError(f"ds must be a finite number > 0, got {ds!r}")
    x0, y0 = _as_point("start", start)
    x1, y1 = _as_point("end", end)
    length = math.hypot(x1 - x0, y1 - y0)
    if length == 0.0:
        return np.array([[x0, y0]], dtype=float)
    segments = max(1, math.ceil(length / ds))
    t = np.linspace(0.0, 1.0, segments + 1)
    return np.column_stack((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))


def straight_line(
    workspace: Workspace,
    start: Point,
    end: Point,
    ds: float | None = None,
) -> Trajectory:
    """Plan a straight-line trajectory; validate EVERY point before returning.

    Args:
        workspace: provides the kinematic model and the reach / mu check.
        start, end: gripper points in mm.
        ds: maximum segment length in mm (default: motion.trajectory_step_mm).

    Raises:
        TrajectoryError: a point is out of reach or has mu below the workspace
            margin; ``index`` and ``point`` identify the first such point. Raised
            before any joint solution is returned.
        KinematicsError: invalid ``start``, ``end`` or ``ds``.
    """
    points = straight_line_points(start, end, ds)

    # Phase 1: check every point, in order. Nothing is built until all pass.
    margins = np.empty(len(points), dtype=float)
    for index, (x, y) in enumerate(points):
        try:
            margins[index] = workspace.check_point(float(x), float(y))
        except KinematicsError as exc:
            raise TrajectoryError(index, (float(x), float(y)), exc) from exc

    # Phase 2: all points are valid, so the joint solutions cannot fail on reach.
    joints = np.array([workspace.model.ik(float(x), float(y)) for x, y in points], dtype=float)
    return Trajectory(points_xy=points, joints_deg=joints, margins_deg=margins)
