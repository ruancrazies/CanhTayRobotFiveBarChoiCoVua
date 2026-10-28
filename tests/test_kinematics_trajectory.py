"""Tests for desktop_app/kinematics/trajectory.py (SPEC section 6.3)."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import pytest

from desktop_app.kinematics.dimension import BoardGeometry, Workspace
from desktop_app.kinematics.five_bar import (
    FiveBarKinematics,
    KinematicsError,
    OutOfReachError,
    SingularityError,
)
from desktop_app.kinematics.trajectory import (
    Trajectory,
    TrajectoryError,
    straight_line,
    straight_line_points,
)
from desktop_app.utils.settings import get_settings
from phase2_support import MIN_SINGULARITY_MARGIN_DEG

DEFAULT_MAX_SEGMENT_MM = get_settings().motion.trajectory_step_mm


@pytest.fixture(scope="module")
def model() -> FiveBarKinematics:
    return FiveBarKinematics.from_config()


@pytest.fixture(scope="module")
def workspace(model: FiveBarKinematics) -> Workspace:
    return Workspace(model)


@pytest.fixture(scope="module")
def squares() -> dict[str, tuple[float, float]]:
    return BoardGeometry.from_config().all_square_centers()


def _segment_lengths(points: np.ndarray) -> np.ndarray:
    return np.hypot(*np.diff(points, axis=0).T)


# --------------------------------------------------------------------------- #
# Point generation
# --------------------------------------------------------------------------- #


def test_default_segment_length_of_settings_is_2_mm() -> None:
    assert DEFAULT_MAX_SEGMENT_MM == 2.0


def test_segments_are_never_longer_than_ds_by_default() -> None:
    start, end = (-108.5, 191.5), (108.5, 408.5)
    points = straight_line_points(start, end)
    assert points[0] == pytest.approx(start) and points[-1] == pytest.approx(end)
    lengths = _segment_lengths(points)
    assert lengths.max() <= DEFAULT_MAX_SEGMENT_MM + 1e-9
    assert len(points) - 1 == math.ceil(math.dist(start, end) / DEFAULT_MAX_SEGMENT_MM)
    assert np.allclose(lengths, lengths[0])  # uniform spacing


@pytest.mark.parametrize("ds", [0.5, 1.0, 2.0, 5.0, 7.3, 50.0])
def test_ds_is_configurable(ds: float) -> None:
    start, end = (-100.0, 200.0), (120.0, 350.0)
    points = straight_line_points(start, end, ds)
    assert _segment_lengths(points).max() <= ds + 1e-9
    assert len(points) - 1 == math.ceil(math.dist(start, end) / ds)
    assert points[0] == pytest.approx(start) and points[-1] == pytest.approx(end)


def test_points_lie_on_the_straight_line() -> None:
    start, end = (-90.0, 210.0), (110.0, 390.0)
    points = straight_line_points(start, end)
    direction = np.array(end) - np.array(start)
    for point in points:
        offset = point - np.array(start)
        assert abs(direction[0] * offset[1] - direction[1] * offset[0]) < 1e-9 * np.linalg.norm(direction)


def test_path_shorter_than_ds_is_two_points() -> None:
    points = straight_line_points((0.0, 300.0), (0.0, 301.0))
    assert points.shape == (2, 2)
    assert points[0] == pytest.approx((0.0, 300.0)) and points[1] == pytest.approx((0.0, 301.0))


def test_path_of_exactly_ds_is_two_points() -> None:
    assert straight_line_points((0.0, 300.0), (0.0, 302.0)).shape == (2, 2)


def test_zero_length_path_is_a_single_point() -> None:
    points = straight_line_points((10.0, 300.0), (10.0, 300.0))
    assert points.shape == (1, 2)
    assert points[0] == pytest.approx((10.0, 300.0))


@pytest.mark.parametrize("ds", [0.0, -1.0, math.nan, math.inf, True, "2"])
def test_invalid_ds_raises(ds: Any) -> None:
    with pytest.raises(KinematicsError):
        straight_line_points((0.0, 300.0), (10.0, 300.0), ds)


def test_no_ds_means_the_step_of_settings_yaml() -> None:
    explicit = straight_line_points((0.0, 300.0), (10.0, 300.0), DEFAULT_MAX_SEGMENT_MM)
    assert straight_line_points((0.0, 300.0), (10.0, 300.0)) == pytest.approx(explicit)
    assert straight_line_points((0.0, 300.0), (10.0, 300.0), None) == pytest.approx(explicit)


@pytest.mark.parametrize(
    "start, end",
    [((math.nan, 300.0), (0.0, 300.0)), ((0.0, 300.0), (0.0, math.inf)), ((0.0,), (0.0, 300.0)), ("ab", (0.0, 300.0)), (None, (0.0, 3.0))],
)
def test_invalid_end_points_raise(start: Any, end: Any) -> None:
    with pytest.raises(KinematicsError):
        straight_line_points(start, end)


# --------------------------------------------------------------------------- #
# Planning
# --------------------------------------------------------------------------- #


def test_trajectory_over_the_board_diagonal(workspace: Workspace, model: FiveBarKinematics) -> None:
    traj = straight_line(workspace, (-108.5, 191.5), (108.5, 408.5))
    assert isinstance(traj, Trajectory)
    n = len(traj)
    assert traj.points_xy.shape == (n, 2) and traj.joints_deg.shape == (n, 2) and traj.margins_deg.shape == (n,)
    assert _segment_lengths(traj.points_xy).max() <= DEFAULT_MAX_SEGMENT_MM + 1e-9
    assert (traj.margins_deg >= MIN_SINGULARITY_MARGIN_DEG).all()
    assert traj.min_margin_deg == pytest.approx(26.0, abs=0.5)  # A1 -> H8 passes through A8/H8-like margins
    for (x, y), (alpha, beta) in zip(traj.points_xy, traj.joints_deg):
        assert model.fk(alpha, beta) == pytest.approx((x, y), abs=1e-6)


def test_trajectory_joint_solutions_are_the_ik_of_each_point(workspace: Workspace, model: FiveBarKinematics) -> None:
    traj = straight_line(workspace, (-50.0, 250.0), (60.0, 330.0), ds=5.0)
    for (x, y), joints in zip(traj.points_xy, traj.joints_deg):
        assert tuple(joints) == pytest.approx(model.ik(x, y))
    assert traj.margins_deg[0] == pytest.approx(model.margin_at_point(-50.0, 250.0))


def test_every_move_between_squares_is_valid(workspace: Workspace, squares: dict[str, tuple[float, float]]) -> None:
    # Corners and edge midpoints: every pair is a pick-and-place move the robot may be asked for.
    names = ["a1", "h1", "a8", "h8", "d1", "e8", "a4", "h5", "d4", "e5"]
    for source in names:
        for target in names:
            traj = straight_line(workspace, squares[source], squares[target])
            assert traj.min_margin_deg >= MIN_SINGULARITY_MARGIN_DEG, (source, target)


def test_start_and_end_are_checked_and_returned(workspace: Workspace) -> None:
    traj = straight_line(workspace, (-100.0, 200.0), (100.0, 400.0))
    assert traj.points_xy[0] == pytest.approx((-100.0, 200.0))
    assert traj.points_xy[-1] == pytest.approx((100.0, 400.0))


def test_zero_length_trajectory_is_one_point_not_two(workspace: Workspace, model: FiveBarKinematics) -> None:
    traj = straight_line(workspace, (0.0, 300.0), (0.0, 300.0))
    assert len(traj) == 1
    assert traj.points_xy.shape == (1, 2) and traj.joints_deg.shape == (1, 2) and traj.margins_deg.shape == (1,)
    assert tuple(traj.joints_deg[0]) == pytest.approx(model.ik(0.0, 300.0))


def test_zero_length_trajectory_at_a_bad_point_raises_at_index_0(workspace: Workspace) -> None:
    with pytest.raises(TrajectoryError) as info:
        straight_line(workspace, (0.0, 600.0), (0.0, 600.0))
    assert info.value.index == 0


def test_ds_is_passed_through(workspace: Workspace) -> None:
    coarse = straight_line(workspace, (-100.0, 200.0), (100.0, 200.0), ds=10.0)
    fine = straight_line(workspace, (-100.0, 200.0), (100.0, 200.0), ds=1.0)
    assert len(coarse) == 21 and len(fine) == 201


# --------------------------------------------------------------------------- #
# Errors: reported with the index of the bad point, before anything is returned
# --------------------------------------------------------------------------- #


def _expected_first_bad_index(workspace: Workspace, start: tuple[float, float], end: tuple[float, float], ds: float) -> int:
    """Independent re-derivation of the first failing index."""
    points = straight_line_points(start, end, ds)
    for index, (x, y) in enumerate(points):
        if not workspace.contains(float(x), float(y)):
            return index
    raise AssertionError("path is entirely inside the workspace")


def test_path_leaving_the_workspace_raises_with_the_index(workspace: Workspace) -> None:
    start, end = (0.0, 300.0), (0.0, 460.0)
    expected = _expected_first_bad_index(workspace, start, end, DEFAULT_MAX_SEGMENT_MM)
    assert 0 < expected < len(straight_line_points(start, end)) - 1  # fails in the middle, not at an end
    with pytest.raises(TrajectoryError) as info:
        straight_line(workspace, start, end)
    error = info.value
    assert error.index == expected
    assert error.point == pytest.approx(tuple(straight_line_points(start, end)[expected]))
    assert f"point {expected}" in str(error)
    assert isinstance(error.reason, SingularityError)  # the margin goes below 15 before reach is lost
    assert error.__cause__ is error.reason
    assert not workspace.contains(*error.point)
    assert workspace.contains(*straight_line_points(start, end)[expected - 1])


def test_unreachable_end_raises_with_the_last_index(workspace: Workspace) -> None:
    # One long segment: only the end point (index 1) is out of reach.
    with pytest.raises(TrajectoryError) as info:
        straight_line(workspace, (0.0, 300.0), (0.0, 500.0), ds=300.0)
    assert info.value.index == 1
    assert isinstance(info.value.reason, OutOfReachError)


def test_bad_start_raises_at_index_0(workspace: Workspace) -> None:
    with pytest.raises(TrajectoryError) as info:
        straight_line(workspace, (-50.0, 10.0), (0.0, 300.0))
    assert info.value.index == 0
    assert isinstance(info.value.reason, OutOfReachError)


def test_path_that_dips_out_of_the_workspace_in_the_middle_raises(workspace: Workspace) -> None:
    # Both end points are inside the workspace, but the straight line between them passes too
    # close to motor 1 (reach and margin are lost in the middle), so the end points alone are not enough.
    start, end = (-200.0, 20.0), (0.0, 100.0)
    assert workspace.contains(*start) and workspace.contains(*end)
    expected = _expected_first_bad_index(workspace, start, end, DEFAULT_MAX_SEGMENT_MM)
    last = len(straight_line_points(start, end)) - 1
    assert 0 < expected < last
    with pytest.raises(TrajectoryError) as info:
        straight_line(workspace, start, end)
    assert info.value.index == expected


def test_margin_override_changes_what_is_allowed(model: FiveBarKinematics) -> None:
    start, end = (0.0, 300.0), (0.0, 442.0)
    assert straight_line(Workspace(model, min_margin_deg=5.0), start, end).min_margin_deg < 20.0
    with pytest.raises(TrajectoryError):
        straight_line(Workspace(model, min_margin_deg=25.0), start, end)


class _CountingModel(FiveBarKinematics):
    """Records every ik() call in a shared event log."""

    events: list[str] = []  # replaced per test

    def ik(self, x: float, y: float) -> tuple[float, float]:
        self.events.append("ik")
        return super().ik(x, y)


class _CountingWorkspace(Workspace):
    """A workspace whose check_point is counted and can be told to fail at a given call number."""

    def __init__(self, model: FiveBarKinematics, events: list[str], fail_at_call: int | None = None) -> None:
        super().__init__(model)
        self.events = events
        self.fail_at_call = fail_at_call
        self.calls: list[tuple[float, float]] = []

    def check_point(self, x: float, y: float) -> float:
        self.events.append("check")
        self.calls.append((x, y))
        if self.fail_at_call is not None and len(self.calls) - 1 == self.fail_at_call:
            raise OutOfReachError(f"forced failure at call {self.fail_at_call}")
        return 30.0


def _counting_setup(model: FiveBarKinematics, fail_at_call: int | None) -> tuple[_CountingWorkspace, list[str]]:
    events: list[str] = []
    counting_model = _CountingModel(l=model.l, m=model.m, n=model.n, pulses_per_rev=model.pulses_per_rev)
    _CountingModel.events = events
    return _CountingWorkspace(counting_model, events, fail_at_call), events


def test_every_point_is_checked_before_any_joint_solution_is_computed(model: FiveBarKinematics) -> None:
    counting, events = _counting_setup(model, fail_at_call=None)
    start, end = (-100.0, 200.0), (100.0, 400.0)
    traj = straight_line(counting, start, end)
    n = len(traj)
    assert len(counting.calls) == n
    assert events == ["check"] * n + ["ik"] * n  # all checks first, then all IK solves


def test_a_bad_point_raises_before_anything_is_computed_or_returned(model: FiveBarKinematics) -> None:
    for fail_at in (0, 1, 37, 100):
        counting, events = _counting_setup(model, fail_at_call=fail_at)
        start, end = (-100.0, 200.0), (100.0, 400.0)
        total = len(straight_line_points(start, end))
        assert fail_at < total
        with pytest.raises(TrajectoryError) as info:
            straight_line(counting, start, end)
        assert info.value.index == fail_at
        assert info.value.point == pytest.approx(tuple(straight_line_points(start, end)[fail_at]))
        assert isinstance(info.value.reason, OutOfReachError)
        assert len(counting.calls) == fail_at + 1  # points were checked in order, stopping at the first bad one
        assert "ik" not in events  # no joint solution was computed, so nothing could be sent or returned


def test_a_failing_last_point_still_prevents_any_result(model: FiveBarKinematics) -> None:
    start, end = (-100.0, 200.0), (100.0, 400.0)
    last = len(straight_line_points(start, end)) - 1
    counting, events = _counting_setup(model, fail_at_call=last)
    with pytest.raises(TrajectoryError) as info:
        straight_line(counting, start, end)
    assert info.value.index == last
    assert len(counting.calls) == last + 1 and "ik" not in events


def test_the_workspace_is_asked_about_the_end_points_of_a_single_point_path(model: FiveBarKinematics) -> None:
    counting, events = _counting_setup(model, fail_at_call=None)
    traj = straight_line(counting, (0.0, 300.0), (0.0, 300.0))
    assert len(traj) == 1 and len(counting.calls) == 1 and events == ["check", "ik"]
