"""Tests for desktop_app/kinematics/dimension.py: board geometry, workspace, zones."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest
import yaml

from desktop_app.kinematics.dimension import (
    BoardExtent,
    BoardGeometry,
    Workspace,
    ZoneNotDefinedError,
    ZoneValidationError,
    Zones,
)
from desktop_app.kinematics.five_bar import (
    MIN_SINGULARITY_MARGIN_DEG,
    FiveBarKinematics,
    KinematicsError,
    OutOfReachError,
    SingularityError,
)
from desktop_app.utils.hardware_config import HardwareConfig, load_hardware_config

# TEST-ONLY coordinates. They are NOT the robot's capture zone, reserve zone or
# park position (those are [TO CONFIRM] and unset); they are simply points that
# are off the board and inside the workspace so the validation code can be exercised.
TEST_OFF_BOARD_LEFT = (-200.0, 300.0)
TEST_OFF_BOARD_RIGHT = (200.0, 300.0)
TEST_OFF_BOARD_NEAR = (0.0, 100.0)


@pytest.fixture(scope="module")
def cfg() -> HardwareConfig:
    return load_hardware_config()


@pytest.fixture(scope="module")
def model(cfg: HardwareConfig) -> FiveBarKinematics:
    return FiveBarKinematics.from_config(cfg)


@pytest.fixture(scope="module")
def board(cfg: HardwareConfig) -> BoardGeometry:
    return BoardGeometry.from_config(cfg)


@pytest.fixture(scope="module")
def workspace(model: FiveBarKinematics) -> Workspace:
    return Workspace(model)


# --------------------------------------------------------------------------- #
# Square centres and board extent
# --------------------------------------------------------------------------- #


def test_corner_square_centres_match_the_spec(board: BoardGeometry) -> None:
    assert board.square_center(0, 0) == pytest.approx((-108.5, 191.5))  # A1
    assert board.square_center(7, 0) == pytest.approx((108.5, 191.5))  # H1
    assert board.square_center(0, 7) == pytest.approx((-108.5, 408.5))  # A8
    assert board.square_center(7, 7) == pytest.approx((108.5, 408.5))  # H8
    assert board.square_center_by_name("e4") == pytest.approx((-108.5 + 31 * 4, 191.5 + 31 * 3))


def test_square_formula_for_every_square(board: BoardGeometry) -> None:
    for file in range(8):
        for rank in range(8):
            assert board.square_center(file, rank) == pytest.approx((-108.5 + 31.0 * file, 191.5 + 31.0 * rank))


def test_all_square_centres(board: BoardGeometry) -> None:
    centers = board.all_square_centers()
    assert len(centers) == 64
    assert list(centers)[:3] == ["a1", "a2", "a3"] and list(centers)[-1] == "h8"
    assert len(set(centers.values())) == 64
    assert centers["a1"] == pytest.approx((-108.5, 191.5))
    assert centers["h8"] == pytest.approx((108.5, 408.5))
    assert all(board.is_on_board(*xy) for xy in centers.values())


def test_square_names_round_trip(board: BoardGeometry) -> None:
    for file in range(8):
        for rank in range(8):
            name = board.square_name(file, rank)
            assert board.square_center_by_name(name) == board.square_center(file, rank)
            assert board.square_center_by_name(name.upper()) == board.square_center(file, rank)
    assert board.square_name(0, 0) == "a1" and board.square_name(7, 7) == "h8"


def test_rank_1_is_nearest_and_file_a_is_left(board: BoardGeometry) -> None:
    assert board.square_center(0, 0)[0] < 0 < board.square_center(7, 0)[0]
    assert board.square_center(0, 0)[1] < board.square_center(0, 7)[1]


@pytest.mark.parametrize("name", ["", "a", "i1", "a0", "a9", "1a", "a-1", "aa1", "e", None, 41])
def test_invalid_square_names_raise(board: BoardGeometry, name: Any) -> None:
    with pytest.raises(KinematicsError):
        board.square_center_by_name(name)


@pytest.mark.parametrize("file, rank", [(-1, 0), (0, -1), (8, 0), (0, 8), (0.0, 0), (0, 1.5), (True, 0), ("a", 0)])
def test_invalid_square_indices_raise(board: BoardGeometry, file: Any, rank: Any) -> None:
    with pytest.raises(KinematicsError):
        board.square_center(file, rank)


def test_board_extent_is_the_300_mm_board_centred_on_the_y_axis(board: BoardGeometry) -> None:
    assert board.extent == BoardExtent(x_min=-150.0, x_max=150.0, y_min=150.0, y_max=450.0)
    assert board.extent.contains(0.0, 300.0)  # board centre (0, 300)
    assert board.is_on_board(150.0, 150.0)  # border corner counts as board
    assert not board.is_on_board(150.1, 300.0)
    assert not board.is_on_board(0.0, 149.9)
    assert not board.is_on_board(0.0, 450.1)


def test_board_follows_the_config(tmp_path: Path, hardware_yaml_data: dict[str, Any]) -> None:
    hardware_yaml_data["geometry"]["square_size_mm"] = 30.0
    hardware_yaml_data["geometry"]["board_border_mm"] = 20.0
    hardware_yaml_data["geometry"]["motor_to_board_edge_mm"] = 160.0
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(hardware_yaml_data), encoding="utf-8")
    changed = BoardGeometry.from_config(load_hardware_config(path))

    # board = 8 * 30 + 2 * 20 = 280 mm
    assert changed.extent == BoardExtent(-140.0, 140.0, 160.0, 440.0)
    assert changed.square_center(0, 0) == pytest.approx((-140.0 + 20.0 + 15.0, 160.0 + 20.0 + 15.0))
    assert changed.square_center(7, 7) == pytest.approx((140.0 - 20.0 - 15.0, 440.0 - 20.0 - 15.0))


def test_default_geometry_loads_from_the_default_config(board: BoardGeometry) -> None:
    assert BoardGeometry.from_config() == board


# --------------------------------------------------------------------------- #
# Workspace
# --------------------------------------------------------------------------- #


def test_every_square_is_in_the_workspace(workspace: Workspace, board: BoardGeometry) -> None:
    for name, (x, y) in board.all_square_centers().items():
        assert workspace.contains(x, y), name
        assert workspace.check_point(x, y) >= MIN_SINGULARITY_MARGIN_DEG


def test_workspace_default_margin_is_the_named_constant(workspace: Workspace) -> None:
    assert workspace.min_margin_deg == MIN_SINGULARITY_MARGIN_DEG == 15.0


def test_check_point_returns_the_margin(workspace: Workspace, model: FiveBarKinematics) -> None:
    assert workspace.check_point(0.0, 300.0) == pytest.approx(model.margin_at_point(0.0, 300.0))


def test_out_of_reach_point_raises(workspace: Workspace) -> None:
    with pytest.raises(OutOfReachError):
        workspace.check_point(0.0, 500.0)
    assert not workspace.contains(0.0, 500.0)


def test_low_margin_point_raises(workspace: Workspace, model: FiveBarKinematics) -> None:
    assert model.is_reachable(0.0, 445.0)  # reachable, yet mu is about 11 degrees
    with pytest.raises(SingularityError):
        workspace.check_point(0.0, 445.0)
    assert not workspace.contains(0.0, 445.0)


def test_workspace_margin_can_be_overridden(model: FiveBarKinematics) -> None:
    strict = Workspace(model, min_margin_deg=25.0)
    lenient = Workspace(model, min_margin_deg=5.0)
    assert not strict.contains(0.0, 440.0)  # mu about 20.6
    assert Workspace(model).contains(0.0, 440.0)
    assert lenient.contains(0.0, 445.0)  # mu about 11.4


@pytest.mark.parametrize("bad", [-1.0, 90.0, 120.0, math.nan, math.inf])
def test_workspace_rejects_a_nonsensical_margin(model: FiveBarKinematics, bad: float) -> None:
    with pytest.raises(KinematicsError):
        Workspace(model, min_margin_deg=bad)


def test_workspace_exposes_its_model(workspace: Workspace, model: FiveBarKinematics) -> None:
    assert workspace.model is model


# --------------------------------------------------------------------------- #
# Zones: [TO CONFIRM], unset by default
# --------------------------------------------------------------------------- #


def test_zones_are_none_until_provided(board: BoardGeometry, workspace: Workspace) -> None:
    zones = Zones(board, workspace)
    assert zones.undefined == ("capture_zone", "reserve_zone", "park_position")
    for name in ("capture_zone", "reserve_zone", "park_position"):
        assert not zones.is_defined(name)
        with pytest.raises(ZoneNotDefinedError, match="TO CONFIRM"):
            getattr(zones, name)


def test_undefined_zone_error_names_the_zone(board: BoardGeometry, workspace: Workspace) -> None:
    zones = Zones(board, workspace, capture_zone=TEST_OFF_BOARD_LEFT)
    with pytest.raises(ZoneNotDefinedError, match="reserve_zone"):
        zones.reserve_zone
    with pytest.raises(ZoneNotDefinedError, match="park_position"):
        zones.park_position


def test_zone_not_defined_is_a_kinematics_error() -> None:
    assert issubclass(ZoneNotDefinedError, KinematicsError)
    assert issubclass(ZoneValidationError, KinematicsError)


def test_provided_zones_are_returned(board: BoardGeometry, workspace: Workspace) -> None:
    zones = Zones(
        board,
        workspace,
        capture_zone=TEST_OFF_BOARD_LEFT,
        reserve_zone=TEST_OFF_BOARD_RIGHT,
        park_position=TEST_OFF_BOARD_NEAR,
    )
    assert zones.capture_zone == TEST_OFF_BOARD_LEFT
    assert zones.reserve_zone == TEST_OFF_BOARD_RIGHT
    assert zones.park_position == TEST_OFF_BOARD_NEAR
    assert zones.undefined == ()
    assert all(zones.is_defined(name) for name in ("capture_zone", "reserve_zone", "park_position"))


def test_zones_can_be_provided_one_at_a_time(board: BoardGeometry, workspace: Workspace) -> None:
    zones = Zones(board, workspace, park_position=TEST_OFF_BOARD_NEAR)
    assert zones.park_position == TEST_OFF_BOARD_NEAR
    assert zones.undefined == ("capture_zone", "reserve_zone")


def test_is_defined_rejects_unknown_names(board: BoardGeometry, workspace: Workspace) -> None:
    with pytest.raises(KeyError):
        Zones(board, workspace).is_defined("bin")


@pytest.mark.parametrize("name", ["capture_zone", "reserve_zone", "park_position"])
def test_zone_on_the_board_is_rejected(board: BoardGeometry, workspace: Workspace, name: str) -> None:
    for on_board in [(0.0, 300.0), (-108.5, 191.5), (150.0, 300.0), (0.0, 150.0)]:  # last two: on the border
        with pytest.raises(ZoneValidationError, match="on the board"):
            Zones(board, workspace, **{name: on_board})


@pytest.mark.parametrize("name", ["capture_zone", "reserve_zone", "park_position"])
def test_zone_outside_the_workspace_is_rejected(board: BoardGeometry, workspace: Workspace, name: str) -> None:
    with pytest.raises(ZoneValidationError, match="out of reach") as info:
        Zones(board, workspace, **{name: (0.0, 600.0)})
    assert isinstance(info.value.__cause__, OutOfReachError)


@pytest.mark.parametrize("name", ["capture_zone", "reserve_zone", "park_position"])
def test_zone_with_low_margin_is_rejected(
    board: BoardGeometry, workspace: Workspace, model: FiveBarKinematics, name: str
) -> None:
    # Off the board and reachable, but with the left arm fully stretched: mu = 0.
    angle = math.radians(60.0)
    x = model.motor1_xy[0] + (model.m + model.n) * math.cos(angle)
    y = (model.m + model.n) * math.sin(angle)
    assert not board.is_on_board(x, y) and model.is_reachable(x, y)
    with pytest.raises(ZoneValidationError, match="singularity margin") as info:
        Zones(board, workspace, **{name: (x, y)})
    assert isinstance(info.value.__cause__, SingularityError)


def test_zone_validation_uses_the_workspace_margin(board: BoardGeometry, model: FiveBarKinematics) -> None:
    point = TEST_OFF_BOARD_NEAR  # mu about 24.4 degrees
    assert model.margin_at_point(*point) < 25.0
    Zones(board, Workspace(model, min_margin_deg=15.0), park_position=point)
    with pytest.raises(ZoneValidationError):
        Zones(board, Workspace(model, min_margin_deg=25.0), park_position=point)


@pytest.mark.parametrize("bad", [(1.0,), (1.0, 2.0, 3.0), "ab", (math.nan, 100.0), (0.0, math.inf), ("x", "y"), 5])
def test_malformed_zone_values_are_rejected(board: BoardGeometry, workspace: Workspace, bad: Any) -> None:
    with pytest.raises(ZoneValidationError):
        Zones(board, workspace, capture_zone=bad)


def test_zone_validation_follows_the_board_size(tmp_path: Path, hardware_yaml_data: dict[str, Any], model: FiveBarKinematics) -> None:
    # A point just left of the default board edge becomes "on the board" for a wider board.
    point = (-170.0, 300.0)
    assert Zones(BoardGeometry.from_config(), Workspace(model), capture_zone=point).capture_zone == point
    hardware_yaml_data["geometry"]["board_border_mm"] = 40.0  # board 328 mm, half 164 ... point still off
    hardware_yaml_data["geometry"]["square_size_mm"] = 34.0  # board 8 * 34 + 80 = 352, half 176: point on board
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(hardware_yaml_data), encoding="utf-8")
    wide = BoardGeometry.from_config(load_hardware_config(path))
    with pytest.raises(ZoneValidationError, match="on the board"):
        Zones(wide, Workspace(model), capture_zone=point)
