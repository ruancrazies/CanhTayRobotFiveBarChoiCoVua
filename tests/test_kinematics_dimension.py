"""Tests for desktop_app/kinematics/dimension.py: board geometry, workspace, zones."""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

import pytest
import yaml

from desktop_app.kinematics.dimension import (
    PROPOSED_CAPTURE_SLOTS,
    PROPOSED_PARK_POSITION,
    PROPOSED_RESERVE_SLOTS,
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

# TEST-ONLY slots for exercising the validation code with small lists. The robot's
# PROPOSED layout (still [TO CONFIRM]) is tested separately through Zones.proposed.
TEST_SLOTS_LEFT = ((-200.0, 300.0), (-200.0, 250.0))
TEST_SLOTS_RIGHT = ((200.0, 300.0), (200.0, 250.0))
TEST_PARK = (0.0, 100.0)


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
# Zones: lists of slots, [TO CONFIRM], unset by default
# --------------------------------------------------------------------------- #

ZONE_NAMES = ("capture_zone", "reserve_zone", "park_position")


def test_zones_are_none_until_provided(board: BoardGeometry, workspace: Workspace) -> None:
    zones = Zones(board, workspace)
    assert zones.undefined == ZONE_NAMES
    for name in ZONE_NAMES:
        assert not zones.is_defined(name)
        with pytest.raises(ZoneNotDefinedError, match="TO CONFIRM"):
            getattr(zones, name)


def test_undefined_zone_error_names_the_zone(board: BoardGeometry, workspace: Workspace) -> None:
    zones = Zones(board, workspace, capture_zone=TEST_SLOTS_LEFT)
    with pytest.raises(ZoneNotDefinedError, match="reserve_zone"):
        zones.reserve_zone
    with pytest.raises(ZoneNotDefinedError, match="park_position"):
        zones.park_position


def test_zone_errors_are_kinematics_errors() -> None:
    assert issubclass(ZoneNotDefinedError, KinematicsError)
    assert issubclass(ZoneValidationError, KinematicsError)


def test_provided_slots_are_returned_as_tuples_in_order(board: BoardGeometry, workspace: Workspace) -> None:
    zones = Zones(
        board, workspace, capture_zone=list(TEST_SLOTS_LEFT), reserve_zone=TEST_SLOTS_RIGHT, park_position=TEST_PARK
    )
    assert zones.capture_zone == TEST_SLOTS_LEFT and isinstance(zones.capture_zone, tuple)
    assert zones.reserve_zone == TEST_SLOTS_RIGHT and isinstance(zones.reserve_zone, tuple)
    assert zones.park_position == TEST_PARK
    assert zones.undefined == ()
    assert all(zones.is_defined(name) for name in ZONE_NAMES)


def test_zones_can_be_provided_one_at_a_time(board: BoardGeometry, workspace: Workspace) -> None:
    zones = Zones(board, workspace, park_position=TEST_PARK)
    assert zones.park_position == TEST_PARK
    assert zones.undefined == ("capture_zone", "reserve_zone")
    only_reserve = Zones(board, workspace, reserve_zone=TEST_SLOTS_RIGHT)
    assert only_reserve.undefined == ("capture_zone", "park_position")


def test_is_defined_rejects_unknown_names(board: BoardGeometry, workspace: Workspace) -> None:
    with pytest.raises(KeyError):
        Zones(board, workspace).is_defined("bin")


# ---- the proposed layout (still [TO CONFIRM]) ------------------------------ #


def test_proposed_slot_layout_matches_the_owners_proposal() -> None:
    assert len(PROPOSED_CAPTURE_SLOTS) == 15
    assert {x for x, _ in PROPOSED_CAPTURE_SLOTS} == {-190.0, -225.0, -260.0}
    assert {y for _, y in PROPOSED_CAPTURE_SLOTS} == {160.0, 195.0, 230.0, 265.0, 300.0}
    assert set(PROPOSED_CAPTURE_SLOTS) == {(x, y) for x in (-190.0, -225.0, -260.0) for y in (160.0, 195.0, 230.0, 265.0, 300.0)}
    assert len(PROPOSED_RESERVE_SLOTS) == 4
    assert set(PROPOSED_RESERVE_SLOTS) == {(190.0, 200.0), (190.0, 235.0), (225.0, 200.0), (225.0, 235.0)}
    assert PROPOSED_PARK_POSITION == (0.0, 120.0)
    assert len(set(PROPOSED_CAPTURE_SLOTS)) == 15 and len(set(PROPOSED_RESERVE_SLOTS)) == 4


def test_proposed_fill_order_is_x_major(board: BoardGeometry) -> None:
    assert PROPOSED_CAPTURE_SLOTS[:5] == tuple((-190.0, y) for y in (160.0, 195.0, 230.0, 265.0, 300.0))
    assert PROPOSED_CAPTURE_SLOTS[5] == (-225.0, 160.0)
    assert PROPOSED_RESERVE_SLOTS == ((190.0, 200.0), (190.0, 235.0), (225.0, 200.0), (225.0, 235.0))


def test_proposed_zones_validate_every_slot(
    board: BoardGeometry, workspace: Workspace, model: FiveBarKinematics, caplog: pytest.LogCaptureFixture
) -> None:
    with caplog.at_level(logging.WARNING, logger="desktop_app.kinematics.dimension"):
        zones = Zones.proposed(board, workspace)
    assert zones.capture_zone == PROPOSED_CAPTURE_SLOTS
    assert zones.reserve_zone == PROPOSED_RESERVE_SLOTS
    assert zones.park_position == PROPOSED_PARK_POSITION
    assert zones.undefined == ()
    # independent re-check of every single slot, against the model, not through Zones
    for slot in (*zones.capture_zone, *zones.reserve_zone, zones.park_position):
        assert not board.is_on_board(*slot), slot
        assert model.is_reachable(*slot), slot
        assert model.margin_at_point(*slot) >= MIN_SINGULARITY_MARGIN_DEG, slot
    assert any("PROPOSED" in r.getMessage() and "TO CONFIRM" in r.getMessage() for r in caplog.records)


def test_proposed_margins(model: FiveBarKinematics) -> None:
    margins = {slot: model.margin_at_point(*slot) for slot in (*PROPOSED_CAPTURE_SLOTS, *PROPOSED_RESERVE_SLOTS, PROPOSED_PARK_POSITION)}
    assert min(margins.values()) == pytest.approx(margins[PROPOSED_PARK_POSITION])
    assert margins[PROPOSED_PARK_POSITION] == pytest.approx(29.4, abs=0.05)
    assert margins[(-260.0, 300.0)] == pytest.approx(33.3, abs=0.05)  # tightest capture slot


def test_every_square_can_reach_every_proposed_slot_by_a_straight_line(
    board: BoardGeometry, workspace: Workspace
) -> None:
    from desktop_app.kinematics.trajectory import straight_line

    slots = (*PROPOSED_CAPTURE_SLOTS, *PROPOSED_RESERVE_SLOTS, PROPOSED_PARK_POSITION)
    for square, center in board.all_square_centers().items():
        for slot in slots:
            for start, end in ((center, slot), (slot, center)):
                assert straight_line(workspace, start, end).min_margin_deg >= MIN_SINGULARITY_MARGIN_DEG, (square, slot)


def test_proposed_zones_follow_a_stricter_workspace(board: BoardGeometry, model: FiveBarKinematics) -> None:
    with pytest.raises(ZoneValidationError):  # the park point has mu about 29.4
        Zones.proposed(board, Workspace(model, min_margin_deg=35.0))


def test_proposed_zones_do_not_fit_a_larger_board(model: FiveBarKinematics, tmp_path: Path, hardware_yaml_data: dict[str, Any]) -> None:
    hardware_yaml_data["geometry"]["square_size_mm"] = 40.0  # board 8 * 40 + 52 = 372, half 186: slot x = -190 is still off
    hardware_yaml_data["geometry"]["board_border_mm"] = 40.0  # board 400, half 200: x = -190 is now on the board
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(hardware_yaml_data), encoding="utf-8")
    wide = BoardGeometry.from_config(load_hardware_config(path))
    with pytest.raises(ZoneValidationError, match="on the board"):
        Zones.proposed(wide, Workspace(model))


# ---- validation of every slot ----------------------------------------------- #

SLOT_ZONES = ("capture_zone", "reserve_zone")


@pytest.mark.parametrize("name", SLOT_ZONES)
def test_one_bad_slot_among_good_ones_is_rejected_and_named(
    board: BoardGeometry, workspace: Workspace, name: str
) -> None:
    good = [(-200.0, 300.0), (-200.0, 250.0), (-200.0, 200.0)]
    for bad_index in range(3):
        slots = list(good)
        slots[bad_index] = (0.0, 300.0)  # on the board
        with pytest.raises(ZoneValidationError, match=rf"{name} slot {bad_index}.*on the board"):
            Zones(board, workspace, **{name: slots})


@pytest.mark.parametrize("name", SLOT_ZONES)
def test_slot_on_the_board_or_its_border_is_rejected(board: BoardGeometry, workspace: Workspace, name: str) -> None:
    for on_board in [(0.0, 300.0), (-108.5, 191.5), (150.0, 300.0), (0.0, 150.0), (-150.0, 450.0)]:
        with pytest.raises(ZoneValidationError, match="on the board"):
            Zones(board, workspace, **{name: [(-200.0, 300.0), on_board]})


@pytest.mark.parametrize("name", SLOT_ZONES)
def test_slot_outside_the_workspace_is_rejected(board: BoardGeometry, workspace: Workspace, name: str) -> None:
    with pytest.raises(ZoneValidationError, match="out of reach") as info:
        Zones(board, workspace, **{name: [(-200.0, 300.0), (0.0, 600.0)]})
    assert isinstance(info.value.__cause__, OutOfReachError)


@pytest.mark.parametrize("name", SLOT_ZONES)
def test_slot_with_low_margin_is_rejected(
    board: BoardGeometry, workspace: Workspace, model: FiveBarKinematics, name: str
) -> None:
    # Off the board and reachable, but with the left arm fully stretched: mu = 0.
    angle = math.radians(60.0)
    stretched = (model.motor1_xy[0] + (model.m + model.n) * math.cos(angle), (model.m + model.n) * math.sin(angle))
    assert not board.is_on_board(*stretched) and model.is_reachable(*stretched)
    with pytest.raises(ZoneValidationError, match="singularity margin") as info:
        Zones(board, workspace, **{name: [(-200.0, 300.0), stretched]})
    assert isinstance(info.value.__cause__, SingularityError)


def test_park_position_is_validated_like_a_slot(board: BoardGeometry, workspace: Workspace, model: FiveBarKinematics) -> None:
    with pytest.raises(ZoneValidationError, match="on the board"):
        Zones(board, workspace, park_position=(0.0, 300.0))
    with pytest.raises(ZoneValidationError, match="out of reach"):
        Zones(board, workspace, park_position=(0.0, 600.0))
    angle = math.radians(60.0)
    with pytest.raises(ZoneValidationError, match="singularity margin"):
        Zones(board, workspace, park_position=(model.motor1_xy[0] + (model.m + model.n) * math.cos(angle), (model.m + model.n) * math.sin(angle)))


def test_slot_validation_uses_the_workspace_margin(board: BoardGeometry, model: FiveBarKinematics) -> None:
    assert model.margin_at_point(*TEST_PARK) < 25.0  # about 24.4
    Zones(board, Workspace(model, min_margin_deg=15.0), park_position=TEST_PARK)
    Zones(board, Workspace(model, min_margin_deg=15.0), capture_zone=[TEST_PARK])
    for kwargs in ({"park_position": TEST_PARK}, {"capture_zone": [TEST_PARK]}, {"reserve_zone": [TEST_PARK]}):
        with pytest.raises(ZoneValidationError):
            Zones(board, Workspace(model, min_margin_deg=25.0), **kwargs)


@pytest.mark.parametrize("name", SLOT_ZONES)
def test_empty_slot_list_is_rejected(board: BoardGeometry, workspace: Workspace, name: str) -> None:
    for empty in ([], ()):
        with pytest.raises(ZoneValidationError, match="at least one slot"):
            Zones(board, workspace, **{name: empty})


@pytest.mark.parametrize("name", SLOT_ZONES)
def test_a_single_point_is_no_longer_a_valid_zone(board: BoardGeometry, workspace: Workspace, name: str) -> None:
    # The previous API took one (x, y). Passing that now must fail loudly, not be misread as two slots.
    with pytest.raises(ZoneValidationError, match="slot 0 must be an"):
        Zones(board, workspace, **{name: (-200.0, 300.0)})


@pytest.mark.parametrize("name", SLOT_ZONES)
@pytest.mark.parametrize(
    "bad", ["ab", 5, [(1.0,)], [(1.0, 2.0, 3.0)], [(math.nan, 100.0)], [(0.0, math.inf)], [("x", "y")], ["ab"], [None]]
)
def test_malformed_slots_are_rejected(board: BoardGeometry, workspace: Workspace, name: str, bad: Any) -> None:
    with pytest.raises(ZoneValidationError):
        Zones(board, workspace, **{name: bad})


@pytest.mark.parametrize("bad", [(1.0,), (1.0, 2.0, 3.0), "ab", (math.nan, 100.0), (0.0, math.inf), ("x", "y"), 5])
def test_malformed_park_position_is_rejected(board: BoardGeometry, workspace: Workspace, bad: Any) -> None:
    with pytest.raises(ZoneValidationError):
        Zones(board, workspace, park_position=bad)


@pytest.mark.parametrize("name", SLOT_ZONES)
def test_repeated_slot_is_rejected(board: BoardGeometry, workspace: Workspace, name: str) -> None:
    with pytest.raises(ZoneValidationError, match="slot 2.*repeats slot 0"):
        Zones(board, workspace, **{name: [(-200.0, 300.0), (-200.0, 250.0), (-200.0, 300.0)]})


def test_a_point_shared_by_two_zones_is_rejected(board: BoardGeometry, workspace: Workspace) -> None:
    shared = (-200.0, 300.0)
    with pytest.raises(ZoneValidationError, match="coincides"):
        Zones(board, workspace, capture_zone=[shared], reserve_zone=[(-200.0, 250.0), shared])
    with pytest.raises(ZoneValidationError, match="coincides"):
        Zones(board, workspace, capture_zone=[shared], park_position=shared)


def test_slots_are_normalised_to_floats(board: BoardGeometry, workspace: Workspace) -> None:
    zones = Zones(board, workspace, capture_zone=[(-200, 300)], park_position=(0, 100))
    assert zones.capture_zone == ((-200.0, 300.0),)
    assert all(isinstance(v, float) for v in (*zones.capture_zone[0], *zones.park_position))
